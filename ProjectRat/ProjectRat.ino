#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <usb_mouse.h>

#include "config.h"
#include "imu_lsm6dsv16x.h"
#include "sensor_fusion.h"
#include "buttons.h"          // --- BUTTON SUBSYSTEM ---

IMU lsm6dsv16x;

float accelXg = 0.0f, accelYg = 0.0f, accelZg = 0.0f;
float gyroXdps = 0.0f, gyroYdps = 0.0f, gyroZdps = 0.0f;
uint8_t imuMlResult = 0;
bool imuMlValid = false;
bool imuQvarContact = false;
bool imuQvarValid = false;
bool debouncedGateHeld = false;  // debounced MOTION_GATE_PIN state, maintained in readSensors()
FusedMotion fusedMotion = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};

#if SD_LOG_ENABLE
bool sdCardAvailable = false;
File driftSummaryFile;
bool driftSummaryFileReady = false;
#endif

#define DRIFT_TEST_DURATION_MS   30000
#define DRIFT_TEST_SAMPLE_MS     20   // nominal target only — actual dt is now measured via micros()
#define DRIFT_TEST_RUN_COUNT     50    // number of consecutive automated runs
#define DRIFT_TEST_PAUSE_MS      5000 // pause between consecutive runs
#define DRIFT_SUMMARY_FILE_NAME  "drift_summary.csv"

struct DriftResult {
  int sampleCount;
  float meanAx, meanAy, meanAz;
  float stdAx, stdAy, stdAz;
  float meanGx, meanGy, meanGz;
  float stdGx, stdGy, stdGz;
  float meanTempC;      // mean die temperature over the run (degC)
  float meanDtMs;        // mean measured sample interval over the run (ms) — diagnostic,
                         // confirms actual vs. nominal DRIFT_TEST_SAMPLE_MS
  float posXDriftMm, posYDriftMm;
};

void setupHardware();
void calibrateGyroBiasAtBoot();
void readSensors();
void updateHID();
void reportStatus();
DriftResult runDriftCharacterization(int runNumber);
void runDriftBatch();
#if SD_LOG_ENABLE
bool initSdCard();
bool initDriftSummaryLog();
void logDriftSummary(int runNumber, const DriftResult& r);
#endif

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println(F("Project Rat prototype starting..."));
  Wire.begin();
  setupHardware();

#if RUN_DRIFT_BATCH_AT_BOOT
  runDriftBatch();
#endif
}

void loop() {
  readSensors();

  Buttons::update();        // --- BUTTON SUBSYSTEM ---

  updateHID();

  // Periodic status print, rate-limited independent of the main loop
  // rate (MAIN_LOOP_DELAY_MS) so the serial monitor is readable rather
  // than flooded. reportStatus() previously only ran once at the end
  // of setupHardware() — this is what actually makes gate/button/motion
  // state visible live while bench testing.
  static unsigned long lastReportMs = 0;
  if (millis() - lastReportMs >= STATUS_REPORT_INTERVAL_MS) {
    reportStatus();
    lastReportMs = millis();
  }

  delay(MAIN_LOOP_DELAY_MS);
}

void calibrateGyroBiasAtBoot() {
  const int sampleCount = 64;
  float sumGx = 0.0f;
  float sumGy = 0.0f;
  float sumGz = 0.0f;

  for (int i = 0; i < sampleCount; ++i) {
    int16_t rawAx = 0, rawAy = 0, rawAz = 0;
    int16_t rawGx = 0, rawGy = 0, rawGz = 0;
    if (!lsm6dsv16x.readAll(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz)) {
      continue;
    }

    sumGx += (float)rawGx * LSM6DSV16X::GYRO_SENSITIVITY_DPS_PER_LSB;
    sumGy += (float)rawGy * LSM6DSV16X::GYRO_SENSITIVITY_DPS_PER_LSB;
    sumGz += (float)rawGz * LSM6DSV16X::GYRO_SENSITIVITY_DPS_PER_LSB;
    delay(10);
  }

  SensorCalibration calibration = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  calibration.gyroBiasXDps = -sumGx / max(1, sampleCount);
  calibration.gyroBiasYDps = -sumGy / max(1, sampleCount);
  calibration.gyroBiasZDps = -sumGz / max(1, sampleCount);
  setSensorCalibration(calibration);

  Serial.print(F("Gyro bias calibration: X/Y/Z = "));
  Serial.print(calibration.gyroBiasXDps, 3);
  Serial.print(F(" / "));
  Serial.print(calibration.gyroBiasYDps, 3);
  Serial.print(F(" / "));
  Serial.print(calibration.gyroBiasZDps, 3);
  Serial.println(F(" dps"));
}

void setupHardware() {
  bool imuOk = lsm6dsv16x.begin(Wire, IMU_I2C_ADDR);
  if (!imuOk) {
    Serial.println(F("IMU initialization failed."));
  } else {
    Serial.println(F("IMU initialized."));

    calibrateGyroBiasAtBoot();

    bool qvarOk = lsm6dsv16x.configureQvar();
    bool mlOk = lsm6dsv16x.configureMl();


    if (!qvarOk) {
      Serial.println(F("IMU Qvar configuration unavailable."));
    } else {
      Serial.println(F("IMU Qvar configured."));
    }

    if (!mlOk) {
      Serial.println(F("IMU ML configuration unavailable."));
    } else {
      Serial.println(F("IMU ML configured."));
    }
  }

#if SD_LOG_ENABLE
  if (initSdCard()) {
    Serial.println(F("SD card initialized."));
  } else {
    Serial.println(F("SD card init failed."));
  }

  if (initDriftSummaryLog()) {
    Serial.println(F("Drift summary log initialized."));
  } else {
    Serial.println(F("Drift summary log failed."));
  }
#endif

  Buttons::begin();         // --- BUTTON SUBSYSTEM ---

  pinMode(MOTION_GATE_PIN, INPUT_PULLUP);  // --- MOTION GATE (acoustic stand-in) ---

  readSensors();
  reportStatus();
}

void readSensors() {
  // Raw counts feed fuseSensorData() directly. Physical g/dps units are
  // still derived here for reportStatus(), via a single readAll() +
  // manual scale multiply (no second I2C transaction through
  // readAllPhysical()).
  int16_t rawAx = 0, rawAy = 0, rawAz = 0, rawGx = 0, rawGy = 0, rawGz = 0;
  bool imuOk = lsm6dsv16x.readAll(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);

  if (!imuOk) {
    Serial.println(F("IMU read failed."));
  } else {
    accelXg = (float)rawAx * LSM6DSV16X::ACCEL_SENSITIVITY_G_PER_LSB;
    accelYg = (float)rawAy * LSM6DSV16X::ACCEL_SENSITIVITY_G_PER_LSB;
    accelZg = (float)rawAz * LSM6DSV16X::ACCEL_SENSITIVITY_G_PER_LSB;
    gyroXdps = (float)rawGx * LSM6DSV16X::GYRO_SENSITIVITY_DPS_PER_LSB;
    gyroYdps = (float)rawGy * LSM6DSV16X::GYRO_SENSITIVITY_DPS_PER_LSB;
    gyroZdps = (float)rawGz * LSM6DSV16X::GYRO_SENSITIVITY_DPS_PER_LSB;
  }

  imuQvarValid = lsm6dsv16x.readQvarState(imuQvarContact);
  imuMlValid = lsm6dsv16x.readMlState(imuMlResult);

  // Real measured dt for this cycle -- same class of fix already
  // applied to the drift characterization test, now needed here too:
  // both orientation integration (gyro) and velocity integration
  // (residual accel) accumulate a systematic error from any gap
  // between assumed and actual sample interval.
  static unsigned long lastMainLoopMicros = 0;
  unsigned long nowMicros = micros();
  float dtSeconds = (lastMainLoopMicros == 0)
                       ? (MAIN_LOOP_DELAY_MS / 1000.0f)
                       : (nowMicros - lastMainLoopMicros) / 1000000.0f;
  lastMainLoopMicros = nowMicros;

  // Orientation (roll/pitch) is a real physical property of the mouse
  // and must be tracked continuously, independent of whether the
  // acoustic gate currently permits translational motion -- otherwise
  // the first gravity-compensated sample after a gate opens would be
  // working from stale tilt data.
  if (dtSeconds <= 0.0f) {
    dtSeconds = (MAIN_LOOP_DELAY_MS / 1000.0f);
  }

  if (imuOk) {
    updateOrientation(rawAx, rawAy, rawAz, rawGx, rawGy, dtSeconds);
  }

  float axG = (float)rawAx * LSM6DSV16X::ACCEL_SENSITIVITY_G_PER_LSB;
  float ayG = (float)rawAy * LSM6DSV16X::ACCEL_SENSITIVITY_G_PER_LSB;
  float gxDps = (float)rawGx * LSM6DSV16X::GYRO_SENSITIVITY_DPS_PER_LSB;
  float gyDps = (float)rawGy * LSM6DSV16X::GYRO_SENSITIVITY_DPS_PER_LSB;
  SensorCalibration calibration;
  getSensorCalibration(calibration);

  axG -= calibration.accelBiasXg;
  ayG -= calibration.accelBiasYg;
  gxDps -= calibration.gyroBiasXDps;
  gyDps -= calibration.gyroBiasYDps;

  float gravityX = 0.0f;
  float gravityY = 0.0f;
  getGravityReference(gravityX, gravityY);
  float residualAxG = axG - gravityX;
  float residualAyG = ayG - gravityY;
  float gyroMagDps = sqrtf(gxDps * gxDps + gyDps * gyDps);

  // --- ACOUSTIC STILLNESS GATE (single-pair stand-in, MOTION_GATE_PIN) ---
  // Checked FIRST and authoritative. While withheld, fuseSensorData()
  // is not called at all — the mouse simply produces no motion, full
  // stop, not "IMU said something but we discarded it." The instant
  // the gate transitions from held to released, resetFusionState()
  // zeroes the smoothing filter so residual motion from just before
  // the stop can't leak into the next held cycle.
  //
  // Debounced: raw pin state must stay stable for
  // MOTION_GATE_DEBOUNCE_MS before a change is accepted, matching the
  // protection Buttons A/B already get. Without this, a brief noise
  // glitch or marginal breadboard contact could register as a false
  // "held" long enough to produce real, if brief, unintended motion —
  // exactly the kind of thing that could show up as unexplained drift
  // while genuinely not touching the switch.
  static bool motionGateHeldPrev = false;
  static bool lastRawGateHeld = false;
  static unsigned long lastGateChangeMs = 0;

  bool rawGateHeld = (digitalRead(MOTION_GATE_PIN) == LOW);
  unsigned long nowMs = millis();
  if (rawGateHeld != lastRawGateHeld) {
    lastGateChangeMs = nowMs;
    lastRawGateHeld = rawGateHeld;
  }
  if ((nowMs - lastGateChangeMs) >= MOTION_GATE_DEBOUNCE_MS) {
    debouncedGateHeld = rawGateHeld;
  }
  bool motionGateHeld = debouncedGateHeld;

  static bool motionGateReady = false;
  static unsigned long motionGateOpenStartMs = 0;

  updateMotionState(motionGateHeld, residualAxG, residualAyG, gyroMagDps, dtSeconds);
  MotionState motionState = getMotionState();

  if (motionGateHeld) {
    if (!motionGateReady) {
      if (motionGateHeldPrev) {
        motionGateReady = false;
      } else {
        motionGateOpenStartMs = nowMs;
      }

      if ((nowMs - motionGateOpenStartMs) >= GATE_OPEN_SETTLE_MS) {
        motionGateReady = true;
      }
    }

    bool allowMotion = motionGateReady && (motionState == MotionState::Active || motionState == MotionState::Settling);

    if (allowMotion && imuOk) {
      fuseSensorData(rawAx, rawAy, rawGx, rawGy, dtSeconds, fusedMotion);
    } else {
      fusedMotion.dx = 0.0f;
      fusedMotion.dy = 0.0f;
    }
    fusedMotion.atRest = (motionState == MotionState::Still || motionState == MotionState::Idle);

    // Existing qvar/ML contact checks still apply as an additional
    // override on top of the acoustic gate (e.g. mouse lifted mid-move).
    if (imuQvarValid && !imuQvarContact) {
      fusedMotion.dx = 0.0f;
      fusedMotion.dy = 0.0f;
      fusedMotion.atRest = true;
    }
    if (imuMlValid && imuMlResult == 0) {
      fusedMotion.dx = 0.0f;
      fusedMotion.dy = 0.0f;
      fusedMotion.atRest = true;
    }
  } else {
    if (motionGateHeldPrev) {
      resetFusionState();
    }
    motionGateReady = false;
    fusedMotion.dx = 0.0f;
    fusedMotion.dy = 0.0f;
    fusedMotion.atRest = true;
  }

  motionGateHeldPrev = motionGateHeld;
}

void updateHID() {
#ifdef MOUSE_INTERFACE
  int16_t xMove = (int16_t)constrain((int32_t)roundf(fusedMotion.dx), -12, 12);
  int16_t yMove = (int16_t)constrain((int32_t)roundf(fusedMotion.dy), -12, 12);

  if (xMove != 0 || yMove != 0) {
    Mouse.move((int8_t)xMove, (int8_t)yMove, 0);
  }

  // --- BUTTON SUBSYSTEM: HID OUTPUT ---
  ButtonEvent aEvent = Buttons::getEvent(ButtonId::A);
  ButtonEvent bEvent = Buttons::getEvent(ButtonId::B);

  if (aEvent == ButtonEvent::Pressed) {
    Mouse.press(MOUSE_LEFT);
  } else if (aEvent == ButtonEvent::Released) {
    Mouse.release(MOUSE_LEFT);
  }

  if (bEvent == ButtonEvent::Pressed) {
    Mouse.press(MOUSE_RIGHT);
  } else if (bEvent == ButtonEvent::Released) {
    Mouse.release(MOUSE_RIGHT);
  }
#endif
}

void reportStatus() {
  Serial.print(F("IMU Accel X/Y/Z (g): "));
  Serial.print(accelXg, 5);
  Serial.print(F(" / "));
  Serial.print(accelYg, 5);
  Serial.print(F(" / "));
  Serial.println(accelZg, 5);

  Serial.print(F("IMU Gyro X/Y/Z (dps): "));
  Serial.print(gyroXdps, 5);
  Serial.print(F(" / "));
  Serial.print(gyroYdps, 5);
  Serial.print(F(" / "));
  Serial.println(gyroZdps, 5);

  Serial.print(F("Surface tilt: "));
  Serial.print(fusedMotion.surfaceTilt);
  Serial.print(F(", atRest: "));
  Serial.print(fusedMotion.atRest ? F("yes") : F("no"));
  Serial.print(F(", fused dx/dy: "));
  Serial.print(fusedMotion.dx, 4);
  Serial.print(F(" / "));
  Serial.print(fusedMotion.dy, 4);

  {
    float dbgResAx, dbgResAy;
    int dbgStillCount;
    bool dbgGyroStill;
    getFusionDebugState(dbgResAx, dbgResAy, dbgStillCount, dbgGyroStill);
    Serial.print(F(", residual ax/ay: "));
    Serial.print(dbgResAx, 4);
    Serial.print(F(" / "));
    Serial.print(dbgResAy, 4);
    Serial.print(F(", gyroStill: "));
    Serial.print(dbgGyroStill ? F("yes") : F("no"));
    Serial.print(F(", stillCount: "));
    Serial.print(dbgStillCount);
  }

  Serial.print(F(" IMU Qvar: "));
  if (imuQvarValid) {
    Serial.print(imuQvarContact ? F("contact") : F("no contact"));
  } else {
    Serial.print(F("unknown"));
  }

  Serial.print(F(" ML: "));
  if (imuMlValid) {
    Serial.print(imuMlResult);
  } else {
    Serial.print(F("unknown"));
  }

  Serial.print(F(" Motion gate: "));
  Serial.print(debouncedGateHeld ? F("permitted") : F("withheld"));

  Serial.println();
}

void runDriftBatch() {
  Serial.println(F("=========================================="));
  Serial.print(F("Starting automated drift batch: "));
  Serial.print(DRIFT_TEST_RUN_COUNT);
  Serial.println(F(" consecutive runs (no power cycle between)."));
  Serial.println(F("NOTE: for power-cycle bias variation, run separate"));
  Serial.println(F("batches with a manual USB unplug/replug in between."));
  Serial.println(F("=========================================="));

  for (int i = 1; i <= DRIFT_TEST_RUN_COUNT; i++) {
    DriftResult result = runDriftCharacterization(i);

#if SD_LOG_ENABLE
    logDriftSummary(i, result);
#endif

    if (i < DRIFT_TEST_RUN_COUNT) {
      Serial.print(F("Pausing "));
      Serial.print(DRIFT_TEST_PAUSE_MS / 1000);
      Serial.println(F("s before next run..."));
      delay(DRIFT_TEST_PAUSE_MS);
    }
  }

  Serial.println(F("=========================================="));
  Serial.println(F("Drift batch complete."));
  Serial.println(F("=========================================="));
}

DriftResult runDriftCharacterization(int runNumber) {
  Serial.print(F("=== Drift Characterization Test — Run "));
  Serial.print(runNumber);
  Serial.println(F(" ==="));
  Serial.println(F("Keep the mouse perfectly stationary for 30 seconds."));
  delay(3000); // grace period to stop touching it

  int sampleCount = 0;

  float sumAx = 0, sumAy = 0, sumAz = 0;
  float sumGx = 0, sumGy = 0, sumGz = 0;
  float sumAx2 = 0, sumAy2 = 0, sumAz2 = 0;
  float sumGx2 = 0, sumGy2 = 0, sumGz2 = 0;
  float sumTempC = 0;
  int tempSampleCount = 0;
  float sumDtMs = 0;

  float velX = 0, velY = 0;
  float posX = 0, posY = 0;

  unsigned long startTime = millis();

  // Measured-dt integration: dt is now the actual elapsed time between
  // consecutive successful reads, not the nominal DRIFT_TEST_SAMPLE_MS.
  // The previous fixed-dt version assumed 20 ms/sample; actual I2C read +
  // processing overhead measured ~21.4 ms/sample in the first 50-run batch,
  // which was a ~7% systematic error compounding through the double
  // integration. This removes that error source.
  unsigned long lastSampleMicros = micros();

  while (millis() - startTime < DRIFT_TEST_DURATION_MS) {
    float ax, ay, az, gx, gy, gz;
    if (lsm6dsv16x.readAllPhysical(ax, ay, az, gx, gy, gz)) {
      unsigned long nowMicros = micros();
      float dt = (nowMicros - lastSampleMicros) / 1000000.0f;
      lastSampleMicros = nowMicros;

      sumAx += ax; sumAy += ay; sumAz += az;
      sumGx += gx; sumGy += gy; sumGz += gz;
      sumAx2 += ax * ax; sumAy2 += ay * ay; sumAz2 += az * az;
      sumGx2 += gx * gx; sumGy2 += gy * gy; sumGz2 += gz * gz;
      sumDtMs += dt * 1000.0f;

      float tempC = 0.0f;
      if (lsm6dsv16x.readTemperatureC(tempC)) {
        sumTempC += tempC;
        tempSampleCount++;
      }

      float axMps2 = ax * LSM6DSV16X::G_TO_MPS2;
      float ayMps2 = ay * LSM6DSV16X::G_TO_MPS2;
      velX += axMps2 * dt;
      velY += ayMps2 * dt;
      posX += velX * dt;
      posY += velY * dt;

      sampleCount++;
    }
    delay(DRIFT_TEST_SAMPLE_MS);
  }

  DriftResult result = {0};
  result.sampleCount = sampleCount;

  if (sampleCount == 0) {
    Serial.println(F("No valid samples collected — check IMU connection."));
    return result;
  }

  result.meanAx = sumAx / sampleCount;
  result.meanAy = sumAy / sampleCount;
  result.meanAz = sumAz / sampleCount;
  result.meanGx = sumGx / sampleCount;
  result.meanGy = sumGy / sampleCount;
  result.meanGz = sumGz / sampleCount;

  result.stdAx = sqrt(max(0.0f, sumAx2 / sampleCount - result.meanAx * result.meanAx));
  result.stdAy = sqrt(max(0.0f, sumAy2 / sampleCount - result.meanAy * result.meanAy));
  result.stdAz = sqrt(max(0.0f, sumAz2 / sampleCount - result.meanAz * result.meanAz));
  result.stdGx = sqrt(max(0.0f, sumGx2 / sampleCount - result.meanGx * result.meanGx));
  result.stdGy = sqrt(max(0.0f, sumGy2 / sampleCount - result.meanGy * result.meanGy));
  result.stdGz = sqrt(max(0.0f, sumGz2 / sampleCount - result.meanGz * result.meanGz));

  result.meanTempC = (tempSampleCount > 0) ? (sumTempC / tempSampleCount) : NAN;
  result.meanDtMs = sumDtMs / sampleCount;

  result.posXDriftMm = posX * 1000.0f;
  result.posYDriftMm = posY * 1000.0f;

  Serial.println(F("--- Results ---"));
  Serial.print(F("Samples collected: ")); Serial.println(sampleCount);

  Serial.print(F("Accel X: mean=")); Serial.print(result.meanAx, 5);
  Serial.print(F("g  std=")); Serial.print(result.stdAx, 5); Serial.println(F("g"));

  Serial.print(F("Accel Y: mean=")); Serial.print(result.meanAy, 5);
  Serial.print(F("g  std=")); Serial.print(result.stdAy, 5); Serial.println(F("g"));

  Serial.print(F("Accel Z: mean=")); Serial.print(result.meanAz, 5);
  Serial.print(F("g  std=")); Serial.print(result.stdAz, 5); Serial.println(F("g  (expect ~1.0g if level)"));

  Serial.print(F("Gyro X: mean=")); Serial.print(result.meanGx, 5);
  Serial.print(F(" dps  std=")); Serial.print(result.stdGx, 5); Serial.println(F(" dps"));

  Serial.print(F("Gyro Y: mean=")); Serial.print(result.meanGy, 5);
  Serial.print(F(" dps  std=")); Serial.print(result.stdGy, 5); Serial.println(F(" dps"));

  Serial.print(F("Gyro Z: mean=")); Serial.print(result.meanGz, 5);
  Serial.print(F(" dps  std=")); Serial.print(result.stdGz, 5); Serial.println(F(" dps"));

  Serial.print(F("Die temperature: mean="));
  if (tempSampleCount > 0) {
    Serial.print(result.meanTempC, 2);
    Serial.print(F(" degC ("));
    Serial.print(tempSampleCount);
    Serial.print(F("/"));
    Serial.print(sampleCount);
    Serial.println(F(" reads valid)"));
  } else {
    Serial.println(F("unavailable"));
  }

  Serial.print(F("Measured mean sample interval: "));
  Serial.print(result.meanDtMs, 3);
  Serial.print(F(" ms (nominal target: "));
  Serial.print(DRIFT_TEST_SAMPLE_MS);
  Serial.println(F(" ms)"));

  Serial.print(F("Naive double-integrated drift over "));
  Serial.print(DRIFT_TEST_DURATION_MS / 1000);
  Serial.println(F("s (gravity-uncompensated, illustrative only, now using measured dt):"));
  Serial.print(F("  posX drift: ")); Serial.print(result.posXDriftMm, 2); Serial.println(F(" mm"));
  Serial.print(F("  posY drift: ")); Serial.print(result.posYDriftMm, 2); Serial.println(F(" mm"));

  Serial.println(F("=== Run Complete ==="));

  return result;
}

#if SD_LOG_ENABLE
bool initSdCard() {
#ifdef BUILTIN_SDCARD
  const int sdCsPin = BUILTIN_SDCARD;
#else
  const int sdCsPin = 10;
#endif

  if (!SD.begin(sdCsPin)) {
    return false;
  }

  sdCardAvailable = true;
  return true;
}

bool initDriftSummaryLog() {
  if (!sdCardAvailable) {
    return false;
  }

  driftSummaryFile = SD.open(DRIFT_SUMMARY_FILE_NAME, FILE_WRITE);
  if (!driftSummaryFile) {
    driftSummaryFileReady = false;
    return false;
  }

  if (driftSummaryFile.size() == 0) {
    driftSummaryFile.println(F("boot_timestamp_ms,run_number,sample_count,"
                                "mean_ax_g,std_ax_g,mean_ay_g,std_ay_g,mean_az_g,std_az_g,"
                                "mean_gx_dps,std_gx_dps,mean_gy_dps,std_gy_dps,mean_gz_dps,std_gz_dps,"
                                "mean_temp_c,mean_dt_ms,"
                                "posx_drift_mm,posy_drift_mm"));
  }

  driftSummaryFile.flush();
  driftSummaryFileReady = true;
  return true;
}

void logDriftSummary(int runNumber, const DriftResult& r) {
  if (!driftSummaryFileReady || !driftSummaryFile) {
    return;
  }

  driftSummaryFile.print(millis());
  driftSummaryFile.print(',');
  driftSummaryFile.print(runNumber);
  driftSummaryFile.print(',');
  driftSummaryFile.print(r.sampleCount);
  driftSummaryFile.print(',');
  driftSummaryFile.print(r.meanAx, 5); driftSummaryFile.print(',');
  driftSummaryFile.print(r.stdAx, 5); driftSummaryFile.print(',');
  driftSummaryFile.print(r.meanAy, 5); driftSummaryFile.print(',');
  driftSummaryFile.print(r.stdAy, 5); driftSummaryFile.print(',');
  driftSummaryFile.print(r.meanAz, 5); driftSummaryFile.print(',');
  driftSummaryFile.print(r.stdAz, 5); driftSummaryFile.print(',');
  driftSummaryFile.print(r.meanGx, 5); driftSummaryFile.print(',');
  driftSummaryFile.print(r.stdGx, 5); driftSummaryFile.print(',');
  driftSummaryFile.print(r.meanGy, 5); driftSummaryFile.print(',');
  driftSummaryFile.print(r.stdGy, 5); driftSummaryFile.print(',');
  driftSummaryFile.print(r.meanGz, 5); driftSummaryFile.print(',');
  driftSummaryFile.print(r.stdGz, 5); driftSummaryFile.print(',');
  driftSummaryFile.print(r.meanTempC, 2); driftSummaryFile.print(',');
  driftSummaryFile.print(r.meanDtMs, 3); driftSummaryFile.print(',');
  driftSummaryFile.print(r.posXDriftMm, 2); driftSummaryFile.print(',');
  driftSummaryFile.println(r.posYDriftMm, 2);

  driftSummaryFile.flush();
}
#endif
