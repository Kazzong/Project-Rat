#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <usb_mouse.h>

#include "config.h"
#include "imu_lsm6dsv16x.h"
#include "piezo_strip.h"
#include "sensor_fusion.h"
#include "buttons.h"          // --- BUTTON SUBSYSTEM ---

IMU lsm6dsv16x;
PiezoStrip piezoStrip(PIEZO_INPUT_PIN);

float accelXg = 0.0f, accelYg = 0.0f, accelZg = 0.0f;
float gyroXdps = 0.0f, gyroYdps = 0.0f, gyroZdps = 0.0f;
uint16_t piezoRawValue = 0;
float piezoMagnitude = 0.0f;
uint8_t imuMlResult = 0;
bool imuMlValid = false;
bool imuQvarContact = false;
bool imuQvarValid = false;
FusedMotion fusedMotion = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};

#if SD_LOG_ENABLE
File sdLogFile;
bool sdCardAvailable = false;
File driftSummaryFile;
bool driftSummaryFileReady = false;
#endif

#define DRIFT_TEST_DURATION_MS   30000
#define DRIFT_TEST_SAMPLE_MS     20   // matches MAIN_LOOP_DELAY_MS
#define DRIFT_TEST_RUN_COUNT     50    // number of consecutive automated runs
#define DRIFT_TEST_PAUSE_MS      5000 // pause between consecutive runs
#define DRIFT_SUMMARY_FILE_NAME  "drift_summary.csv"

struct DriftResult {
  int sampleCount;
  float meanAx, meanAy, meanAz;
  float stdAx, stdAy, stdAz;
  float meanGx, meanGy, meanGz;
  float stdGx, stdGy, stdGz;
  float posXDriftMm, posYDriftMm;
};

void setupHardware();
void readSensors();
void updateHID();
void reportStatus();
DriftResult runDriftCharacterization(int runNumber);
void runDriftBatch();
#if SD_LOG_ENABLE
bool initSdLogging();
void logSurfaceSample();
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

  // Uncomment to run a batch of consecutive drift characterization passes at boot:
  runDriftBatch();
}

void loop() {
  readSensors();

  Buttons::update();        // --- BUTTON SUBSYSTEM ---

  updateHID();
  delay(MAIN_LOOP_DELAY_MS);
}

void setupHardware() {
  bool imuOk = lsm6dsv16x.begin(Wire, IMU_I2C_ADDR);
  if (!imuOk) {
    Serial.println(F("IMU initialization failed."));
  } else {
    Serial.println(F("IMU initialized."));

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

  piezoStrip.begin();
  Serial.println(F("Piezo strip sensor initialized."));

#if SD_LOG_ENABLE
  if (initSdLogging()) {
    Serial.println(F("SD logging initialized."));
  } else {
    Serial.println(F("SD logging failed."));
  }

  if (initDriftSummaryLog()) {
    Serial.println(F("Drift summary log initialized."));
  } else {
    Serial.println(F("Drift summary log failed."));
  }
#endif

  Buttons::begin();         // --- BUTTON SUBSYSTEM ---

  readSensors();
  reportStatus();
}

void readSensors() {
  if (!lsm6dsv16x.readAllPhysical(accelXg, accelYg, accelZg,
                                   gyroXdps, gyroYdps, gyroZdps)) {
    Serial.println(F("IMU read failed."));
  }

  imuQvarValid = lsm6dsv16x.readQvarState(imuQvarContact);
  imuMlValid = lsm6dsv16x.readMlState(imuMlResult);

  piezoRawValue = piezoStrip.readRaw();
  piezoMagnitude = piezoStrip.getMagnitude();

#if SD_LOG_ENABLE
  if (sdCardAvailable) {
    logSurfaceSample();
  }
#endif

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
}

void updateHID() {
#ifdef MOUSE_INTERFACE
  int16_t xMove = (int16_t)constrain((int32_t)roundf(fusedMotion.dx), -12, 12);
  int16_t yMove = (int16_t)constrain((int32_t)roundf(fusedMotion.dy), -12, 12);

  if (xMove != 0 || yMove != 0) {
    usb_mouse_move((int8_t)xMove, (int8_t)yMove, 0, 0);
  }

  // --- BUTTON SUBSYSTEM: HID OUTPUT ---
  ButtonEvent aEvent = Buttons::getEvent(ButtonId::A);
  ButtonEvent bEvent = Buttons::getEvent(ButtonId::B);

  if (aEvent == ButtonEvent::Pressed) {
    usb_mouse_press(MOUSE_LEFT);
  } else if (aEvent == ButtonEvent::Released) {
    usb_mouse_release(MOUSE_LEFT);
  }

  if (bEvent == ButtonEvent::Pressed) {
    usb_mouse_press(MOUSE_RIGHT);
  } else if (bEvent == ButtonEvent::Released) {
    usb_mouse_release(MOUSE_RIGHT);
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

  Serial.print(F("Piezo raw: "));
  Serial.print(piezoRawValue);
  Serial.print(F(", magnitude: "));
  Serial.print(piezoMagnitude);
  Serial.print(F(", texture: "));
  Serial.print(fusedMotion.surfaceTilt);
  Serial.print(F(", atRest: "));
  Serial.print(fusedMotion.atRest ? F("yes") : F("no"));

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

  float velX = 0, velY = 0;
  float posX = 0, posY = 0;
  float dt = DRIFT_TEST_SAMPLE_MS / 1000.0f;

  unsigned long startTime = millis();

  while (millis() - startTime < DRIFT_TEST_DURATION_MS) {
    float ax, ay, az, gx, gy, gz;
    if (lsm6dsv16x.readAllPhysical(ax, ay, az, gx, gy, gz)) {
      sumAx += ax; sumAy += ay; sumAz += az;
      sumGx += gx; sumGy += gy; sumGz += gz;
      sumAx2 += ax * ax; sumAy2 += ay * ay; sumAz2 += az * az;
      sumGx2 += gx * gx; sumGy2 += gy * gy; sumGz2 += gz * gz;

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

  Serial.print(F("Naive double-integrated drift over "));
  Serial.print(DRIFT_TEST_DURATION_MS / 1000);
  Serial.println(F("s (gravity-uncompensated, illustrative only):"));
  Serial.print(F("  posX drift: ")); Serial.print(result.posXDriftMm, 2); Serial.println(F(" mm"));
  Serial.print(F("  posY drift: ")); Serial.print(result.posYDriftMm, 2); Serial.println(F(" mm"));

  Serial.println(F("=== Run Complete ==="));

  return result;
}

#if SD_LOG_ENABLE
bool initSdLogging() {
#ifdef BUILTIN_SDCARD
  const int sdCsPin = BUILTIN_SDCARD;
#else
  const int sdCsPin = 10;
#endif

  if (!SD.begin(sdCsPin)) {
    return false;
  }

  sdCardAvailable = true;
  sdLogFile = SD.open(SD_LOG_FILE_NAME, FILE_WRITE);
  if (!sdLogFile) {
    sdCardAvailable = false;
    return false;
  }

  if (sdLogFile.size() == 0) {
    sdLogFile.println(F("timestamp_ms,piezo_raw,piezo_magnitude,accel_x_g,accel_y_g,accel_z_g,gyro_x_dps,gyro_y_dps,gyro_z_dps,qvar_contact,ml_result"));
  }

  sdLogFile.flush();
  return true;
}

void logSurfaceSample() {
  if (!sdCardAvailable) {
    return;
  }

  if (!sdLogFile) {
    sdCardAvailable = false;
    return;
  }

  sdLogFile.print(millis());
  sdLogFile.print(',');
  sdLogFile.print(piezoRawValue);
  sdLogFile.print(',');
  sdLogFile.print(piezoMagnitude);
  sdLogFile.print(',');
  sdLogFile.print(accelXg, 5);
  sdLogFile.print(',');
  sdLogFile.print(accelYg, 5);
  sdLogFile.print(',');
  sdLogFile.print(accelZg, 5);
  sdLogFile.print(',');
  sdLogFile.print(gyroXdps, 5);
  sdLogFile.print(',');
  sdLogFile.print(gyroYdps, 5);
  sdLogFile.print(',');
  sdLogFile.print(gyroZdps, 5);
  sdLogFile.print(',');
  sdLogFile.print(imuQvarValid ? (imuQvarContact ? 1 : 0) : 0);
  sdLogFile.print(',');
  sdLogFile.println(imuMlValid ? imuMlResult : 255);

  sdLogFile.flush();
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
  driftSummaryFile.print(r.posXDriftMm, 2); driftSummaryFile.print(',');
  driftSummaryFile.println(r.posYDriftMm, 2);

  driftSummaryFile.flush();
}
#endif