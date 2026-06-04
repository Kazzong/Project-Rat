#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <usb_mouse.h>

#include "config.h"
#include "imu_lsm6dsv16x.h"
#include "tof_sensor.h"
#include "sensor_fusion.h"

IMU lsm6dsv16x;
ToFSensor icu10201Left;
ToFSensor icu10201Right;

int16_t imuAccelX = 0;
int16_t imuAccelY = 0;
int16_t imuGyroX = 0;
int16_t imuGyroY = 0;
uint16_t tofDistanceLeftMm = 0;
uint16_t tofDistanceRightMm = 0;
uint16_t tofDistanceMm = 0;
uint8_t imuMlResult = 0;
bool imuMlValid = false;
bool imuQvarContact = false;
bool imuQvarValid = false;
FusedMotion fusedMotion = {0.0f, 0.0f, 0.0f, 0.0f, false};

#if SD_LOG_ENABLE
File sdLogFile;
bool sdCardAvailable = false;
#endif

void setupHardware();
void readSensors();
void updateHID();
void reportStatus();
#if SD_LOG_ENABLE
bool initSdLogging();
void logTofSurfaceSample();
#endif

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println(F("Project Rat prototype starting..."));
  Wire.begin();
  setupHardware();
}

void loop() {
  readSensors();
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

  bool icu10201LeftOk = icu10201Left.begin(Wire, TOF_LEFT_ID);
  bool icu10201RightOk = icu10201Right.begin(Wire, TOF_RIGHT_ID);

  if (!icu10201LeftOk || !icu10201RightOk) {
    Serial.println(F("ToF initialization failed. Using stub sensor data."));
  } else {
    Serial.println(F("ToF sensors initialized."));
  }

#if SD_LOG_ENABLE
  if (initSdLogging()) {
    Serial.println(F("SD logging initialized."));
  } else {
    Serial.println(F("SD logging failed."));
  }
#endif

  readSensors();
  reportStatus();
}

void readSensors() {
  if (!lsm6dsv16x.readXY(imuAccelX, imuAccelY, imuGyroX, imuGyroY)) {
    Serial.println(F("IMU read failed."));
  }

  imuQvarValid = lsm6dsv16x.readQvarState(imuQvarContact);
  imuMlValid = lsm6dsv16x.readMlState(imuMlResult);

  bool haveLeft = icu10201Left.readDistance(tofDistanceLeftMm);
  bool haveRight = icu10201Right.readDistance(tofDistanceRightMm);

  if (haveLeft && haveRight) {
    tofDistanceMm = (uint16_t)((tofDistanceLeftMm + tofDistanceRightMm) / 2);
  } else if (haveLeft) {
    tofDistanceMm = tofDistanceLeftMm;
  } else if (haveRight) {
    tofDistanceMm = tofDistanceRightMm;
  } else {
    tofDistanceMm = 0;
  }

  fuseSensorData(imuAccelX,
                 imuAccelY,
                 imuGyroX,
                 imuGyroY,
                 tofDistanceLeftMm,
                 tofDistanceRightMm,
                 fusedMotion);

#if SD_LOG_ENABLE
  if (sdCardAvailable) {
    logTofSurfaceSample();
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
#endif
}

void reportStatus() {
  Serial.print(F("IMU Accel X/Y: "));
  Serial.print(imuAccelX);
  Serial.print(F(" / "));
  Serial.println(imuAccelY);

  Serial.print(F("IMU Gyro X/Y: "));
  Serial.print(imuGyroX);
  Serial.print(F(" / "));
  Serial.println(imuGyroY);

  Serial.print(F("ToF distance (avg): "));
  Serial.print(tofDistanceMm);
  Serial.print(F(" mm, left: "));
  Serial.print(tofDistanceLeftMm);
  Serial.print(F(" mm, right: "));
  Serial.print(tofDistanceRightMm);
  Serial.print(F(" mm, tilt: "));
  Serial.print(fusedMotion.surfaceTilt);
  Serial.print(F(" deg, atRest: "));
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
    sdLogFile.println(F("timestamp_ms,left_mm,right_mm,avg_mm,accel_x,accel_y,gyro_x,gyro_y,qvar_contact,ml_result"));
  }

  sdLogFile.flush();
  return true;
}

void logTofSurfaceSample() {
  if (!sdCardAvailable) {
    return;
  }

  if (!sdLogFile) {
    sdCardAvailable = false;
    return;
  }

  sdLogFile.print(millis());
  sdLogFile.print(',');
  sdLogFile.print(tofDistanceLeftMm);
  sdLogFile.print(',');
  sdLogFile.print(tofDistanceRightMm);
  sdLogFile.print(',');
  sdLogFile.print(tofDistanceMm);
  sdLogFile.print(',');
  sdLogFile.print(imuAccelX);
  sdLogFile.print(',');
  sdLogFile.print(imuAccelY);
  sdLogFile.print(',');
  sdLogFile.print(imuGyroX);
  sdLogFile.print(',');
  sdLogFile.print(imuGyroY);
  sdLogFile.print(',');
  sdLogFile.print(imuQvarValid ? (imuQvarContact ? 1 : 0) : 0);
  sdLogFile.print(',');
  sdLogFile.println(imuMlValid ? imuMlResult : 255);

  sdLogFile.flush();
}
#endif
