#include <Arduino.h>
#include <Wire.h>
#include <usb_mouse.h>

#include "config.h"
#include "imu_lsm6dsv16x.h"
#include "tof_sensor.h"
#include "sensor_fusion.h"

LSM6DSV16X imu;
ToFSensor tofLeft;
ToFSensor tofRight;

int16_t imuAccelX = 0;
int16_t imuAccelY = 0;
int16_t imuGyroX = 0;
int16_t imuGyroY = 0;
uint16_t tofDistanceMm = 0;
FusedMotion fusedMotion = {0.0f, 0.0f, 0.0f};

void setupHardware();
void readSensors();
void updateHID();
void reportStatus();

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
  bool imuOk = imu.begin(Wire, IMU_I2C_ADDR);
  if (!imuOk) {
    Serial.println(F("IMU initialization failed."));
  } else {
    Serial.println(F("IMU initialized."));
  }

  bool tofLeftOk = tofLeft.begin(TOF_LEFT_ID);
  bool tofRightOk = tofRight.begin(TOF_RIGHT_ID);

  if (!tofLeftOk || !tofRightOk) {
    Serial.println(F("ToF initialization failed. Using stub sensor data."));
  } else {
    Serial.println(F("ToF sensors initialized."));
  }

  readSensors();
  reportStatus();
}

void readSensors() {
  if (!imu.readXY(imuAccelX, imuAccelY, imuGyroX, imuGyroY)) {
    Serial.println(F("IMU read failed."));
  }

  uint16_t leftDistance = 0;
  uint16_t rightDistance = 0;
  bool haveLeft = tofLeft.readDistance(leftDistance);
  bool haveRight = tofRight.readDistance(rightDistance);

  if (haveLeft && haveRight) {
    tofDistanceMm = (uint16_t)((leftDistance + rightDistance) / 2);
  } else if (haveLeft) {
    tofDistanceMm = leftDistance;
  } else if (haveRight) {
    tofDistanceMm = rightDistance;
  } else {
    tofDistanceMm = 0;
  }

  fuseSensorData(imuAccelX, imuAccelY, imuGyroX, imuGyroY, tofDistanceMm, fusedMotion);
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

  Serial.print(F("ToF distance: "));
  Serial.print(tofDistanceMm);
  Serial.println(F(" mm"));
}
