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

int16_t imuAccelX = 0;
int16_t imuAccelY = 0;
int16_t imuGyroX = 0;
int16_t imuGyroY = 0;
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
#endif

void setupHardware();
void readSensors();
void updateHID();
void reportStatus();
#if SD_LOG_ENABLE
bool initSdLogging();
void logSurfaceSample();
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
#endif

  Buttons::begin();         // --- BUTTON SUBSYSTEM ---

  readSensors();
  reportStatus();
}

void readSensors() {
  if (!lsm6dsv16x.readXY(imuAccelX, imuAccelY, imuGyroX, imuGyroY)) {
    Serial.println(F("IMU read failed."));
  }

  imuQvarValid = lsm6dsv16x.readQvarState(imuQvarContact);
  imuMlValid = lsm6dsv16x.readMlState(imuMlResult);

  piezoRawValue = piezoStrip.readRaw();
  piezoMagnitude = piezoStrip.getMagnitude();

  fuseSensorData(imuAccelX,
                 imuAccelY,
                 imuGyroX,
                 imuGyroY,
                 piezoMagnitude,
                 fusedMotion);

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
  Serial.print(F("IMU Accel X/Y: "));
  Serial.print(imuAccelX);
  Serial.print(F(" / "));
  Serial.println(imuAccelY);

  Serial.print(F("IMU Gyro X/Y: "));
  Serial.print(imuGyroX);
  Serial.print(F(" / "));
  Serial.println(imuGyroY);

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
    sdLogFile.println(F("timestamp_ms,piezo_raw,piezo_magnitude,accel_x,accel_y,gyro_x,gyro_y,qvar_contact,ml_result"));
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
