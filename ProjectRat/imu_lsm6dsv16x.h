#ifndef IMU_LSM6DSV16X_H
#define IMU_LSM6DSV16X_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

class LSM6DSV16X {
public:
  LSM6DSV16X();
  bool begin(TwoWire& wire, uint8_t address = IMU_I2C_ADDR);
  bool readXY(int16_t& outAx, int16_t& outAy, int16_t& outGx, int16_t& outGy);
  bool readAll(int16_t& outAx,
               int16_t& outAy,
               int16_t& outAz,
               int16_t& outGx,
               int16_t& outGy,
               int16_t& outGz);
  bool configureQvar();
  bool configureMl();
  bool readQvarState(bool& contactDetected);
  bool readMlState(uint8_t& mlResult);

private:
  TwoWire* _wire;
  uint8_t _address;
  bool _qvarEnabled;
  bool _mlEnabled;

  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegister(uint8_t reg, uint8_t* buffer, uint8_t count);
  bool readRegister16(uint8_t lowReg, int16_t& value);
};

using IMU = LSM6DSV16X;

#endif // IMU_LSM6DSV16X_H
