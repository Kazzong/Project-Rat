#ifndef IMU_LSM6DSV16X_H
#define IMU_LSM6DSV16X_H

#include <Arduino.h>
#include <Wire.h>

// NOTE: register addresses are defined in config.h — do not redefine here.

class LSM6DSV16X {
public:
  LSM6DSV16X();

  bool begin(TwoWire& wire, uint8_t address = IMU_I2C_ADDR);

  // Raw counts (existing interface — unchanged call sites keep working)
  bool readXY(int16_t& outAx, int16_t& outAy, int16_t& outGx, int16_t& outGy);
  bool readAll(int16_t& outAx, int16_t& outAy, int16_t& outAz,
               int16_t& outGx, int16_t& outGy, int16_t& outGz);

  // Physical units (g and dps), built on readAll()
  static constexpr float ACCEL_SENSITIVITY_G_PER_LSB = 0.000061f;   // ±2g FS, 0.061 mg/LSB
  static constexpr float GYRO_SENSITIVITY_DPS_PER_LSB = 0.070f;     // ±2000dps FS, 70 mdps/LSB
  static constexpr float G_TO_MPS2 = 9.80665f;
  static constexpr float IMU_DEG_TO_RAD = 0.017453293f;  // renamed — Teensy core already #defines DEG_TO_RAD

  bool readAllPhysical(float& outAxG, float& outAyG, float& outAzG,
                        float& outGxDps, float& outGyDps, float& outGzDps);

  // Onboard die temperature (degC). Sensitivity 256 LSB/degC, 0 LSB = 25 degC.
  // Added to correlate stationary bias drift against measured temperature
  // instead of inferring warm-up purely from elapsed run number.
  static constexpr float TEMP_SENSITIVITY_LSB_PER_C = 256.0f;
  static constexpr float TEMP_OFFSET_C = 25.0f;
  bool readTemperatureC(float& outTempC);

  // Qvar / MLC. MLC result polling is wired to the documented result
  // register; configureMl() remains disabled until a generated .ucf file
  // supplies the trained decision-tree register sequence.
  bool configureQvar();
  bool configureMl();
  bool readQvarState(bool& contactDetected);
  bool readMlState(uint8_t& mlResult);
  bool readMlChangeStatus(uint8_t& status);

private:
  TwoWire* _wire;
  uint8_t _address;
  bool _qvarEnabled;
  bool _mlEnabled;

  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegister(uint8_t reg, uint8_t* buffer, uint8_t count);
  bool readRegister16(uint8_t lowReg, int16_t& value);
};

// Alias so existing sketch code (`IMU lsm6dsv16x;`) keeps compiling unchanged.
using IMU = LSM6DSV16X;

#endif // IMU_LSM6DSV16X_H
