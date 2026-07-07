#include "config.h"
#include "imu_lsm6dsv16x.h"

static const uint8_t WHO_AM_I_REG = IMU_WHO_AM_I;
static const uint8_t EXPECTED_WHOAMI1 = 0x70;
static const uint8_t EXPECTED_WHOAMI2 = 0x70;

LSM6DSV16X::LSM6DSV16X()
    : _wire(nullptr), _address(IMU_I2C_ADDR), _qvarEnabled(false), _mlEnabled(false) {}

bool LSM6DSV16X::begin(TwoWire& wire, uint8_t address) {
  _wire = &wire;
  _address = address;

  _wire->beginTransmission(_address);
  _wire->write(WHO_AM_I_REG);
  if (_wire->endTransmission(false) != 0) {
    return false;
  }

  if (_wire->requestFrom((uint8_t)_address, (uint8_t)1, (uint8_t)true) != 1) {
    return false;
  }

  uint8_t whoami = _wire->read();
  if (whoami != EXPECTED_WHOAMI1 && whoami != EXPECTED_WHOAMI2) {
    return false;
  }

  bool configOk = true;

  // CTRL3 (0x12): if_inc = bit 2, bdu = bit 6. Bit 0 (sw_reset) intentionally left at 0.
  configOk &= writeRegister(IMU_CTRL3_C, 0x44);

  // CTRL1 (0x10): odr_xl = bits 0-3, op_mode_xl = bits 4-6.
  // 0x07 = odr_xl 0111 (240 Hz), op_mode_xl 000 (high-performance, valid at all ODRs).
  configOk &= writeRegister(IMU_CTRL1_XL, 0x07);

  // CTRL2 (0x11): odr_g = bits 0-3, op_mode_g = bits 4-6.
  // 0x07 = odr_g 0111 (240 Hz), op_mode_g 000 (high-performance, valid at all ODRs).
  configOk &= writeRegister(IMU_CTRL2_G, 0x07);

  if (!configOk) {
    return false;
  }

  _qvarEnabled = false;
  _mlEnabled = false;

  return true;
}

bool LSM6DSV16X::configureQvar() {
  _qvarEnabled = false;
  return false;
}

bool LSM6DSV16X::configureMl() {
  _mlEnabled = false;
  return false;
}

bool LSM6DSV16X::readQvarState(bool& contactDetected) {
  if (!_qvarEnabled) {
    return false;
  }

  uint8_t status = 0;
  if (!readRegister(IMU_QVAR_STATUS, &status, 1)) {
    return false;
  }

  contactDetected = (status & 0x01) != 0;
  return true;
}

bool LSM6DSV16X::readMlState(uint8_t& mlResult) {
  if (!_mlEnabled) {
    return false;
  }

  if (!readRegister(IMU_MLC_STATUS, &mlResult, 1)) {
    return false;
  }

  return true;
}

bool LSM6DSV16X::readXY(int16_t& outAx,
                        int16_t& outAy,
                        int16_t& outGx,
                        int16_t& outGy) {
  int16_t accelZ = 0;
  int16_t gyroZ = 0;
  return readAll(outAx, outAy, accelZ, outGx, outGy, gyroZ);
}

bool LSM6DSV16X::readAll(int16_t& outAx,
                         int16_t& outAy,
                         int16_t& outAz,
                         int16_t& outGx,
                         int16_t& outGy,
                         int16_t& outGz) {
  const uint8_t bufferCount = 12;
  uint8_t buffer[bufferCount] = {0};

  if (!readRegister(IMU_OUTX_L_G, buffer, bufferCount)) {
    return false;
  }

  outGx = (int16_t)((buffer[1] << 8) | buffer[0]);
  outGy = (int16_t)((buffer[3] << 8) | buffer[2]);
  outGz = (int16_t)((buffer[5] << 8) | buffer[4]);
  outAx = (int16_t)((buffer[7] << 8) | buffer[6]);
  outAy = (int16_t)((buffer[9] << 8) | buffer[8]);
  outAz = (int16_t)((buffer[11] << 8) | buffer[10]);

  return true;
}

bool LSM6DSV16X::readAllPhysical(float& outAxG, float& outAyG, float& outAzG,
                                  float& outGxDps, float& outGyDps, float& outGzDps) {
  int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;

  if (!readAll(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz)) {
    return false;
  }

  outAxG = (float)rawAx * ACCEL_SENSITIVITY_G_PER_LSB;
  outAyG = (float)rawAy * ACCEL_SENSITIVITY_G_PER_LSB;
  outAzG = (float)rawAz * ACCEL_SENSITIVITY_G_PER_LSB;

  outGxDps = (float)rawGx * GYRO_SENSITIVITY_DPS_PER_LSB;
  outGyDps = (float)rawGy * GYRO_SENSITIVITY_DPS_PER_LSB;
  outGzDps = (float)rawGz * GYRO_SENSITIVITY_DPS_PER_LSB;

  return true;
}

bool LSM6DSV16X::readTemperatureC(float& outTempC) {
  int16_t rawTemp = 0;
  if (!readRegister16(IMU_OUT_TEMP_L, rawTemp)) {
    return false;
  }

  outTempC = TEMP_OFFSET_C + ((float)rawTemp / TEMP_SENSITIVITY_LSB_PER_C);
  return true;
}

bool LSM6DSV16X::writeRegister(uint8_t reg, uint8_t value) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(value);
  return (_wire->endTransmission() == 0);
}

bool LSM6DSV16X::readRegister(uint8_t reg, uint8_t* buffer, uint8_t count) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) {
    return false;
  }

  if (_wire->requestFrom((uint8_t)_address, (uint8_t)count, (uint8_t)true) != count) {
    return false;
  }

  for (uint8_t i = 0; i < count; i++) {
    buffer[i] = _wire->read();
  }
  return true;
}

bool LSM6DSV16X::readRegister16(uint8_t lowReg, int16_t& value) {
  uint8_t buffer[2] = {0, 0};
  if (!readRegister(lowReg, buffer, 2)) {
    return false;
  }
  value = (int16_t)((buffer[1] << 8) | buffer[0]);
  return true;
}
