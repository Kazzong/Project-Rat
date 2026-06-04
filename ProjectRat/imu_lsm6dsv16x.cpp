#include "imu_lsm6dsv16x.h"

static const uint8_t WHO_AM_I_REG = IMU_WHO_AM_I;
static const uint8_t EXPECTED_WHOAMI1 = 0x6C;
static const uint8_t EXPECTED_WHOAMI2 = 0x6D;

LSM6DSV16X::LSM6DSV16X()
    : _wire(nullptr), _address(IMU_I2C_ADDR) {}

bool LSM6DSV16X::begin(TwoWire& wire, uint8_t address) {
  _wire = &wire;
  _address = address;

  _wire->beginTransmission(_address);
  _wire->write(WHO_AM_I_REG);
  if (_wire->endTransmission(false) != 0) {
    return false;
  }

  if (_wire->requestFrom((int)_address, 1) != 1) {
    return false;
  }

  uint8_t whoami = _wire->read();
  if (whoami != EXPECTED_WHOAMI1 && whoami != EXPECTED_WHOAMI2) {
    return false;
  }

  writeRegister(IMU_CTRL3_C, 0x01); // IF_INC enable
  writeRegister(IMU_CTRL1_XL, 0x50); // 1.66 kHz, 2g, LPF enabled
  writeRegister(IMU_CTRL2_G, 0x4C);  // 1.66 kHz, 2000 dps

  return true;
}

bool LSM6DSV16X::readXY(int16_t& outAx, int16_t& outAy, int16_t& outGx, int16_t& outGy) {
  if (!readRegister16(IMU_OUTX_L_A, outAx)) {
    return false;
  }
  if (!readRegister16(IMU_OUTY_L_A, outAy)) {
    return false;
  }
  if (!readRegister16(IMU_OUTX_L_G, outGx)) {
    return false;
  }
  if (!readRegister16(IMU_OUTY_L_G, outGy)) {
    return false;
  }
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

  if (_wire->requestFrom(_address, count) != count) {
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
