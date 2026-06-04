#include "tof_sensor.h"
#include "config.h"

static const uint8_t kProbeAddresses[] = {
  0x20, 0x21, 0x22, 0x23,
  0x24, 0x25, 0x26, 0x27,
  0x28, 0x29, 0x2A, 0x2B,
  0x2C, 0x2D, 0x2E, 0x2F,
  0x30, 0x31, 0x32, 0x33,
  0x34, 0x35, 0x36, 0x37,
  0x38, 0x39
};

static const uint8_t kDistanceRegisterCandidates[] = {
  TOF_DISTANCE_REGISTER,
  0x01,
  0x04,
  0x10,
  0x1E,
  0x20,
  0x2C
};

static bool isPlausibleDistance(uint16_t distanceMm) {
  return distanceMm >= 20 && distanceMm <= 2000;
}

ToFSensor::ToFSensor()
    : _wire(nullptr), _sensorId(0), _address(0), _initialized(false) {}

bool ToFSensor::begin(TwoWire& wire, uint8_t sensorId) {
  _wire = &wire;
  _sensorId = sensorId;
  _address = 0;
  _initialized = false;

  if (!probeSensorAddress(sensorId)) {
    return false;
  }

  // The EV_MOD_ICU-10201-00 hardware protocol is not confirmed yet.
  // This implementation uses a generic I2C register probe and will be
  // updated once the exact datasheet and register map are available.
  _initialized = true;
  return true;
}

bool ToFSensor::readDistance(uint16_t& distanceMm) {
  if (!_initialized || _wire == nullptr) {
    return false;
  }

  if (!readDistanceFromCandidateRegister(distanceMm)) {
    return false;
  }

  return true;
}

bool ToFSensor::probeSensorAddress(uint8_t sensorIndex) {
  uint8_t foundIndex = 0;

  for (uint8_t address : kProbeAddresses) {
    _wire->beginTransmission(address);
    if (_wire->endTransmission() == 0) {
      if (foundIndex == sensorIndex) {
        _address = address;
        return true;
      }
      foundIndex++;
    }
  }

  return false;
}

bool ToFSensor::readDistanceFromCandidateRegister(uint16_t& distanceMm) {
  for (uint8_t reg : kDistanceRegisterCandidates) {
    if (!readRegister16(reg, distanceMm)) {
      continue;
    }

    if (isPlausibleDistance(distanceMm)) {
      return true;
    }
  }

  return false;
}

bool ToFSensor::readRegister16(uint8_t reg, uint16_t& value) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) {
    return false;
  }

  constexpr uint8_t bytesToRead = 2;
  if (_wire->requestFrom(_address, bytesToRead) != bytesToRead) {
    return false;
  }

  uint8_t low = _wire->read();
  uint8_t high = _wire->read();
  value = (uint16_t(high) << 8) | low;

  return true;
}
