#include "tof_sensor.h"

ToFSensor::ToFSensor()
    : _sensorId(0), _initialized(false) {}

bool ToFSensor::begin(uint8_t sensorId) {
  _sensorId = sensorId;
  _initialized = true;
  return true; // TODO: Replace stub implementation with actual ToF interface initialization.
}

bool ToFSensor::readDistance(uint16_t& distanceMm) {
  if (!_initialized) {
    return false;
  }

  // Stub measurement values for initial firmware development.
  distanceMm = (_sensorId == 0) ? 500 : 520;
  return true;
}
