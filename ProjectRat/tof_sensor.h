#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <Arduino.h>
#include <Wire.h>

class ToFSensor {
public:
  ToFSensor();
  bool begin(TwoWire& wire, uint8_t sensorId);
  bool readDistance(uint16_t& distanceMm);

private:
  bool probeSensorAddress(uint8_t sensorIndex);
  bool readRegister16(uint8_t reg, uint16_t& value);
  bool readDistanceFromCandidateRegister(uint16_t& distanceMm);

  TwoWire* _wire;
  uint8_t _sensorId;
  uint8_t _address;
  bool _initialized;
};

#endif // TOF_SENSOR_H
