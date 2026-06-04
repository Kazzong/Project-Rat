#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <Arduino.h>

class ToFSensor {
public:
  ToFSensor();
  bool begin(uint8_t sensorId);
  bool readDistance(uint16_t& distanceMm);

private:
  uint8_t _sensorId;
  bool _initialized;
};

#endif // TOF_SENSOR_H
