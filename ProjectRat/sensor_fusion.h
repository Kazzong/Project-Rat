#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <Arduino.h>

struct FusedMotion {
  float dx;
  float dy;
  float height;
};

void fuseSensorData(int16_t accelX,
                    int16_t accelY,
                    int16_t gyroX,
                    int16_t gyroY,
                    uint16_t zMm,
                    FusedMotion& outMotion);

#endif // SENSOR_FUSION_H
