#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <Arduino.h>

struct FusedMotion {
  float dx;
  float dy;
  float height;
  float surfaceTilt;
  float vibrationStrength;
  bool atRest;
};

void fuseSensorData(int16_t accelX,
                    int16_t accelY,
                    int16_t gyroX,
                    int16_t gyroY,
                    float vibrationStrength,
                    FusedMotion& outMotion);

#endif // SENSOR_FUSION_H
