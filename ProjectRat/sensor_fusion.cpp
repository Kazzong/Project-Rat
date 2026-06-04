#include "sensor_fusion.h"

void fuseSensorData(int16_t accelX,
                    int16_t accelY,
                    int16_t gyroX,
                    int16_t gyroY,
                    uint16_t zMm,
                    FusedMotion& outMotion) {
  // Convert raw IMU values into a simple delta movement estimate.
  // This is a placeholder; replace with a proper fusion algorithm.
  float accelFactor = 0.00015f;
  float gyroFactor = 0.00006f;

  outMotion.dx = accelX * accelFactor + gyroY * gyroFactor;
  outMotion.dy = accelY * accelFactor - gyroX * gyroFactor;
  outMotion.height = zMm * 0.001f;
}
