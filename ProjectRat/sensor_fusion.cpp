#include "sensor_fusion.h"

void fuseSensorData(int16_t accelX,
                    int16_t accelY,
                    int16_t gyroX,
                    int16_t gyroY,
                    float vibrationStrength,
                    FusedMotion& outMotion) {
  // IMU is the core mouse input source.
  // Use accelerometer X/Y as the primary motion signal and
  // use gyroscope X/Y as a secondary stabilization input.
  const float accelScale = 0.00018f;   // raw accel → motion units
  const float gyroScale = 0.00005f;    // raw gyro → small correction
  const float deadzone = 0.03f;        // suppress tiny jitter
  const float smoothing = 0.72f;       // exponential smoothing factor
  const float tiltScale = 0.0018f;     // sensitivity of texture compensation
  const float vibrationThreshold = 6.0f; // vibration drift threshold for stillness
  const float motionThreshold = 0.04f; // IMU motion threshold for stillness

  static float filteredDx = 0.0f;
  static float filteredDy = 0.0f;
  static float lastVibration = 0.0f;

  float motionX = accelX * accelScale + gyroY * gyroScale;
  float motionY = accelY * accelScale - gyroX * gyroScale;

  float heightMm = 0.0f;
  float surfaceTiltDeg = vibrationStrength * tiltScale;

  // Attenuate motion when texture-induced vibration is high.
  motionX -= surfaceTiltDeg;

  if (fabs(motionX) < deadzone) {
    motionX = 0.0f;
  }
  if (fabs(motionY) < deadzone) {
    motionY = 0.0f;
  }

  filteredDx = filteredDx * smoothing + motionX * (1.0f - smoothing);
  filteredDy = filteredDy * smoothing + motionY * (1.0f - smoothing);

  const float vibrationDelta = fabsf(vibrationStrength - lastVibration);
  bool vibrationStable = vibrationDelta < vibrationThreshold;
  lastVibration = vibrationStrength;

  bool imuStill = fabsf(motionX) < motionThreshold && fabsf(motionY) < motionThreshold;
  outMotion.atRest = vibrationStable && imuStill;

  if (outMotion.atRest) {
    filteredDx = 0.0f;
    filteredDy = 0.0f;
  }

  outMotion.dx = filteredDx;
  outMotion.dy = filteredDy;
  outMotion.height = heightMm * 0.001f;
  outMotion.surfaceTilt = surfaceTiltDeg;
  outMotion.vibrationStrength = vibrationStrength;
}
