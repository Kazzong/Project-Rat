#include "sensor_fusion.h"

void fuseSensorData(int16_t accelX,
                    int16_t accelY,
                    int16_t gyroX,
                    int16_t gyroY,
                    uint16_t tofLeftMm,
                    uint16_t tofRightMm,
                    FusedMotion& outMotion) {
  // IMU is the core mouse input source.
  // Use accelerometer X/Y as the primary motion signal and
  // use gyroscope X/Y as a secondary stabilization input.
  const float accelScale = 0.00018f;   // raw accel → motion units
  const float gyroScale = 0.00005f;    // raw gyro → small correction
  const float deadzone = 0.03f;        // suppress tiny jitter
  const float smoothing = 0.72f;       // exponential smoothing factor
  const float tofBaselineMm = 30.0f;   // approximate sensor separation
  const float tiltScale = 0.0025f;     // sensitivity of tilt compensation
  const float stillnessThresholdMm = 2.5f; // ToF drift threshold for stillness
  const float motionThreshold = 0.04f; // IMU motion threshold for stillness

  static float filteredDx = 0.0f;
  static float filteredDy = 0.0f;
  static float lastTofLeft = 0.0f;
  static float lastTofRight = 0.0f;

  float motionX = accelX * accelScale + gyroY * gyroScale;
  float motionY = accelY * accelScale - gyroX * gyroScale;

  float heightMm = 0.0f;
  float surfaceTiltDeg = 0.0f;

  if (tofLeftMm > 0 && tofRightMm > 0) {
    heightMm = (tofLeftMm + tofRightMm) * 0.5f;
    surfaceTiltDeg = atan2f((float)tofRightMm - (float)tofLeftMm, tofBaselineMm) * 57.2957795f;

    // Compensate X motion based on estimated surface tilt.
    motionX -= surfaceTiltDeg * tiltScale;
  } else if (tofLeftMm > 0) {
    heightMm = (float)tofLeftMm;
  } else if (tofRightMm > 0) {
    heightMm = (float)tofRightMm;
  }

  if (fabs(motionX) < deadzone) {
    motionX = 0.0f;
  }
  if (fabs(motionY) < deadzone) {
    motionY = 0.0f;
  }

  filteredDx = filteredDx * smoothing + motionX * (1.0f - smoothing);
  filteredDy = filteredDy * smoothing + motionY * (1.0f - smoothing);

  bool tofStable = false;
  if (tofLeftMm > 0 && tofRightMm > 0) {
    const float deltaLeft = fabsf((float)tofLeftMm - lastTofLeft);
    const float deltaRight = fabsf((float)tofRightMm - lastTofRight);
    tofStable = deltaLeft < stillnessThresholdMm && deltaRight < stillnessThresholdMm;
    lastTofLeft = (float)tofLeftMm;
    lastTofRight = (float)tofRightMm;
  }

  bool imuStill = fabsf(motionX) < motionThreshold && fabsf(motionY) < motionThreshold;
  outMotion.atRest = tofStable && imuStill;

  if (outMotion.atRest) {
    filteredDx = 0.0f;
    filteredDy = 0.0f;
  }

  outMotion.dx = filteredDx;
  outMotion.dy = filteredDy;
  outMotion.height = heightMm * 0.001f;
  outMotion.surfaceTilt = surfaceTiltDeg;
}
