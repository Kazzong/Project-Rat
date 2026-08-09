#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <Arduino.h>

enum class MotionState : uint8_t {
  Idle = 0,
  Settling,
  Active,
  Still
};

struct FusedMotion {
  float dx;
  float dy;
  float height;              // reserved: future four-corner acoustic lift data
  float surfaceTilt;         // reserved: future four-corner acoustic tilt data
  float vibrationStrength;   // reserved: future four-corner acoustic data
  bool atRest;
};

struct SensorCalibration {
  float gyroBiasXDps;
  float gyroBiasYDps;
  float gyroBiasZDps;
  float accelBiasXg;
  float accelBiasYg;
  float accelBiasZg;
};

// Updates the persistent roll/pitch orientation estimate from this
// cycle's raw accel + gyro data (complementary filter: gyro integration,
// slowly corrected toward the accel-derived angle).
//
// Must be called every cycle, UNCONDITIONALLY, regardless of acoustic
// gate state. Tilt is a real physical property of the mouse and needs
// continuous tracking even while motion is withheld — otherwise the
// first gravity-compensated sample after the gate opens would be
// working from stale orientation data.
//
// dtSeconds must be the real measured interval since the previous call
// (see readSensors() in ProjectRat.ino) — a nominal constant here
// reintroduces the same systematic timing error already fixed in the
// drift characterization test.
void setSensorCalibration(const SensorCalibration& calibration);
void getSensorCalibration(SensorCalibration& outCalibration);
void zeroSensorCalibration();
void updateMotionState(bool gateHeld, float residualAxG, float residualAyG,
                      float gyroMagDps, float dtSeconds);
MotionState getMotionState();

void updateOrientation(int16_t accelX, int16_t accelY, int16_t accelZ,
                        int16_t gyroX, int16_t gyroY,
                        float dtSeconds);

// Returns the current X/Y gravity reference derived from the private
// roll/pitch orientation estimate.
void getGravityReference(float& outGravityX, float& outGravityY);

// Runs one motion-fusion step using this cycle's raw accel/gyro X/Y
// counts and the orientation state already maintained by
// updateOrientation(). Subtracts the gravity component (via current
// roll/pitch) from raw accel before integrating to velocity, so the
// result reflects actual desk-plane motion rather than tilt.
//
// Also runs an in-motion Zero-Velocity-Update: if residual accel AND
// gyro both stay near-zero for several consecutive cycles WHILE STILL
// GATED ON, velocity is actively reset to zero — this is what actually
// stops bias-driven drift/erratic shifts during real held-gate use,
// rather than relying only on the deadzone filtering every single
// sample or waiting for the external gate to release.
//
// Caller is responsible for gating: only call while the acoustic
// stillness signal (MOTION_GATE_PIN today; the real acoustic pair,
// eventually) confirms the mouse is not stationary. dtSeconds must be
// the real measured interval, matching what was passed to
// updateOrientation() this same cycle.
void fuseSensorData(int16_t accelX, int16_t accelY,
                    int16_t gyroX, int16_t gyroY,
                    float dtSeconds,
                    FusedMotion& outMotion);

// Zeroes the internal velocity integrator only (NOT orientation).
// Call exactly once, at the moment the acoustic stillness signal
// transitions from "moving" to "stopped" — this is the Zero-Velocity-
// Update (ZVU) concept from ST DT0106, preventing residual velocity
// from leaking into the next movement cycle. Orientation (roll/pitch)
// deliberately persists through a stop — tilt doesn't reset just
// because translation did.
void resetFusionState();

// Debug accessor: exposes the most recent cycle's residual accel
// (pre-deadzone), current in-motion-stillness counter, and whether
// gyro was under threshold. Added specifically to see WHY the in-motion
// ZVU is or isn't triggering, rather than guessing from behavior alone.
void getFusionDebugState(float& outResidualAxG, float& outResidualAyG,
                          int& outStillnessCounter, bool& outGyroStill);

#endif // SENSOR_FUSION_H
