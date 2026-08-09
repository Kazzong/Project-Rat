#include "sensor_fusion.h"
#include "config.h"
#include "imu_lsm6dsv16x.h"

// --- Orientation state (roll = rotation about X axis, pitch = rotation
// about Y axis), radians. Persistent across calls; updated every cycle
// by updateOrientation() regardless of acoustic gate state, since tilt
// is a real physical property that must be tracked continuously.
static float rollRad = 0.0f;
static float pitchRad = 0.0f;

// --- Translational velocity state, m/s. Only advanced while the
// acoustic gate permits motion (fuseSensorData() is only called under
// that condition, by readSensors() in ProjectRat.ino); reset to zero
// on the gate's held->released transition via resetFusionState() —
// this is the Zero-Velocity-Update (ZVU) concept from ST DT0106.
static float velX = 0.0f;
static float velY = 0.0f;

// Complementary filter blend factor: how much weight stays on the
// gyro-integrated angle each cycle vs. the accel-derived angle.
//
// Lowered from 0.98 to 0.90: bench data showed CONTINUOUS drift while
// genuinely holding still (not a one-time transient), which points at
// this filter having the same bias-amplification vulnerability the
// velocity leaky integrator has -- a constant gyro bias doesn't decay
// out of a filter like this, it settles into a steady-state angle
// error of roughly (gyro_bias * dt) / (1 - ORIENTATION_COMP_ALPHA).
// At 0.98 that denominator (0.02) is small enough that even the
// modest ~2 dps gyro bias measured in the 50-run drift dataset
// produces a persistent multi-degree orientation error -- which then
// leaves a real, sustained (not decaying) residual after gravity
// subtraction. Dropping to 0.90 (same value already used for
// VELOCITY_LEAK_ALPHA) shrinks that amplification substantially.
// Tradeoff: trusts the accel-derived angle more, which is a worse
// reference during real fast motion (linear acceleration contaminates
// it) -- watch for orientation becoming jittery/noisy during quick
// deliberate moves as the sign this went too far. TUNE ON BENCH.
static const float ORIENTATION_COMP_ALPHA = 0.90f;

// Leaky factor for the velocity integrator (ST DT0106 suggests
// 0.90-0.95; starting at the lower/safer end of that range since this
// specific integration path hasn't been separately characterized yet).
// TUNE ON BENCH.
static const float VELOCITY_LEAK_ALPHA = 0.90f;

// raw accel -> HID motion-unit scale, applied after gravity subtraction
// and velocity integration. Bumped again (400 -> 4000): bench data at
// 400 showed fused dx/dy around 0.3-0.6 -- better than the original 40
// (which gave ~0.1-0.16) but still landing right at the roundf()
// rounding boundary in updateHID(), so real motion was still mostly
// disappearing. NOTE: those test captures also showed the gate only
// held for ~2 loop cycles (~40-50ms) per press before releasing,
// which caps how much the velocity integrator can build up regardless
// of scale -- test with a firm, sustained hold through a full 1-2s
// slide for a fair read, not a quick tap. Still not a validated final
// value. TUNE ON BENCH.
static const float OUTPUT_SCALE = 4000.0f;

// Residual (gravity-subtracted) acceleration deadzone, in g units.
// NOTE: different meaning/scale than the old pre-gravity-compensation
// deadzone (0.03, applied to raw scaled accel) — this one operates on
// residual accel after gravity subtraction, so the old value does not
// carry over. Also NOTE: this is extrapolated from raw (pre-gravity-
// subtraction) accel bias measured in the 50-run stationary drift
// dataset (~0.018g on X) -- it has not been directly measured against
// the actual post-subtraction residual in this pipeline. TUNE ON BENCH.
static const float RESIDUAL_ACCEL_DEADZONE = 0.02f;

// In-motion Zero-Velocity-Update (ZVU) thresholds, per ST DT0106's
// criterion: "when the modulus of acceleration is 1g [i.e. residual
// ~0] and gyro output is near 0, velocity is assumed zero." Checked
// EVERY cycle fuseSensorData() runs (i.e. only while gated on) --
// separate from and in addition to the external gate's own
// held->released reset. This is what actually stops bias-driven
// drift/erratic shifts during real sustained holds, since it doesn't
// wait for the gate to release. TUNE ON BENCH.
static const float STILLNESS_GYRO_THRESHOLD_DPS = 5.0f;
static const int STILLNESS_HOLD_SAMPLES = 5;  // consecutive still cycles required (~100ms at 20ms/cycle)
static int stillnessCounter = 0;

// Low-pass filtered residual acceleration. This smooths the short-lived
// spikes that happen when the gate opens or the device is lightly jostled,
// while still allowing legitimate linear acceleration to remain visible.
static float filteredResidualAxG = 0.0f;
static float filteredResidualAyG = 0.0f;
static const float RESIDUAL_FILTER_ALPHA = 0.35f;

// TODO: verify empirically on the bench. Roll is assumed to track
// +gyroX, pitch +gyroY, matching the standard right-handed convention
// used to derive the gravity-reference-vector formula below. If
// tilting the mouse in a known direction produces cursor motion in the
// wrong direction once this is flashed, flip the corresponding sign
// here.
static const float ROLL_GYRO_SIGN = 1.0f;
static const float PITCH_GYRO_SIGN = 1.0f;

// Gyro bias, dps, measured once at boot via calibrateGyroBias() in
// ProjectRat.ino (setupHardware()) and subtracted everywhere raw gyro
// counts are converted to dps below. Root-cause fix for orientation
// drift: updateOrientation() runs continuously, forever, uncorrected
// gyro bias (measured ~-2 to -2.6 dps on Y in the 50-run drift
// dataset) integrates into a substantial persistent pitch/roll error
// over time -- confirmed on the bench via getFusionDebugState():
// residual ax/ay sitting at ~0.15-0.19g while raw accel was ~0,
// implying ~8-11 degrees of pitch error while genuinely flat and
// still. This is NOT the same problem the in-motion ZVU targets (that
// catches small transient bias in the velocity integrator; this is a
// structural error in the orientation estimate itself, regenerating a
// large fake residual every cycle regardless of stillness).
static SensorCalibration g_sensorCalibration = {
  0.0f, 0.0f, 0.0f,
  0.0f, 0.0f, 0.0f
};

void setSensorCalibration(const SensorCalibration& calibration) {
  g_sensorCalibration = calibration;
}

void zeroSensorCalibration() {
  g_sensorCalibration = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
}

void setGyroBiasDps(float biasXDps, float biasYDps) {
  g_sensorCalibration.gyroBiasXDps = biasXDps;
  g_sensorCalibration.gyroBiasYDps = biasYDps;
}

// Debug snapshot of the most recent fuseSensorData() call -- exposed so
// ProjectRat.ino's reportStatus() can print what the ZVU logic is
// actually seeing, instead of us guessing whether it's triggering.
static float lastResidualAxG = 0.0f;
static float lastResidualAyG = 0.0f;
static bool lastGyroStill = false;
static MotionState g_motionState = MotionState::Idle;

void updateMotionState(bool gateHeld, float residualAxG, float residualAyG,
                      float gyroMagDps, float dtSeconds) {
  static float moveEntryThresholdG = 0.05f;
  static float moveExitThresholdG = 0.025f;
  static float stillEntryGyroDps = 8.0f;
  static float stillExitGyroDps = 5.0f;
  static float settleElapsedSeconds = 0.0f;

  float residualMagG = sqrtf(residualAxG * residualAxG + residualAyG * residualAyG);
  bool isMotionActive = (residualMagG > moveEntryThresholdG) || (gyroMagDps > stillEntryGyroDps);
  bool isMotionStopped = (residualMagG < moveExitThresholdG) && (gyroMagDps < stillExitGyroDps);

  if (!gateHeld) {
    g_motionState = MotionState::Idle;
    settleElapsedSeconds = 0.0f;
    return;
  }

  switch (g_motionState) {
    case MotionState::Idle:
      if (isMotionActive) {
        g_motionState = MotionState::Settling;
        settleElapsedSeconds = 0.0f;
      } else {
        g_motionState = MotionState::Still;
      }
      break;

    case MotionState::Settling:
      settleElapsedSeconds += dtSeconds;
      if (settleElapsedSeconds >= 0.12f) {
        g_motionState = MotionState::Active;
      } else if (isMotionStopped) {
        g_motionState = MotionState::Still;
      }
      break;

    case MotionState::Active:
      if (isMotionStopped) {
        g_motionState = MotionState::Still;
      }
      break;

    case MotionState::Still:
      if (isMotionActive) {
        g_motionState = MotionState::Active;
      }
      break;
  }
}

MotionState getMotionState() {
  return g_motionState;
}

void updateOrientation(int16_t accelX, int16_t accelY, int16_t accelZ,
                        int16_t gyroX, int16_t gyroY,
                        float dtSeconds) {
  float axG = (float)accelX * LSM6DSV16X::ACCEL_SENSITIVITY_G_PER_LSB;
  float ayG = (float)accelY * LSM6DSV16X::ACCEL_SENSITIVITY_G_PER_LSB;
  float azG = (float)accelZ * LSM6DSV16X::ACCEL_SENSITIVITY_G_PER_LSB;

  axG -= g_sensorCalibration.accelBiasXg;
  ayG -= g_sensorCalibration.accelBiasYg;
  azG -= g_sensorCalibration.accelBiasZg;

  float gxDps = (float)gyroX * LSM6DSV16X::GYRO_SENSITIVITY_DPS_PER_LSB;
  float gyDps = (float)gyroY * LSM6DSV16X::GYRO_SENSITIVITY_DPS_PER_LSB;

  // Apply measured gyro bias before integrating. This is the first
  // correction that removes the slow, steady angular drift that otherwise
  // accumulates into a persistent tilt error even while the device is still.
  gxDps -= g_sensorCalibration.gyroBiasXDps;
  gyDps -= g_sensorCalibration.gyroBiasYDps;

  // Gyro-integrated angle prediction (radians).
  float rollGyro = rollRad + ROLL_GYRO_SIGN * gxDps * LSM6DSV16X::IMU_DEG_TO_RAD * dtSeconds;
  float pitchGyro = pitchRad + PITCH_GYRO_SIGN * gyDps * LSM6DSV16X::IMU_DEG_TO_RAD * dtSeconds;

  // Accelerometer-derived angle. Only meaningful when the device is not
  // undergoing significant linear acceleration -- the complementary
  // filter's high ORIENTATION_COMP_ALPHA is what keeps this from
  // dominating during real motion; its main job is correcting long-term
  // gyro drift, not providing the moment-to-moment angle.
  float rollAcc = atan2f(ayG, azG);
  float pitchAcc = atan2f(-axG, sqrtf(ayG * ayG + azG * azG));

  rollRad = ORIENTATION_COMP_ALPHA * rollGyro + (1.0f - ORIENTATION_COMP_ALPHA) * rollAcc;
  pitchRad = ORIENTATION_COMP_ALPHA * pitchGyro + (1.0f - ORIENTATION_COMP_ALPHA) * pitchAcc;
}

void getGravityReference(float& outGravityX, float& outGravityY) {
  outGravityX = -sinf(pitchRad);
  outGravityY = cosf(pitchRad) * sinf(rollRad);
}

void fuseSensorData(int16_t accelX, int16_t accelY,
                    int16_t gyroX, int16_t gyroY,
                    float dtSeconds,
                    FusedMotion& outMotion) {
  float axG = (float)accelX * LSM6DSV16X::ACCEL_SENSITIVITY_G_PER_LSB;
  float ayG = (float)accelY * LSM6DSV16X::ACCEL_SENSITIVITY_G_PER_LSB;
  float gxDps = (float)gyroX * LSM6DSV16X::GYRO_SENSITIVITY_DPS_PER_LSB;
  float gyDps = (float)gyroY * LSM6DSV16X::GYRO_SENSITIVITY_DPS_PER_LSB;

  axG -= g_sensorCalibration.accelBiasXg;
  ayG -= g_sensorCalibration.accelBiasYg;
  gxDps -= g_sensorCalibration.gyroBiasXDps;
  gyDps -= g_sensorCalibration.gyroBiasYDps;

  // Rotated gravity/up-reference vector from current roll/pitch
  // (ST DT0106 Figure 1 rotation construction). Only X/Y needed here --
  // Z (lift) is reserved for later four-corner acoustic work.
  float gx = -sinf(pitchRad);
  float gy = cosf(pitchRad) * sinf(rollRad);

  // Residual (gravity-subtracted) linear acceleration, g units.
  //
  // NOTE: this is (measured - g_ref), NOT ST DT0106's -(acc - g).
  // DT0106 assumes the accelerometer reads positive when an axis points
  // DOWN toward gravity. Our IMU (confirmed by the 50-run stationary
  // drift dataset: azG ~= +1.008g at rest, flat, Z pointing up) reads
  // positive pointing UP, away from the desk -- the opposite
  // convention. Copying DT0106's formula verbatim would invert the
  // result. Derivation: measured_accel = a_true - g_body, where g_body
  // is gravity (pointing down) in body frame; at rest a_true=0 so
  // measured_accel = -g_body = R*[0,0,1] = g_ref as constructed above.
  // Therefore a_true (the residual we want) = measured_accel - g_ref.
  float residualAxG = axG - gx;
  float residualAyG = ayG - gy;

  // In-motion Zero-Velocity-Update: if residual accel AND gyro both
  // stay under their stillness thresholds for STILLNESS_HOLD_SAMPLES
  // consecutive cycles, treat this as genuine stillness (not just a
  // single sample dipping under the deadzone by chance) and actively
  // zero the velocity integrator -- this runs every cycle regardless
  // of the deadzone check below, so it catches sustained small bias
  // that the deadzone alone lets accumulate.
  filteredResidualAxG = (RESIDUAL_FILTER_ALPHA * filteredResidualAxG) +
                        ((1.0f - RESIDUAL_FILTER_ALPHA) * residualAxG);
  filteredResidualAyG = (RESIDUAL_FILTER_ALPHA * filteredResidualAyG) +
                        ((1.0f - RESIDUAL_FILTER_ALPHA) * residualAyG);

  bool residualStill = (fabsf(filteredResidualAxG) < RESIDUAL_ACCEL_DEADZONE) &&
                        (fabsf(filteredResidualAyG) < RESIDUAL_ACCEL_DEADZONE);
  bool gyroStill = (fabsf(gxDps) < STILLNESS_GYRO_THRESHOLD_DPS) &&
                    (fabsf(gyDps) < STILLNESS_GYRO_THRESHOLD_DPS);

  // Debug snapshot, pre-deadzone, for reportStatus() visibility.
  lastResidualAxG = filteredResidualAxG;
  lastResidualAyG = filteredResidualAyG;
  lastGyroStill = gyroStill;

  if (residualStill && gyroStill) {
    stillnessCounter++;
    if (stillnessCounter >= STILLNESS_HOLD_SAMPLES) {
      velX = 0.0f;
      velY = 0.0f;
      stillnessCounter = 0;
    }
  } else {
    stillnessCounter = 0;
  }

  if (fabsf(filteredResidualAxG) < RESIDUAL_ACCEL_DEADZONE) filteredResidualAxG = 0.0f;
  if (fabsf(filteredResidualAyG) < RESIDUAL_ACCEL_DEADZONE) filteredResidualAyG = 0.0f;

  // Keep a gentle decay when the residual falls back below the valid motion
  // band so that tiny noise does not keep the integrator alive after the
  // physical movement has stopped.
  if ((fabsf(filteredResidualAxG) < 0.015f) && (fabsf(filteredResidualAyG) < 0.015f)) {
    velX *= 0.25f;
    velY *= 0.25f;
  }

  float axMps2 = filteredResidualAxG * LSM6DSV16X::G_TO_MPS2;
  float ayMps2 = filteredResidualAyG * LSM6DSV16X::G_TO_MPS2;

  // Leaky velocity integration (ST DT0106 deadreckon_very_simple
  // pattern, first stage only -- a second/position leaky stage isn't
  // needed here since HID mouse reports want a per-cycle DELTA, not an
  // absolute tracked position; see chat for reasoning). Reset to zero
  // on the gate's stop transition via resetFusionState() (ZVU), AND
  // now also on sustained in-motion stillness above -- this is the
  // primary drift control until real acoustic correction exists.
  velX = VELOCITY_LEAK_ALPHA * velX + axMps2 * dtSeconds;
  velY = VELOCITY_LEAK_ALPHA * velY + ayMps2 * dtSeconds;

  // Per-cycle position delta (meters moved this cycle = velocity * dt),
  // scaled to HID motion units.
  outMotion.dx = velX * dtSeconds * OUTPUT_SCALE;
  outMotion.dy = velY * dtSeconds * OUTPUT_SCALE;
  // height / surfaceTilt / vibrationStrength intentionally left
  // untouched here -- reserved for future four-corner acoustic tilt
  // and lift data, out of scope for this single-pair stillness stage.
}

void resetFusionState() {
  velX = 0.0f;
  velY = 0.0f;
  stillnessCounter = 0;
  // rollRad/pitchRad intentionally NOT reset here -- tilt is a real
  // physical property that persists through a stop; only translational
  // velocity resets.
}

void getFusionDebugState(float& outResidualAxG, float& outResidualAyG,
                          int& outStillnessCounter, bool& outGyroStill) {
  outResidualAxG = lastResidualAxG;
  outResidualAyG = lastResidualAyG;
  outStillnessCounter = stillnessCounter;
  outGyroStill = lastGyroStill;
}
