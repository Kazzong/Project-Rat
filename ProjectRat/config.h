#ifndef PROJECT_RAT_CONFIG_H
#define PROJECT_RAT_CONFIG_H

#include <stdint.h>

/*
 * ---------------------------------------------------------
 * IMU (LSM6DSV16X) I2C configuration
 * ---------------------------------------------------------
 * IMU_I2C_ADDR:       The 7-bit I2C address of the IMU.
 * IMU_WHO_AM_I:       Register used to verify the IMU identity.
 */
#define IMU_I2C_ADDR        0x6B
#define IMU_WHO_AM_I        0x0F

// The IMU is now physically aligned with the logical mouse frame, so no
// coordinate remap is required.
#define IMU_ROTATED_CW_90   0

/*
 * ---------------------------------------------------------
 * IMU control registers
 * ---------------------------------------------------------
 * IMU_CTRL1_XL:       Accelerometer configuration (ODR, range, filters).
 * IMU_CTRL2_G:        Gyroscope configuration (ODR, range).
 * IMU_CTRL3_C:        General control (auto-increment, reset, etc.).
 */
#define IMU_CTRL1_XL        0x10
#define IMU_CTRL2_G         0x11
#define IMU_CTRL3_C         0x12

/*
 * ---------------------------------------------------------
 * IMU output registers (temperature)
 * ---------------------------------------------------------
 * OUT_TEMP_L/H:       Low/high bytes for onboard die temperature.
 *                     Sensitivity 256 LSB/degC, 0 LSB = 25 degC.
 *                     Added to correlate stationary bias drift
 *                     against measured die temperature instead of
 *                     inferring warm-up purely from run number.
 */
#define IMU_OUT_TEMP_L      0x20
#define IMU_OUT_TEMP_H      0x21

/*
 * ---------------------------------------------------------
 * IMU output registers (accelerometer)
 * ---------------------------------------------------------
 * OUTX/Y/Z_L/H_A:     Low/high bytes for accel X/Y/Z readings.
 */
#define IMU_OUTX_L_A        0x28
#define IMU_OUTX_H_A        0x29
#define IMU_OUTY_L_A        0x2A
#define IMU_OUTY_H_A        0x2B
#define IMU_OUTZ_L_A        0x2C
#define IMU_OUTZ_H_A        0x2D

/*
 * ---------------------------------------------------------
 * IMU output registers (gyroscope)
 * ---------------------------------------------------------
 * OUTX/Y/Z_L/H_G:     Low/high bytes for gyro X/Y/Z readings.
 */
#define IMU_OUTX_L_G        0x22
#define IMU_OUTX_H_G        0x23
#define IMU_OUTY_L_G        0x24
#define IMU_OUTY_H_G        0x25
#define IMU_OUTZ_L_G        0x26
#define IMU_OUTZ_H_G        0x27

/*
 * ---------------------------------------------------------
 * QVAR + MLC registers
 * ---------------------------------------------------------
 * MLC result/status addresses are documented by AN5804. The MLC
 * configuration itself still requires a generated Unico .ucf file.
 */
#define IMU_QVAR_CTRL1      0x00
#define IMU_QVAR_STATUS     0x00
#define IMU_MLC1_SRC        0x70
#define IMU_MLC_STATUS      0x15
#define IMU_MLC_STATUS_MAINPAGE 0x4B
#define IMU_MLC_ODR_CFG_C   0x60

/*
 * ---------------------------------------------------------
 * Motion pipeline notes
 * ---------------------------------------------------------
 * The two flags formerly here (BYPASS_VIBRATION_INPUT,
 * BYPASS_REST_GATE) are superseded, not just disabled. The piezo
 * strip has been removed from fuseSensorData() entirely — it does
 * not correspond to any real Project RAT hardware (the acoustic feet
 * discs, switch benders, and scroll-strip actuator are three separate
 * piezo elements, none of which is this analog pin). Stillness gating
 * is now structural: see MOTION_GATE_PIN below and the acoustic-first
 * control flow in readSensors() (ProjectRat.ino).
 */

/*
 * ---------------------------------------------------------
 * Piezo strip — removed
 * ---------------------------------------------------------
 * Was: analog pin (PIEZO_INPUT_PIN) feeding a "vibrationStrength"
 * signal into the fusion path. Removed because it does not
 * correspond to any real Project RAT hardware — the acoustic feet
 * discs, switch benders, and scroll-strip actuator are three
 * separate piezo elements, none of which was this pin. Real
 * acoustic-based surface mapping will be implemented later, tied to
 * actual corner-pad hardware rather than this placeholder.
 */

/*
 * ---------------------------------------------------------
 * SD card logging configuration
 * ---------------------------------------------------------
 * SD_LOG_ENABLE:          1 = enable SD card init + drift summary
 *                          CSV logging (drift_summary.csv), 0 = disable.
 *                          Surface-map logging (was piezo-based) has
 *                          been removed; real acoustic-based surface
 *                          mapping will replace it later.
 */
#define SD_LOG_ENABLE       1

/*
 * ---------------------------------------------------------
 * HID interface enable
 * ---------------------------------------------------------
 * MOUSE_INTERFACE is NOT defined here. Teensy's own core
 * (usb_desc.h) defines it automatically based on the Arduino IDE's
 * Tools > USB Type board setting (must be a Mouse-capable mode,
 * e.g. "Keyboard+Mouse+Joystick") — defining it again in this file
 * conflicts with that and only produces a redefinition warning. The
 * #ifdef MOUSE_INTERFACE guard in updateHID() (ProjectRat.ino) picks
 * up the core's definition automatically once that board setting is
 * correct; nothing here needs to change based on it.
 */

/*
 * ---------------------------------------------------------
 * Drift characterization batch
 * ---------------------------------------------------------
 * RUN_DRIFT_BATCH_AT_BOOT:
 *     1 = runDriftBatch() executes in setup() before loop() ever
 *         runs — a 50-run, ~38s/run stationary IMU characterization
 *         (~32 minutes total). Nothing else (gating, buttons, HID
 *         motion) is reachable until it finishes.
 *     0 = skipped entirely; setup() proceeds straight to loop() as
 *         normal.
 *     Set to 1 only when you specifically want another drift
 *     characterization run; leave at 0 for all other bench testing
 *     (motion gate, buttons, HID behavior, etc).
 */
#define RUN_DRIFT_BATCH_AT_BOOT  0

/*
 * ---------------------------------------------------------
 * Serial status reporting
 * ---------------------------------------------------------
 * STATUS_REPORT_INTERVAL_MS:
 *     How often reportStatus() prints in loop() (milliseconds).
 *     Decoupled from MAIN_LOOP_DELAY_MS so the serial monitor stays
 *     readable instead of printing on every ~20ms loop iteration.
 */
#define STATUS_REPORT_INTERVAL_MS  250

/*
 * ---------------------------------------------------------
 * Main loop timing
 * ---------------------------------------------------------
 * MAIN_LOOP_DELAY_MS:     Delay between loop iterations.
 *                         Controls IMU polling rate + HID update rate.
 */
#define MAIN_LOOP_DELAY_MS  20

/*
 * ---------------------------------------------------------
 * Motion permit gate (manual stand-in for acoustic stillness)
 * ---------------------------------------------------------
 * MOTION_GATE_PIN:
 *     A third momentary switch, separate from BUTTON_A/B. Held down
 *     (active-low) = mouse has moved / motion permitted; released =
 *     mouse is stationary / motion withheld.
 *
 *     This stands in for the Phase 0 single acoustic TX/RX pair,
 *     whose only current job is stillness detection (RAT-DOC-001
 *     Section 3.3) — not tilt or lift sensing, which are four-corner
 *     capabilities that arrive later. The control flow in
 *     readSensors() checks this gate FIRST: while withheld,
 *     fuseSensorData() is not called at all (not just discarded
 *     afterward), and the moment it transitions from held to
 *     released, resetFusionState() zeroes the smoothing filter so
 *     nothing leaks into the next movement cycle.
 */
/*
 * MOTION_GATE_DEBOUNCE_MS:
 *     Minimum time MOTION_GATE_PIN's raw reading must stay stable
 *     before a state change is accepted. Buttons A/B already get this
 *     protection via the Buttons class debounce state machine;
 *     MOTION_GATE_PIN was previously read with a bare digitalRead()
 *     every cycle with no debounce at all -- a brief noise glitch or
 *     marginal breadboard contact could register as a false "held"
 *     for long enough to produce real (if brief) unintended motion,
 *     even though the throttled status print might never catch it.
 */
#define MOTION_GATE_DEBOUNCE_MS  25
/*
 * GATE_OPEN_SETTLE_MS:
 *     After MOTION_GATE_PIN transitions to held, fuseSensorData() is
 *     still not called for this many additional milliseconds. Added
 *     after bench testing showed a large cursor jump at the exact
 *     moment of pressing the gate switch, even with the board
 *     completely undisturbed beforehand -- the switch itself (mounted
 *     on the same breadboard as the IMU) produces a real mechanical
 *     snap-action vibration on actuation, which the accelerometer
 *     picks up as a genuine, large, brief acceleration spike right as
 *     the gate opens. This settle window lets that decay before
 *     integration starts trusting the data. Distinct from
 *     MOTION_GATE_DEBOUNCE_MS (which filters the digital pin read) --
 *     this filters analog sensor settling time after a real mechanical
 *     event.
 */
#define GATE_OPEN_SETTLE_MS      150
#define MOTION_GATE_PIN         4

/*
 * ---------------------------------------------------------
 * Button subsystem (Option A — full production-grade)
 * ---------------------------------------------------------
 * BUTTON_A_PIN / BUTTON_B_PIN:
 *     GPIO pins for your two physical buttons.
 *
 * BUTTON_DEBOUNCE_MS:
 *     Minimum time a signal must remain stable before
 *     being accepted as a real press/release.
 *
 * BUTTON_LONG_PRESS_MS:
 *     Duration threshold for long-press detection.
 */
#define BUTTON_A_PIN            2
#define BUTTON_B_PIN            3

#define BUTTON_DEBOUNCE_MS      5
#define BUTTON_LONG_PRESS_MS    400

#endif // PROJECT_RAT_CONFIG_H
