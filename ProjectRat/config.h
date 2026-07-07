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
 * Placeholder QVAR + MLC registers
 * ---------------------------------------------------------
 * These are stubs until the full LSM6DSV16X register map
 * for QVAR and machine-learning core is integrated.
 */
#define IMU_QVAR_CTRL1      0x00
#define IMU_QVAR_STATUS     0x00
#define IMU_MLC_CTRL        0x00
#define IMU_MLC_STATUS      0x00

/*
 * ---------------------------------------------------------
 * Piezo-film vibration sensor configuration
 * ---------------------------------------------------------
 * PIEZO_INPUT_PIN:        Analog pin connected to the PVDF strip.
 * PIEZO_BASELINE_ALPHA:   Smoothing factor for baseline tracking.
 * PIEZO_LOG_THRESHOLD:    Minimum vibration magnitude to log.
 */
#define PIEZO_INPUT_PIN     A0
#define PIEZO_BASELINE_ALPHA 0.025f
#define PIEZO_LOG_THRESHOLD 16

/*
 * ---------------------------------------------------------
 * SD card logging configuration
 * ---------------------------------------------------------
 * SD_LOG_ENABLE:          1 = enable CSV logging, 0 = disable.
 * SD_LOG_FILE_NAME:       Output file name on SD card.
 */
#define SD_LOG_ENABLE       1
#define SD_LOG_FILE_NAME    "surface_map.csv"

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
