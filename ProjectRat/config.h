#ifndef PROJECT_RAT_CONFIG_H
#define PROJECT_RAT_CONFIG_H

#include <stdint.h>

#define IMU_I2C_ADDR        0x6A
#define IMU_WHO_AM_I        0x0F

#define IMU_CTRL1_XL        0x10
#define IMU_CTRL2_G         0x11
#define IMU_CTRL3_C         0x12

#define IMU_OUTX_L_A        0x28
#define IMU_OUTX_H_A        0x29
#define IMU_OUTY_L_A        0x2A
#define IMU_OUTY_H_A        0x2B
#define IMU_OUTZ_L_A        0x2C
#define IMU_OUTZ_H_A        0x2D
#define IMU_OUTX_L_G        0x22
#define IMU_OUTX_H_G        0x23
#define IMU_OUTY_L_G        0x24
#define IMU_OUTY_H_G        0x25
#define IMU_OUTZ_L_G        0x26
#define IMU_OUTZ_H_G        0x27

// Placeholder Qvar and ML register definitions.
// Replace with the actual LSM6DSV16X register map when available.
#define IMU_QVAR_CTRL1      0x00
#define IMU_QVAR_STATUS     0x00
#define IMU_MLC_CTRL        0x00
#define IMU_MLC_STATUS      0x00

#define TOF_LEFT_ID         0
#define TOF_RIGHT_ID        1
#define TOF_DISTANCE_REGISTER 0x00

#define MAIN_LOOP_DELAY_MS  20

#endif // PROJECT_RAT_CONFIG_H
