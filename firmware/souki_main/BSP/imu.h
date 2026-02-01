#ifndef __IMU_H__
#define __IMU_H__

#include <stdint.h>

// Yahboom 9-axis IMU (GD32 co-processor, Kalman filtered)
// I2C address: 0x23 (confirmed by scan)
// I2C clock: 100kHz, supports continuous read with auto-increment
// Data format: little-endian
#define IMU_ADDR  0x23

// Register map (from Communication Protocol.xlsx, I2C sheet)
// Version (uint8 x3)
#define IMU_REG_VERSION    0x01  // 0x01-0x03: VERSION_H, VERSION_M, VERSION_L

// Accelerometer raw data (int16 x3, scale: 16/32767.0 g)
#define IMU_REG_ACCEL      0x04  // 0x04-0x09: AX_L,AX_H, AY_L,AY_H, AZ_L,AZ_H

// Gyroscope raw data (int16 x3, scale: (2000/32767.0)*(pi/180) rad/s)
#define IMU_REG_GYRO       0x0A  // 0x0A-0x0F: GX_L,GX_H, GY_L,GY_H, GZ_L,GZ_H

// Magnetometer raw data (int16 x3, scale: 800/32767.0)
#define IMU_REG_MAG        0x10  // 0x10-0x15: MX_L,MX_H, MY_L,MY_H, MZ_L,MZ_H

// Quaternion (float x4, LE IEEE754)
#define IMU_REG_QUAT       0x16  // 0x16-0x25: W(4B), X(4B), Y(4B), Z(4B)

// Euler angles (float x3, LE IEEE754, in radians)
#define IMU_REG_EULER      0x26  // 0x26-0x31: Roll(4B), Pitch(4B), Yaw(4B)

// Barometer (float x4, LE IEEE754, 10-axis only)
#define IMU_REG_BARO       0x32  // 0x32-0x41: Height(4B), Temp(4B), Press(4B), RefPress(4B)

// Config registers
#define IMU_REG_ALGO_TYPE  0x61  // uint8, R/W: 6=6-axis, 9=9-axis
#define IMU_REG_CALIB_IMU  0x70  // uint8, R/W: 1=start, 0=clear
#define IMU_REG_CALIB_MAG  0x71  // uint8, R/W: 1=start, 0=clear
#define IMU_REG_CALIB_TEMP 0x73  // uint16, R/W: actual_temp * 100

// Functions
int IMU_Init(void);
int IMU_ReadEuler(int16_t *roll, int16_t *pitch, int16_t *yaw);
int IMU_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az);
int IMU_ReadGyro(int16_t *gx, int16_t *gy, int16_t *gz);
int IMU_ReadMag(int16_t *mx, int16_t *my, int16_t *mz);

#endif
