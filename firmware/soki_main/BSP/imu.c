#include "imu.h"
#include "ioi2c.h"
#include "delay.h"

// Verify I2C communication by reading version register
int IMU_Init(void)
{
    uint8_t buf[3];
    int ret = i2cRead(IMU_ADDR, IMU_REG_VERSION, 3, buf);
    return ret;  // 0 = success
}

// Read Euler angles (float, radians) - returns raw bytes
// Caller must parse as 3x LE IEEE754 float, multiply by 57.2958 for degrees
int IMU_ReadEuler(int16_t *roll, int16_t *pitch, int16_t *yaw)
{
    uint8_t buf[12];
    int ret = i2cRead(IMU_ADDR, IMU_REG_EULER, 12, buf);
    if (ret != 0) return ret;

    // Note: Euler data is actually float, not int16.
    // This function is kept for backward compat but values will be wrong.
    // Use float_to_deg_x10() in empty.c for correct parsing.
    *roll  = (int16_t)(buf[1] << 8 | buf[0]);
    *pitch = (int16_t)(buf[5] << 8 | buf[4]);
    *yaw   = (int16_t)(buf[9] << 8 | buf[8]);
    return 0;
}

// Read acceleration (raw int16 values)
// Convert to g: raw / 32768.0 * 16.0
int IMU_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];
    int ret = i2cRead(IMU_ADDR, IMU_REG_ACCEL, 6, buf);
    if (ret != 0) return ret;

    *ax = (int16_t)(buf[1] << 8 | buf[0]);
    *ay = (int16_t)(buf[3] << 8 | buf[2]);
    *az = (int16_t)(buf[5] << 8 | buf[4]);
    return 0;
}

// Read angular velocity (raw int16 values)
// Convert to deg/s: raw / 32768.0 * 2000.0
int IMU_ReadGyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];
    int ret = i2cRead(IMU_ADDR, IMU_REG_GYRO, 6, buf);
    if (ret != 0) return ret;

    *gx = (int16_t)(buf[1] << 8 | buf[0]);
    *gy = (int16_t)(buf[3] << 8 | buf[2]);
    *gz = (int16_t)(buf[5] << 8 | buf[4]);
    return 0;
}

// Read magnetometer (raw int16 values)
// Convert to uT-like unit: raw / 32768.0 * 800.0
int IMU_ReadMag(int16_t *mx, int16_t *my, int16_t *mz)
{
    uint8_t buf[6];
    int ret = i2cRead(IMU_ADDR, IMU_REG_MAG, 6, buf);
    if (ret != 0) return ret;

    *mx = (int16_t)(buf[1] << 8 | buf[0]);
    *my = (int16_t)(buf[3] << 8 | buf[2]);
    *mz = (int16_t)(buf[5] << 8 | buf[4]);
    return 0;
}
