#ifndef __MODBUS_H__
#define __MODBUS_H__

#include <stdint.h>

/* Slave address */
#define MODBUS_SLAVE_ADDR   1

/* Register count (0x00 .. 0x41) */
#define MODBUS_REG_COUNT    0x42

/* Register addresses — Status */
#define REG_STATUS          0x00
#define REG_HEARTBEAT       0x01

/* Register addresses — IMU raw */
#define REG_ACCEL_X         0x10
#define REG_ACCEL_Y         0x11
#define REG_ACCEL_Z         0x12
#define REG_GYRO_X          0x13
#define REG_GYRO_Y          0x14
#define REG_GYRO_Z          0x15

/* Register addresses — Magnetometer raw */
#define REG_MAG_X           0x16
#define REG_MAG_Y           0x17
#define REG_MAG_Z           0x18

/* Register addresses — Quaternion (2 regs each, LE word order) */
#define REG_QUAT_W_LO       0x20
#define REG_QUAT_W_HI       0x21
#define REG_QUAT_X_LO       0x22
#define REG_QUAT_X_HI       0x23
#define REG_QUAT_Y_LO       0x24
#define REG_QUAT_Y_HI       0x25
#define REG_QUAT_Z_LO       0x26
#define REG_QUAT_Z_HI       0x27

/* Register addresses — Encoder / Speed */
#define REG_ENC_L_HI        0x30
#define REG_ENC_L_LO        0x31
#define REG_ENC_R_HI        0x32
#define REG_ENC_R_LO        0x33
#define REG_SPEED_L          0x34
#define REG_SPEED_R          0x35

/* Register addresses — Motor command (R/W) */
#define REG_CMD_SPEED_L      0x40
#define REG_CMD_SPEED_R      0x41

/* Writable range */
#define REG_WRITE_START      0x40
#define REG_WRITE_END        0x41

/* Status bits */
#define STATUS_IMU_OK        (1 << 0)
#define STATUS_MOTOR_OK      (1 << 1)

/* RX buffer size */
#define MODBUS_RX_BUF_SIZE   128

/* Modbus exception codes */
#define MODBUS_EX_ILLEGAL_FUNCTION   0x01
#define MODBUS_EX_ILLEGAL_ADDRESS    0x02
#define MODBUS_EX_ILLEGAL_VALUE      0x03

/* Shared data */
extern uint16_t holding_regs[MODBUS_REG_COUNT];
extern uint8_t  modbus_rx_buf[MODBUS_RX_BUF_SIZE];
extern uint16_t modbus_rx_len;
extern volatile uint8_t modbus_frame_ready;
extern volatile uint8_t modbus_cmd_written;  /* set on write to 0x40-0x41 */

/* Functions */
uint16_t modbus_crc16(const uint8_t *data, uint16_t len);
void     modbus_process_frame(void);
void     modbus_send_response(const uint8_t *data, uint16_t len);

#endif
