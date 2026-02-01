/*
 * souki_main — Modbus RTU Slave Firmware
 *
 * MSPM0G3507 + Yahboom Expansion Board
 * RPi connects via UART0 (Modbus RTU, 115200 baud)
 * Motor driver on UART1, IMU on software I2C (PA12/PA13)
 *
 * Registers: see modbus.h for full map
 *   0x00-0x01  Status / Heartbeat
 *   0x10-0x15  IMU raw (accel + gyro)
 *   0x20-0x27  Quaternion (float, 2 regs each)
 *   0x30-0x35  Encoder + Speed
 *   0x40-0x41  Motor speed command (R/W)
 */

#include "ti_msp_dl_config.h"
#include "usart.h"
#include "delay.h"
#include "ioi2c.h"
#include "imu.h"
#include "modbus.h"
#include "motor.h"

/* ---- Axis mapping (passthrough, adjust after real-hardware test) ---- */
#define AXIS_MAP_ACCEL_X(ax, ay, az)  (ax)
#define AXIS_MAP_ACCEL_Y(ax, ay, az)  (ay)
#define AXIS_MAP_ACCEL_Z(ax, ay, az)  (az)
#define AXIS_MAP_GYRO_X(gx, gy, gz)   (gx)
#define AXIS_MAP_GYRO_Y(gx, gy, gz)   (gy)
#define AXIS_MAP_GYRO_Z(gx, gy, gz)   (gz)

/* ---- Safety timeout ---- */
#define CMD_TIMEOUT_COUNT   5   /* 5 x 100ms = 500ms */

static volatile uint8_t timer0_flag = 0;
static uint16_t cmd_timeout_cnt = CMD_TIMEOUT_COUNT;

/* ---- Store a float (LE) into two holding registers [lo_word, hi_word] ---- */
static void store_float_le(uint16_t reg_base, const uint8_t *bytes)
{
    /* bytes[0..3] = IEEE754 LE: byte0(LSB)..byte3(MSB) */
    /* lo_word = bytes[0] | bytes[1]<<8,  hi_word = bytes[2] | bytes[3]<<8 */
    holding_regs[reg_base]     = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
    holding_regs[reg_base + 1] = (uint16_t)bytes[2] | ((uint16_t)bytes[3] << 8);
}

/* ---- Update IMU registers ---- */
static void update_imu_regs(void)
{
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
    uint8_t qbuf[16];

    /* Accel + Gyro raw */
    if (IMU_ReadAccel(&ax, &ay, &az) == 0) {
        holding_regs[REG_ACCEL_X] = (uint16_t)AXIS_MAP_ACCEL_X(ax, ay, az);
        holding_regs[REG_ACCEL_Y] = (uint16_t)AXIS_MAP_ACCEL_Y(ax, ay, az);
        holding_regs[REG_ACCEL_Z] = (uint16_t)AXIS_MAP_ACCEL_Z(ax, ay, az);
    }

    if (IMU_ReadGyro(&gx, &gy, &gz) == 0) {
        holding_regs[REG_GYRO_X] = (uint16_t)AXIS_MAP_GYRO_X(gx, gy, gz);
        holding_regs[REG_GYRO_Y] = (uint16_t)AXIS_MAP_GYRO_Y(gx, gy, gz);
        holding_regs[REG_GYRO_Z] = (uint16_t)AXIS_MAP_GYRO_Z(gx, gy, gz);
    }

    /* Magnetometer raw */
    if (IMU_ReadMag(&mx, &my, &mz) == 0) {
        holding_regs[REG_MAG_X] = (uint16_t)mx;
        holding_regs[REG_MAG_Y] = (uint16_t)my;
        holding_regs[REG_MAG_Z] = (uint16_t)mz;
    }

    /* Quaternion (4 x float LE = 16 bytes from register 0x16) */
    if (i2cRead(IMU_ADDR, IMU_REG_QUAT, 16, qbuf) == 0) {
        store_float_le(REG_QUAT_W_LO, &qbuf[0]);   /* W */
        store_float_le(REG_QUAT_X_LO, &qbuf[4]);   /* X */
        store_float_le(REG_QUAT_Y_LO, &qbuf[8]);   /* Y */
        store_float_le(REG_QUAT_Z_LO, &qbuf[12]);  /* Z */
    }
}

/* ---- Update motor/encoder registers ---- */
static void update_motor_regs(void)
{
    int32_t enc_l = motor_get_encoder_l();
    int32_t enc_r = motor_get_encoder_r();

    holding_regs[REG_ENC_L_HI] = (uint16_t)((enc_l >> 16) & 0xFFFF);
    holding_regs[REG_ENC_L_LO] = (uint16_t)(enc_l & 0xFFFF);
    holding_regs[REG_ENC_R_HI] = (uint16_t)((enc_r >> 16) & 0xFFFF);
    holding_regs[REG_ENC_R_LO] = (uint16_t)(enc_r & 0xFFFF);

    holding_regs[REG_SPEED_L] = (uint16_t)motor_get_speed_l();
    holding_regs[REG_SPEED_R] = (uint16_t)motor_get_speed_r();
}

/* ---- Apply motor commands from holding registers ---- */
static void apply_motor_cmd(void)
{
    /* Check if a Modbus write to CMD registers occurred */
    if (modbus_cmd_written) {
        modbus_cmd_written = 0;
        cmd_timeout_cnt = 0;
    } else {
        if (cmd_timeout_cnt < CMD_TIMEOUT_COUNT) {
            cmd_timeout_cnt++;
        }
    }

    /* Safety: stop if no write for 500ms */
    if (cmd_timeout_cnt >= CMD_TIMEOUT_COUNT) {
        motor_stop();
    } else {
        int16_t cmd_l = (int16_t)holding_regs[REG_CMD_SPEED_L];
        int16_t cmd_r = (int16_t)holding_regs[REG_CMD_SPEED_R];
        motor_set_speed(cmd_l, cmd_r);
    }
}

/* ================================================================ */
int main(void)
{
    /* Initialize peripherals */
    USART_Init();
    delay_ms(100);

    /* Initialize motor driver */
    motor_init();

    /* Initialize IMU (retry up to 5 times) */
    int imu_ok = 0;
    {
        int i;
        for (i = 0; i < 5; i++) {
            if (IMU_Init() == 0) {
                imu_ok = 1;
                break;
            }
            delay_ms(100);
        }
    }

    /* Enable timer interrupts */
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_1_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_1_INST_INT_IRQN);

    /* Set initial status */
    holding_regs[REG_STATUS] = 0;
    if (imu_ok) {
        holding_regs[REG_STATUS] |= STATUS_IMU_OK;
    }
    holding_regs[REG_STATUS] |= STATUS_MOTOR_OK;

    /* Main loop */
    while (1) {
        /* 1. Modbus frame processing */
        if (modbus_frame_ready) {
            modbus_frame_ready = 0;
            modbus_process_frame();
        }

        /* 2. Motor driver response processing */
        motor_poll();

        /* 3. 100ms periodic tasks */
        if (timer0_flag) {
            timer0_flag = 0;

            /* IMU read → registers */
            update_imu_regs();

            /* Encoder / speed → registers */
            update_motor_regs();

            /* Apply speed command from Modbus registers */
            apply_motor_cmd();

            /* Heartbeat */
            holding_regs[REG_HEARTBEAT]++;
        }
    }
}

/* ---- TIMER_0 ISR: 100ms periodic ---- */
void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            timer0_flag = 1;
            break;
        default:
            break;
    }
}

/* ---- TIMER_1 ISR: Modbus inter-frame timeout (~2ms) ---- */
void TIMER_1_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_1_INST)) {
        case DL_TIMER_IIDX_ZERO:
            modbus_frame_ready = 1;
            break;
        default:
            break;
    }
}
