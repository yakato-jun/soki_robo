/*
 * IMU Read - Correct I2C register protocol
 * Based on Communication Protocol.xlsx I2C sheet
 * Euler angles: 12 bytes (3x float LE) from register 0x26
 */

#include "ti_msp_dl_config.h"
#include "usart.h"
#include "delay.h"
#include "ioi2c.h"
#include "imu.h"

void print_str(const char *s)
{
    while (*s) {
        USART_SendData(*s++);
    }
}

void print_int(int val)
{
    char buf[12];
    int i = 0;
    int neg = 0;

    if (val < 0) {
        neg = 1;
        val = -val;
    }
    if (val == 0) {
        USART_SendData('0');
        return;
    }
    while (val > 0) {
        buf[i++] = '0' + (val % 10);
        val /= 10;
    }
    if (neg) USART_SendData('-');
    while (i > 0) {
        USART_SendData(buf[--i]);
    }
}

void print_hex(uint8_t val)
{
    const char hex[] = "0123456789ABCDEF";
    USART_SendData('0');
    USART_SendData('x');
    USART_SendData(hex[(val >> 4) & 0x0F]);
    USART_SendData(hex[val & 0x0F]);
}

void print_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        print_hex(buf[i]);
        print_str(" ");
    }
}

/* Scan I2C bus */
void i2c_scan(void)
{
    uint8_t addr;
    int found = 0;

    print_str("I2C Scan: ");

    for (addr = 0x01; addr < 0x78; addr++) {
        IIC_Start();
        IIC_Send_Byte((addr << 1) | 0);
        if (IIC_Wait_Ack() == 0) {
            print_hex(addr);
            print_str(" ");
            found++;
        }
        IIC_Stop();
        delay_ms(2);
    }

    if (found == 0) {
        print_str("none");
    }
    print_str("\r\n");
}

/*
 * Convert IEEE 754 LE float (radians) to degrees x 10
 * Uses Q15 fixed-point intermediate to avoid float math.
 *   q15_val = mantissa >> (8 - exp)   [Q15: 32768 = 1.0]
 *   deg_x10 = q15_val * 573 / 32768   [573 ≈ 5729.578/10]
 */
int32_t float_to_deg_x10(uint8_t *b)
{
    uint32_t raw = (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
                   ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
    if (raw == 0 || (raw & 0x7F800000) == 0) return 0;

    int neg = (raw >> 31) & 1;
    int exp = ((raw >> 23) & 0xFF) - 127;
    uint32_t mantissa = (raw & 0x7FFFFF) | 0x800000;

    int32_t q15_val;
    int shift = 8 - exp;
    if (shift >= 24) return 0;
    if (shift > 0)
        q15_val = mantissa >> shift;
    else if (shift > -8)
        q15_val = mantissa << (-shift);
    else
        return neg ? -9999 : 9999;

    int32_t deg_x10 = (int32_t)q15_val * 573 / 32768;

    return neg ? -deg_x10 : deg_x10;
}

/* Print degrees with 1 decimal place from deg_x10 value */
void print_deg(int32_t deg_x10)
{
    int neg = 0;
    if (deg_x10 < 0) {
        neg = 1;
        deg_x10 = -deg_x10;
    }
    if (neg) USART_SendData('-');
    print_int((int)(deg_x10 / 10));
    USART_SendData('.');
    USART_SendData('0' + (int)(deg_x10 % 10));
}

int main(void)
{
    int ret;
    uint8_t buf[12];
    uint8_t abuf[6];
    uint8_t ver[3];
    uint8_t algo;

    USART_Init();
    delay_ms(100);

    print_str("IMU Read v2\r\n");
    delay_ms(1000);

    i2c_scan();

    /* Read version: 3 bytes from register 0x01 */
    ret = i2cRead(IMU_ADDR, IMU_REG_VERSION, 3, ver);
    print_str("Ver: ");
    if (ret == 0) {
        print_int(ver[0]); USART_SendData('.');
        print_int(ver[1]); USART_SendData('.');
        print_int(ver[2]);
    } else {
        print_str("err="); print_int(ret);
    }
    print_str("\r\n");

    /* Read algorithm type: 1 byte from register 0x61 */
    ret = i2cRead(IMU_ADDR, IMU_REG_ALGO_TYPE, 1, &algo);
    print_str("Algo: ");
    if (ret == 0) {
        print_int(algo);
    } else {
        print_str("err="); print_int(ret);
    }
    print_str("\r\n");

    /* One-shot raw hex dump of Euler registers */
    ret = i2cRead(IMU_ADDR, IMU_REG_EULER, 12, buf);
    print_str("Raw: ");
    if (ret == 0) {
        print_buf(buf, 12);
    } else {
        print_str("err="); print_int(ret);
    }
    print_str("\r\n\r\n");

    /* Continuous Euler angle + accelerometer read */
    while (1) {
        ret = i2cRead(IMU_ADDR, IMU_REG_EULER, 12, buf);
        if (ret == 0) {
            int32_t roll  = float_to_deg_x10(&buf[0]);
            int32_t pitch = float_to_deg_x10(&buf[4]);
            int32_t yaw   = float_to_deg_x10(&buf[8]);

            print_str("R:");
            print_deg(roll);
            print_str(" P:");
            print_deg(pitch);
            print_str(" Y:");
            print_deg(yaw);
        }

        ret = i2cRead(IMU_ADDR, IMU_REG_ACCEL, 6, abuf);
        if (ret == 0) {
            int16_t ax = (int16_t)(abuf[1] << 8 | abuf[0]);
            int16_t ay = (int16_t)(abuf[3] << 8 | abuf[2]);
            int16_t az = (int16_t)(abuf[5] << 8 | abuf[4]);
            /* g x 100 = raw * 100 * 16 / 32768 = raw * 25 / 512 */
            int32_t gx = (int32_t)ax * 25 / 512;
            int32_t gy = (int32_t)ay * 25 / 512;
            int32_t gz = (int32_t)az * 25 / 512;

            print_str(" A:");
            print_int(gx); USART_SendData(',');
            print_int(gy); USART_SendData(',');
            print_int(gz);
        }

        print_str("\r\n");
        delay_ms(100);
    }
}
