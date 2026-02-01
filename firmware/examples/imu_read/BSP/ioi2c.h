#ifndef __IOI2C_H
#define __IOI2C_H

#include "ti_msp_dl_config.h"
#include "stdint.h"
#include "delay.h"

#define u8 uint8_t

// IO direction setting
// I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, I2C_SDA_IOMUX are defined
// in ti_msp_dl_config.h (auto-generated from empty.syscfg)
#define SDA_IN()  { DL_GPIO_disableOutput(I2C_PORT, I2C_SDA_PIN); \
                    DL_GPIO_initDigitalInput(I2C_SDA_IOMUX); }
#define SDA_OUT() {                                                \
                        DL_GPIO_initDigitalOutput(I2C_SDA_IOMUX);    \
                        DL_GPIO_setPins(I2C_PORT, I2C_SDA_PIN);      \
                        DL_GPIO_enableOutput(I2C_PORT, I2C_SDA_PIN); \
                  }

// IO operation
#define SCL(x)    ( (x) ? DL_GPIO_setPins(I2C_PORT,I2C_SCL_PIN) : DL_GPIO_clearPins(I2C_PORT,I2C_SCL_PIN) )
#define SDA(x)    ( (x) ? DL_GPIO_setPins(I2C_PORT,I2C_SDA_PIN) : DL_GPIO_clearPins(I2C_PORT,I2C_SDA_PIN) )
#define SDA_GET() ( ( ( DL_GPIO_readPins(I2C_PORT,I2C_SDA_PIN) & I2C_SDA_PIN ) > 0 ) ? 1 : 0 )


// I2C protocol functions
int IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Byte(u8 txd);
u8 IIC_Read_Byte(void);
int IIC_Wait_Ack(void);
void IIC_Send_Ack(unsigned char ack);

// High-level I2C read/write
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif
