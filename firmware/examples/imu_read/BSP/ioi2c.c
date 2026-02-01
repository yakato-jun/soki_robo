#include "ioi2c.h"

// Generate I2C START condition
int IIC_Start(void)
{
	SDA_OUT();
	SDA(1);
	SCL(1);
	delay_us(1);
	SDA(0);
	delay_us(1);
	SCL(0);
	return 0;
}

// Generate I2C STOP condition
void IIC_Stop(void)
{
	SDA_OUT();
	SCL(0);
	SDA(0);

	SCL(1);
	delay_us(1);
	SDA(1);
	delay_us(1);
}

// Wait for ACK from slave
// Returns 0: ACK received, 1: NACK (timeout)
int IIC_Wait_Ack(void)
{
	char ack = 0;
	unsigned char ack_flag = 10;
	SCL(0);
	SDA(1);
	SDA_IN();
	delay_us(5);

	SCL(1);
	delay_us(5);
	while( (SDA_GET()==1) && ( ack_flag ) )
	{
		ack_flag--;
		delay_us(1);
	}

	if( ack_flag <= 0 )
	{
		IIC_Stop();
		return 1;
	}
	else
	{
		SCL(0);
		SDA_OUT();
	}
	return ack;
}

// Send ACK or NACK
// ack=0: send ACK, ack=1: send NACK
void IIC_Send_Ack(unsigned char ack)
{
	SDA_OUT();
	SCL(0);
	SDA(0);
	delay_us(5);
	if(!ack) SDA(0);
	else     SDA(1);
	SCL(1);
	delay_us(5);
	SCL(0);
	SDA(1);
}

// Send one byte
void IIC_Send_Byte(u8 txd)
{
	int i = 0;
	SDA_OUT();
	SCL(0);

	for( i = 0; i < 8; i++ )
	{
		SDA( (txd & 0x80) >> 7 );
		delay_us(1);
		SCL(1);
		delay_us(5);
		SCL(0);
		delay_us(5);
		txd<<=1;
	}
}

// Read one byte
u8 IIC_Read_Byte(void)
{
	unsigned char i, receive = 0;
	SDA_IN();
	delay_us(5);
	for(i = 0; i < 8; i++)
	{
		SCL(0);
		delay_us(5);
		SCL(1);
		delay_us(5);
		receive <<= 1;
		if( SDA_GET() )
		{
			receive |= 1;
		}
		delay_us(5);
	}
	SCL(0);
	return receive;
}

// Write data to register
// Returns 0: success, non-zero: error
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	uint16_t i = 0;
	IIC_Start();
	IIC_Send_Byte((addr<<1)|0);
	if( IIC_Wait_Ack() == 1 ) {IIC_Stop();return 1;}
	IIC_Send_Byte(reg);
	if( IIC_Wait_Ack() == 1 ) {IIC_Stop();return 2;}

	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(data[i]);
		if( IIC_Wait_Ack() == 1 ) {IIC_Stop();return (3+i);}
	}
	IIC_Stop();
	return 0;
}

// Read data from register
// Returns 0: success, non-zero: error
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	uint8_t i;
	IIC_Start();
	IIC_Send_Byte((addr<<1)|0);
	if( IIC_Wait_Ack() == 1 ) {IIC_Stop();return 1;}
	IIC_Send_Byte(reg);
	if( IIC_Wait_Ack() == 1 ) {IIC_Stop();return 2;}

	IIC_Start();
	IIC_Send_Byte((addr<<1)|1);
	if( IIC_Wait_Ack() == 1 ) {IIC_Stop();return 3;}

	for(i=0;i<(len-1);i++){
		buf[i]=IIC_Read_Byte();
		IIC_Send_Ack(0);
	}
	buf[i]=IIC_Read_Byte();
	IIC_Send_Ack(1);
	IIC_Stop();
	return 0;
}
