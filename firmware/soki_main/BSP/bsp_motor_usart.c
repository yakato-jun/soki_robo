#include "bsp_motor_usart.h"

/************************************************
�������� �� Send_Motor_U8		Function name: Send_Motor_U8
��    �� �� USART1����һ���ַ�	Function: USART1 sends a character
��    �� �� Data --- ����		Parameter: Data --- data
�� �� ֵ �� ��					Return value: None
*************************************************/
void Send_Motor_U8(uint8_t Data)
{
	while( DL_UART_isBusy(UART_1_INST) == true );
	DL_UART_Main_transmitData(UART_1_INST, Data);
}

/************************************************
�������� �� Send_Motor_ArrayU8	Function name: Send_Motor_ArrayU8
��    �� �� ����1����N���ַ�		Function: Serial port 1 sends N characters
��    �� �� pData ---- �ַ���	Parameter: pData ---- string
            Length --- ����		Length --- length
�� �� ֵ �� ��					Return value: None
*************************************************/
void Send_Motor_ArrayU8(uint8_t *pData, uint16_t Length)
{
	while (Length--)
	{
		Send_Motor_U8(*pData);
		pData++;
	}
}


/* UART_1_INST_IRQHandler is defined in usart.c (centralized ISR) */
