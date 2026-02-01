/*
 * Simple UART Echo for Yahboom MSPM0 Expansion Board
 * Receives data from PC and echoes it back
 */

#include "ti_msp_dl_config.h"
#include "usart.h"

// External variables from usart.c
extern volatile uint8_t  recv0_buff[];
extern volatile uint16_t recv0_length;
extern volatile uint8_t  recv0_flag;

int main(void)
{
    uint16_t i;

    // Initialize UART
    USART_Init();

    // Send startup message
    printf("UART Echo Ready\r\n");

    while(1)
    {
        // Check if data received
        if(recv0_flag == 1)
        {
            recv0_flag = 0;

            // Echo back received data
            for(i = 0; i < recv0_length; i++)
            {
                USART_SendData(recv0_buff[i]);
            }

            // Clear buffer
            recv0_length = 0;
        }
    }
}
