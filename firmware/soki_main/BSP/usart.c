#include "usart.h"
#include "modbus.h"
#include "app_motor_usart.h"

void USART_Init(void)
{
    SYSCFG_DL_init();

    /* Enable UART interrupts */
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

/* Send one byte via UART0 (Modbus response) */
void USART_SendData(unsigned char data)
{
    while (DL_UART_isBusy(UART_0_INST) == true)
        ;
    DL_UART_Main_transmitData(UART_0_INST, data);
}

/*
 * UART0 ISR — Modbus RTU frame reception
 * Each received byte is stored in modbus_rx_buf.
 * TIMER_1 (one-shot ~2ms) is restarted on each byte;
 * when it expires, it sets modbus_frame_ready = 1.
 */
void UART_0_INST_IRQHandler(void)
{
    uint8_t rx_byte;

    switch (DL_UART_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_IIDX_RX:
            rx_byte = DL_UART_Main_receiveData(UART_0_INST);

            if (modbus_rx_len < MODBUS_RX_BUF_SIZE) {
                modbus_rx_buf[modbus_rx_len++] = rx_byte;
            }

            /* Restart TIMER_1 (Modbus inter-frame timeout) */
            DL_TimerG_stopCounter(TIMER_1_INST);
            DL_TimerG_setTimerCount(TIMER_1_INST,
                                    DL_TimerG_getLoadValue(TIMER_1_INST));
            DL_TimerG_startCounter(TIMER_1_INST);
            break;

        default:
            break;
    }
}

/*
 * UART1 ISR — Motor driver protocol reception (unchanged)
 */
void UART_1_INST_IRQHandler(void)
{
    uint8_t rx_byte;

    switch (DL_UART_getPendingInterrupt(UART_1_INST)) {
        case DL_UART_IIDX_RX:
            rx_byte = DL_UART_Main_receiveData(UART_1_INST);
            Deal_Control_Rxtemp(rx_byte);
            break;

        default:
            break;
    }
}
