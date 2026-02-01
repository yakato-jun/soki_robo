/*
 * Encoder Read Test for Yahboom MSPM0 Expansion Board
 * Reads encoder values from motor driver (no motor movement)
 */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "usart.h"
#include "app_motor_usart.h"

#define MOTOR_TYPE 2   // 1:520 motor 2:310 motor 3:TT encoder 4:TT DC 5:L type 520

uint8_t timer_count = 0;

// Helper function to print integer
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

int main(void)
{
    USART_Init();

    printf("Encoder Read Test\r\n");
    printf("Initializing...\r\n");

    // Stop motors first
    Contrl_Pwm(0, 0, 0, 0);
    delay_ms(100);

    // Disable all data upload initially
    send_upload_data(false, false, false);
    delay_ms(10);

    // Configure motor type
    #if MOTOR_TYPE == 2
    send_motor_type(2);
    delay_ms(100);
    send_pulse_phase(20);
    delay_ms(100);
    send_pulse_line(13);
    delay_ms(100);
    send_wheel_diameter(48.00);
    delay_ms(100);
    send_motor_deadzone(1600);
    delay_ms(100);
    #endif

    // Enable speed data upload
    send_upload_data(false, false, true);
    delay_ms(10);

    // Enable timer interrupt
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    printf("Ready!\r\n");

    while(1)
    {
        // Print every 500ms
        if(timer_count >= 5)
        {
            timer_count = 0;

            // Print speed values manually
            printf("M1:");
            print_int((int)g_Speed[0]);
            printf(" M2:");
            print_int((int)g_Speed[1]);
            printf(" M3:");
            print_int((int)g_Speed[2]);
            printf(" M4:");
            print_int((int)g_Speed[3]);
            USART_SendData('\r');
            USART_SendData('\n');
        }

        // Process received data from motor driver
        if(g_recv_flag == 1)
        {
            g_recv_flag = 0;
            Deal_data_real();
        }
    }
}

// Timer interrupt handler (100ms interval)
void TIMER_0_INST_IRQHandler(void)
{
    switch(DL_TimerG_getPendingInterrupt(TIMER_0_INST))
    {
        case DL_TIMER_IIDX_ZERO:
            timer_count++;
            break;
        default:
            break;
    }
}
