/*
 * Motor Channel Test (Speed Mode)
 * M1, M2 を速度指定で順番に回転させ、エンコーダ値を確認する。
 *
 * 各モーターを3秒間回転 → 2秒停止 → 次のモーターへ
 * エンコーダ速度値を500msごとに出力
 */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "usart.h"
#include "app_motor_usart.h"

/* L type 520 motor config (MOTOR_TYPE 5) */
#define MOTOR_TYPE_ID    1
#define PULSE_PHASE      40
#define PULSE_LINE       11
#define WHEEL_DIAMETER   67.00f
#define DEADZONE         1900

#define TEST_SPEED       200   /* mm/s */
#define RUN_TIME_MS      3000
#define PAUSE_TIME_MS    2000

volatile uint8_t timer_count = 0;

void print_str(const char *s)
{
    while (*s) USART_SendData(*s++);
}

void print_int(int val)
{
    char buf[12];
    int i = 0, neg = 0;
    if (val < 0) { neg = 1; val = -val; }
    if (val == 0) { USART_SendData('0'); return; }
    while (val > 0) { buf[i++] = '0' + (val % 10); val /= 10; }
    if (neg) USART_SendData('-');
    while (i > 0) USART_SendData(buf[--i]);
}

void poll_and_print(void)
{
    if (g_recv_flag) {
        g_recv_flag = 0;
        Deal_data_real();
    }
    print_str("  SPD: M1=");
    print_int((int)g_Speed[0]);
    print_str(" M2=");
    print_int((int)g_Speed[1]);
    print_str("\r\n");
}

void test_motor(int m_index, int16_t speed)
{
    int16_t spd[4] = {0, 0, 0, 0};
    int elapsed = 0;

    print_str("\r\n=== M");
    print_int(m_index + 1);
    print_str(" @ ");
    print_int(speed);
    print_str(" mm/s ===\r\n");

    spd[m_index] = speed;
    Contrl_Speed(spd[0], spd[1], spd[2], spd[3]);

    while (elapsed < RUN_TIME_MS) {
        delay_ms(500);
        elapsed += 500;

        if (g_recv_flag) {
            g_recv_flag = 0;
            Deal_data_real();
        }

        print_str("  [");
        print_int(elapsed / 1000);
        print_str(".");
        print_int((elapsed % 1000) / 100);
        print_str("s]");
        poll_and_print();
    }

    Contrl_Speed(0, 0, 0, 0);
    print_str("=== M");
    print_int(m_index + 1);
    print_str(" STOP ===\r\n");

    delay_ms(PAUSE_TIME_MS);
}

int main(void)
{
    USART_Init();
    delay_ms(100);

    print_str("\r\n--- Motor Speed Test ---\r\n");

    /* Stop all motors */
    Contrl_Pwm(0, 0, 0, 0);
    delay_ms(100);

    /* Configure motor driver */
    send_upload_data(false, false, false);
    delay_ms(10);
    send_motor_type(MOTOR_TYPE_ID);
    delay_ms(100);
    send_pulse_phase(PULSE_PHASE);
    delay_ms(100);
    send_pulse_line(PULSE_LINE);
    delay_ms(100);
    send_wheel_diameter(WHEEL_DIAMETER);
    delay_ms(100);
    send_motor_deadzone(DEADZONE);
    delay_ms(100);

    /* Enable speed upload */
    send_upload_data(false, false, true);
    delay_ms(100);

    /* Enable timer */
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    print_str("Config done. Starting in 3s...\r\n");
    delay_ms(3000);

    /* Test M1 positive and negative */
    test_motor(0, TEST_SPEED);
    test_motor(0, -TEST_SPEED);

    /* Test M2 positive and negative */
    test_motor(1, TEST_SPEED);
    test_motor(1, -TEST_SPEED);

    print_str("\r\n>> Test complete. <<\r\n");

    /* Idle */
    while (1) {
        delay_ms(1000);
    }
}

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            timer_count++;
            break;
        default:
            break;
    }
}
