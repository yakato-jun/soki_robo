#include "motor.h"
#include "app_motor_usart.h"
#include "delay.h"

void motor_init(void)
{
    /* Stop all motors first */
    Contrl_Pwm(0, 0, 0, 0);
    delay_ms(100);

    /* Disable uploads during config */
    send_upload_data(false, false, false);
    delay_ms(10);

    /* Motor type and parameters */
    send_motor_type(MOTOR_TYPE_ID);
    delay_ms(100);
    send_pulse_phase(MOTOR_PULSE_PHASE);
    delay_ms(100);
    send_pulse_line(MOTOR_PULSE_LINE);
    delay_ms(100);
    send_wheel_diameter(MOTOR_WHEEL_DIA);
    delay_ms(100);
    send_motor_deadzone(MOTOR_DEADZONE);
    delay_ms(100);

    /* Enable speed + cumulative encoder upload */
    send_upload_data(true, false, true);
    delay_ms(10);
}

/*
 * Set motor speed.
 * M1 = +left, M2 = -right (M2 is sign-reversed for differential drive).
 */
void motor_set_speed(int16_t left_mm_s, int16_t right_mm_s)
{
    Contrl_Speed(left_mm_s, -right_mm_s, 0, 0);
}

void motor_stop(void)
{
    Contrl_Pwm(0, 0, 0, 0);
}

/* Poll for motor driver UART responses */
void motor_poll(void)
{
    if (g_recv_flag) {
        g_recv_flag = 0;
        Deal_data_real();
    }
}

int16_t motor_get_speed_l(void)
{
    return (int16_t)g_Speed[0];
}

int16_t motor_get_speed_r(void)
{
    return -(int16_t)g_Speed[1];  /* negate: M2 runs reversed */
}

int32_t motor_get_encoder_l(void)
{
    return (int32_t)Encoder_Now[0];
}

int32_t motor_get_encoder_r(void)
{
    return -(int32_t)Encoder_Now[1];  /* negate: M2 runs reversed */
}
