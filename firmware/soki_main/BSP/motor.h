#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <stdint.h>

/* Motor driver configuration (L-type 520 motor, from motor_test) */
#define MOTOR_TYPE_ID      1
#define MOTOR_PULSE_PHASE  40
#define MOTOR_PULSE_LINE   11
#define MOTOR_WHEEL_DIA    67.00f
#define MOTOR_DEADZONE     1900

void    motor_init(void);
void    motor_set_speed(int16_t left_mm_s, int16_t right_mm_s);
void    motor_stop(void);
void    motor_poll(void);
int16_t motor_get_speed_l(void);
int16_t motor_get_speed_r(void);
int32_t motor_get_encoder_l(void);
int32_t motor_get_encoder_r(void);

#endif
