#ifndef __PWM_H__
#define __PWM_H__

#include "main.h"

#define MOTOR_1_CH TIM_CHANNEL_1
#define MOTOR_2_CH TIM_CHANNEL_3
#define MOTOR_3_CH TIM_CHANNEL_2
#define MOTOR_4_CH TIM_CHANNEL_4

void pwm_init_start(void);
void motor_set_speed(int motor, int speed);

#endif
