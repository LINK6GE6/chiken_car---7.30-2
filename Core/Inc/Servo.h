#ifndef __SERVO_H__
#define __SERVO_H__

#include "main.h"

/* TIM9 PWM 参数（1MHz计数，50Hz = 20ms周期） */
#define SERVO_PULSE_MIN   500    // 0.5ms →  0度
#define SERVO_PULSE_MID   1500   // 1.5ms → 90度（中位）
#define SERVO_PULSE_MAX   2500   // 2.5ms → 180度

#define SERVO_ANGLE_MIN   0.0f
#define SERVO_ANGLE_MAX   180.0f

/* 通道宏，对应 TIM9 */
#define SERVO_X_CH  TIM_CHANNEL_1   // PA2
#define SERVO_Y_CH  TIM_CHANNEL_2   // PA3

void Servo_Init(void);
void ServoX_SetAngle(float angle);
void ServoY_SetAngle(float angle);

#endif
