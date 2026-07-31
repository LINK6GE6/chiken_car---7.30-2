#include "servo.h"
#include "tim.h"

/**
 * @brief  舵机初始化：启动 TIM9 两个 PWM 通道，舵机回到中位（90度）
 */
void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim9, SERVO_X_CH);
    HAL_TIM_PWM_Start(&htim9, SERVO_Y_CH);

    // 两个舵机回到中位
    __HAL_TIM_SET_COMPARE(&htim9, SERVO_X_CH, SERVO_PULSE_MID);
    __HAL_TIM_SET_COMPARE(&htim9, SERVO_Y_CH, SERVO_PULSE_MID);
}

/**
 * @brief  将角度转换为 CCR 比较值
 * @param  angle  角度（0 ~ 180 度）
 * @retval CCR 值（500 ~ 2500）
 */
static uint16_t servo_angle_to_pulse(float angle)
{
    if (angle < SERVO_ANGLE_MIN) angle = SERVO_ANGLE_MIN;
    if (angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;

    return (uint16_t)(SERVO_PULSE_MIN + angle * (SERVO_PULSE_MAX - SERVO_PULSE_MIN)
                      / (SERVO_ANGLE_MAX - SERVO_ANGLE_MIN));
}

/**
 * @brief  设置舵机X角度（PA2, TIM9_CH1）
 * @param  angle  角度（0 ~ 180 度）
 */
void ServoX_SetAngle(float angle)
{
    uint16_t pulse = servo_angle_to_pulse(angle);
    __HAL_TIM_SET_COMPARE(&htim9, SERVO_X_CH, pulse);
}

/**
 * @brief  设置舵机Y角度（PA3, TIM9_CH2）
 * @param  angle  角度（0 ~ 180 度）
 */
void ServoY_SetAngle(float angle)
{
    uint16_t pulse = servo_angle_to_pulse(angle);
    __HAL_TIM_SET_COMPARE(&htim9, SERVO_Y_CH, pulse);
}
