#include "pwm.h"
#include "math.h"
#include <stdlib.h>
#include "tim.h"

static int my_abs(int value)
{
	if (value<0) value = -value;
	return value;
}

void pwm_init_start(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

// 参数motor
// 1是左前 2是右前 3是左后 4是右后
// 参数speed
// 速度值 可正负
void motor_set_speed(int motor, int speed)
{
	switch(motor)
	{
		case 1:
			if(speed > 0){
				HAL_GPIO_WritePin(MOTOR1_AIN1_GPIO_Port, MOTOR1_AIN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(MOTOR1_AIN2_GPIO_Port, MOTOR1_AIN2_Pin, GPIO_PIN_RESET);
			}else{
				HAL_GPIO_WritePin(MOTOR1_AIN1_GPIO_Port, MOTOR1_AIN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(MOTOR1_AIN2_GPIO_Port, MOTOR1_AIN2_Pin, GPIO_PIN_SET);
			}

			__HAL_TIM_SET_COMPARE(&htim1, MOTOR_1_CH, my_abs(speed));
			// TIM1->CCR1 = my_abs(speed);
		break;

		case 2:
			if(speed > 0){
				HAL_GPIO_WritePin(MOTOR2_BIN1_GPIO_Port, MOTOR2_BIN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(MOTOR2_BIN2_GPIO_Port, MOTOR2_BIN2_Pin, GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(MOTOR2_BIN1_GPIO_Port, MOTOR2_BIN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(MOTOR2_BIN2_GPIO_Port, MOTOR2_BIN2_Pin, GPIO_PIN_RESET);
			}

			__HAL_TIM_SET_COMPARE(&htim1, MOTOR_2_CH, my_abs(speed));
			// TIM1->CCR3 = my_abs(speed);
		break;

		case 3:
			if(speed > 0){
				HAL_GPIO_WritePin(MOTOR3_CIN1_GPIO_Port, MOTOR3_CIN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(MOTOR3_CIN2_GPIO_Port, MOTOR3_CIN2_Pin, GPIO_PIN_RESET);
			}else{
				HAL_GPIO_WritePin(MOTOR3_CIN1_GPIO_Port, MOTOR3_CIN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(MOTOR3_CIN2_GPIO_Port, MOTOR3_CIN2_Pin, GPIO_PIN_SET);
			}

			__HAL_TIM_SET_COMPARE(&htim1, MOTOR_3_CH, my_abs(speed));
			// TIM1->CCR2 = my_abs(speed);
		break;

		case 4:
			if(speed > 0){
				HAL_GPIO_WritePin(MOTOR4_DIN1_GPIO_Port, MOTOR4_DIN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(MOTOR4_DIN2_GPIO_Port, MOTOR4_DIN2_Pin, GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(MOTOR4_DIN1_GPIO_Port, MOTOR4_DIN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(MOTOR4_DIN2_GPIO_Port, MOTOR4_DIN2_Pin, GPIO_PIN_RESET);
			}

			__HAL_TIM_SET_COMPARE(&htim1, MOTOR_4_CH, my_abs(speed));
			// TIM1->CCR4 = my_abs(speed);
		break;
	}
}
