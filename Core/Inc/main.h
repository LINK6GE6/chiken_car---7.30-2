/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR3_CIN2_Pin GPIO_PIN_2
#define MOTOR3_CIN2_GPIO_Port GPIOE
#define MOTOR2_BIN2_Pin GPIO_PIN_3
#define MOTOR2_BIN2_GPIO_Port GPIOE
#define MOTOR2_BIN1_Pin GPIO_PIN_4
#define MOTOR2_BIN1_GPIO_Port GPIOE
#define MOTOR4_DIN2_Pin GPIO_PIN_5
#define MOTOR4_DIN2_GPIO_Port GPIOE
#define MOTOR4_DIN1_Pin GPIO_PIN_6
#define MOTOR4_DIN1_GPIO_Port GPIOE
#define Encoder3_A_Pin GPIO_PIN_0
#define Encoder3_A_GPIO_Port GPIOA
#define Encoder3_B_Pin GPIO_PIN_1
#define Encoder3_B_GPIO_Port GPIOA
#define ServoX_Pin GPIO_PIN_2
#define ServoX_GPIO_Port GPIOA
#define ServoY_Pin GPIO_PIN_3
#define ServoY_GPIO_Port GPIOA
#define WaterLevel_Pin GPIO_PIN_4
#define WaterLevel_GPIO_Port GPIOA
#define Encoder1_A_Pin GPIO_PIN_6
#define Encoder1_A_GPIO_Port GPIOA
#define Encoder1_B_Pin GPIO_PIN_7
#define Encoder1_B_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOB
#define HD_KEY_Pin GPIO_PIN_11
#define HD_KEY_GPIO_Port GPIOD
#define Encoder2_A_Pin GPIO_PIN_12
#define Encoder2_A_GPIO_Port GPIOD
#define Encoder2_B_Pin GPIO_PIN_13
#define Encoder2_B_GPIO_Port GPIOD
#define HD_I2C_SDA_Pin GPIO_PIN_14
#define HD_I2C_SDA_GPIO_Port GPIOD
#define HD_I2C_SCL_Pin GPIO_PIN_15
#define HD_I2C_SCL_GPIO_Port GPIOD
#define Encoder4_A_Pin GPIO_PIN_6
#define Encoder4_A_GPIO_Port GPIOC
#define Encoder4_B_Pin GPIO_PIN_7
#define Encoder4_B_GPIO_Port GPIOC
#define LED_Light_Pin GPIO_PIN_8
#define LED_Light_GPIO_Port GPIOC
#define Water_Pump_Pin GPIO_PIN_9
#define Water_Pump_GPIO_Port GPIOC
#define PS2_CS_Pin GPIO_PIN_12
#define PS2_CS_GPIO_Port GPIOC
#define PS2_CMD_Pin GPIO_PIN_0
#define PS2_CMD_GPIO_Port GPIOD
#define PS2_CLK_Pin GPIO_PIN_1
#define PS2_CLK_GPIO_Port GPIOD
#define PS2_DIN_Pin GPIO_PIN_2
#define PS2_DIN_GPIO_Port GPIOD
#define BT_STATE_Pin GPIO_PIN_3
#define BT_STATE_GPIO_Port GPIOD
#define KEY1_Pin GPIO_PIN_4
#define KEY1_GPIO_Port GPIOD
#define KEY1_EXTI_IRQn EXTI4_IRQn
#define KEY2_Pin GPIO_PIN_5
#define KEY2_GPIO_Port GPIOD
#define KEY2_EXTI_IRQn EXTI9_5_IRQn
#define KEY3_Pin GPIO_PIN_6
#define KEY3_GPIO_Port GPIOD
#define KEY3_EXTI_IRQn EXTI9_5_IRQn
#define KEY4_Pin GPIO_PIN_7
#define KEY4_GPIO_Port GPIOD
#define KEY4_EXTI_IRQn EXTI9_5_IRQn
#define GZ_SOFT_SDA_Pin GPIO_PIN_3
#define GZ_SOFT_SDA_GPIO_Port GPIOB
#define GZ_SOFT_SCL_Pin GPIO_PIN_4
#define GZ_SOFT_SCL_GPIO_Port GPIOB
#define MPU6050_SCL_Pin GPIO_PIN_6
#define MPU6050_SCL_GPIO_Port GPIOB
#define MPU6050_SDA_Pin GPIO_PIN_7
#define MPU6050_SDA_GPIO_Port GPIOB
#define MOTOR1_AIN2_Pin GPIO_PIN_9
#define MOTOR1_AIN2_GPIO_Port GPIOB
#define MOTOR1_AIN1_Pin GPIO_PIN_0
#define MOTOR1_AIN1_GPIO_Port GPIOE
#define MOTOR3_CIN1_Pin GPIO_PIN_1
#define MOTOR3_CIN1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
