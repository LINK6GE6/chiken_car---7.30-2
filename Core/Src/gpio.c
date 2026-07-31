/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, MOTOR3_CIN2_Pin|MOTOR2_BIN2_Pin|MOTOR2_BIN1_Pin|MOTOR4_DIN2_Pin
                          |MOTOR4_DIN1_Pin|MOTOR1_AIN1_Pin|MOTOR3_CIN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|MOTOR1_AIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, HD_KEY_Pin|HD_I2C_SDA_Pin|HD_I2C_SCL_Pin|PS2_CMD_Pin
                          |PS2_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Light_Pin|Water_Pump_Pin|PS2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GZ_SOFT_SDA_Pin|GZ_SOFT_SCL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : MOTOR3_CIN2_Pin MOTOR2_BIN2_Pin MOTOR2_BIN1_Pin MOTOR4_DIN2_Pin
                           MOTOR4_DIN1_Pin MOTOR1_AIN1_Pin MOTOR3_CIN1_Pin */
  GPIO_InitStruct.Pin = MOTOR3_CIN2_Pin|MOTOR2_BIN2_Pin|MOTOR2_BIN1_Pin|MOTOR4_DIN2_Pin
                          |MOTOR4_DIN1_Pin|MOTOR1_AIN1_Pin|MOTOR3_CIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : WaterLevel_Pin */
  GPIO_InitStruct.Pin = WaterLevel_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(WaterLevel_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin MOTOR1_AIN2_Pin */
  GPIO_InitStruct.Pin = LED_Pin|MOTOR1_AIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : HD_KEY_Pin HD_I2C_SDA_Pin HD_I2C_SCL_Pin PS2_CMD_Pin
                           PS2_CLK_Pin */
  GPIO_InitStruct.Pin = HD_KEY_Pin|HD_I2C_SDA_Pin|HD_I2C_SCL_Pin|PS2_CMD_Pin
                          |PS2_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Light_Pin Water_Pump_Pin PS2_CS_Pin */
  GPIO_InitStruct.Pin = LED_Light_Pin|Water_Pump_Pin|PS2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PS2_DIN_Pin BT_STATE_Pin */
  GPIO_InitStruct.Pin = PS2_DIN_Pin|BT_STATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY1_Pin KEY2_Pin KEY3_Pin KEY4_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin|KEY2_Pin|KEY3_Pin|KEY4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : GZ_SOFT_SDA_Pin GZ_SOFT_SCL_Pin */
  GPIO_InitStruct.Pin = GZ_SOFT_SDA_Pin|GZ_SOFT_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
