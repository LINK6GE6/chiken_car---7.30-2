/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "pid.h"
#include "pwm.h"
#include "encoder.h"
#include "Sensor.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define RX_BUFFER_SIZE 128
uint8_t rx_buffer[RX_BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 宏定义
#define DEBUG 1

// 全局变量
uint8_t read_encoder_flag;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_UART4_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */

  pwm_init_start();       // 电机PWM初始化

  encoder_init_start();   // 编码器初始化

  PID_Init();

//  BH1750_GPIO_Init();   // 初始化 PB3/PB4 为开漏输出
//  BH1750_Init();        // BH1750 上电 + 设置模式

	HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_buffer, RX_BUFFER_SIZE); //UART4 的空闲中断+DMA接收  蓝牙


#ifdef DEBUG
  motor_set_speed(1, 0);
  motor_set_speed(2, 0);
  motor_set_speed(3, 0);
  motor_set_speed(4, 0);

  // pid_fl.kp = 0.5;
  // pid_fr.kp = 0.5;
  // pid_bl.kp = 0.5;
  // pid_br.kp = 0.5;

  // 目标速度值范围-412~415
#endif

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1);
    return ch;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t last_interrupt_time = 0;
  uint32_t current_time = HAL_GetTick();

  if (current_time - last_interrupt_time < 200)
  {
    return;
  }

  switch (GPIO_Pin)
  {
    // case KEY1_Pin: // D4 按键
    //   if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
		// 			target_speed_fl += 20;
		// 			target_speed_fr += 20;
		// 			target_speed_bl += 20;
		// 			target_speed_br += 20;
    //     last_interrupt_time = current_time;
    //   }
    //   break;

    // case KEY2_Pin: // D5 按键
    //   if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET) {
		// 			target_speed_fl -= 20;
		// 			target_speed_fr -= 20;
		// 			target_speed_bl -= 20;
		// 			target_speed_br -= 20;
    //     last_interrupt_time = current_time;
    //   }
    //   break;

    // case KEY3_Pin: // D6 按键
    //   if (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_RESET) {
		// 			target_speed_fl += 150;
		// 			target_speed_fr += 150;
		// 			target_speed_bl += 150;
		// 			target_speed_br += 150;
    //     last_interrupt_time = current_time;
    //   }
    //   break;

    // case KEY4_Pin: // D7 按键
    //   if (HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin) == GPIO_PIN_RESET) {
		// 			target_speed_fl -= 150;
		// 			target_speed_fr -= 150;
		// 			target_speed_bl -= 150;
		// 			target_speed_br -= 150;
    //     last_interrupt_time = current_time;
    //   }
    //   break;

    // default:
    //   break;
  }
}

extern osMessageQueueId_t BtCmdQueueHandle;
extern uint8_t rx_buffer[128];

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART4)
    {
        // 1. 遍历这次收到的所有字符，依次压入队列
        for (uint16_t i = 0; i < Size; i++)
        {
            // 在中断里发数据，超时时间设为 0
//            osMessageQueuePut(BtCmdQueueHandle, &rx_buffer[i], 0, 0);
        }

        // 2. 立刻重启 DMA 接收！
        // 把这行代码放在中断里执行，最大限度缩短了漏接数据的真空期
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_buffer, 128);
    }

    //     memset(rx_buffer, 0, RX_BUFFER_SIZE);

    //     HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_buffer, RX_BUFFER_SIZE);
    //

}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
