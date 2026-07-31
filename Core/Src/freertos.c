/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "pid.h"
#include "pwm.h"
#include "encoder.h"
#include "usart.h"
#include "Sensor.h"
#include "uart3_to_htp.h"
#include "oled.h"
#include "ax_ps2.h"
#include "MPU6050.h"
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
/* USER CODE BEGIN Variables */
extern int current_speed_fl, current_speed_fr, current_speed_bl, current_speed_br;
extern int current_speed_fl_10, current_speed_fr_10, current_speed_bl_10, current_speed_br_10;
extern int target_speed_fl, target_speed_fr, target_speed_bl, target_speed_br;
extern SensorData_t g_SensorData;
osMessageQueueId_t BtCmdQueueHandle;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal7,
};
/* Definitions for BtTask */
osThreadId_t BtTaskHandle;
const osThreadAttr_t BtTask_attributes = {
  .name = "BtTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal7,
};
/* Definitions for IMUTask */
osThreadId_t IMUTaskHandle;
const osThreadAttr_t IMUTask_attributes = {
  .name = "IMUTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal6,
};
/* Definitions for LEDFlashingTask */
osThreadId_t LEDFlashingTaskHandle;
const osThreadAttr_t LEDFlashingTask_attributes = {
  .name = "LEDFlashingTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal7,
};
/* Definitions for HTPComuTask */
osThreadId_t HTPComuTaskHandle;
const osThreadAttr_t HTPComuTask_attributes = {
  .name = "HTPComuTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal6,
};
/* Definitions for OLEDTask */
osThreadId_t OLEDTaskHandle;
const osThreadAttr_t OLEDTask_attributes = {
  .name = "OLEDTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal6,
};
/* Definitions for PS2Task */
osThreadId_t PS2TaskHandle;
const osThreadAttr_t PS2Task_attributes = {
  .name = "PS2Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal7,
};
/* Definitions for ActuatorTask */
osThreadId_t ActuatorTaskHandle;
const osThreadAttr_t ActuatorTask_attributes = {
  .name = "ActuatorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ExSensorTask */
osThreadId_t ExSensorTaskHandle;
const osThreadAttr_t ExSensorTask_attributes = {
  .name = "ExSensorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for global_sensor_data_mutex */
osMutexId_t global_sensor_data_mutexHandle;
const osMutexAttr_t global_sensor_data_mutex_attributes = {
  .name = "global_sensor_data_mutex"
};
/* Definitions for speed_mutex */
osMutexId_t speed_mutexHandle;
const osMutexAttr_t speed_mutex_attributes = {
  .name = "speed_mutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartMotorTask(void *argument);
void StartBtTask(void *argument);
void StartIMUTask(void *argument);
void StartLEDFlashingTask(void *argument);
void StartHTPComuTask(void *argument);
void StartOLEDTask(void *argument);
void StartPS2Task(void *argument);
void StartActuatorTask(void *argument);
void StartExSensorTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of global_sensor_data_mutex */
  global_sensor_data_mutexHandle = osMutexNew(&global_sensor_data_mutex_attributes);

  /* creation of speed_mutex */
  speed_mutexHandle = osMutexNew(&speed_mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  BtCmdQueueHandle = osMessageQueueNew(32, sizeof(uint8_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* creation of BtTask */
  BtTaskHandle = osThreadNew(StartBtTask, NULL, &BtTask_attributes);

  /* creation of IMUTask */
  IMUTaskHandle = osThreadNew(StartIMUTask, NULL, &IMUTask_attributes);

  /* creation of LEDFlashingTask */
  LEDFlashingTaskHandle = osThreadNew(StartLEDFlashingTask, NULL, &LEDFlashingTask_attributes);

  /* creation of HTPComuTask */
  HTPComuTaskHandle = osThreadNew(StartHTPComuTask, NULL, &HTPComuTask_attributes);

  /* creation of OLEDTask */
  OLEDTaskHandle = osThreadNew(StartOLEDTask, NULL, &OLEDTask_attributes);

  /* creation of PS2Task */
  PS2TaskHandle = osThreadNew(StartPS2Task, NULL, &PS2Task_attributes);

  /* creation of ActuatorTask */
  ActuatorTaskHandle = osThreadNew(StartActuatorTask, NULL, &ActuatorTask_attributes);

  /* creation of ExSensorTask */
  ExSensorTaskHandle = osThreadNew(StartExSensorTask, NULL, &ExSensorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
  uint32_t tick = osKernelGetTickCount();

  /* Infinite loop */
  for(;;)
  {
    // 蓝牙连接断开自停
		// if(HAL_GPIO_ReadPin(BT_STATE_GPIO_Port, BT_STATE_Pin) == GPIO_PIN_RESET)
    // {
//        target_speed_fl = 0;
//        target_speed_fr = 0;
//        target_speed_bl = 0;
//        target_speed_br = 0;
    // }

    PID_Task();

							//调试
//							printf("MOTOR task stack free: %u\r\n",
//              uxTaskGetStackHighWaterMark(NULL));

    vTaskDelayUntil(&tick, pdMS_TO_TICKS(50));
  }
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartBtTask */
/**
* @brief Function implementing the BtTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBtTask */
void StartBtTask(void *argument)
{
  /* USER CODE BEGIN StartBtTask */
  extern uint8_t rx_buffer[128];

  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_buffer, 128);

//  uint8_t cmd = 0;
  /* Infinite loop */
  for(;;)
  {
//      if (osMessageQueueGet(BtCmdQueueHandle, &cmd, NULL, osWaitForever) == osOK)
//      {

//          switch(cmd)
//          {
//              case 'F':
////                  target_speed_fl = 160; target_speed_fr = 160;
////                  target_speed_bl = 160; target_speed_br = 160;
//										motor_set_speed(1, 150);
//										motor_set_speed(2, 150);
//										motor_set_speed(3, 150);
//										motor_set_speed(4, 150);

//							break;
//              case 'B':
////                  target_speed_fl = -160; target_speed_fr = -160;
////                  target_speed_bl = -160; target_speed_br = -160;
//										motor_set_speed(1, -150);
//										motor_set_speed(2, -150);
//										motor_set_speed(3, -150);
//										motor_set_speed(4, -150);


//							break;

//              case 'L':
////                  target_speed_fl = -480; target_speed_fr = 250;
////                  target_speed_bl = -250; target_speed_br = 250;
//										motor_set_speed(1, -450);
//										motor_set_speed(2, 450);
//										motor_set_speed(3, -450);
//										motor_set_speed(4, 450);


//							break;

//              case 'R':
////                  target_speed_fl = 250; target_speed_fr = -250;
////                  target_speed_bl = 250; target_speed_br = -250;
//										motor_set_speed(1, 450);
//										motor_set_speed(2, -450);
//										motor_set_speed(3, 450);
//										motor_set_speed(4, -450);

//							break;

//              case 'S':
////                  target_speed_fl = 0; target_speed_fr = 0;
////                  target_speed_bl = 0; target_speed_br = 0;
//										motor_set_speed(1, 0);
//										motor_set_speed(2, 0);
//										motor_set_speed(3, 0);
//										motor_set_speed(4, 0);
//							break;

//              default:
//              break;
//          }
//      }
										//调试
							// printf("BT task stack free: %u\r\n",
              // uxTaskGetStackHighWaterMark(NULL));
              osDelay(pdMS_TO_TICKS(5));
  }
  /* USER CODE END StartBtTask */
}

/* USER CODE BEGIN Header_StartIMUTask */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void *argument)
{
  /* USER CODE BEGIN StartIMUTask */
  static uint8_t first_read_flag = 1;
  static float last_yaw = 0.0f;
	
	float current_pitch, current_roll, current_yaw;

  MPU6050_Init();
  //静置校准零偏 (此函数内部会停留约1秒，此时手不要抖板子！)
  MPU6050_Calibrate_Offset();
    printf("IMU init done\r\n");

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);  // 10ms

  /* Infinite loop */
  for(;;)
  {

    MPU6050_ReadData_DMA();

    if (!g_mpu_dma_ready)
        {
        g_mpu_dma_ready = 1;

          // 1. 获取三姿态角
          MPU6050_GetAngle(&current_pitch, &current_roll, &current_yaw);

          // 2. 将数据更新到你的全局结构体中供 OLED 等其他任务使用
          if(osMutexAcquire(global_sensor_data_mutexHandle, 5) == osOK)
          {
            g_SensorData.pitch = current_pitch;
            g_SensorData.roll  = current_roll;
            g_SensorData.yaw   = current_yaw;
            osMutexRelease(global_sensor_data_mutexHandle);
          }
        }

		float current_yaw = 0.0f;

    if(osMutexAcquire(global_sensor_data_mutexHandle, 5) == osOK)
    {
			current_yaw = g_SensorData.yaw;

      osMutexRelease(global_sensor_data_mutexHandle);
    }

		if(first_read_flag)
		{
			first_read_flag = 0;
			last_yaw = current_yaw;

			goto delay;
		}

		// 10ms的Yaw角差值
		float delta_yaw = current_yaw - last_yaw;

    last_yaw = current_yaw;

    // 处理角度环绕
    if (delta_yaw > 180.0f)  delta_yaw -= 360.0f;
    if (delta_yaw < -180.0f) delta_yaw += 360.0f;

    // 二阶低通滤波：级联两个一阶IIR，抑制突变噪声
    {
        static uint8_t  lpf_inited = 0;
        static float    lpf_y1 = 0.0f;
        static float    lpf_y2 = 0.0f;
        const  float    alpha = 0.3f;   // fc ≈ 5.7 Hz @ 100 Hz采样

        if (!lpf_inited) {
            lpf_y1 = delta_yaw;
            lpf_y2 = delta_yaw;
            lpf_inited = 1;
        }

        lpf_y1 = alpha * delta_yaw + (1.0f - alpha) * lpf_y1;
        lpf_y2 = alpha * lpf_y1    + (1.0f - alpha) * lpf_y2;

        delta_yaw = lpf_y2;
    }

    // 放大100倍 保证两位小数的精度
    int16_t delta_yaw_16 = (int16_t)(delta_yaw * 100.0f);

    // 拼凑发送数据
    uint8_t data[3] = {0};
    if(delta_yaw_16 < 0)
    {
      data[0] = 1;
      delta_yaw_16 = - delta_yaw_16;
    }
    else
    {
      data[0] = 0;
    }
    data[1] = delta_yaw_16 >> 8;
    data[2] = delta_yaw_16 & 0xff;

    printf("%d\r\n",delta_yaw_16);
    // 发送到核桃派
    uart3_send_to_htp(STM_SEND_CMD_IMU, data, 3);

    //调试
    // printf("IMU task stack free: %u\r\n",
    // uxTaskGetStackHighWaterMark(NULL));
		delay:
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }

  /* USER CODE END StartIMUTask */
}

/* USER CODE BEGIN Header_StartLEDFlashingTask */
/**
* @brief Function implementing the LEDFlashingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLEDFlashingTask */
void StartLEDFlashingTask(void *argument)
{
  /* USER CODE BEGIN StartLEDFlashingTask */
  /* Infinite loop */
  for(;;)
	{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    osDelay(500);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		osDelay(500);
  }
  /* USER CODE END StartLEDFlashingTask */
}

/* USER CODE BEGIN Header_StartHTPComuTask */
/**
* @brief Function implementing the HTPComuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHTPComuTask */
void StartHTPComuTask(void *argument)
{
  /* USER CODE BEGIN StartHTPComuTask */
  uint32_t tick = osKernelGetTickCount();

  /* Infinite loop */
  for(;;)
  {
		int16_t left_encoder_value_int16 = 0;
		int16_t right_encoder_value_int16 = 0;

    if(osMutexAcquire(speed_mutexHandle, 10) == osOK)
    {
      current_speed_fl_10 = encoder_read_speed_10(1);
      current_speed_fr_10 = encoder_read_speed_10(2);
      current_speed_bl_10 = encoder_read_speed_10(3);
      current_speed_br_10 = encoder_read_speed_10(4);

      float left_encoder_value = ((float)current_speed_fl_10 + (float)current_speed_bl_10)/2;
      float right_encoder_value = ((float)current_speed_fr_10 + (float)current_speed_br_10)/2;

			left_encoder_value_int16 = (int16_t)left_encoder_value;
			right_encoder_value_int16 = (int16_t)right_encoder_value;

      osMutexRelease(speed_mutexHandle);
    }

		uint8_t signal = 0;
		uint8_t data[6] = {0};

		if(left_encoder_value_int16 < 0){
			signal = 1;
			left_encoder_value_int16 = -left_encoder_value_int16;
		}
		data[0] = signal;
		data[1] = left_encoder_value_int16 >> 8;
		data[2] = left_encoder_value_int16 & 0xFF;

		signal = 0;
		if(right_encoder_value_int16 < 0){
			signal = 1;
			right_encoder_value_int16 = -right_encoder_value_int16;
		}
		data[3] = signal;
		data[4] = right_encoder_value_int16 >> 8;
		data[5] = right_encoder_value_int16 & 0xFF;
    // for(int i = 0;i<6;i++)
    // {
    //       printf("%#x ",data[i]);
    // }
    // printf("\r\n");
		uart3_send_to_htp(STM_SEND_CMD_ENCODER, data, sizeof(data));

//		motor_set_speed(1, 1680);
//		motor_set_speed(2, 1680);
//		motor_set_speed(3, 1680);
//		motor_set_speed(4, 1680);
//		printf("l: %.2f ",left_encoder_value);
//		printf("r: %.2f",right_encoder_value);
//		printf("\r\n");

							//调试
							// printf("HTP task stack free: %u\r\n",
              // uxTaskGetStackHighWaterMark(NULL));

		vTaskDelayUntil(&tick, pdMS_TO_TICKS(10));

  }
  /* USER CODE END StartHTPComuTask */
}

/* USER CODE BEGIN Header_StartOLEDTask */
/**
* @brief Function implementing the OLEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOLEDTask */
void StartOLEDTask(void *argument)
{
  /* USER CODE BEGIN StartOLEDTask */
  /* Infinite loop */
  OLED_Init();
  uint8_t page = 0;
  char buf[32];

  for(;;)
  {
    OLED_Clear();
    page = 2;
    switch(page){
      case 0:// 当前速度
        if(osMutexAcquire(speed_mutexHandle, 10) == osOK)
        {
          OLED_ShowString(0,0,"Speed:",16);
          sprintf(buf, "FL:%d FR:%d", current_speed_fl, current_speed_fr);
          OLED_ShowString(0, 2, (u8*)buf, 16);
          sprintf(buf, "BL:%d BR:%d", current_speed_bl, current_speed_br);
          OLED_ShowString(0, 4, (u8*)buf, 16);

          osMutexRelease(speed_mutexHandle);
        }
        break;

      case 1:  // 目标速度
        if(osMutexAcquire(speed_mutexHandle, 10) == osOK)
        {
          OLED_ShowString(0, 0, "Target:", 16);
          sprintf(buf, "TFL:%d TFR:%d", target_speed_fl, target_speed_fr);
          OLED_ShowString(0, 2, (u8*)buf, 16);
          sprintf(buf, "TBL:%d TBR:%d", target_speed_bl, target_speed_br);
          OLED_ShowString(0, 4, (u8*)buf, 16);

          osMutexRelease(speed_mutexHandle);
        }
        break;
      case 2:  // MPU6050
        if(osMutexAcquire(global_sensor_data_mutexHandle, 100) == osOK)
        {
          OLED_ShowString(0, 0, "MPU6050:", 16);
          sprintf(buf, "P:%.1f R:%.1f", g_SensorData.pitch, g_SensorData.roll);
          OLED_ShowString(0, 2, (u8*)buf, 16);
          sprintf(buf, "Y:%.1f", g_SensorData.yaw);
          OLED_ShowString(0, 40, (u8*)buf, 16);

          osMutexRelease(global_sensor_data_mutexHandle);
        }
        break;
//      case 3:  //
//        OLED_ShowString(0, 0, "bh1750_lux:", 16);
//        sprintf(buf, "LUX:%d", g_SensorData.bh1750_lux);
//        OLED_ShowString(0, 2, (u8*)buf, 16);
//        break;
    }
    OLED_Refresh();

//    page = (page + 1) % 3;

    //调试
    // printf("OLED task stack free: %u\r\n",
    // uxTaskGetStackHighWaterMark(NULL));
    osDelay(50);
  }
  /* USER CODE END StartOLEDTask */
}

/* USER CODE BEGIN Header_StartPS2Task */
/**
* @brief Function implementing the PS2Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPS2Task */
void StartPS2Task(void *argument)
{
  /* USER CODE BEGIN StartPS2Task */
  JOYSTICK_TypeDef js;
  AX_PS2_Init();
  /* Infinite loop */
  for(;;)
  {
    AX_PS2_ScanKey(&js);

    if(osMutexAcquire(speed_mutexHandle, 100) == osOK)
    {
      switch (js.btn1)
      {
        case 0x10:  //↑ UP：前进
          target_speed_fl =  83; target_speed_fr =  83;
          target_speed_bl =  83; target_speed_br =  83;

  //										motor_set_speed(1, 200);
  //										motor_set_speed(2, 200);
  //										motor_set_speed(3, 200);
  //										motor_set_speed(4, 200);

//  			target_speed_fl += 20;
  //			target_speed_fr += 20;
  //			target_speed_bl += 20;
  //			target_speed_br += 20;
          break;

        case 0x40:  //↓ DOWN：后退
          target_speed_fl =  -83; target_speed_fr =  -83;
          target_speed_bl =  -83; target_speed_br =  -83;

  //										motor_set_speed(1, -200);
  //										motor_set_speed(2, -200);
  //										motor_set_speed(3, -200);
  //										motor_set_speed(4, -200);

//  			target_speed_fl -= 20;
  //			target_speed_fr -= 20;
  //			target_speed_bl -= 20;
  //			target_speed_br -= 20;
          break;

        case 0x80:  //← LEFT：左转
          target_speed_fl =  -91; target_speed_fr =  91;
          target_speed_bl =  -91; target_speed_br =  91;

  //								motor_set_speed(1, -300);
  //								motor_set_speed(2, 300);
  //								motor_set_speed(3, -300);
  //								motor_set_speed(4, 300);
  //								target_speed_fl += 100;

//  			target_speed_fl += 100;
  //			target_speed_fr += 100;
  //			target_speed_bl += 100;
  //			target_speed_br += 100;
          break;

        case 0x20:  //→ RIGHT：右转
          target_speed_fl =  91; target_speed_fr =  -91;
          target_speed_bl =  91; target_speed_br =  -91;

  //										motor_set_speed(1, 300);
  //										motor_set_speed(2, -300);
  //										motor_set_speed(3, 300);
  //										motor_set_speed(4, -300);
  //										target_speed_fl += 100;

//  			target_speed_fl -= 100;
  //			target_speed_fr -= 100;
  //			target_speed_bl -= 100;
  //			target_speed_br -= 100;
          break;

        default:
          break;
      }
      switch (js.btn2)
      {
        case 0x10:  //△ 三角：停车
  //        target_speed_fl =  0; target_speed_fr =  0;
  //        target_speed_bl =  0; target_speed_br =  0;
  //										motor_set_speed(1, 0);
  //										motor_set_speed(2, 0);
  //										motor_set_speed(3, 0);
  //										motor_set_speed(4, 0);
          target_speed_fl =  0; target_speed_fr =  0;
          target_speed_bl =  0; target_speed_br =  0;
          break;

        default:
        break;
      }

      osMutexRelease(speed_mutexHandle);
    }
       // ==================== 其他按键 ====================
       // js.btn1 & 0x01  // SELECT
       // js.btn1 & 0x02  // L3
       // js.btn1 & 0x04  // R3
       // js.btn1 & 0x08  // START
       // js.btn2 & 0x01  // L2
       // js.btn2 & 0x02  // R2
       // js.btn2 & 0x04  // L1
       // js.btn2 & 0x08  // R1
       // js.btn2 & 0x20  // ○ 圆圈
       // js.btn2 & 0x40  // × 叉号
       // js.btn2 & 0x80  // □ 方块

       // js.LJoy_LR  // 左摇杆左右  0x00~0xFF 中位0x80
       // js.LJoy_UD  // 左摇杆上下  0x00~0xFF 中位0x80
       // js.RJoy_LR  // 右摇杆左右  0x00~0xFF 中位0x80
       // js.RJoy_UD  // 右摇杆上下  0x00~0xFF 中位0x80

    osDelay(75);
							//调试
							// printf("PS2 task stack free: %u\r\n",
              // uxTaskGetStackHighWaterMark(NULL));
  }
  /* USER CODE END StartPS2Task */
}

/* USER CODE BEGIN Header_StartActuatorTask */
/**
* @brief Function implementing the ActuatorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartActuatorTask */
void StartActuatorTask(void *argument)
{
  /* USER CODE BEGIN StartActuatorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartActuatorTask */
}

/* USER CODE BEGIN Header_StartExSensorTask */
/**
* @brief Function implementing the ExSensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartExSensorTask */
void StartExSensorTask(void *argument)
{
  /* USER CODE BEGIN StartExSensorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartExSensorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

