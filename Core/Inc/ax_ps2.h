/**
 * @file    ax_ps2.h
 * @brief   PS2 无线手柄驱动 — HAL 库版本（阻塞式）
 * @note
 *         引脚映射:
 *           PC12 = CS   (片选)
 *           PD0  = CMD  (命令)
 *           PD1  = CLK  (时钟)
 *           PD2  = DIN  (数据输入，PS2_Init 中重配为输入上拉)
 *
 *         数据帧 (9 字节):
 *           Byte1: idle   Byte2: 0x73(模式)   Byte3: 0x5A
 *           Byte4: btn1   Byte5: btn2
 *           Byte6~7: 右摇杆   Byte8~9: 左摇杆
 */

#ifndef __AX_PS2_H
#define __AX_PS2_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "delay.h"

/* 引脚定义 ------------------------------------------------------------------*/
#define PS2_CS_PORT              GPIOC
#define PS2_CS_PIN               GPIO_PIN_12

#define PS2_CMD_PORT             GPIOD
#define PS2_CMD_PIN              GPIO_PIN_0

#define PS2_CLK_PORT             GPIOD
#define PS2_CLK_PIN              GPIO_PIN_1

#define PS2_DIN_PORT             GPIOD
#define PS2_DIN_PIN              GPIO_PIN_2

/* 手柄数据结构体 ------------------------------------------------------------*/
typedef struct
{
    uint8_t mode;               /* 手柄工作模式           */
    uint8_t btn1;               /* 按键组1 (按下=1)      */
    uint8_t btn2;               /* 按键组2 (按下=1)      */
    uint8_t RJoy_LR;            /* 右摇杆左右  0x80中位  */
    uint8_t RJoy_UD;            /* 右摇杆上下  0x80中位  */
    uint8_t LJoy_LR;            /* 左摇杆左右  0x80中位  */
    uint8_t LJoy_UD;            /* 左摇杆上下  0x80中位  */
} JOYSTICK_TypeDef;

/* 接口函数 ------------------------------------------------------------------*/
void AX_PS2_Init(void);
void AX_PS2_ScanKey(JOYSTICK_TypeDef *JoystickStruct);

#ifdef __cplusplus
}
#endif

#endif /* __AX_PS2_H */
