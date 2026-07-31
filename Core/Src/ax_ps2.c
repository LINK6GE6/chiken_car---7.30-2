/**
 * @file    ax_ps2.c
 * @brief   PS2 无线手柄驱动 — HAL 库版本（阻塞式）
 * @note
 *         - 时序: CLK 低/高各约 16us，时钟约 31kHz
 *         - 单次 PS2_ScanKey() 阻塞约 2.4ms
 *         - 延时用软件空循环，编译优化需关闭该部分
 */

#include "ax_ps2.h"



/* ---------------------------------------------------------------------------*/
/* 引脚快捷操作宏                                                              */
/* ---------------------------------------------------------------------------*/
#define CS_H()    HAL_GPIO_WritePin(PS2_CS_PORT,  PS2_CS_PIN,  GPIO_PIN_SET)
#define CS_L()    HAL_GPIO_WritePin(PS2_CS_PORT,  PS2_CS_PIN,  GPIO_PIN_RESET)

#define CMD_H()   HAL_GPIO_WritePin(PS2_CMD_PORT, PS2_CMD_PIN, GPIO_PIN_SET)
#define CMD_L()   HAL_GPIO_WritePin(PS2_CMD_PORT, PS2_CMD_PIN, GPIO_PIN_RESET)

#define CLK_H()   HAL_GPIO_WritePin(PS2_CLK_PORT, PS2_CLK_PIN, GPIO_PIN_SET)
#define CLK_L()   HAL_GPIO_WritePin(PS2_CLK_PORT, PS2_CLK_PIN, GPIO_PIN_RESET)

#define DIN()     HAL_GPIO_ReadPin(PS2_DIN_PORT, PS2_DIN_PIN)

/* PS2 查询命令序列 -----------------------------------------------------------*/
static const uint8_t PS2_cmnd[9] = {
    0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static uint8_t PS2_data[9] = {0};

/* ---------------------------------------------------------------------------*/
/**
 * @brief  PS2 初始化
 * @note   CS/CMD/CLK → 推挽输出；DIN → 输入上拉
 *         调用前需已完成 SystemClock_Config()
 */
/* ---------------------------------------------------------------------------*/
void AX_PS2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能 GPIO 时钟 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* CS: PC12  推挽输出 */
    GPIO_InitStruct.Pin   = PS2_CS_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PS2_CS_PORT, &GPIO_InitStruct);

    /* CMD: PD0  推挽输出 */
    GPIO_InitStruct.Pin   = PS2_CMD_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PS2_CMD_PORT, &GPIO_InitStruct);

    /* CLK: PD1  推挽输出 */
    GPIO_InitStruct.Pin   = PS2_CLK_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PS2_CLK_PORT, &GPIO_InitStruct);

    /* DIN: PD2  输入上拉（覆盖 CubeMX 默认输出配置） */
    GPIO_InitStruct.Pin   = PS2_DIN_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PS2_DIN_PORT, &GPIO_InitStruct);

    /* 初始电平：CS/CLK/CMD 全拉高 */
    CS_H();
    CLK_H();
    CMD_H();
}

/* ---------------------------------------------------------------------------*/
/**
 * @brief  PS2 单字节读写
 * @param  data: 待发送字节 (LSB first)
 * @return 接收到的手柄数据
 *
 * 时序 (每 bit):
 *   CLK↓ → 放 CMD → 等 16us → CLK↑ → 读 DIN → 等 16us
 */
/* ---------------------------------------------------------------------------*/
static uint8_t PS2_ReadWriteData(uint8_t data)
{
    uint8_t ref, res = 0;

    for (ref = 0x01; ref > 0x00; ref <<= 1)
    {
        CLK_L();

        if (ref & data)
            CMD_H();
        else
            CMD_L();

        Delay_us(16);

        CLK_H();

        if (DIN())
            res |= ref;

        Delay_us(16);
    }

    CMD_H();
    return res;
}

/* ---------------------------------------------------------------------------*/
/**
 * @brief  扫描手柄按键与摇杆
 * @param  JoystickStruct: 数据输出结构体指针
 *
 * 阻塞时间约 2.4ms。
 * 典型调用周期 30ms，CPU 占用约 8%。
 *
 * FreeRTOS 任务中使用示例:
 * @code
 *   JOYSTICK_TypeDef js;
 *   AX_PS2_Init();
 *   for (;;) {
 *       AX_PS2_ScanKey(&js);
 *       if (js.btn1 & 0x10) { ... }  // UP 键
 *       vTaskDelay(pdMS_TO_TICKS(30));
 *   }
 * @endcode
 */
/* ---------------------------------------------------------------------------*/
void AX_PS2_ScanKey(JOYSTICK_TypeDef *JoystickStruct)
{
    uint8_t i;

    /* 拉低片选，开始通信 */
    CS_L();

    /* 收发 9 字节 */
    for (i = 0; i < 9; i++)
    {
        PS2_data[i] = PS2_ReadWriteData(PS2_cmnd[i]);
        Delay_us(16);
    }

    /* 拉高片选，结束通信 */
    CS_H();

    /* 解析数据 */
    if (JoystickStruct != NULL)
    {
        JoystickStruct->mode    = PS2_data[1];
        JoystickStruct->btn1    = ~PS2_data[3];
        JoystickStruct->btn2    = ~PS2_data[4];
        JoystickStruct->RJoy_LR = PS2_data[5];
        JoystickStruct->RJoy_UD = PS2_data[6];
        JoystickStruct->LJoy_LR = PS2_data[7];
        JoystickStruct->LJoy_UD = PS2_data[8];
    }
}
