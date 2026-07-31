#include "encoder.h"

// 启动所有的编码器定时器
void encoder_init_start(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
}

// 参数motor
// 1是左前 2是右前 3是左后 4是右后
int16_t encoder_read_speed(uint8_t motor)
{
    static int16_t encoder_last[4] = {0, 0, 0, 0};
    int16_t encoder_speed = 0;
    uint32_t read_cnt = 0;

    switch(motor)
    {
        case 1:
            read_cnt = htim3.Instance->CNT;         // 读取定时器的计数值

            encoder_speed = (int16_t)read_cnt - encoder_last[0];        //计算差值
            encoder_last[0] = (int16_t)read_cnt;
        break;

        case 2:
            read_cnt = htim4.Instance->CNT;

            encoder_speed = (int16_t)read_cnt - encoder_last[1];
            encoder_last[1] = (int16_t)read_cnt;
        break;

        case 3:
            read_cnt = htim5.Instance->CNT;

            encoder_speed = (int16_t)read_cnt - encoder_last[2];
            encoder_last[2] = (int16_t)read_cnt;
        break;

        case 4:
            read_cnt = htim8.Instance->CNT;

            encoder_speed = (int16_t)read_cnt - encoder_last[3];
            encoder_last[3] = (int16_t)read_cnt;
        break;
    }

    return encoder_speed;
}

int16_t encoder_read_speed_10(uint8_t motor)//调试用
{
    static int16_t encoder_last[4] = {0, 0, 0, 0};
    int16_t encoder_speed = 0;
    uint32_t read_cnt = 0;

    switch(motor)
    {
        case 1:
            read_cnt = htim3.Instance->CNT;         // 读取定时器的计数值

            encoder_speed = (int16_t)read_cnt - encoder_last[0];        //计算差值
            encoder_last[0] = (int16_t)read_cnt;
        break;

        case 2:
            read_cnt = htim4.Instance->CNT;

            encoder_speed = (int16_t)read_cnt - encoder_last[1];
            encoder_last[1] = (int16_t)read_cnt;
        break;

        case 3:
            read_cnt = htim5.Instance->CNT;

            encoder_speed = (int16_t)read_cnt - encoder_last[2];
            encoder_last[2] = (int16_t)read_cnt;
        break;

        case 4:
            read_cnt = htim8.Instance->CNT;

            encoder_speed = (int16_t)read_cnt - encoder_last[3];
            encoder_last[3] = (int16_t)read_cnt;
        break;
    }

    return encoder_speed;
}

