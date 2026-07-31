#include "delay.h"

// 168M精准5us延时
#pragma push
#pragma O0
void Delay5us(void)
{
    // 强制使用 volatile 阻止编译器将空循环优化掉
    volatile uint32_t i;
    for(i=0;i<105;i++);
}

void Delay_us(uint32_t us)
{
    volatile uint32_t i;
    while(us--)
    {
        for(i = 0; i < 21; i++);
    }
}

void Delay_ms(uint32_t ms)
{
    while(ms--)
    {
        Delay_us(1000);
    }
}

#pragma pop
