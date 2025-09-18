#include "zf_device_systick.h"

static volatile uint32 g_systick_time_ms = 0;

// SysTick中断服务函数
void SysTick_Handler(void)
{
    g_systick_time_ms++;
}

// 初始化SysTick定时器
void systick_init(void)
{
    // 配置SysTick定时器每1ms中断一次
    // SystemCoreClock 是在 airc18xx.h 中定义的，代表了系统主时钟频率
    SysTick_Config(SystemCoreClock / 1000);
}

// 获取系统运行时间，单位ms
uint32 systick_get_time_ms(void)
{
    return g_systick_time_ms;
}

// 毫秒级延时
void systick_delay_ms(uint32 ms)
{
    uint32 start_time = systick_get_time_ms();
    while(systick_get_time_ms() - start_time < ms);
}

