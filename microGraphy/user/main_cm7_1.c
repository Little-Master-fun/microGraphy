/*********************************************************************************************************************
 * @file        main_cm7_1.c
 * @brief       Core M7_1 (Co-processor) main program body
 * @version     v1.0
 * @author      AI Assistant
 * @date        2024-XX-XX
 *
 * @note
 *              该核心作为协处理器，专门负责高频次的传感器数据采集和状态估计。
 *              1. 初始化IMU、编码器和IPC通信。
 *              2. 设置一个2ms的定时器中断 (PIT_CH0)。
 *              3. 在中断中执行航位推算，并将结果通过IPC发送给主控核心M7_0。
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "zf_common_interrupt.h"

#include "driver_sch16tk10.h"
#include "driver_encoder.h"
#include "state_estimator.h"
#include "ipc_protocol.h"

// 定义状态估计的周期 (ms)
#define ESTIMATOR_UPDATE_PERIOD_MS (2)

// IPC消息回调函数 (M7_1接收来自M7_0的消息)
void m7_1_ipc_callback(uint32_t receive_data)
{
    // 解析M7_0发来的指令
    switch(receive_data)
    {
        case IPC_ID_RESET_ESTIMATOR:
            state_estimator_init(); // 复位状态估计器
            break;
            
        // 可以扩展其他指令
        default:
            break;
    }
}


int main(void)
{
    // 1. 初始化时钟和中断
    clock_init(SYSTEM_CLOCK_250M);
    interrupt_global_enable(0);

    // 2. 初始化硬件驱动
    // 注意：确保这里的引脚初始化不会与M7_0冲突
    SCH1_filter filter = {0};
    SCH1_sensitivity sensitivity = {0};
    SCH1_decimation decimation = {0};
    SCH1_init(filter, sensitivity, decimation, false);
    encoder_init();

    // 3. 初始化功能模块
    state_estimator_init();
    ipc_communicate_init(IPC_PORT_1, m7_1_ipc_callback);

    // 4. 设置并启动PIT0定时器，作为状态估计的心跳
    pit_ms_init(PIT_CH0, ESTIMATOR_UPDATE_PERIOD_MS);

    // 5. M7_1的主循环可以保持空闲，或者执行一些低优先级的后台任务
    while(true)
    {
        // WFI(); // 进入低功耗模式等待中断
    }
}
