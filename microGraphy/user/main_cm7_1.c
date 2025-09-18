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
#include "ipc/ipc_protocol.h"
#include "motor_control.h"

// 定义时序周期 (ms)
#define ESTIMATOR_UPDATE_PERIOD_MS (2)  // PIT_CH0: 状态估计
#define MOTOR_CONTROL_PERIOD_MS (5)     // PIT_CH10: 电机控制

// 运动指令缓存 (M7_0 -> M7_1) - 全局变量，供中断访问
volatile float s_target_linear_speed = 0.0f;
volatile float s_target_angular_speed = 0.0f;
volatile bool s_motion_command_updated = false;

// IPC状态机
typedef enum {
    IPC_STATE_WAIT_ID,
    IPC_STATE_WAIT_LINEAR_SPEED,
    IPC_STATE_WAIT_ANGULAR_SPEED
} ipc_receive_state_t;

static ipc_receive_state_t s_ipc_state = IPC_STATE_WAIT_ID;
static ipc_data_converter_t s_ipc_converter;

// IPC消息回调函数 (M7_1接收来自M7_0的消息)
void m7_1_ipc_callback(uint32_t receive_data)
{
    switch(s_ipc_state)
    {
        case IPC_STATE_WAIT_ID:
            if(receive_data == IPC_ID_RESET_ESTIMATOR)
            {
                state_estimator_init(); // 复位状态估计器
            }
            else if(receive_data == IPC_ID_MOTION_COMMAND)
            {
                s_ipc_state = IPC_STATE_WAIT_LINEAR_SPEED;
            }
            break;
            
        case IPC_STATE_WAIT_LINEAR_SPEED:
            s_ipc_converter.u32 = receive_data;
            s_target_linear_speed = s_ipc_converter.f32;
            s_ipc_state = IPC_STATE_WAIT_ANGULAR_SPEED;
            break;
            
        case IPC_STATE_WAIT_ANGULAR_SPEED:
            s_ipc_converter.u32 = receive_data;
            s_target_angular_speed = s_ipc_converter.f32;
            s_motion_command_updated = true;
            s_ipc_state = IPC_STATE_WAIT_ID;
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
    motor_control_init();
    ipc_communicate_init(IPC_PORT_1, m7_1_ipc_callback);

    // 4. 设置双定时器：高精度状态估计 + 电机控制
    pit_ms_init(PIT_CH0, ESTIMATOR_UPDATE_PERIOD_MS);   // 2ms: 传感器+状态估计
    pit_ms_init(PIT_CH10, MOTOR_CONTROL_PERIOD_MS);     // 5ms: 电机PID控制

    // 5. M7_1的主循环保持空闲，所有实时任务在中断中完成
    while(true)
    {
        // 可以添加一些非实时的低优先级任务
        // 例如：自诊断、统计信息更新等
        // WFI(); // 进入低功耗模式等待中断
    }
}
