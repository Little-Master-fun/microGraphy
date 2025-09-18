/*********************************************************************************************************************
* 文件名称          system_control.c
* 功能说明          主控流程与状态机管理模块 实现文件 (双核版)
* 作者              AI Assistant
* 版本信息          v2.0
* 修改记录
* 日期              作者                版本              备注
* 2024-XX-XX        AI Assistant        v2.0              适配双核架构，通过IPC接收状态
********************************************************************************************************************/
#include "system_control.h"
#include "motor_control.h"
#include "navigation_flash_improved.h"
#include "zf_device_systick.h"
#include "ipc_protocol.h"

//-------------------------------------------------------------------------------------------------------------------
// 内部变量
//-------------------------------------------------------------------------------------------------------------------

// M7_0的主状态机
static system_state_enum s_current_state = SYS_STATE_INIT;

// 用于缓存从M7_1接收到的最新车辆状态
static VehicleState s_vehicle_state_cache;

// IPC接收状态机
typedef enum {
    IPC_RX_STATE_WAIT_ID,
    IPC_RX_STATE_WAIT_X,
    IPC_RX_STATE_WAIT_Y,
    IPC_RX_STATE_WAIT_HEADING,
    IPC_RX_STATE_WAIT_SPEED,
} ipc_rx_state_enum;
static ipc_rx_state_enum s_ipc_rx_state = IPC_RX_STATE_WAIT_ID;


//-------------------------------------------------------------------------------------------------------------------
// 内部函数声明
//-------------------------------------------------------------------------------------------------------------------
static void m7_0_ipc_callback(uint32_t receive_data);


//-------------------------------------------------------------------------------------------------------------------
// 主接口函数
//-------------------------------------------------------------------------------------------------------------------

void system_control_init(void)
{
    // 初始化硬件驱动和功能模块
    systick_init();
    ui_init();
    motor_control_init();
    Navigation_Init();
    ipc_communicate_init(IPC_PORT_1, m7_0_ipc_callback);
    
    // 初始化状态
    memset(&s_vehicle_state_cache, 0, sizeof(VehicleState));
    s_current_state = SYS_STATE_IDLE;
}

void system_control_update(void)
{
    // UI处理 和 模式切换
    ui_update();
    system_mode_enum ui_mode = ui_get_current_mode();
    if(ui_mode == SYS_MODE_CONFIRM_START && s_current_state == SYS_STATE_IDLE)
    {
        Navigation_GenerateTestPath(1000.0f, 100);
        motor_control_start();
        ui_set_mode(SYS_MODE_PATH_FOLLOWING);
        s_current_state = SYS_STATE_PATH_FOLLOWING;
    }
    
    // 主状态机逻辑
    switch(s_current_state)
    {
        case SYS_STATE_IDLE:
            motor_control_stop();
            break;
            
        case SYS_STATE_PATH_FOLLOWING:
        {
            // M7_0不再自己计算状态，而是直接使用从M7_1接收到的最新状态 s_vehicle_state_cache
            MotionCommand motion_cmd = Navigation_PathTrack(&s_vehicle_state_cache);
            motor_set_robot_motion(motion_cmd.desired_linear_speed, motion_cmd.desired_angular_speed);
        }
            break;
            
        case SYS_STATE_STOPPED:
            motor_control_stop();
            s_current_state = SYS_STATE_IDLE;
            ui_set_mode(SYS_MODE_IDLE);
            break;
            
        default:
            motor_control_stop();
            break;
    }
}

system_state_enum system_control_get_state(void)
{
    return s_current_state;
}


//-------------------------------------------------------------------------------------------------------------------
// IPC回调与数据解析
//-------------------------------------------------------------------------------------------------------------------

// M7_0的IPC回调函数，用于接收M7_1发来的数据
static void m7_0_ipc_callback(uint32_t receive_data)
{
    ipc_data_converter_t converter;

    switch(s_ipc_rx_state)
    {
        // 1. 等待消息ID
        case IPC_RX_STATE_WAIT_ID:
            if(receive_data == IPC_ID_VEHICLE_STATE)
            {
                s_ipc_rx_state = IPC_RX_STATE_WAIT_X;
            }
            break;
            
        // 2. 接收X坐标
        case IPC_RX_STATE_WAIT_X:
            converter.u32 = receive_data;
            s_vehicle_state_cache.x = converter.f32;
            s_ipc_rx_state = IPC_RX_STATE_WAIT_Y;
            break;
            
        // 3. 接收Y坐标
        case IPC_RX_STATE_WAIT_Y:
            converter.u32 = receive_data;
            s_vehicle_state_cache.y = converter.f32;
            s_ipc_rx_state = IPC_RX_STATE_WAIT_HEADING;
            break;
            
        // 4. 接收航向角
        case IPC_RX_STATE_WAIT_HEADING:
            converter.u32 = receive_data;
            s_vehicle_state_cache.heading = converter.f32;
            s_ipc_rx_state = IPC_RX_STATE_WAIT_SPEED;
            break;
            
        // 5. 接收速度 (一包数据接收完毕)
        case IPC_RX_STATE_WAIT_SPEED:
            converter.u32 = receive_data;
            s_vehicle_state_cache.linear_speed = converter.f32;
            s_ipc_rx_state = IPC_RX_STATE_WAIT_ID; // 复位状态机，等待下一包数据
            break;
    }
}
