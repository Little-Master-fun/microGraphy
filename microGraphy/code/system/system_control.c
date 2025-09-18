/*********************************************************************************************************************
* 文件名称          system_control.c
* 功能说明          主控流程与状态机管理模块 实现文件
* 作者              AI Assistant
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本              备注
* 2024-XX-XX        AI Assistant        v1.0              创建文件，实现状态机逻辑
********************************************************************************************************************/
#include "system_control.h"
#include "motor_control.h"
#include "navigation_flash_improved.h"
#include "zf_device_systick.h"


// 内部状态变量
static system_state_enum s_current_state = SYS_STATE_INIT;

// 定时器相关
static uint32_t s_last_update_time_ms = 0;

void system_control_init(void)
{
    // 初始化各个硬件驱动和功能模块
    systick_init();
    ui_init();
    motor_control_init();
    Navigation_Init();
    
    // 设置初始状态
    s_current_state = SYS_STATE_IDLE;
    s_last_update_time_ms = systick_get_time_ms();
}

void system_control_update(void)
{
    // ------------------ 1. UI处理 (所有状态下都执行) ------------------
    ui_update();
    
    // ------------------ 2. 根据UI指令，切换系统主状态 ------------------
    system_mode_enum ui_mode = ui_get_current_mode();
    if(ui_mode == SYS_MODE_CONFIRM_START && s_current_state == SYS_STATE_IDLE)
    {
        // 收到UI的发车确认指令，切换到循迹状态
        Navigation_GenerateTestPath(1000.0f, 100); // 加载测试路径
        motor_control_start();
        ui_set_mode(SYS_MODE_PATH_FOLLOWING); // 更新UI界面为运行模式
        s_current_state = SYS_STATE_PATH_FOLLOWING;
    }
    
    // ------------------ 3. 主状态机逻辑 ------------------
    switch(s_current_state)
    {
        case SYS_STATE_IDLE:
            // 在待机状态下，确保电机是停止的
            motor_control_stop();
            // 此状态下主要由 ui_update() 负责交互
            break;
            
        case SYS_STATE_PATH_FOLLOWING:
        {
            // 计算时间间隔 dt
            uint32_t current_time_ms = systick_get_time_ms();
            float dt = (current_time_ms - s_last_update_time_ms) / 1000.0f;
            
            // 控制周期 safeguarding
            if(dt < 0.01f) // 限制控制频率最高为100Hz
            {
                return;
            }
            s_last_update_time_ms = current_time_ms;
            
            // 执行核心导航控制流程 (感知 -> 决策 -> 执行)
            Navigation_UpdateState(dt);
            MotionCommand motion_cmd = Navigation_PathTrack();
            motor_set_robot_motion(motion_cmd.desired_linear_speed, motion_cmd.desired_angular_speed);
            
            // 此处可以添加任务完成的判断逻辑
            // if(is_task_finished()) {
            //     s_current_state = SYS_STATE_STOPPED;
            //     ui_set_mode(SYS_MODE_IDLE);
            // }
        }
            break;
            
        case SYS_STATE_PATH_RECORDING:
            // 路径记录逻辑 (待实现)
            break;
            
        case SYS_STATE_STOPPED:
            motor_control_stop();
            // 可以切换回IDLE状态
            s_current_state = SYS_STATE_IDLE;
            ui_set_mode(SYS_MODE_IDLE);
            break;
            
        case SYS_STATE_INIT:
        case SYS_STATE_ERROR:
        default:
            // 错误处理，停止所有动作
            motor_control_stop();
            break;
    }
}

system_state_enum system_control_get_state(void)
{
    return s_current_state;
}
