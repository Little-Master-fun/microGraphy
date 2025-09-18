/*********************************************************************************************************************
* 文件名称          test_square_path.c
* 功能说明          【优化版】正方形路径测试 实现文件
* 作者              AI Assistant
* 版本信息          v2.0
* 修改记录
* 日期              作者                版本              备注
* 2024-XX-XX        AI Assistant        v2.0              重构为调用核心导航模块的测试程序
*
* 文件作用说明：
* 本文件通过调用'navigation_flash_improved'模块的接口，执行一个完整的导航测试。
* 它负责初始化硬件、生成测试路径、运行主控制循环，并显示导航状态。
* 这种结构实现了测试逻辑和核心导航算法的完全分离。
********************************************************************************************************************/

#include "test_square_path.h"
#include "navigation_flash_improved.h" // 引入核心导航模块
#include "motor_control.h"
#include "driver_sch16tk10.h"
#include "driver_encoder.h"
#include "zf_device_ips114.h"
#include "zf_common_headfile.h"
#include <stdio.h>

#define RAD_TO_DEG(rad) ((rad) * (180.0f / M_PI))

//================================================= 内部函数声明 =================================================
static void display_navigation_status(float dt);
static uint32 get_system_time_ms(void);

//================================================= 主测试函数 =================================================
void test_square_path_optimized(void)
{
    // -------- 1. 初始化硬件和核心模块 --------
    ips114_init();
    ips114_clear();
    ips114_show_string(0, 0, "Optimized Nav Test");
    
    motor_control_init();
    encoder_init();
    SCH1_init((SCH1_filter){0}, (SCH1_sensitivity){0}, (SCH1_decimation){0}, false);
    
    // 初始化导航系统
    Navigation_Init();
    ips114_show_string(0, 16, "Navigation System OK");
    
    // 让导航模块生成一个1x1m的正方形测试路径
    Navigation_GenerateTestPath(1000.0f, 100);
    ips114_show_string(0, 32, "Test Path Generated OK");
    
    motor_control_start();
    ips114_show_string(0, 48, "Test Running...");
    
    uint32 last_time = get_system_time_ms();
    
    // -------- 2. 主控制循环 (目标100Hz) --------
    while(1)
    {
        uint32 current_time = get_system_time_ms();
        float dt = (current_time - last_time) / 1000.0f;
        if (dt < 0.01f) {
            continue; // 保证循环频率不高于100Hz
        }
        last_time = current_time;

        // I. 更新状态估计 (调用导航模块)
        Navigation_UpdateState(dt);
        
        // II. 计算导航决策 (调用导航模块)
        MotionCommand motion_cmd = Navigation_PathTrack();

        // III. 执行运动指令 (调用电机控制模块)
        motor_set_robot_motion(motion_cmd.desired_linear_speed, motion_cmd.desired_angular_speed);
        
        // IV. 数据显示
        static uint32 last_display_time = 0;
        if (current_time - last_display_time > 200) // 200ms更新一次
        {
            display_navigation_status(dt);
            last_display_time = current_time;
        }
    }
}

//================================================= 辅助函数实现 =================================================

static void display_navigation_status(float dt)
{
    char info[50];
    // 从导航模块的全局变量中获取状态信息用于显示
    VehicleState* s = &g_nav_system.state;
    
    ips114_clear();
    
    sprintf(info, "X:%.1f Y:%.1f (mm)", s->x, s->y);
    ips114_show_string(0, 0, info);
    
    sprintf(info, "Head:%.1fdeg Spd:%.2f", RAD_TO_DEG(s->heading), s->linear_speed);
    ips114_show_string(0, 16, info);

    sprintf(info, "FPS: %.1f", 1.0f / dt);
    ips114_show_string(0, 32, info);
}

static uint32 get_system_time_ms(void)
{
    // 实际使用时应替换为真实的系统时间函数
    return systick_get_time_ms();
}
