/*********************************************************************************************************************
* 文件名称          mpc_ui_extension.c
* 功能说明          MPC控制器UI扩展功能 实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本              备注
* 2025-09-18        LittleMaster        v1.0              创建文件
*
* 文件作用说明：
* 本文件实现了MPC控制器的UI界面功能，包括性能监控、自动调参界面等。
********************************************************************************************************************/

#include "mpc_ui_extension.h"
#include "mpc_controller.h"
#include "navigation_flash_improved.h"
#include "zf_device_ips114.h"
#include <stdio.h>

void mpc_ui_show_performance(void)
{
    const mpc_performance_t* perf = mpc_get_performance_stats();
    nav_controller_type_t current_controller = Navigation_GetControllerType();
    
    // 第一行：当前控制器类型
    if (current_controller == NAV_CONTROLLER_MPC) {
        ips114_show_string(0, 0, "Controller: MPC");
    } else {
        ips114_show_string(0, 0, "Controller: Stanley");
    }
    
    // 如果是MPC模式，显示性能信息
    if (current_controller == NAV_CONTROLLER_MPC) {
        char buffer[64];
        
        // 第二行：计算时间
        snprintf(buffer, sizeof(buffer), "Comp Time: %lu us", perf->computation_time_us);
        ips114_show_string(0, 20, buffer);
        
        // 第三行：CPU使用率
        snprintf(buffer, sizeof(buffer), "CPU Usage: %.1f%%", perf->cpu_usage_percent);
        ips114_show_string(0, 40, buffer);
        
        // 第四行：收敛失败次数
        snprintf(buffer, sizeof(buffer), "Conv Failures: %lu", perf->convergence_failures);
        ips114_show_string(0, 60, buffer);
        
        // 第五行：最大计算时间
        snprintf(buffer, sizeof(buffer), "Max Time: %lu us", perf->max_computation_time_us);
        ips114_show_string(0, 80, buffer);
    }
}

void mpc_ui_show_controller_selection(void)
{
    nav_controller_type_t current_controller = Navigation_GetControllerType();
    
    ips114_show_string(0, 0, "=== Controller Select ===");
    
    // 显示当前选择
    if (current_controller == NAV_CONTROLLER_STANLEY) {
        ips114_show_string(0, 20, "-> Stanley (Classic)");
        ips114_show_string(0, 40, "   MPC (Advanced)");
    } else {
        ips114_show_string(0, 20, "   Stanley (Classic)");
        ips114_show_string(0, 40, "-> MPC (Advanced)");
    }
    
    ips114_show_string(0, 80, "KEY1: Switch  KEY2: OK");
    ips114_show_string(0, 100, "KEY3: Back");
}

bool mpc_ui_handle_controller_key(uint8_t key_pressed)
{
    nav_controller_type_t current_controller = Navigation_GetControllerType();
    
    switch (key_pressed) {
        case 1: // KEY1: 切换控制器类型
            if (current_controller == NAV_CONTROLLER_STANLEY) {
                Navigation_SetControllerType(NAV_CONTROLLER_MPC);
                ips114_show_string(0, 120, "Switched to MPC!");
            } else {
                Navigation_SetControllerType(NAV_CONTROLLER_STANLEY);
                ips114_show_string(0, 120, "Switched to Stanley!");
            }
            return true;
            
        case 2: // KEY2: 确认选择（当前暂时无额外操作）
            return true;
            
        case 3: // KEY3: 返回上级菜单
            return false; // 返回false表示UI应该切换回上级模式
            
        default:
            return false;
    }
}

//================================================= 自动调参UI功能实现 =================================================

void mpc_ui_show_auto_tuning(void)
{
    ips114_clear();
    ips114_show_string(0, 0, "=== MPC Auto-Tuning ===");
    
    const mpc_performance_t* perf = mpc_get_performance_stats();
    mpc_tuning_state_t tuning_state = mpc_get_tuning_state();
    char buffer[64];
    
    // 第二行：自动调参状态
    const char* state_str = "Unknown";
    switch (tuning_state) {
        case TUNING_STATE_IDLE: state_str = "Idle"; break;
        case TUNING_STATE_EVALUATING: state_str = "Evaluating"; break;
        case TUNING_STATE_OPTIMIZING: state_str = "Optimizing"; break;
        case TUNING_STATE_CONVERGED: state_str = "Converged"; break;
        case TUNING_STATE_FAILED: state_str = "Failed"; break;
    }
    snprintf(buffer, sizeof(buffer), "Status: %s", state_str);
    ips114_show_string(0, 20, buffer);
    
    // 第三行：性能评分
    snprintf(buffer, sizeof(buffer), "Score: %.1f/100", perf->overall_performance_score);
    ips114_show_string(0, 40, buffer);
    
    // 第四行：位置误差RMS
    snprintf(buffer, sizeof(buffer), "Pos RMS: %.2f mm", perf->position_error_rms);
    ips114_show_string(0, 60, buffer);
    
    // 第五行：航向误差RMS
    snprintf(buffer, sizeof(buffer), "Head RMS: %.3f rad", perf->heading_error_rms);
    ips114_show_string(0, 80, buffer);
    
    // 第六行：计算时间
    snprintf(buffer, sizeof(buffer), "Comp: %lu us", perf->computation_time_us);
    ips114_show_string(0, 100, buffer);
    
    // 操作提示
    ips114_show_string(0, 120, "KEY1:Enable KEY2:Config");
    ips114_show_string(0, 140, "KEY3:Weights KEY4:Trend");
}

void mpc_ui_show_tuning_config(void)
{
    ips114_clear();
    ips114_show_string(0, 0, "=== Tuning Config ===");
    
    char buffer[64];
    
    // 显示当前配置（这里需要添加获取配置的接口）
    ips114_show_string(0, 20, "Step Size: 0.1");
    ips114_show_string(0, 40, "Threshold: 2.0%");
    ips114_show_string(0, 60, "Window: 50 cycles");
    ips114_show_string(0, 80, "Interval: 100 steps");
    ips114_show_string(0, 100, "Max Iter: 10");
    
    ips114_show_string(0, 120, "KEY1:Edit KEY2:Save");
    ips114_show_string(0, 140, "KEY3:Reset KEY4:Back");
}

void mpc_ui_show_weights(void)
{
    ips114_clear();
    ips114_show_string(0, 0, "=== MPC Weights ===");
    
    // 获取当前权重
    mpc_weights_t best_weights;
    char buffer[64];
    
    if (mpc_get_best_weights(&best_weights)) {
        // 显示最优权重
        snprintf(buffer, sizeof(buffer), "Q_pos: %.1f", best_weights.Q_x);
        ips114_show_string(0, 20, buffer);
        
        snprintf(buffer, sizeof(buffer), "Q_head: %.1f", best_weights.Q_heading);
        ips114_show_string(0, 40, buffer);
        
        snprintf(buffer, sizeof(buffer), "Q_vel: %.1f", best_weights.Q_velocity);
        ips114_show_string(0, 60, buffer);
        
        snprintf(buffer, sizeof(buffer), "R_ctrl: %.1f", best_weights.R_linear_vel);
        ips114_show_string(0, 80, buffer);
        
        snprintf(buffer, sizeof(buffer), "S_smooth: %.1f", best_weights.S_linear_vel);
        ips114_show_string(0, 100, buffer);
    } else {
        ips114_show_string(0, 20, "No optimized weights");
        ips114_show_string(0, 40, "available yet.");
    }
    
    ips114_show_string(0, 120, "KEY1:Default KEY2:Apply");
    ips114_show_string(0, 140, "KEY3:Export KEY4:Back");
}

void mpc_ui_show_performance_trend(void)
{
    ips114_clear();
    ips114_show_string(0, 0, "=== Performance Trend ===");
    
    const mpc_performance_t* perf = mpc_get_performance_stats();
    char buffer[64];
    
    // 显示性能趋势
    snprintf(buffer, sizeof(buffer), "Current: %.1f", perf->overall_performance_score);
    ips114_show_string(0, 20, buffer);
    
    snprintf(buffer, sizeof(buffer), "Trend: %.2f", perf->performance_trend);
    ips114_show_string(0, 40, buffer);
    
    snprintf(buffer, sizeof(buffer), "Samples: %lu", perf->samples_count);
    ips114_show_string(0, 60, buffer);
    
    // 简单的ASCII图表 (模拟)
    ips114_show_string(0, 80, "History: [||||||||__|");
    
    // 显示最大误差
    snprintf(buffer, sizeof(buffer), "Max Pos Err: %.1f mm", perf->max_position_error);
    ips114_show_string(0, 100, buffer);
    
    snprintf(buffer, sizeof(buffer), "Max Head Err: %.3f rad", perf->max_heading_error);
    ips114_show_string(0, 120, buffer);
    
    ips114_show_string(0, 140, "Press any key to return");
}

bool mpc_ui_handle_tuning_key(uint8_t key_pressed)
{
    static uint8_t current_page = 0; // 0=主页, 1=配置, 2=权重, 3=趋势
    
    switch (current_page) {
        case 0: // 主页
            switch (key_pressed) {
                case 1: // KEY1: 启用/禁用自动调参
                    mpc_ui_toggle_auto_tuning(true);
                    ips114_show_string(0, 160, "Auto-tuning started!");
                    return true;
                    
                case 2: // KEY2: 配置页面
                    current_page = 1;
                    mpc_ui_show_tuning_config();
                    return true;
                    
                case 3: // KEY3: 权重页面
                    current_page = 2;
                    mpc_ui_show_weights();
                    return true;
                    
                case 4: // KEY4: 趋势页面
                    current_page = 3;
                    mpc_ui_show_performance_trend();
                    return true;
            }
            break;
            
        case 1: // 配置页面
            switch (key_pressed) {
                case 4: // KEY4: 返回主页
                    current_page = 0;
                    mpc_ui_show_auto_tuning();
                    return true;
            }
            break;
            
        case 2: // 权重页面
            switch (key_pressed) {
                case 1: // KEY1: 恢复默认权重
                    {
                        mpc_weights_t default_weights;
                        mpc_get_default_weights(&default_weights);
                        mpc_set_best_weights(&default_weights);
                        ips114_show_string(0, 160, "Default weights applied");
                    }
                    return true;
                    
                case 4: // KEY4: 返回主页
                    current_page = 0;
                    mpc_ui_show_auto_tuning();
                    return true;
            }
            break;
            
        case 3: // 趋势页面
            // 任意键返回主页
            current_page = 0;
            mpc_ui_show_auto_tuning();
            return true;
    }
    
    return false;
}

bool mpc_ui_toggle_auto_tuning(bool enable)
{
    if (enable) {
        // 配置默认调参参数
        mpc_auto_tuning_config_t config;
        mpc_get_default_tuning_config(&config);
        config.auto_tuning_enabled = true;
        
        mpc_auto_tuning_configure(&config);
        return mpc_auto_tuning_enable(true);
    } else {
        return mpc_auto_tuning_enable(false);
    }
}
