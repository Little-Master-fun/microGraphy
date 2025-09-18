/*********************************************************************************************************************
* 文件名称          kalman_ui_extension.c
* 功能说明          卡尔曼滤波器UI扩展功能 实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本              备注
* 2025-09-18        LittleMaster        v1.0              创建文件
*
* 文件作用说明：
* 本文件实现了卡尔曼滤波器的UI界面功能。
********************************************************************************************************************/

#include "kalman_ui_extension.h"
#include "filter/kalman_filter.h"
#include "state_estimator.h"
#include "zf_device_ips114.h"
#include <stdio.h>

void kalman_ui_show_performance(void)
{
    estimator_mode_t current_mode = state_estimator_get_mode();
    
    // 第一行：当前估计器模式
    if (current_mode == ESTIMATOR_MODE_KALMAN) {
        ips114_show_string(0, 0, "Estimator: Kalman");
    } else {
        ips114_show_string(0, 0, "Estimator: Simple");
    }
    
    // 第二行：陀螺仪偏置信息
    float gyro_bias = gyro_kalman_filter_get_bias();
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "Gyro Bias: %.4f", gyro_bias);
    ips114_show_string(0, 20, buffer);
    
    // 如果是卡尔曼模式，显示详细性能信息
    if (current_mode == ESTIMATOR_MODE_KALMAN) {
        const kf_performance_t* perf = state_estimator_get_kalman_performance();
        if (perf != NULL) {
            // 第三行：新息信息
            snprintf(buffer, sizeof(buffer), "Innov X: %.2f Y: %.2f", 
                     perf->innovation_x, perf->innovation_y);
            ips114_show_string(0, 40, buffer);
            
            // 第四行：协方差迹
            snprintf(buffer, sizeof(buffer), "P Trace: %.3f", perf->trace_P);
            ips114_show_string(0, 60, buffer);
            
            // 第五行：异常值计数
            snprintf(buffer, sizeof(buffer), "Outliers: %lu", perf->outlier_count);
            ips114_show_string(0, 80, buffer);
        }
    } else {
        // 简单模式显示状态信息
        const VehicleState* state = state_estimator_get_state();
        snprintf(buffer, sizeof(buffer), "Pos: (%.0f,%.0f)", state->x, state->y);
        ips114_show_string(0, 40, buffer);
        
        snprintf(buffer, sizeof(buffer), "Head: %.2f deg", state->heading * 180.0f / 3.14159f);
        ips114_show_string(0, 60, buffer);
        
        snprintf(buffer, sizeof(buffer), "Speed: %.2f m/s", state->linear_speed);
        ips114_show_string(0, 80, buffer);
    }
}

void kalman_ui_show_estimator_selection(void)
{
    estimator_mode_t current_mode = state_estimator_get_mode();
    
    ips114_show_string(0, 0, "=== Estimator Select ===");
    
    // 显示当前选择
    if (current_mode == ESTIMATOR_MODE_SIMPLE) {
        ips114_show_string(0, 20, "-> Simple (Fast)");
        ips114_show_string(0, 40, "   Kalman (Precise)");
    } else {
        ips114_show_string(0, 20, "   Simple (Fast)");
        ips114_show_string(0, 40, "-> Kalman (Precise)");
    }
    
    ips114_show_string(0, 80, "KEY1: Switch  KEY2: OK");
    ips114_show_string(0, 100, "KEY3: Back    KEY4: Debug");
}

bool kalman_ui_handle_estimator_key(uint8_t key_pressed)
{
    estimator_mode_t current_mode = state_estimator_get_mode();
    
    switch (key_pressed) {
        case 1: // KEY1: 切换估计器类型
            if (current_mode == ESTIMATOR_MODE_SIMPLE) {
                state_estimator_set_mode(ESTIMATOR_MODE_KALMAN);
                ips114_show_string(0, 120, "Switched to Kalman!");
            } else {
                state_estimator_set_mode(ESTIMATOR_MODE_SIMPLE);
                ips114_show_string(0, 120, "Switched to Simple!");
            }
            return true;
            
        case 2: // KEY2: 确认选择
            return true;
            
        case 3: // KEY3: 返回上级菜单
            return false;
            
        case 4: // KEY4: 显示调试信息
            kalman_ui_show_gyro_info();
            return true;
            
        default:
            return false;
    }
}

void kalman_ui_show_gyro_info(void)
{
    ips114_clear();
    ips114_show_string(0, 0, "=== Gyro Filter Info ===");
    
    // 显示陀螺仪偏置
    float gyro_bias = gyro_kalman_filter_get_bias();
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "Bias: %.5f rad/s", gyro_bias);
    ips114_show_string(0, 20, buffer);
    
    snprintf(buffer, sizeof(buffer), "Bias: %.3f deg/s", gyro_bias * 180.0f / 3.14159f);
    ips114_show_string(0, 40, buffer);
    
    // 显示当前车辆状态
    const VehicleState* state = state_estimator_get_state();
    snprintf(buffer, sizeof(buffer), "Position: (%.1f, %.1f)", state->x, state->y);
    ips114_show_string(0, 60, buffer);
    
    snprintf(buffer, sizeof(buffer), "Heading: %.2f deg", state->heading * 180.0f / 3.14159f);
    ips114_show_string(0, 80, buffer);
    
    snprintf(buffer, sizeof(buffer), "Speed: %.3f m/s", state->linear_speed);
    ips114_show_string(0, 100, buffer);
    
    ips114_show_string(0, 120, "Press any key to return");
}
