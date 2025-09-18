/*********************************************************************************************************************
* 文件名称          test_kalman_filter.c
* 功能说明          卡尔曼滤波器测试文件
* 作者              sonnet
* 版本信息          v1.0
********************************************************************************************************************/

#include "filter/kalman_filter.h"
#include "state_estimator.h"
#include "kalman_ui_extension.h"
#include "zf_device_systick.h"
#include <stdio.h>
#include <math.h>

//================================================= 测试配置 =================================================
#define TEST_SIMULATION_TIME    (10.0f)    // 仿真时间 (秒)
#define TEST_DT                 (0.002f)   // 时间步长 (2ms)
#define TEST_NOISE_LEVEL        (0.1f)     // 噪声水平

//================================================= 测试函数实现 =================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     陀螺仪卡尔曼滤波器测试
// 参数说明     void
// 返回参数     void
// 使用示例     test_gyro_kalman_filter();
//-------------------------------------------------------------------------------------------------------------------
void test_gyro_kalman_filter(void)
{
    printf("=== Gyroscope Kalman Filter Test ===\n");
    
    // 初始化陀螺仪滤波器
    gyro_kalman_filter_init();
    
    // 模拟测试：陀螺仪有固定偏置和噪声
    float true_bias = 0.05f;      // 真实偏置 0.05 rad/s
    float true_omega = 1.0f;      // 真实角速度 1.0 rad/s
    
    printf("True bias: %.3f rad/s, True omega: %.3f rad/s\n", true_bias, true_omega);
    
    float total_bias_error = 0.0f;
    float total_omega_error = 0.0f;
    int test_steps = (int)(TEST_SIMULATION_TIME / TEST_DT);
    
    for (int i = 0; i < test_steps; i++) {
        // 模拟带噪声的陀螺仪测量
        float noise = ((float)rand() / RAND_MAX - 0.5f) * 2.0f * TEST_NOISE_LEVEL;
        float measured_gyro = true_omega + true_bias + noise;
        
        // 卡尔曼滤波
        float filtered_omega = gyro_kalman_filter_update(measured_gyro, TEST_DT);
        float estimated_bias = gyro_kalman_filter_get_bias();
        
        // 计算误差
        float bias_error = fabsf(estimated_bias - true_bias);
        float omega_error = fabsf(filtered_omega - true_omega);
        
        total_bias_error += bias_error;
        total_omega_error += omega_error;
        
        // 每秒输出一次结果
        if (i % 500 == 0) {
            printf("Step %d: Bias=%.4f (err=%.4f), Omega=%.3f (err=%.3f)\n",
                   i, estimated_bias, bias_error, filtered_omega, omega_error);
        }
    }
    
    float avg_bias_error = total_bias_error / test_steps;
    float avg_omega_error = total_omega_error / test_steps;
    
    printf("\n=== Test Results ===\n");
    printf("Average bias error: %.5f rad/s\n", avg_bias_error);
    printf("Average omega error: %.4f rad/s\n", avg_omega_error);
    printf("Final bias estimate: %.5f rad/s\n", gyro_kalman_filter_get_bias());
    
    // 性能评估
    if (avg_bias_error < 0.01f && avg_omega_error < 0.1f) {
        printf("? Gyro Kalman filter test PASSED!\n");
    } else {
        printf("? Gyro Kalman filter test FAILED!\n");
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     主卡尔曼滤波器测试
// 参数说明     void
// 返回参数     void
// 使用示例     test_main_kalman_filter();
//-------------------------------------------------------------------------------------------------------------------
void test_main_kalman_filter(void)
{
    printf("=== Main Kalman Filter Test ===\n");
    
    // 初始化卡尔曼滤波器
    kf_state_t initial_state = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    kalman_filter_init(&initial_state);
    
    // 模拟直线运动：v=1m/s, omega=0.5rad/s
    float true_vx = 1000.0f;     // 1000 mm/s
    float true_vy = 0.0f;
    float true_omega = 0.5f;     // 0.5 rad/s
    
    float true_x = 0.0f, true_y = 0.0f, true_heading = 0.0f;
    float encoder_x = 0.0f, encoder_y = 0.0f;
    
    int test_steps = (int)(TEST_SIMULATION_TIME / TEST_DT);
    
    for (int i = 0; i < test_steps; i++) {
        // 模拟真实运动
        true_x += true_vx * cosf(true_heading) * TEST_DT;
        true_y += true_vx * sinf(true_heading) * TEST_DT;
        true_heading += true_omega * TEST_DT;
        
        // 模拟编码器积分 (带噪声)
        float encoder_noise_x = ((float)rand() / RAND_MAX - 0.5f) * 10.0f; // ±5mm噪声
        float encoder_noise_y = ((float)rand() / RAND_MAX - 0.5f) * 10.0f;
        encoder_x = true_x + encoder_noise_x;
        encoder_y = true_y + encoder_noise_y;
        
        // 模拟陀螺仪 (带噪声)
        float gyro_noise = ((float)rand() / RAND_MAX - 0.5f) * 0.1f; // ±0.05rad/s噪声
        float measured_omega = true_omega + gyro_noise;
        
        // 卡尔曼滤波器预测和更新
        kalman_filter_predict(TEST_DT);
        
        kf_measurement_t measurement = {
            .encoder_x = encoder_x,
            .encoder_y = encoder_y,
            .gyro_omega = measured_omega,
            .valid = true
        };
        
        kalman_filter_update(&measurement);
        
        // 每秒输出一次结果
        if (i % 500 == 0) {
            kf_state_t estimated_state;
            kalman_filter_get_state(&estimated_state);
            
            printf("Step %d: True(%.1f,%.1f,%.2f) Est(%.1f,%.1f,%.2f)\n",
                   i, true_x, true_y, true_heading,
                   estimated_state.x, estimated_state.y, estimated_state.heading);
        }
    }
    
    // 最终精度评估
    kf_state_t final_state;
    kalman_filter_get_state(&final_state);
    
    float pos_error = sqrtf(powf(final_state.x - true_x, 2) + powf(final_state.y - true_y, 2));
    float heading_error = fabsf(final_state.heading - true_heading);
    
    printf("\n=== Final Results ===\n");
    printf("Position error: %.2f mm\n", pos_error);
    printf("Heading error: %.3f rad (%.1f deg)\n", heading_error, heading_error * 180.0f / 3.14159f);
    
    const kf_performance_t* perf = kalman_filter_get_performance();
    printf("Outlier count: %lu\n", perf->outlier_count);
    printf("P trace: %.3f\n", perf->trace_P);
    
    // 性能评估
    if (pos_error < 50.0f && heading_error < 0.1f) {
        printf("? Main Kalman filter test PASSED!\n");
    } else {
        printf("? Main Kalman filter test FAILED!\n");
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     状态估计器模式对比测试
// 参数说明     void
// 返回参数     void
// 使用示例     test_estimator_comparison();
//-------------------------------------------------------------------------------------------------------------------
void test_estimator_comparison(void)
{
    printf("=== State Estimator Comparison Test ===\n");
    
    // 初始化状态估计器
    state_estimator_init();
    
    // 测试简单模式
    state_estimator_set_mode(ESTIMATOR_MODE_SIMPLE);
    printf("\n--- Simple Estimator Test ---\n");
    
    float simple_total_time = 0.0f;
    uint32_t simple_start_time = systick_get_time_ms() * 1000;
    
    for (int i = 0; i < 1000; i++) {
        state_estimator_update(TEST_DT);
    }
    
    uint32_t simple_end_time = systick_get_time_ms() * 1000;
    simple_total_time = (simple_end_time - simple_start_time) / 1000.0f;
    
    const VehicleState* simple_state = state_estimator_get_state();
    printf("Simple mode time: %.3f ms (avg: %.1f us/update)\n", 
           simple_total_time, simple_total_time * 1000.0f / 1000);
    printf("Final state: (%.1f, %.1f, %.3f)\n", 
           simple_state->x, simple_state->y, simple_state->heading);
    
    // 重置并测试卡尔曼模式
    state_estimator_init();
    state_estimator_set_mode(ESTIMATOR_MODE_KALMAN);
    printf("\n--- Kalman Estimator Test ---\n");
    
    float kalman_total_time = 0.0f;
    uint32_t kalman_start_time = systick_get_time_ms() * 1000;
    
    for (int i = 0; i < 1000; i++) {
        state_estimator_update(TEST_DT);
    }
    
    uint32_t kalman_end_time = systick_get_time_ms() * 1000;
    kalman_total_time = (kalman_end_time - kalman_start_time) / 1000.0f;
    
    const VehicleState* kalman_state = state_estimator_get_state();
    printf("Kalman mode time: %.3f ms (avg: %.1f us/update)\n", 
           kalman_total_time, kalman_total_time * 1000.0f / 1000);
    printf("Final state: (%.1f, %.1f, %.3f)\n", 
           kalman_state->x, kalman_state->y, kalman_state->heading);
    
    // 性能对比
    const kf_performance_t* perf = state_estimator_get_kalman_performance();
    if (perf != NULL) {
        printf("Kalman performance:\n");
        printf("  Outliers: %lu\n", perf->outlier_count);
        printf("  P trace: %.3f\n", perf->trace_P);
    }
    
    printf("\n--- Comparison Results ---\n");
    printf("Time overhead: %.1fx\n", kalman_total_time / simple_total_time);
    printf("Gyro bias estimate: %.5f rad/s\n", gyro_kalman_filter_get_bias());
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     噪声环境鲁棒性测试
// 参数说明     void
// 返回参数     void
// 使用示例     test_noise_robustness();
//-------------------------------------------------------------------------------------------------------------------
void test_noise_robustness(void)
{
    printf("=== Noise Robustness Test ===\n");
    
    float noise_levels[] = {0.05f, 0.1f, 0.2f, 0.5f};
    int num_levels = sizeof(noise_levels) / sizeof(noise_levels[0]);
    
    for (int level = 0; level < num_levels; level++) {
        printf("\n--- Noise Level: %.2f ---\n", noise_levels[level]);
        
        // 重新初始化
        gyro_kalman_filter_init();
        
        float true_bias = 0.03f;
        float true_omega = 0.8f;
        float current_noise = noise_levels[level];
        
        float bias_errors[100];
        
        // 运行100次短测试
        for (int test = 0; test < 100; test++) {
            float test_bias_error = 0.0f;
            
            for (int i = 0; i < 100; i++) {
                float noise = ((float)rand() / RAND_MAX - 0.5f) * 2.0f * current_noise;
                float measured = true_omega + true_bias + noise;
                gyro_kalman_filter_update(measured, TEST_DT);
            }
            
            float estimated_bias = gyro_kalman_filter_get_bias();
            bias_errors[test] = fabsf(estimated_bias - true_bias);
        }
        
        // 统计结果
        float avg_error = 0.0f, max_error = 0.0f;
        for (int i = 0; i < 100; i++) {
            avg_error += bias_errors[i];
            if (bias_errors[i] > max_error) max_error = bias_errors[i];
        }
        avg_error /= 100.0f;
        
        printf("Avg bias error: %.5f, Max error: %.5f\n", avg_error, max_error);
        
        // 性能评估
        if (avg_error < current_noise * 0.5f) {
            printf("? Good performance under noise level %.2f\n", current_noise);
        } else {
            printf("?? Degraded performance under noise level %.2f\n", current_noise);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UI显示测试
// 参数说明     void
// 返回参数     void
// 使用示例     test_kalman_ui_display();
//-------------------------------------------------------------------------------------------------------------------
void test_kalman_ui_display(void)
{
    printf("=== Kalman UI Display Test ===\n");
    
    // 初始化并运行一些滤波
    state_estimator_init();
    state_estimator_set_mode(ESTIMATOR_MODE_KALMAN);
    
    for (int i = 0; i < 100; i++) {
        state_estimator_update(TEST_DT);
    }
    
    // 显示性能信息
    printf("Displaying Kalman performance on screen...\n");
    kalman_ui_show_performance();
    
    systick_delay_ms(2000);
    
    // 显示选择界面
    printf("Displaying estimator selection interface...\n");
    kalman_ui_show_estimator_selection();
    
    systick_delay_ms(2000);
    
    // 显示陀螺仪信息
    printf("Displaying gyro filter info...\n");
    kalman_ui_show_gyro_info();
    
    printf("UI display test completed. Check the IPS114 screen.\n");
}
