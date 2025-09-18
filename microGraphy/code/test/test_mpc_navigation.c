/*********************************************************************************************************************
* 文件名称          test_mpc_navigation.c
* 功能说明          MPC导航控制器测试文件
* 作者              LittleMaster
* 版本信息          v1.0
********************************************************************************************************************/

#include "navigation_flash_improved.h"
#include "mpc_controller.h"
#include "mpc_ui_extension.h"
#include "zf_device_ips114.h"
#include "zf_device_systick.h"
#include <stdio.h>

//================================================= 测试配置 =================================================
#define TEST_PATH_SIZE_MM       (800.0f)    // 测试路径大小
#define TEST_PATH_POINTS        (40)        // 测试路径点数
#define TEST_SIMULATION_STEPS   (200)       // 仿真步数

//================================================= 测试函数实现 =================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     MPC控制器性能测试
// 参数说明     void
// 返回参数     void
// 使用示例     test_mpc_performance();
//-------------------------------------------------------------------------------------------------------------------
void test_mpc_performance(void)
{
    printf("=== MPC Controller Performance Test ===\n");
    
    // 1. 初始化导航系统
    Navigation_Init();
    Navigation_GenerateTestPath(TEST_PATH_SIZE_MM, TEST_PATH_POINTS);
    
    // 2. 切换到MPC控制器
    Navigation_SetControllerType(NAV_CONTROLLER_MPC);
    printf("Switched to MPC Controller\n");
    
    // 3. 模拟车辆状态
    VehicleState test_state = {
        .x = 0.0f,
        .y = 0.0f, 
        .heading = 1.57f, // 90度
        .linear_speed = 1.0f
    };
    
    // 4. 性能测试
    uint32_t total_time = 0;
    uint32_t max_time = 0;
    uint32_t min_time = 999999;
    
    for (int i = 0; i < TEST_SIMULATION_STEPS; i++) {
        uint32_t start_time = systick_get_time_ms() * 1000; // 转换为微秒
        
        // 执行MPC计算
        MotionCommand cmd = Navigation_PathTrack(&test_state);
        
        uint32_t end_time = systick_get_time_ms() * 1000;
        uint32_t elapsed = end_time - start_time;
        
        total_time += elapsed;
        if (elapsed > max_time) max_time = elapsed;
        if (elapsed < min_time) min_time = elapsed;
        
        // 简单的运动学更新
        float dt = 0.02f; // 20ms
        test_state.x += test_state.linear_speed * cosf(test_state.heading) * dt * 1000.0f;
        test_state.y += test_state.linear_speed * sinf(test_state.heading) * dt * 1000.0f;
        test_state.heading += cmd.desired_angular_speed * dt;
        test_state.linear_speed = cmd.desired_linear_speed;
        
        // 每10步打印一次状态
        if (i % 10 == 0) {
            printf("Step %d: Time=%lu us, Pos=(%.1f,%.1f), Speed=%.2f\n", 
                   i, elapsed, test_state.x, test_state.y, test_state.linear_speed);
        }
    }
    
    // 5. 打印统计结果
    uint32_t avg_time = total_time / TEST_SIMULATION_STEPS;
    printf("\n=== Performance Results ===\n");
    printf("Average Time: %lu us\n", avg_time);
    printf("Maximum Time: %lu us\n", max_time);
    printf("Minimum Time: %lu us\n", min_time);
    printf("CPU Usage (20ms cycle): %.2f%%\n", (float)avg_time / 20000.0f * 100.0f);
    
    // 6. 获取MPC性能统计
    const mpc_performance_t* perf = mpc_get_performance_stats();
    printf("\nMPC Internal Stats:\n");
    printf("Total Iterations: %lu\n", perf->total_iterations);
    printf("Convergence Failures: %lu\n", perf->convergence_failures);
    printf("Success Rate: %.1f%%\n", 
           (1.0f - (float)perf->convergence_failures / TEST_SIMULATION_STEPS) * 100.0f);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     MPC与Stanley控制器对比测试
// 参数说明     void
// 返回参数     void
// 使用示例     test_mpc_vs_stanley();
//-------------------------------------------------------------------------------------------------------------------
void test_mpc_vs_stanley(void)
{
    printf("=== MPC vs Stanley Comparison Test ===\n");
    
    // 初始化导航系统
    Navigation_Init();
    Navigation_GenerateTestPath(TEST_PATH_SIZE_MM, TEST_PATH_POINTS);
    
    VehicleState test_state = {0.0f, 0.0f, 1.57f, 1.0f};
    
    // 测试Stanley控制器
    Navigation_SetControllerType(NAV_CONTROLLER_STANLEY);
    printf("\n--- Stanley Controller Test ---\n");
    
    float stanley_total_error = 0.0f;
    uint32_t stanley_total_time = 0;
    
    for (int i = 0; i < 50; i++) {
        uint32_t start_time = systick_get_time_ms() * 1000;
        MotionCommand cmd = Navigation_PathTrack(&test_state);
        uint32_t end_time = systick_get_time_ms() * 1000;
        stanley_total_time += (end_time - start_time);
        
        // 计算路径跟踪误差（简化）
        float path_error = sqrtf(test_state.x * test_state.x + test_state.y * test_state.y);
        stanley_total_error += path_error;
        
        // 更新状态
        float dt = 0.02f;
        test_state.x += test_state.linear_speed * cosf(test_state.heading) * dt * 1000.0f;
        test_state.y += test_state.linear_speed * sinf(test_state.heading) * dt * 1000.0f;
        test_state.heading += cmd.desired_angular_speed * dt;
        test_state.linear_speed = cmd.desired_linear_speed;
    }
    
    printf("Stanley Avg Time: %lu us\n", stanley_total_time / 50);
    printf("Stanley Avg Error: %.2f mm\n", stanley_total_error / 50);
    
    // 重置状态，测试MPC控制器
    test_state.x = 0.0f; test_state.y = 0.0f; test_state.heading = 1.57f; test_state.linear_speed = 1.0f;
    Navigation_SetControllerType(NAV_CONTROLLER_MPC);
    printf("\n--- MPC Controller Test ---\n");
    
    float mpc_total_error = 0.0f;
    uint32_t mpc_total_time = 0;
    
    for (int i = 0; i < 50; i++) {
        uint32_t start_time = systick_get_time_ms() * 1000;
        MotionCommand cmd = Navigation_PathTrack(&test_state);
        uint32_t end_time = systick_get_time_ms() * 1000;
        mpc_total_time += (end_time - start_time);
        
        // 计算路径跟踪误差
        float path_error = sqrtf(test_state.x * test_state.x + test_state.y * test_state.y);
        mpc_total_error += path_error;
        
        // 更新状态
        float dt = 0.02f;
        test_state.x += test_state.linear_speed * cosf(test_state.heading) * dt * 1000.0f;
        test_state.y += test_state.linear_speed * sinf(test_state.heading) * dt * 1000.0f;
        test_state.heading += cmd.desired_angular_speed * dt;
        test_state.linear_speed = cmd.desired_linear_speed;
    }
    
    printf("MPC Avg Time: %lu us\n", mpc_total_time / 50);
    printf("MPC Avg Error: %.2f mm\n", mpc_total_error / 50);
    
    // 对比结果
    printf("\n--- Comparison Summary ---\n");
    printf("Time Overhead: %.1fx\n", (float)mpc_total_time / stanley_total_time);
    printf("Error Improvement: %.1f%%\n", 
           (1.0f - mpc_total_error / stanley_total_error) * 100.0f);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     MPC参数调优测试
// 参说明     void
// 返回参数     void
// 使用示例     test_mpc_parameter_tuning();
//-------------------------------------------------------------------------------------------------------------------
void test_mpc_parameter_tuning(void)
{
    printf("=== MPC Parameter Tuning Test ===\n");
    
    Navigation_Init();
    Navigation_GenerateTestPath(TEST_PATH_SIZE_MM, TEST_PATH_POINTS);
    Navigation_SetControllerType(NAV_CONTROLLER_MPC);
    
    // 测试不同的权重参数组合
    mpc_weights_t test_weights[] = {
        // 配置1：高精度跟踪
        {10.0f, 10.0f, 8.0f, 1.0f, 1.0f, 0.1f, 0.1f, 2.0f, 2.0f, 15.0f, 15.0f, 10.0f},
        // 配置2：低能耗
        {5.0f, 5.0f, 4.0f, 0.5f, 0.5f, 1.0f, 1.0f, 1.0f, 1.0f, 8.0f, 8.0f, 5.0f},
        // 配置3：高平滑性
        {8.0f, 8.0f, 6.0f, 1.0f, 1.0f, 0.5f, 0.5f, 5.0f, 5.0f, 12.0f, 12.0f, 8.0f}
    };
    
    const char* config_names[] = {"High Precision", "Low Energy", "High Smoothness"};
    
    for (int config = 0; config < 3; config++) {
        printf("\n--- Testing %s Config ---\n", config_names[config]);
        
        // 设置权重
        mpc_set_weights(&test_weights[config]);
        mpc_reset_controller();
        
        VehicleState test_state = {0.0f, 0.0f, 1.57f, 1.0f};
        float total_error = 0.0f;
        uint32_t total_time = 0;
        
        for (int i = 0; i < 30; i++) {
            uint32_t start_time = systick_get_time_ms() * 1000;
            MotionCommand cmd = Navigation_PathTrack(&test_state);
            uint32_t end_time = systick_get_time_ms() * 1000;
            
            total_time += (end_time - start_time);
            
            float path_error = sqrtf(test_state.x * test_state.x + test_state.y * test_state.y);
            total_error += path_error;
            
            // 更新状态
            float dt = 0.02f;
            test_state.x += test_state.linear_speed * cosf(test_state.heading) * dt * 1000.0f;
            test_state.y += test_state.linear_speed * sinf(test_state.heading) * dt * 1000.0f;
            test_state.heading += cmd.desired_angular_speed * dt;
            test_state.linear_speed = cmd.desired_linear_speed;
        }
        
        printf("Avg Time: %lu us\n", total_time / 30);
        printf("Avg Error: %.2f mm\n", total_error / 30);
        
        const mpc_performance_t* perf = mpc_get_performance_stats();
        printf("Conv Failures: %lu\n", perf->convergence_failures);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UI显示MPC信息的测试
// 参数说明     void
// 返回参数     void
// 使用示例     test_mpc_ui_display();
//-------------------------------------------------------------------------------------------------------------------
void test_mpc_ui_display(void)
{
    printf("=== MPC UI Display Test ===\n");
    
    Navigation_Init();
    Navigation_SetControllerType(NAV_CONTROLLER_MPC);
    
    // 执行一些计算以产生性能数据
    VehicleState test_state = {0.0f, 0.0f, 1.57f, 1.0f};
    Navigation_GenerateTestPath(TEST_PATH_SIZE_MM, TEST_PATH_POINTS);
    
    for (int i = 0; i < 10; i++) {
        Navigation_PathTrack(&test_state);
    }
    
    // 显示性能信息
    ips114_clear();
    mpc_ui_show_performance();
    
    printf("MPC performance information displayed on screen.\n");
    printf("Check the IPS114 display for real-time MPC stats.\n");
    
    // 显示控制器选择界面
    systick_delay_ms(2000);
    ips114_clear();
    mpc_ui_show_controller_selection();
    
    printf("Controller selection interface displayed.\n");
    printf("Use keys to switch between Stanley and MPC controllers.\n");
}
