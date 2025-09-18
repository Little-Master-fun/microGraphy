/*********************************************************************************************************************
* 文件名称          test_mpc_auto_tuning.c
* 功能说明          MPC自动调参测试程序 实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本              备注
* 2025-09-18        LittleMaster        v1.0              创建文件
*
* 文件作用说明：
* 本文件实现了MPC自动调参功能的全面测试程序。
********************************************************************************************************************/

#include "test_mpc_auto_tuning.h"
#include "mpc_controller.h"
#include "mpc_ui_extension.h"
#include "navigation_flash_improved.h"
#include "zf_device_systick.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

//================================================= 测试辅助函数 =================================================

static void generate_test_path(OptimalPathPoint* path, uint16_t* path_length, float size)
{
    // 生成正方形测试路径
    *path_length = 4;
    
    path[0].x = 0.0f;
    path[0].y = 0.0f;
    path[0].heading = 0.0f;
    path[0].speed = 0.5f;
    
    path[1].x = size;
    path[1].y = 0.0f;
    path[1].heading = M_PI/2;
    path[1].speed = 0.5f;
    
    path[2].x = size;
    path[2].y = size;
    path[2].heading = M_PI;
    path[2].speed = 0.5f;
    
    path[3].x = 0.0f;
    path[3].y = size;
    path[3].heading = -M_PI/2;
    path[3].speed = 0.5f;
}

static void simulate_vehicle_motion(VehicleState* state, const MotionCommand* cmd, float dt)
{
    // 简单的车辆运动学仿真
    state->linear_speed = cmd->desired_linear_speed;
    
    // 更新位置和航向
    state->x += state->linear_speed * cosf(state->heading) * dt * 1000.0f; // 转换为mm
    state->y += state->linear_speed * sinf(state->heading) * dt * 1000.0f;
    state->heading += cmd->desired_angular_speed * dt;
    
    // 归一化航向角
    while (state->heading > M_PI) state->heading -= 2 * M_PI;
    while (state->heading < -M_PI) state->heading += 2 * M_PI;
}

static void print_performance_summary(const char* test_name, const mpc_performance_t* perf)
{
    printf("\n=== %s Performance Summary ===\n", test_name);
    printf("Overall Score: %.1f/100\n", perf->overall_performance_score);
    printf("Position RMS Error: %.2f mm\n", perf->position_error_rms);
    printf("Heading RMS Error: %.4f rad (%.1f deg)\n", 
           perf->heading_error_rms, perf->heading_error_rms * 180.0f / M_PI);
    printf("Max Position Error: %.2f mm\n", perf->max_position_error);
    printf("Max Heading Error: %.4f rad (%.1f deg)\n", 
           perf->max_heading_error, perf->max_heading_error * 180.0f / M_PI);
    printf("Average Computation Time: %lu us\n", perf->computation_time_us);
    printf("CPU Usage: %.1f%%\n", perf->cpu_usage_percent);
    printf("Convergence Failures: %lu\n", perf->convergence_failures);
    printf("Samples Count: %lu\n", perf->samples_count);
}

//================================================= 测试函数实现 =================================================

void test_mpc_auto_tuning_basic(void)
{
    printf("=== MPC Auto-Tuning Basic Test ===\n");
    
    // 1. 初始化MPC控制器
    mpc_weights_t default_weights;
    mpc_get_default_weights(&default_weights);
    
    if (!mpc_controller_init(&default_weights)) {
        printf("? MPC controller initialization failed!\n");
        return;
    }
    
    // 2. 配置自动调参
    mpc_auto_tuning_config_t config;
    mpc_get_default_tuning_config(&config);
    config.auto_tuning_enabled = true;
    config.evaluation_window = 20; // 缩短评估窗口用于测试
    config.tuning_interval = 50;   // 缩短调参间隔
    
    if (!mpc_auto_tuning_configure(&config)) {
        printf("? Auto-tuning configuration failed!\n");
        return;
    }
    
    // 3. 启用自动调参
    if (!mpc_auto_tuning_enable(true)) {
        printf("? Auto-tuning enable failed!\n");
        return;
    }
    
    printf("? Auto-tuning enabled successfully\n");
    
    // 4. 生成测试路径
    OptimalPathPoint test_path[10];
    uint16_t path_length;
    generate_test_path(test_path, &path_length, 1000.0f);
    
    // 5. 模拟控制循环
    VehicleState current_state = {0};
    current_state.heading = 0.0f;
    
    printf("Running simulation for 200 steps...\n");
    
    for (int step = 0; step < 200; step++) {
        // 计算MPC控制指令
        MotionCommand cmd = mpc_compute_control(&current_state, test_path, path_length, 300.0f);
        
        // 模拟车辆运动
        simulate_vehicle_motion(&current_state, &cmd, 0.02f);
        
        // 显示调参状态变化
        if (step % 50 == 0) {
            mpc_tuning_state_t state = mpc_get_tuning_state();
            printf("Step %d: Tuning State = %d, Position = (%.1f, %.1f)\n", 
                   step, state, current_state.x, current_state.y);
        }
    }
    
    // 6. 检查结果
    const mpc_performance_t* perf = mpc_get_performance_stats();
    print_performance_summary("Basic Auto-Tuning", perf);
    
    // 7. 获取最优权重
    mpc_weights_t best_weights;
    if (mpc_get_best_weights(&best_weights)) {
        printf("\n=== Optimized Weights ===\n");
        printf("Q_x: %.2f -> %.2f\n", default_weights.Q_x, best_weights.Q_x);
        printf("Q_y: %.2f -> %.2f\n", default_weights.Q_y, best_weights.Q_y);
        printf("Q_heading: %.2f -> %.2f\n", default_weights.Q_heading, best_weights.Q_heading);
        printf("R_linear_vel: %.2f -> %.2f\n", default_weights.R_linear_vel, best_weights.R_linear_vel);
        printf("S_linear_vel: %.2f -> %.2f\n", default_weights.S_linear_vel, best_weights.S_linear_vel);
    }
    
    printf("? Basic auto-tuning test completed!\n");
}

void test_mpc_performance_evaluation(void)
{
    printf("=== MPC Performance Evaluation Test ===\n");
    
    // 初始化MPC
    mpc_weights_t weights;
    mpc_get_default_weights(&weights);
    mpc_controller_init(&weights);
    
    // 生成测试路径
    OptimalPathPoint test_path[10];
    uint16_t path_length;
    generate_test_path(test_path, &path_length, 800.0f);
    
    // 测试不同的权重配置
    struct {
        const char* name;
        float Q_scale;
        float R_scale;
        float S_scale;
    } test_configs[] = {
        {"High Precision", 2.0f, 0.5f, 1.0f},
        {"Smooth Control", 1.0f, 0.1f, 3.0f},
        {"Fast Response", 1.5f, 1.0f, 0.5f},
        {"Balanced", 1.0f, 1.0f, 1.0f}
    };
    
    for (int config_idx = 0; config_idx < 4; config_idx++) {
        printf("\n--- Testing %s Configuration ---\n", test_configs[config_idx].name);
        
        // 设置权重
        mpc_get_default_weights(&weights);
        weights.Q_x *= test_configs[config_idx].Q_scale;
        weights.Q_y *= test_configs[config_idx].Q_scale;
        weights.Q_heading *= test_configs[config_idx].Q_scale;
        weights.R_linear_vel *= test_configs[config_idx].R_scale;
        weights.R_angular_vel *= test_configs[config_idx].R_scale;
        weights.S_linear_vel *= test_configs[config_idx].S_scale;
        weights.S_angular_vel *= test_configs[config_idx].S_scale;
        
        mpc_set_weights(&weights);
        mpc_reset_controller();
        
        // 运行仿真
        VehicleState state = {0};
        for (int step = 0; step < 100; step++) {
            MotionCommand cmd = mpc_compute_control(&state, test_path, path_length, 300.0f);
            simulate_vehicle_motion(&state, &cmd, 0.02f);
        }
        
        const mpc_performance_t* perf = mpc_get_performance_stats();
        print_performance_summary(test_configs[config_idx].name, perf);
    }
    
    printf("? Performance evaluation test completed!\n");
}

void test_mpc_adaptive_weights(void)
{
    printf("=== MPC Adaptive Weights Test ===\n");
    
    // 初始化MPC
    mpc_weights_t weights;
    mpc_get_default_weights(&weights);
    mpc_controller_init(&weights);
    
    // 启用自动调参
    mpc_auto_tuning_config_t config;
    mpc_get_default_tuning_config(&config);
    config.auto_tuning_enabled = true;
    mpc_auto_tuning_configure(&config);
    mpc_auto_tuning_enable(true);
    
    // 测试不同工况
    struct {
        const char* name;
        float speed;
        float path_size;
        bool is_curved;
    } scenarios[] = {
        {"Low Speed Straight", 0.3f, 1500.0f, false},
        {"Medium Speed Straight", 0.8f, 1500.0f, false},
        {"High Speed Straight", 1.2f, 1500.0f, false},
        {"Medium Speed Curved", 0.8f, 800.0f, true}
    };
    
    for (int scenario_idx = 0; scenario_idx < 4; scenario_idx++) {
        printf("\n--- Testing %s Scenario ---\n", scenarios[scenario_idx].name);
        
        // 生成对应的测试路径
        OptimalPathPoint test_path[10];
        uint16_t path_length;
        generate_test_path(test_path, &path_length, scenarios[scenario_idx].path_size);
        
        // 设置路径速度
        for (int i = 0; i < path_length; i++) {
            test_path[i].speed = scenarios[scenario_idx].speed;
        }
        
        // 重置控制器状态
        mpc_reset_controller();
        
        // 运行仿真
        VehicleState state = {0};
        printf("Running adaptive tuning simulation...\n");
        
        for (int step = 0; step < 150; step++) {
            MotionCommand cmd = mpc_compute_control(&state, test_path, path_length, 300.0f);
            simulate_vehicle_motion(&state, &cmd, 0.02f);
            
            // 每30步显示一次权重变化
            if (step % 30 == 0 && step > 0) {
                mpc_weights_t current_weights;
                if (mpc_get_best_weights(&current_weights)) {
                    printf("Step %d: Q_heading=%.2f, S_angular=%.2f\n", 
                           step, current_weights.Q_heading, current_weights.S_angular_vel);
                }
            }
        }
        
        const mpc_performance_t* perf = mpc_get_performance_stats();
        print_performance_summary(scenarios[scenario_idx].name, perf);
    }
    
    printf("? Adaptive weights test completed!\n");
}

void test_mpc_tuning_convergence(void)
{
    printf("=== MPC Tuning Convergence Test ===\n");
    
    // 初始化MPC
    mpc_weights_t initial_weights;
    mpc_get_default_weights(&initial_weights);
    mpc_controller_init(&initial_weights);
    
    // 配置快速收敛的调参参数
    mpc_auto_tuning_config_t config;
    mpc_get_default_tuning_config(&config);
    config.auto_tuning_enabled = true;
    config.evaluation_window = 15;     // 缩短评估窗口
    config.tuning_interval = 30;       // 缩短调参间隔  
    config.performance_threshold = 1.0f; // 降低性能阈值
    config.max_tuning_iterations = 20;   // 增加最大迭代次数
    
    mpc_auto_tuning_configure(&config);
    mpc_auto_tuning_enable(true);
    
    // 生成测试路径
    OptimalPathPoint test_path[10];
    uint16_t path_length;
    generate_test_path(test_path, &path_length, 1000.0f);
    
    // 记录收敛过程
    float performance_history[50];
    int convergence_steps = 0;
    bool converged = false;
    
    VehicleState state = {0};
    
    printf("Monitoring convergence process...\n");
    
    for (int step = 0; step < 500 && !converged; step++) {
        MotionCommand cmd = mpc_compute_control(&state, test_path, path_length, 300.0f);
        simulate_vehicle_motion(&state, &cmd, 0.02f);
        
        // 记录性能
        if (step % 10 == 0) {
            const mpc_performance_t* perf = mpc_get_performance_stats();
            int history_idx = step / 10;
            if (history_idx < 50) {
                performance_history[history_idx] = perf->overall_performance_score;
            }
            
            // 检查是否收敛
            mpc_tuning_state_t tuning_state = mpc_get_tuning_state();
            if (tuning_state == TUNING_STATE_CONVERGED && !converged) {
                convergence_steps = step;
                converged = true;
                printf("? Convergence achieved at step %d!\n", step);
            }
            
            // 每50步输出一次状态
            if (step % 50 == 0) {
                printf("Step %d: Score=%.1f, State=%d\n", 
                       step, perf->overall_performance_score, tuning_state);
            }
        }
    }
    
    // 分析收敛结果
    if (converged) {
        printf("\n=== Convergence Analysis ===\n");
        printf("Convergence Time: %d steps (%.1f seconds)\n", 
               convergence_steps, convergence_steps * 0.02f);
        
        const mpc_performance_t* final_perf = mpc_get_performance_stats();
        print_performance_summary("Final Converged", final_perf);
        
        // 比较初始和最终权重
        mpc_weights_t final_weights;
        if (mpc_get_best_weights(&final_weights)) {
            printf("\n=== Weight Optimization Results ===\n");
            printf("Q_x: %.2f -> %.2f (%.1f%% change)\n", 
                   initial_weights.Q_x, final_weights.Q_x, 
                   (final_weights.Q_x - initial_weights.Q_x) / initial_weights.Q_x * 100);
            printf("Q_heading: %.2f -> %.2f (%.1f%% change)\n", 
                   initial_weights.Q_heading, final_weights.Q_heading,
                   (final_weights.Q_heading - initial_weights.Q_heading) / initial_weights.Q_heading * 100);
            printf("R_linear_vel: %.2f -> %.2f (%.1f%% change)\n", 
                   initial_weights.R_linear_vel, final_weights.R_linear_vel,
                   (final_weights.R_linear_vel - initial_weights.R_linear_vel) / initial_weights.R_linear_vel * 100);
        }
    } else {
        printf("?? Convergence not achieved within 500 steps\n");
    }
    
    printf("? Convergence test completed!\n");
}

void test_mpc_multi_speed_tuning(void)
{
    printf("=== MPC Multi-Speed Tuning Test ===\n");
    
    float test_speeds[] = {0.2f, 0.5f, 0.8f, 1.0f, 1.3f};
    int num_speeds = sizeof(test_speeds) / sizeof(test_speeds[0]);
    
    // 为每个速度进行调参测试
    for (int speed_idx = 0; speed_idx < num_speeds; speed_idx++) {
        float target_speed = test_speeds[speed_idx];
        printf("\n--- Tuning for Speed %.1f m/s ---\n", target_speed);
        
        // 重新初始化MPC
        mpc_weights_t weights;
        mpc_get_default_weights(&weights);
        mpc_controller_init(&weights);
        
        // 启用自动调参
        mpc_auto_tuning_config_t config;
        mpc_get_default_tuning_config(&config);
        config.auto_tuning_enabled = true;
        config.evaluation_window = 25;
        config.tuning_interval = 50;
        
        mpc_auto_tuning_configure(&config);
        mpc_auto_tuning_enable(true);
        
        // 生成对应速度的测试路径
        OptimalPathPoint test_path[10];
        uint16_t path_length;
        generate_test_path(test_path, &path_length, 1200.0f);
        
        for (int i = 0; i < path_length; i++) {
            test_path[i].speed = target_speed;
        }
        
        // 运行调参
        VehicleState state = {0};
        for (int step = 0; step < 200; step++) {
            MotionCommand cmd = mpc_compute_control(&state, test_path, path_length, 300.0f);
            simulate_vehicle_motion(&state, &cmd, 0.02f);
        }
        
        // 记录该速度下的最优性能
        const mpc_performance_t* perf = mpc_get_performance_stats();
        printf("Speed %.1f m/s: Score=%.1f, Pos_RMS=%.2f mm\n", 
               target_speed, perf->overall_performance_score, perf->position_error_rms);
        
        // 保存最优权重（实际应用中可以建立速度-权重映射表）
        mpc_weights_t optimized_weights;
        if (mpc_get_best_weights(&optimized_weights)) {
            printf("  Optimized Q_heading: %.2f\n", optimized_weights.Q_heading);
        }
    }
    
    printf("? Multi-speed tuning test completed!\n");
}

void test_mpc_tuning_performance_comparison(void)
{
    printf("=== MPC Tuning Performance Comparison Test ===\n");
    
    // 生成测试路径
    OptimalPathPoint test_path[10];
    uint16_t path_length;
    generate_test_path(test_path, &path_length, 1000.0f);
    
    // 测试1: 默认参数性能
    printf("\n--- Test 1: Default Parameters ---\n");
    mpc_weights_t default_weights;
    mpc_get_default_weights(&default_weights);
    mpc_controller_init(&default_weights);
    
    VehicleState state1 = {0};
    for (int step = 0; step < 150; step++) {
        MotionCommand cmd = mpc_compute_control(&state1, test_path, path_length, 300.0f);
        simulate_vehicle_motion(&state1, &cmd, 0.02f);
    }
    const mpc_performance_t* default_perf = mpc_get_performance_stats();
    print_performance_summary("Default Parameters", default_perf);
    
    // 测试2: 自动调参性能
    printf("\n--- Test 2: Auto-Tuned Parameters ---\n");
    mpc_controller_init(&default_weights);
    
    // 启用自动调参
    mpc_auto_tuning_config_t config;
    mpc_get_default_tuning_config(&config);
    config.auto_tuning_enabled = true;
    config.evaluation_window = 20;
    config.tuning_interval = 40;
    
    mpc_auto_tuning_configure(&config);
    mpc_auto_tuning_enable(true);
    
    VehicleState state2 = {0};
    for (int step = 0; step < 250; step++) { // 更多步数用于调参
        MotionCommand cmd = mpc_compute_control(&state2, test_path, path_length, 300.0f);
        simulate_vehicle_motion(&state2, &cmd, 0.02f);
    }
    const mpc_performance_t* tuned_perf = mpc_get_performance_stats();
    print_performance_summary("Auto-Tuned Parameters", tuned_perf);
    
    // 性能对比分析
    printf("\n=== Performance Comparison ===\n");
    float score_improvement = tuned_perf->overall_performance_score - default_perf->overall_performance_score;
    float pos_error_improvement = (default_perf->position_error_rms - tuned_perf->position_error_rms) / default_perf->position_error_rms * 100.0f;
    float heading_error_improvement = (default_perf->heading_error_rms - tuned_perf->heading_error_rms) / default_perf->heading_error_rms * 100.0f;
    
    printf("Overall Score Improvement: %.1f points\n", score_improvement);
    printf("Position Error Improvement: %.1f%%\n", pos_error_improvement);
    printf("Heading Error Improvement: %.1f%%\n", heading_error_improvement);
    
    if (score_improvement > 5.0f) {
        printf("? Significant performance improvement achieved!\n");
    } else if (score_improvement > 0.0f) {
        printf("? Moderate performance improvement achieved!\n");
    } else {
        printf("?? No significant improvement or performance degraded\n");
    }
    
    printf("? Performance comparison test completed!\n");
}

void test_mpc_tuning_ui(void)
{
    printf("=== MPC Tuning UI Test ===\n");
    
    // 初始化MPC和自动调参
    mpc_weights_t weights;
    mpc_get_default_weights(&weights);
    mpc_controller_init(&weights);
    
    mpc_auto_tuning_config_t config;
    mpc_get_default_tuning_config(&config);
    config.auto_tuning_enabled = true;
    mpc_auto_tuning_configure(&config);
    mpc_auto_tuning_enable(true);
    
    // 生成一些测试数据
    OptimalPathPoint test_path[10];
    uint16_t path_length;
    generate_test_path(test_path, &path_length, 800.0f);
    
    VehicleState state = {0};
    for (int step = 0; step < 50; step++) {
        MotionCommand cmd = mpc_compute_control(&state, test_path, path_length, 300.0f);
        simulate_vehicle_motion(&state, &cmd, 0.02f);
    }
    
    printf("Testing UI display functions...\n");
    
    // 测试主要UI界面
    printf("1. Testing auto-tuning main UI...\n");
    mpc_ui_show_auto_tuning();
    systick_delay_ms(1000);
    
    printf("2. Testing tuning configuration UI...\n");
    mpc_ui_show_tuning_config();
    systick_delay_ms(1000);
    
    printf("3. Testing weights display UI...\n");
    mpc_ui_show_weights();
    systick_delay_ms(1000);
    
    printf("4. Testing performance trend UI...\n");
    mpc_ui_show_performance_trend();
    systick_delay_ms(1000);
    
    // 测试按键处理
    printf("5. Testing key handling...\n");
    
    // 模拟按键1（启用自动调参）
    if (mpc_ui_handle_tuning_key(1)) {
        printf("   Key 1 handled successfully\n");
    }
    
    // 模拟按键2（配置界面）
    if (mpc_ui_handle_tuning_key(2)) {
        printf("   Key 2 handled successfully\n");
    }
    
    // 模拟按键4返回
    if (mpc_ui_handle_tuning_key(4)) {
        printf("   Key 4 handled successfully\n");
    }
    
    printf("? UI test completed! Check the IPS114 display.\n");
}

void test_mpc_auto_tuning_full_system(void)
{
    printf("=== MPC Auto-Tuning Full System Test ===\n");
    
    // 这是一个综合测试，模拟真实使用场景
    printf("Initializing full system test...\n");
    
    // 1. 系统初始化
    mpc_weights_t initial_weights;
    mpc_get_default_weights(&initial_weights);
    mpc_controller_init(&initial_weights);
    
    // 2. 配置自动调参系统
    mpc_auto_tuning_config_t config;
    mpc_get_default_tuning_config(&config);
    config.auto_tuning_enabled = true;
    config.evaluation_window = 30;
    config.tuning_interval = 60;
    config.performance_threshold = 1.5f;
    config.max_tuning_iterations = 15;
    
    mpc_auto_tuning_configure(&config);
    mpc_auto_tuning_enable(true);
    
    printf("Auto-tuning system configured and enabled\n");
    
    // 3. 生成复杂测试场景
    OptimalPathPoint complex_path[20];
    uint16_t path_length = 8;
    
    // 生成8字形路径
    for (int i = 0; i < path_length; i++) {
        float angle = (float)i / path_length * 4 * M_PI;
        complex_path[i].x = 800.0f * cosf(angle) + 800.0f;
        complex_path[i].y = 400.0f * sinf(2 * angle) + 400.0f;
        complex_path[i].heading = atan2f(800.0f * cosf(2 * angle), -800.0f * sinf(angle));
        complex_path[i].speed = 0.6f + 0.3f * sinf(angle); // 变速
    }
    
    printf("Complex 8-shaped path generated with variable speed\n");
    
    // 4. 运行完整的自动调参会话
    VehicleState vehicle_state = {0};
    vehicle_state.x = complex_path[0].x;
    vehicle_state.y = complex_path[0].y;
    vehicle_state.heading = complex_path[0].heading;
    
    float total_distance = 0.0f;
    float prev_x = vehicle_state.x, prev_y = vehicle_state.y;
    
    printf("Starting full system simulation...\n");
    
    for (int step = 0; step < 400; step++) {
        // 计算控制指令
        MotionCommand cmd = mpc_compute_control(&vehicle_state, complex_path, path_length, 400.0f);
        
        // 模拟车辆运动
        simulate_vehicle_motion(&vehicle_state, &cmd, 0.02f);
        
        // 计算行驶距离
        float dx = vehicle_state.x - prev_x;
        float dy = vehicle_state.y - prev_y;
        total_distance += sqrtf(dx * dx + dy * dy);
        prev_x = vehicle_state.x;
        prev_y = vehicle_state.y;
        
        // 每100步显示系统状态
        if (step % 100 == 0 && step > 0) {
            mpc_tuning_state_t tuning_state = mpc_get_tuning_state();
            const mpc_performance_t* perf = mpc_get_performance_stats();
            
            printf("Step %d: State=%d, Score=%.1f, Distance=%.1f mm\n", 
                   step, tuning_state, perf->overall_performance_score, total_distance);
            
            // 显示当前最优权重
            mpc_weights_t current_best;
            if (mpc_get_best_weights(&current_best)) {
                printf("  Current best Q_heading: %.2f\n", current_best.Q_heading);
            }
        }
    }
    
    // 5. 最终结果分析
    printf("\n=== Full System Test Results ===\n");
    
    const mpc_performance_t* final_perf = mpc_get_performance_stats();
    print_performance_summary("Full System", final_perf);
    
    printf("Total Distance Traveled: %.1f mm\n", total_distance);
    printf("Average Speed: %.2f m/s\n", total_distance / (400 * 0.02f * 1000.0f));
    
    // 权重优化结果
    mpc_weights_t final_weights;
    if (mpc_get_best_weights(&final_weights)) {
        printf("\n=== Final Optimized Weights ===\n");
        printf("Q_x: %.2f (initial: %.2f)\n", final_weights.Q_x, initial_weights.Q_x);
        printf("Q_y: %.2f (initial: %.2f)\n", final_weights.Q_y, initial_weights.Q_y);
        printf("Q_heading: %.2f (initial: %.2f)\n", final_weights.Q_heading, initial_weights.Q_heading);
        printf("Q_velocity: %.2f (initial: %.2f)\n", final_weights.Q_velocity, initial_weights.Q_velocity);
        printf("R_linear_vel: %.2f (initial: %.2f)\n", final_weights.R_linear_vel, initial_weights.R_linear_vel);
        printf("S_linear_vel: %.2f (initial: %.2f)\n", final_weights.S_linear_vel, initial_weights.S_linear_vel);
    }
    
    // 系统性能评估
    if (final_perf->overall_performance_score > 75.0f) {
        printf("? Excellent system performance achieved!\n");
    } else if (final_perf->overall_performance_score > 60.0f) {
        printf("? Good system performance achieved!\n");
    } else {
        printf("?? System performance needs improvement\n");
    }
    
    printf("? Full system test completed successfully!\n");
}
