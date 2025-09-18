/*********************************************************************************************************************
* 文件名称          mpc_controller.c
* 功能说明          基于模型预测控制(MPC)的导航控制器 实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本              备注
* 2025-09-18        LittleMaster        v1.0              创建MPC导航控制器实现
********************************************************************************************************************/

#include "mpc_controller.h"
#include "math_utils.h"
#include "zf_device_systick.h"
#include <string.h>

//================================================= 全局变量定义 =================================================
mpc_controller_t g_mpc_controller;

//================================================= 内部函数声明 =================================================
static void mpc_predict_trajectory(mpc_controller_t* mpc, const mpc_state_t* initial_state);
static void mpc_generate_reference_trajectory(mpc_controller_t* mpc, const VehicleState* current_state,
                                            const OptimalPathPoint* reference_path, uint16_t path_length,
                                            float lookahead_distance);
static float mpc_calculate_cost_function(const mpc_controller_t* mpc);
static void mpc_optimize_control_sequence(mpc_controller_t* mpc);
static void mpc_calculate_gradient(const mpc_controller_t* mpc, float gradient[][2]);
static void mpc_apply_constraints(mpc_control_t* control);
static void mpc_shift_control_sequence(mpc_controller_t* mpc);
static void mpc_state_transition(const mpc_state_t* current_state, const mpc_control_t* control, 
                                mpc_state_t* next_state, float dt);
static void vehicle_state_to_mpc_state(const VehicleState* vehicle_state, mpc_state_t* mpc_state);
static void mpc_control_to_motion_command(const mpc_control_t* mpc_control, MotionCommand* motion_cmd);
static float normalize_angle_mpc(float angle);
static float calculate_path_distance(float x1, float y1, float x2, float y2);

//================================================= 主要接口函数实现 =================================================

bool mpc_controller_init(const mpc_weights_t* weights)
{
    // 清零控制器结构
    memset(&g_mpc_controller, 0, sizeof(mpc_controller_t));
    
    // 设置权重参数
    if (weights != NULL) {
        memcpy(&g_mpc_controller.weights, weights, sizeof(mpc_weights_t));
    } else {
        // 使用默认权重
        mpc_get_default_weights(&g_mpc_controller.weights);
    }
    
    // 设置系统模型参数
    g_mpc_controller.wheelbase = 150.0f; // 150mm轮距
    g_mpc_controller.velocity_time_constant = 0.2f;        // 速度响应时间常数
    g_mpc_controller.angular_velocity_time_constant = 0.15f; // 角速度响应时间常数
    
    // 初始化控制序列（零初始化）
    memset(g_mpc_controller.prediction.control_sequence, 0, 
           sizeof(g_mpc_controller.prediction.control_sequence));
    
    g_mpc_controller.initialized = true;
    return true;
}

MotionCommand mpc_compute_control(const VehicleState* current_state,
                                 const OptimalPathPoint* reference_path,
                                 uint16_t path_length,
                                 float lookahead_distance)
{
    MotionCommand result = {0};
    
    if (!g_mpc_controller.initialized || current_state == NULL || 
        reference_path == NULL || path_length == 0) {
        return result;
    }
    
    uint32_t start_time = systick_get_time_ms() * 1000; // 转换为微秒
    
    // 1. 转换车辆状态为MPC状态
    mpc_state_t mpc_current_state;
    vehicle_state_to_mpc_state(current_state, &mpc_current_state);
    
    // 2. 生成参考轨迹
    mpc_generate_reference_trajectory(&g_mpc_controller, current_state, reference_path, 
                                    path_length, lookahead_distance);
    
    // 3. 状态预测
    mpc_predict_trajectory(&g_mpc_controller, &mpc_current_state);
    
    // 4. 控制序列优化
    mpc_optimize_control_sequence(&g_mpc_controller);
    
    // 5. 提取第一个控制量
    mpc_control_to_motion_command(&g_mpc_controller.prediction.control_sequence[0], &result);
    
    // 6. 为下一步做准备（滚动优化）
    mpc_shift_control_sequence(&g_mpc_controller);
    
    // 7. 性能统计
    uint32_t end_time = systick_get_time_ms() * 1000;
    g_mpc_controller.performance.computation_time_us = end_time - start_time;
    if (g_mpc_controller.performance.computation_time_us > g_mpc_controller.performance.max_computation_time_us) {
        g_mpc_controller.performance.max_computation_time_us = g_mpc_controller.performance.computation_time_us;
    }
    g_mpc_controller.performance.cpu_usage_percent = 
        (float)g_mpc_controller.performance.computation_time_us / 20000.0f * 100.0f; // 20ms周期
    
    g_mpc_controller.control_step_count++;
    g_mpc_controller.last_control = g_mpc_controller.prediction.control_sequence[0];
    
    // 8. 执行自动调参步骤 (在控制计算完成后)
    mpc_auto_tuning_step(current_state, reference_path, path_length);
    
    return result;
}

const mpc_performance_t* mpc_get_performance_stats(void)
{
    return &g_mpc_controller.performance;
}

bool mpc_set_weights(const mpc_weights_t* weights)
{
    if (weights == NULL) {
        return false;
    }
    
    memcpy(&g_mpc_controller.weights, weights, sizeof(mpc_weights_t));
    return true;
}

void mpc_reset_controller(void)
{
    // 重置控制序列
    memset(g_mpc_controller.prediction.control_sequence, 0, 
           sizeof(g_mpc_controller.prediction.control_sequence));
    
    // 重置性能统计
    memset(&g_mpc_controller.performance, 0, sizeof(mpc_performance_t));
    
    // 重置计数器
    g_mpc_controller.control_step_count = 0;
}

//================================================= 核心算法实现 =================================================

static void mpc_predict_trajectory(mpc_controller_t* mpc, const mpc_state_t* initial_state)
{
    // 将初始状态作为预测轨迹的起点
    mpc->prediction.predicted_states[0] = *initial_state;
    
    // 逐步预测未来状态
    for (int i = 0; i < MPC_PREDICTION_HORIZON - 1; i++) {
        int control_index = (i < MPC_CONTROL_HORIZON) ? i : (MPC_CONTROL_HORIZON - 1);
        
        mpc_state_transition(&mpc->prediction.predicted_states[i],
                           &mpc->prediction.control_sequence[control_index],
                           &mpc->prediction.predicted_states[i + 1],
                           MPC_SAMPLING_TIME);
    }
}

static void mpc_generate_reference_trajectory(mpc_controller_t* mpc, const VehicleState* current_state,
                                            const OptimalPathPoint* reference_path, uint16_t path_length,
                                            float lookahead_distance)
{
    if (path_length == 0) return;
    
    // 找到最近的路径点
    int closest_index = 0;
    float min_distance = 1e10f;
    
    for (int i = 0; i < path_length; i++) {
        float dx = reference_path[i].x - current_state->x;
        float dy = reference_path[i].y - current_state->y;
        float distance = sqrtf(dx * dx + dy * dy);
        
        if (distance < min_distance) {
            min_distance = distance;
            closest_index = i;
        }
    }
    
    // 生成预测地平线内的参考轨迹
    for (int i = 0; i < MPC_PREDICTION_HORIZON; i++) {
        // 计算前瞻点索引
        float prediction_distance = lookahead_distance + i * current_state->linear_speed * MPC_SAMPLING_TIME * 1000.0f;
        int reference_index = closest_index;
        
        // 寻找适当前瞻距离的点
        float accumulated_distance = 0;
        for (int j = closest_index; j < path_length - 1; j++) {
            float segment_distance = calculate_path_distance(reference_path[j].x, reference_path[j].y,
                                                           reference_path[j+1].x, reference_path[j+1].y);
            accumulated_distance += segment_distance;
            
            if (accumulated_distance >= prediction_distance) {
                reference_index = j + 1;
                break;
            }
        }
        
        // 环形路径处理
        if (reference_index >= path_length) {
            reference_index = reference_index % path_length;
        }
        
        // 填充参考轨迹
        mpc->prediction.reference_trajectory[i].x = reference_path[reference_index].x;
        mpc->prediction.reference_trajectory[i].y = reference_path[reference_index].y;
        mpc->prediction.reference_trajectory[i].heading = reference_path[reference_index].heading;
        mpc->prediction.reference_trajectory[i].linear_velocity = reference_path[reference_index].reference_speed;
        mpc->prediction.reference_trajectory[i].angular_velocity = 0.0f; // 从路径推导
    }
}

static float mpc_calculate_cost_function(const mpc_controller_t* mpc)
{
    float total_cost = 0.0f;
    const mpc_weights_t* w = &mpc->weights;
    const mpc_prediction_data_t* pred = &mpc->prediction;
    
    // 1. 轨迹跟踪误差代价
    for (int i = 0; i < MPC_PREDICTION_HORIZON; i++) {
        float dx = pred->predicted_states[i].x - pred->reference_trajectory[i].x;
        float dy = pred->predicted_states[i].y - pred->reference_trajectory[i].y;
        float dheading = normalize_angle_mpc(pred->predicted_states[i].heading - pred->reference_trajectory[i].heading);
        float dv = pred->predicted_states[i].linear_velocity - pred->reference_trajectory[i].linear_velocity;
        float domega = pred->predicted_states[i].angular_velocity - pred->reference_trajectory[i].angular_velocity;
        
        // 状态误差权重
        float state_cost = w->Q_x * dx * dx +
                          w->Q_y * dy * dy +
                          w->Q_heading * dheading * dheading +
                          w->Q_velocity * dv * dv +
                          w->Q_angular_vel * domega * domega;
        
        // 终端权重（最后一步加权更大）
        if (i == MPC_PREDICTION_HORIZON - 1) {
            state_cost += w->P_x * dx * dx +
                         w->P_y * dy * dy +
                         w->P_heading * dheading * dheading;
        }
        
        total_cost += state_cost;
    }
    
    // 2. 控制量代价（能耗最小化）
    for (int i = 0; i < MPC_CONTROL_HORIZON; i++) {
        float u_v = pred->control_sequence[i].target_linear_velocity;
        float u_omega = pred->control_sequence[i].target_angular_velocity;
        
        total_cost += w->R_linear_vel * u_v * u_v +
                     w->R_angular_vel * u_omega * u_omega;
    }
    
    // 3. 控制变化率代价（平滑性）
    for (int i = 1; i < MPC_CONTROL_HORIZON; i++) {
        float delta_v = pred->control_sequence[i].target_linear_velocity - pred->control_sequence[i-1].target_linear_velocity;
        float delta_omega = pred->control_sequence[i].target_angular_velocity - pred->control_sequence[i-1].target_angular_velocity;
        
        total_cost += w->S_linear_vel * delta_v * delta_v +
                     w->S_angular_vel * delta_omega * delta_omega;
    }
    
    return total_cost;
}

static void mpc_optimize_control_sequence(mpc_controller_t* mpc)
{
    float gradient[MPC_CONTROL_HORIZON][2];
    float current_cost, previous_cost = 1e10f;
    
    mpc->prediction.iteration_count = 0;
    mpc->prediction.converged = false;
    
    for (int iter = 0; iter < MPC_MAX_ITERATIONS; iter++) {
        // 计算当前代价
        current_cost = mpc_calculate_cost_function(mpc);
        
        // 收敛检查
        if (iter > 0 && fabsf(current_cost - previous_cost) < MPC_CONVERGENCE_TOLERANCE) {
            mpc->prediction.converged = true;
            break;
        }
        
        // 计算梯度
        mpc_calculate_gradient(mpc, gradient);
        
        // 梯度下降更新
        for (int i = 0; i < MPC_CONTROL_HORIZON; i++) {
            mpc->prediction.control_sequence[i].target_linear_velocity -= 
                MPC_GRADIENT_STEP_SIZE * gradient[i][0];
            mpc->prediction.control_sequence[i].target_angular_velocity -= 
                MPC_GRADIENT_STEP_SIZE * gradient[i][1];
            
            // 应用约束
            mpc_apply_constraints(&mpc->prediction.control_sequence[i]);
        }
        
        // 重新预测轨迹
        mpc_predict_trajectory(mpc, &mpc->prediction.predicted_states[0]);
        
        previous_cost = current_cost;
        mpc->prediction.iteration_count++;
    }
    
    mpc->prediction.cost_function_value = current_cost;
    
    // 统计收敛失败次数
    if (!mpc->prediction.converged) {
        mpc->performance.convergence_failures++;
    }
    
    mpc->performance.total_iterations += mpc->prediction.iteration_count;
}

static void mpc_calculate_gradient(const mpc_controller_t* mpc, float gradient[][2])
{
    const float epsilon = 0.001f; // 数值微分步长
    mpc_controller_t temp_mpc = *mpc; // 临时拷贝
    
    float original_cost = mpc_calculate_cost_function(mpc);
    
    for (int i = 0; i < MPC_CONTROL_HORIZON; i++) {
        // 对线速度求偏导
        temp_mpc.prediction.control_sequence[i].target_linear_velocity += epsilon;
        mpc_predict_trajectory(&temp_mpc, &temp_mpc.prediction.predicted_states[0]);
        float cost_plus = mpc_calculate_cost_function(&temp_mpc);
        gradient[i][0] = (cost_plus - original_cost) / epsilon;
        temp_mpc.prediction.control_sequence[i].target_linear_velocity -= epsilon; // 恢复
        
        // 对角速度求偏导
        temp_mpc.prediction.control_sequence[i].target_angular_velocity += epsilon;
        mpc_predict_trajectory(&temp_mpc, &temp_mpc.prediction.predicted_states[0]);
        cost_plus = mpc_calculate_cost_function(&temp_mpc);
        gradient[i][1] = (cost_plus - original_cost) / epsilon;
        temp_mpc.prediction.control_sequence[i].target_angular_velocity -= epsilon; // 恢复
    }
}

//================================================= 辅助函数实现 =================================================

static void mpc_apply_constraints(mpc_control_t* control)
{
    // 线速度约束
    if (control->target_linear_velocity > MPC_MAX_LINEAR_VELOCITY) {
        control->target_linear_velocity = MPC_MAX_LINEAR_VELOCITY;
    }
    if (control->target_linear_velocity < MPC_MIN_LINEAR_VELOCITY) {
        control->target_linear_velocity = MPC_MIN_LINEAR_VELOCITY;
    }
    
    // 角速度约束
    if (control->target_angular_velocity > MPC_MAX_ANGULAR_VELOCITY) {
        control->target_angular_velocity = MPC_MAX_ANGULAR_VELOCITY;
    }
    if (control->target_angular_velocity < MPC_MIN_ANGULAR_VELOCITY) {
        control->target_angular_velocity = MPC_MIN_ANGULAR_VELOCITY;
    }
}

static void mpc_shift_control_sequence(mpc_controller_t* mpc)
{
    // 控制序列向前滚动一步
    for (int i = 0; i < MPC_CONTROL_HORIZON - 1; i++) {
        mpc->prediction.control_sequence[i] = mpc->prediction.control_sequence[i + 1];
    }
    // 最后一个控制量保持不变
}

static void mpc_state_transition(const mpc_state_t* current_state, const mpc_control_t* control, 
                                mpc_state_t* next_state, float dt)
{
    // 非线性车辆运动学模型 + 一阶动态响应
    float v = current_state->linear_velocity;
    float omega = current_state->angular_velocity;
    float u_v = control->target_linear_velocity;
    float u_omega = control->target_angular_velocity;
    
    // 位置更新（运动学）
    next_state->x = current_state->x + v * cosf(current_state->heading) * dt * 1000.0f; // 转换为mm
    next_state->y = current_state->y + v * sinf(current_state->heading) * dt * 1000.0f;
    next_state->heading = normalize_angle_mpc(current_state->heading + omega * dt);
    
    // 速度更新（一阶动态）
    float tau_v = g_mpc_controller.velocity_time_constant;
    float tau_omega = g_mpc_controller.angular_velocity_time_constant;
    
    next_state->linear_velocity = v + (u_v - v) * dt / tau_v;
    next_state->angular_velocity = omega + (u_omega - omega) * dt / tau_omega;
}

static void vehicle_state_to_mpc_state(const VehicleState* vehicle_state, mpc_state_t* mpc_state)
{
    mpc_state->x = vehicle_state->x;
    mpc_state->y = vehicle_state->y;
    mpc_state->heading = vehicle_state->heading;
    mpc_state->linear_velocity = vehicle_state->linear_speed;
    mpc_state->angular_velocity = 0.0f; // 需要从历史数据或编码器推导
}

static void mpc_control_to_motion_command(const mpc_control_t* mpc_control, MotionCommand* motion_cmd)
{
    motion_cmd->desired_linear_speed = mpc_control->target_linear_velocity;
    motion_cmd->desired_angular_speed = mpc_control->target_angular_velocity;
}

static float normalize_angle_mpc(float angle)
{
    return normalize_angle(angle); // 使用已有的角度归一化函数
}

static float calculate_path_distance(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

void mpc_get_default_weights(mpc_weights_t* weights)
{
    // 经过调试的默认权重参数
    weights->Q_x = 10.0f;           // 位置误差权重
    weights->Q_y = 10.0f;
    weights->Q_heading = 8.0f;      // 航向误差权重  
    weights->Q_velocity = 1.0f;     // 速度误差权重
    weights->Q_angular_vel = 1.0f;  // 角速度误差权重
    
    weights->R_linear_vel = 0.1f;   // 控制量权重（能耗）
    weights->R_angular_vel = 0.1f;
    
    weights->S_linear_vel = 2.0f;   // 平滑性权重
    weights->S_angular_vel = 2.0f;
    
    weights->P_x = 15.0f;           // 终端权重（提高稳定性）
    weights->P_y = 15.0f;
    weights->P_heading = 10.0f;
}

void mpc_print_debug_info(void)
{
    // 输出MPC状态信息，用于调试
    printf("MPC Debug Info:\n");
    printf("  Control Steps: %lu\n", g_mpc_controller.control_step_count);
    printf("  Last Computation: %lu us\n", g_mpc_controller.performance.computation_time_us);
    printf("  Max Computation: %lu us\n", g_mpc_controller.performance.max_computation_time_us);
    printf("  CPU Usage: %.1f%%\n", g_mpc_controller.performance.cpu_usage_percent);
    printf("  Convergence Failures: %lu\n", g_mpc_controller.performance.convergence_failures);
    printf("  Last Cost: %.3f\n", g_mpc_controller.prediction.cost_function_value);
    printf("  Last Iterations: %lu\n", g_mpc_controller.prediction.iteration_count);
    printf("  Converged: %s\n", g_mpc_controller.prediction.converged ? "Yes" : "No");
    
    // 显示性能统计
    printf("Performance Stats:\n");
    printf("  Position RMS Error: %.2f mm\n", g_mpc_controller.performance.position_error_rms);
    printf("  Heading RMS Error: %.4f rad\n", g_mpc_controller.performance.heading_error_rms);
    printf("  Performance Score: %.1f\n", g_mpc_controller.performance.overall_performance_score);
    
    // 显示自动调参状态
    if (g_mpc_controller.auto_tuning.config.auto_tuning_enabled) {
        printf("Auto Tuning: %s\n", 
               g_mpc_controller.auto_tuning.state == TUNING_STATE_IDLE ? "Idle" :
               g_mpc_controller.auto_tuning.state == TUNING_STATE_EVALUATING ? "Evaluating" :
               g_mpc_controller.auto_tuning.state == TUNING_STATE_OPTIMIZING ? "Optimizing" :
               g_mpc_controller.auto_tuning.state == TUNING_STATE_CONVERGED ? "Converged" : "Failed");
        printf("  Best Score: %.1f\n", g_mpc_controller.auto_tuning.best_performance_score);
        printf("  Tuning Steps: %lu\n", g_mpc_controller.auto_tuning.tuning_step_count);
    }
}

//================================================= 自动调参功能实现 =================================================

//-------------------------------------------------------------------------------------------------------------------
// 内部自动调参函数声明
//-------------------------------------------------------------------------------------------------------------------
static void mpc_update_performance_stats(const VehicleState* current_state, 
                                        const OptimalPathPoint* reference_path, 
                                        uint16_t path_length);
static float mpc_calculate_performance_score(void);
static void mpc_analyze_path_characteristics(const OptimalPathPoint* reference_path, 
                                           uint16_t path_length, 
                                           const VehicleState* current_state);
static bool mpc_should_tune_now(void);
static void mpc_estimate_gradient(void);
static void mpc_update_weights_with_gradient(void);
static void mpc_apply_adaptive_weights(void);
static float mpc_get_weight_by_index(const mpc_weights_t* weights, int index);
static void mpc_set_weight_by_index(mpc_weights_t* weights, int index, float value);
static void mpc_backup_current_weights(void);
static void mpc_restore_best_weights(void);

bool mpc_auto_tuning_enable(bool enable)
{
    if (!g_mpc_controller.initialized) {
        return false;
    }
    
    g_mpc_controller.auto_tuning.config.auto_tuning_enabled = enable;
    
    if (enable) {
        // 初始化自动调参
        g_mpc_controller.auto_tuning.state = TUNING_STATE_IDLE;
        g_mpc_controller.auto_tuning.tuning_step_count = 0;
        g_mpc_controller.auto_tuning.evaluation_count = 0;
        g_mpc_controller.auto_tuning.best_performance_score = 0.0f;
        
        // 保存当前权重作为初始最优权重
        memcpy(&g_mpc_controller.auto_tuning.best_weights, &g_mpc_controller.weights, 
               sizeof(mpc_weights_t));
        memcpy(&g_mpc_controller.auto_tuning.current_weights, &g_mpc_controller.weights, 
               sizeof(mpc_weights_t));
        
        // 清零梯度和动量
        memset(g_mpc_controller.auto_tuning.gradient, 0, sizeof(g_mpc_controller.auto_tuning.gradient));
        memset(g_mpc_controller.auto_tuning.momentum, 0, sizeof(g_mpc_controller.auto_tuning.momentum));
        memset(g_mpc_controller.auto_tuning.performance_history, 0, 
               sizeof(g_mpc_controller.auto_tuning.performance_history));
        
        printf("MPC Auto-tuning enabled\n");
    } else {
        g_mpc_controller.auto_tuning.state = TUNING_STATE_IDLE;
        printf("MPC Auto-tuning disabled\n");
    }
    
    return true;
}

bool mpc_auto_tuning_configure(const mpc_auto_tuning_config_t* config)
{
    if (config == NULL) {
        return false;
    }
    
    memcpy(&g_mpc_controller.auto_tuning.config, config, sizeof(mpc_auto_tuning_config_t));
    return true;
}

void mpc_get_default_tuning_config(mpc_auto_tuning_config_t* config)
{
    if (config == NULL) return;
    
    config->auto_tuning_enabled = false;        // 默认禁用
    config->tuning_step_size = 0.1f;            // 10% 参数调整步长
    config->performance_threshold = 2.0f;       // 2% 性能改善阈值
    config->evaluation_window = 50;             // 50个控制周期评估窗口
    config->tuning_interval = 100;              // 每100步进行一次调参
    config->convergence_tolerance = 0.5f;       // 0.5% 收敛容差
    config->max_tuning_iterations = 10;         // 最大调参迭代次数
}

bool mpc_auto_tuning_step(const VehicleState* current_state, 
                          const OptimalPathPoint* reference_path, 
                          uint16_t path_length)
{
    if (!g_mpc_controller.auto_tuning.config.auto_tuning_enabled || 
        current_state == NULL || reference_path == NULL) {
        return false;
    }
    
    // 1. 更新性能统计
    mpc_update_performance_stats(current_state, reference_path, path_length);
    
    // 2. 分析路径特征
    mpc_analyze_path_characteristics(reference_path, path_length, current_state);
    
    // 3. 应用工况自适应权重
    mpc_apply_adaptive_weights();
    
    // 4. 判断是否需要进行参数优化
    if (!mpc_should_tune_now()) {
        return false;
    }
    
    bool parameters_changed = false;
    
    switch (g_mpc_controller.auto_tuning.state) {
        case TUNING_STATE_IDLE:
            // 开始新的调参周期
            g_mpc_controller.auto_tuning.state = TUNING_STATE_EVALUATING;
            g_mpc_controller.auto_tuning.evaluation_count = 0;
            mpc_backup_current_weights();
            break;
            
        case TUNING_STATE_EVALUATING:
            // 评估当前参数性能
            g_mpc_controller.auto_tuning.evaluation_count++;
            
            if (g_mpc_controller.auto_tuning.evaluation_count >= 
                g_mpc_controller.auto_tuning.config.evaluation_window) {
                
                float current_score = mpc_calculate_performance_score();
                
                // 记录性能历史
                for (int i = 9; i > 0; i--) {
                    g_mpc_controller.auto_tuning.performance_history[i] = 
                        g_mpc_controller.auto_tuning.performance_history[i-1];
                }
                g_mpc_controller.auto_tuning.performance_history[0] = current_score;
                
                // 判断是否需要优化
                if (current_score > g_mpc_controller.auto_tuning.best_performance_score + 
                    g_mpc_controller.auto_tuning.config.performance_threshold) {
                    // 性能有改善，保存为最优参数
                    g_mpc_controller.auto_tuning.best_performance_score = current_score;
                    memcpy(&g_mpc_controller.auto_tuning.best_weights, &g_mpc_controller.weights, 
                           sizeof(mpc_weights_t));
                    g_mpc_controller.auto_tuning.state = TUNING_STATE_CONVERGED;
                } else if (g_mpc_controller.auto_tuning.tuning_step_count < 
                          g_mpc_controller.auto_tuning.config.max_tuning_iterations) {
                    // 性能未改善，开始优化
                    g_mpc_controller.auto_tuning.state = TUNING_STATE_OPTIMIZING;
                } else {
                    // 达到最大迭代次数，结束调参
                    mpc_restore_best_weights();
                    g_mpc_controller.auto_tuning.state = TUNING_STATE_CONVERGED;
                }
            }
            break;
            
        case TUNING_STATE_OPTIMIZING:
            // 执行参数优化
            mpc_estimate_gradient();
            mpc_update_weights_with_gradient();
            g_mpc_controller.auto_tuning.tuning_step_count++;
            g_mpc_controller.auto_tuning.state = TUNING_STATE_EVALUATING;
            g_mpc_controller.auto_tuning.evaluation_count = 0;
            parameters_changed = true;
            break;
            
        case TUNING_STATE_CONVERGED:
            // 调参收敛，重置状态等待下次调参
            g_mpc_controller.auto_tuning.state = TUNING_STATE_IDLE;
            g_mpc_controller.auto_tuning.tuning_step_count = 0;
            break;
            
        case TUNING_STATE_FAILED:
            // 调参失败，恢复最优参数并重置
            mpc_restore_best_weights();
            g_mpc_controller.auto_tuning.state = TUNING_STATE_IDLE;
            g_mpc_controller.auto_tuning.tuning_step_count = 0;
            break;
    }
    
    return parameters_changed;
}

mpc_tuning_state_t mpc_get_tuning_state(void)
{
    return g_mpc_controller.auto_tuning.state;
}

bool mpc_get_best_weights(mpc_weights_t* weights)
{
    if (weights == NULL || g_mpc_controller.auto_tuning.best_performance_score <= 0.0f) {
        return false;
    }
    
    memcpy(weights, &g_mpc_controller.auto_tuning.best_weights, sizeof(mpc_weights_t));
    return true;
}

bool mpc_set_best_weights(const mpc_weights_t* weights)
{
    if (weights == NULL) {
        return false;
    }
    
    memcpy(&g_mpc_controller.auto_tuning.best_weights, weights, sizeof(mpc_weights_t));
    memcpy(&g_mpc_controller.weights, weights, sizeof(mpc_weights_t));
    g_mpc_controller.auto_tuning.best_performance_score = 100.0f; // 标记为有效
    return true;
}

//================================================= 自动调参内部函数实现 =================================================

static void mpc_update_performance_stats(const VehicleState* current_state, 
                                        const OptimalPathPoint* reference_path, 
                                        uint16_t path_length)
{
    if (current_state == NULL || reference_path == NULL || path_length == 0) {
        return;
    }
    
    // 找到最近的参考点
    float min_distance = 1e6f;
    int closest_index = 0;
    
    for (int i = 0; i < path_length; i++) {
        float dx = current_state->x - reference_path[i].x;
        float dy = current_state->y - reference_path[i].y;
        float distance = sqrtf(dx * dx + dy * dy);
        
        if (distance < min_distance) {
            min_distance = distance;
            closest_index = i;
        }
    }
    
    // 计算位置误差
    float pos_error_x = current_state->x - reference_path[closest_index].x;
    float pos_error_y = current_state->y - reference_path[closest_index].y;
    float position_error = sqrtf(pos_error_x * pos_error_x + pos_error_y * pos_error_y);
    
    // 计算航向误差
    float heading_error = normalize_angle_mpc(current_state->heading - reference_path[closest_index].heading);
    
    // 累积误差统计
    g_mpc_controller.error_accumulator_x += pos_error_x * pos_error_x;
    g_mpc_controller.error_accumulator_y += pos_error_y * pos_error_y;
    g_mpc_controller.error_accumulator_heading += heading_error * heading_error;
    
    // 计算控制变化率
    static mpc_control_t previous_control = {0};
    float control_change_linear = fabsf(g_mpc_controller.last_control.target_linear_velocity - 
                                       previous_control.target_linear_velocity);
    float control_change_angular = fabsf(g_mpc_controller.last_control.target_angular_velocity - 
                                        previous_control.target_angular_velocity);
    
    g_mpc_controller.control_change_accumulator_linear += control_change_linear;
    g_mpc_controller.control_change_accumulator_angular += control_change_angular;
    
    previous_control = g_mpc_controller.last_control;
    
    // 更新统计样本数
    g_mpc_controller.performance.samples_count++;
    
    // 计算RMS误差 (每N步更新一次)
    if (g_mpc_controller.performance.samples_count % 20 == 0) {
        uint32_t n = g_mpc_controller.performance.samples_count;
        
        g_mpc_controller.performance.position_error_rms = 
            sqrtf((g_mpc_controller.error_accumulator_x + g_mpc_controller.error_accumulator_y) / n);
        g_mpc_controller.performance.heading_error_rms = 
            sqrtf(g_mpc_controller.error_accumulator_heading / n);
        g_mpc_controller.performance.control_smoothness_linear = 
            g_mpc_controller.control_change_accumulator_linear / n;
        g_mpc_controller.performance.control_smoothness_angular = 
            g_mpc_controller.control_change_accumulator_angular / n;
        
        // 更新最大误差
        if (position_error > g_mpc_controller.performance.max_position_error) {
            g_mpc_controller.performance.max_position_error = position_error;
        }
        if (fabsf(heading_error) > g_mpc_controller.performance.max_heading_error) {
            g_mpc_controller.performance.max_heading_error = fabsf(heading_error);
        }
    }
}

static float mpc_calculate_performance_score(void)
{
    // 综合性能评分 (0-100分)
    float score = 100.0f;
    
    // 位置精度评分 (权重50%)
    float position_penalty = g_mpc_controller.performance.position_error_rms / 10.0f; // 10mm为基准
    score -= position_penalty * 50.0f;
    
    // 航向精度评分 (权重30%)
    float heading_penalty = g_mpc_controller.performance.heading_error_rms / 0.1f; // 0.1rad为基准
    score -= heading_penalty * 30.0f;
    
    // 控制平滑性评分 (权重15%)
    float smoothness_penalty = (g_mpc_controller.performance.control_smoothness_linear + 
                               g_mpc_controller.performance.control_smoothness_angular) / 2.0f;
    score -= smoothness_penalty * 15.0f;
    
    // 计算效率评分 (权重5%)
    float efficiency_penalty = (float)g_mpc_controller.performance.computation_time_us / 1000.0f; // 1ms为基准
    score -= efficiency_penalty * 5.0f;
    
    // 确保评分在合理范围内
    if (score < 0.0f) score = 0.0f;
    if (score > 100.0f) score = 100.0f;
    
    g_mpc_controller.performance.overall_performance_score = score;
    return score;
}

static void mpc_analyze_path_characteristics(const OptimalPathPoint* reference_path, 
                                           uint16_t path_length, 
                                           const VehicleState* current_state)
{
    if (reference_path == NULL || path_length < 3 || current_state == NULL) {
        return;
    }
    
    // 更新当前速度
    g_mpc_controller.auto_tuning.current_speed = current_state->linear_speed;
    
    // 分析路径曲率 (前瞻5个点)
    int analysis_points = (path_length < 5) ? path_length : 5;
    float total_curvature = 0.0f;
    
    for (int i = 1; i < analysis_points - 1; i++) {
        // 计算曲率 (简化版本)
        float dx1 = reference_path[i].x - reference_path[i-1].x;
        float dy1 = reference_path[i].y - reference_path[i-1].y;
        float dx2 = reference_path[i+1].x - reference_path[i].x;
        float dy2 = reference_path[i+1].y - reference_path[i].y;
        
        float heading1 = atan2f(dy1, dx1);
        float heading2 = atan2f(dy2, dx2);
        float curvature = fabsf(normalize_angle_mpc(heading2 - heading1));
        
        total_curvature += curvature;
    }
    
    g_mpc_controller.auto_tuning.path_curvature = total_curvature / (analysis_points - 2);
    
    // 判断路径类型
    g_mpc_controller.auto_tuning.is_straight_line = (g_mpc_controller.auto_tuning.path_curvature < 0.1f);
    g_mpc_controller.auto_tuning.is_sharp_turn = (g_mpc_controller.auto_tuning.path_curvature > 0.5f);
}

static bool mpc_should_tune_now(void)
{
    // 检查调参间隔
    if (g_mpc_controller.control_step_count % g_mpc_controller.auto_tuning.config.tuning_interval != 0) {
        return false;
    }
    
    // 检查是否有足够的性能数据
    if (g_mpc_controller.performance.samples_count < 20) {
        return false;
    }
    
    // 避免在急转弯时调参
    if (g_mpc_controller.auto_tuning.is_sharp_turn) {
        return false;
    }
    
    return true;
}

static void mpc_estimate_gradient(void)
{
    // 使用有限差分估计梯度
    float perturbation = g_mpc_controller.auto_tuning.config.tuning_step_size * 0.1f;
    float baseline_score = mpc_calculate_performance_score();
    
    // 计算每个权重参数的梯度
    for (int i = 0; i < 9; i++) {
        float original_weight = mpc_get_weight_by_index(&g_mpc_controller.weights, i);
        
        // 向上扰动
        mpc_set_weight_by_index(&g_mpc_controller.weights, i, original_weight + perturbation);
        float score_up = mpc_calculate_performance_score();
        
        // 向下扰动
        mpc_set_weight_by_index(&g_mpc_controller.weights, i, original_weight - perturbation);
        float score_down = mpc_calculate_performance_score();
        
        // 计算梯度
        g_mpc_controller.auto_tuning.gradient[i] = (score_up - score_down) / (2.0f * perturbation);
        
        // 恢复原值
        mpc_set_weight_by_index(&g_mpc_controller.weights, i, original_weight);
    }
}

static void mpc_update_weights_with_gradient(void)
{
    float learning_rate = g_mpc_controller.auto_tuning.config.tuning_step_size;
    float momentum_factor = 0.9f; // 动量系数
    
    for (int i = 0; i < 9; i++) {
        // 动量更新
        g_mpc_controller.auto_tuning.momentum[i] = momentum_factor * g_mpc_controller.auto_tuning.momentum[i] + 
                                                  learning_rate * g_mpc_controller.auto_tuning.gradient[i];
        
        // 更新权重
        float current_weight = mpc_get_weight_by_index(&g_mpc_controller.weights, i);
        float new_weight = current_weight + g_mpc_controller.auto_tuning.momentum[i];
        
        // 应用权重约束
        if (new_weight < 0.01f) new_weight = 0.01f;  // 最小权重
        if (new_weight > 100.0f) new_weight = 100.0f; // 最大权重
        
        mpc_set_weight_by_index(&g_mpc_controller.weights, i, new_weight);
    }
}

static void mpc_apply_adaptive_weights(void)
{
    // 根据工况自适应调整权重
    mpc_weights_t* weights = &g_mpc_controller.weights;
    float speed = g_mpc_controller.auto_tuning.current_speed;
    
    // 速度自适应因子
    float speed_factor = 1.0f;
    if (speed < 0.5f) {
        // 低速：强调精度
        speed_factor = 1.2f;
    } else if (speed > 1.0f) {
        // 高速：强调稳定性
        speed_factor = 0.8f;
    }
    
    // 路径特征自适应
    if (g_mpc_controller.auto_tuning.is_straight_line) {
        // 直线段：降低航向权重，提高速度跟踪
        weights->Q_heading *= 0.9f;
        weights->Q_velocity *= 1.1f;
    } else if (g_mpc_controller.auto_tuning.is_sharp_turn) {
        // 急转弯：提高航向权重，增加平滑性
        weights->Q_heading *= 1.2f;
        weights->S_angular_vel *= 1.3f;
    }
    
    // 应用速度因子
    weights->Q_x *= speed_factor;
    weights->Q_y *= speed_factor;
}

static float mpc_get_weight_by_index(const mpc_weights_t* weights, int index)
{
    switch (index) {
        case 0: return weights->Q_x;
        case 1: return weights->Q_y;
        case 2: return weights->Q_heading;
        case 3: return weights->Q_velocity;
        case 4: return weights->Q_angular_vel;
        case 5: return weights->R_linear_vel;
        case 6: return weights->R_angular_vel;
        case 7: return weights->S_linear_vel;
        case 8: return weights->S_angular_vel;
        default: return 0.0f;
    }
}

static void mpc_set_weight_by_index(mpc_weights_t* weights, int index, float value)
{
    switch (index) {
        case 0: weights->Q_x = value; break;
        case 1: weights->Q_y = value; break;
        case 2: weights->Q_heading = value; break;
        case 3: weights->Q_velocity = value; break;
        case 4: weights->Q_angular_vel = value; break;
        case 5: weights->R_linear_vel = value; break;
        case 6: weights->R_angular_vel = value; break;
        case 7: weights->S_linear_vel = value; break;
        case 8: weights->S_angular_vel = value; break;
    }
}

static void mpc_backup_current_weights(void)
{
    memcpy(&g_mpc_controller.auto_tuning.current_weights, &g_mpc_controller.weights, 
           sizeof(mpc_weights_t));
}

static void mpc_restore_best_weights(void)
{
    memcpy(&g_mpc_controller.weights, &g_mpc_controller.auto_tuning.best_weights, 
           sizeof(mpc_weights_t));
}
