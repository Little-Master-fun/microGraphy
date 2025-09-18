/*********************************************************************************************************************
* 文件名称          mpc_controller.h
* 功能说明          基于模型预测控制(MPC)的导航控制器 头文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本              备注
* 2025-09-18        LittleMaster        v1.0              创建MPC导航控制器
*
* 文件作用说明：
* 本文件定义了基于MPC的路径跟踪控制器，实现预测性最优控制。
********************************************************************************************************************/

#ifndef _MPC_CONTROLLER_H_
#define _MPC_CONTROLLER_H_

#include "zf_common_typedef.h"
#include "navigation_flash_improved.h"
#include <math.h>

//================================================= 宏定义 =================================================

// MPC预测参数
#define MPC_PREDICTION_HORIZON      (8)     // 预测步数（平衡精度和计算量）
#define MPC_CONTROL_HORIZON         (4)     // 控制步数
#define MPC_SAMPLING_TIME           (0.02f) // 采样时间 20ms
#define MPC_MAX_ITERATIONS          (15)    // 最大优化迭代次数

// 系统约束
#define MPC_MAX_LINEAR_VELOCITY     (3.0f)  // 最大线速度 m/s
#define MPC_MIN_LINEAR_VELOCITY     (-1.0f) // 最小线速度 m/s
#define MPC_MAX_ANGULAR_VELOCITY    (4.0f)  // 最大角速度 rad/s
#define MPC_MIN_ANGULAR_VELOCITY    (-4.0f) // 最小角速度 rad/s
#define MPC_MAX_ACCELERATION        (3.0f)  // 最大线加速度 m/s?
#define MPC_MAX_ANGULAR_ACCEL       (5.0f)  // 最大角加速度 rad/s?

// 收敛判据
#define MPC_CONVERGENCE_TOLERANCE   (0.001f) // 代价函数收敛容差
#define MPC_GRADIENT_STEP_SIZE      (0.01f)  // 梯度下降步长

//================================================= 数据结构定义 =================================================

// MPC系统状态向量 [x, y, heading, linear_vel, angular_vel]
typedef struct {
    float x;                // 位置X坐标 (mm)
    float y;                // 位置Y坐标 (mm)
    float heading;          // 航向角 (rad)
    float linear_velocity;  // 线速度 (m/s)
    float angular_velocity; // 角速度 (rad/s)
} mpc_state_t;

// MPC控制输入向量 [target_linear_vel, target_angular_vel]
typedef struct {
    float target_linear_velocity;   // 目标线速度 (m/s)
    float target_angular_velocity;  // 目标角速度 (rad/s)
} mpc_control_t;

// MPC权重参数
typedef struct {
    // 状态误差权重
    float Q_x;              // 位置X误差权重
    float Q_y;              // 位置Y误差权重  
    float Q_heading;        // 航向误差权重
    float Q_velocity;       // 速度误差权重
    float Q_angular_vel;    // 角速度误差权重
    
    // 控制量权重（能耗）
    float R_linear_vel;     // 线速度控制量权重
    float R_angular_vel;    // 角速度控制量权重
    
    // 控制变化率权重（平滑性）
    float S_linear_vel;     // 线速度变化率权重
    float S_angular_vel;    // 角速度变化率权重
    
    // 终端权重（提高稳定性）
    float P_x;              // 终端位置X权重
    float P_y;              // 终端位置Y权重
    float P_heading;        // 终端航向权重
} mpc_weights_t;

// MPC预测数据
typedef struct {
    mpc_state_t predicted_states[MPC_PREDICTION_HORIZON];       // 预测状态序列
    mpc_control_t control_sequence[MPC_CONTROL_HORIZON];        // 控制序列
    mpc_state_t reference_trajectory[MPC_PREDICTION_HORIZON];   // 参考轨迹
    float cost_function_value;                                  // 当前代价函数值
    uint32_t iteration_count;                                   // 优化迭代次数
    bool converged;                                             // 是否收敛
} mpc_prediction_data_t;

// MPC性能监控
typedef struct {
    // 计算性能
    uint32_t computation_time_us;       // 计算时间 (微秒)
    uint32_t max_computation_time_us;   // 最大计算时间
    uint32_t total_iterations;          // 总迭代次数
    uint32_t convergence_failures;     // 收敛失败次数
    float average_cost;                 // 平均代价值
    float cpu_usage_percent;            // CPU使用率百分比
    
    // 跟踪性能统计
    float position_error_rms;           // 位置误差RMS (mm)
    float heading_error_rms;            // 航向误差RMS (rad)
    float max_position_error;           // 最大位置误差 (mm)
    float max_heading_error;            // 最大航向误差 (rad)
    float control_smoothness_linear;    // 线速度控制平滑性
    float control_smoothness_angular;   // 角速度控制平滑性
    
    // 综合性能指标
    float overall_performance_score;    // 综合性能评分 (0-100)
    uint32_t samples_count;             // 统计样本数量
    float performance_trend;            // 性能趋势 (正数表示改善)
} mpc_performance_t;

// MPC自动调参配置
typedef struct {
    bool auto_tuning_enabled;           // 是否启用自动调参
    float tuning_step_size;             // 参数调整步长
    float performance_threshold;        // 性能改善阈值
    uint32_t evaluation_window;         // 性能评估窗口大小
    uint32_t tuning_interval;           // 调参间隔 (控制步数)
    float convergence_tolerance;        // 调参收敛容差
    uint32_t max_tuning_iterations;     // 最大调参迭代次数
} mpc_auto_tuning_config_t;

// 参数调整状态
typedef enum {
    TUNING_STATE_IDLE = 0,              // 空闲状态
    TUNING_STATE_EVALUATING,            // 性能评估中
    TUNING_STATE_OPTIMIZING,            // 参数优化中
    TUNING_STATE_CONVERGED,             // 已收敛
    TUNING_STATE_FAILED,                // 调参失败
} mpc_tuning_state_t;

// 自动调参数据
typedef struct {
    mpc_auto_tuning_config_t config;    // 调参配置
    mpc_tuning_state_t state;           // 当前状态
    mpc_weights_t best_weights;         // 历史最优权重
    float best_performance_score;       // 历史最优性能
    mpc_weights_t current_weights;      // 当前测试权重
    
    // 优化算法状态
    float gradient[9];                  // 参数梯度估计
    float momentum[9];                  // 动量项
    uint32_t tuning_step_count;        // 调参步数计数
    uint32_t evaluation_count;         // 评估计数
    float performance_history[10];     // 性能历史记录
    
    // 工况自适应
    float current_speed;                // 当前速度
    float path_curvature;               // 路径曲率
    bool is_straight_line;              // 是否直线段
    bool is_sharp_turn;                 // 是否急转弯
} mpc_auto_tuning_data_t;

// MPC控制器主结构
typedef struct {
    mpc_weights_t weights;              // 权重参数
    mpc_prediction_data_t prediction;   // 预测数据
    mpc_performance_t performance;      // 性能监控
    mpc_auto_tuning_data_t auto_tuning; // 自动调参数据
    
    // 系统模型参数
    float wheelbase;                    // 轮距 (mm)
    float velocity_time_constant;      // 速度时间常数
    float angular_velocity_time_constant; // 角速度时间常数
    
    // 控制器状态
    bool initialized;                   // 初始化标志
    uint32_t control_step_count;       // 控制步数计数
    mpc_control_t last_control;        // 上一次控制量
    
    // 跟踪误差累积 (用于性能统计)
    float error_accumulator_x;          // X方向误差累积
    float error_accumulator_y;          // Y方向误差累积
    float error_accumulator_heading;    // 航向误差累积
    float control_change_accumulator_linear;   // 线速度变化累积
    float control_change_accumulator_angular;  // 角速度变化累积
} mpc_controller_t;

//================================================= 全局变量声明 =================================================
extern mpc_controller_t g_mpc_controller;

//================================================= 核心函数声明 =================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化MPC控制器
// 参数说明     weights - 权重参数结构指针
// 返回参数     bool - 初始化是否成功
// 使用示例     mpc_controller_init(&default_weights);
// 备注信息     必须在使用MPC前调用，设置权重和系统参数
//-------------------------------------------------------------------------------------------------------------------
bool mpc_controller_init(const mpc_weights_t* weights);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     MPC控制器主计算函数
// 参数说明     current_state - 当前车辆状态
// 参数说明     reference_path - 参考路径数组
// 参数说明     path_length - 路径点数量
// 参数说明     lookahead_distance - 前瞻距离 (mm)
// 返回参数     MotionCommand - 计算得到的运动控制指令
// 使用示例     cmd = mpc_compute_control(&state, path, 100, 300.0f);
// 备注信息     主要的MPC计算接口，每个控制周期调用一次
//-------------------------------------------------------------------------------------------------------------------
MotionCommand mpc_compute_control(const VehicleState* current_state,
                                 const OptimalPathPoint* reference_path,
                                 uint16_t path_length,
                                 float lookahead_distance);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取MPC性能统计信息
// 参数说明     void
// 返回参数     const mpc_performance_t* - 性能统计数据指针
// 使用示例     perf = mpc_get_performance_stats();
// 备注信息     用于性能监控和调试
//-------------------------------------------------------------------------------------------------------------------
const mpc_performance_t* mpc_get_performance_stats(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置MPC权重参数
// 参数说明     weights - 新的权重参数
// 返回参数     bool - 设置是否成功
// 使用示例     mpc_set_weights(&tuned_weights);
// 备注信息     运行时动态调整MPC性能
//-------------------------------------------------------------------------------------------------------------------
bool mpc_set_weights(const mpc_weights_t* weights);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置MPC控制器状态
// 参数说明     void
// 返回参数     void
// 使用示例     mpc_reset_controller();
// 备注信息     在路径切换或异常情况下使用
//-------------------------------------------------------------------------------------------------------------------
void mpc_reset_controller(void);

//================================================= 自动调参函数声明 =================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启用/禁用MPC自动调参
// 参数说明     enable - 是否启用自动调参
// 返回参数     bool - 设置是否成功
// 使用示例     mpc_auto_tuning_enable(true);
//-------------------------------------------------------------------------------------------------------------------
bool mpc_auto_tuning_enable(bool enable);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     配置自动调参参数
// 参数说明     config - 调参配置结构指针
// 返回参数     bool - 配置是否成功
// 使用示例     mpc_auto_tuning_configure(&config);
//-------------------------------------------------------------------------------------------------------------------
bool mpc_auto_tuning_configure(const mpc_auto_tuning_config_t* config);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取默认调参配置
// 参数说明     config - 输出的配置结构指针
// 返回参数     void
// 使用示例     mpc_get_default_tuning_config(&config);
//-------------------------------------------------------------------------------------------------------------------
void mpc_get_default_tuning_config(mpc_auto_tuning_config_t* config);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行一步自动调参
// 参数说明     current_state - 当前车辆状态
// 参数说明     reference_path - 参考路径
// 参数说明     path_length - 路径长度
// 返回参数     bool - 是否进行了参数调整
// 使用示例     mpc_auto_tuning_step(&state, path, 100);
// 备注信息     在每个控制周期后调用，自动评估和调整参数
//-------------------------------------------------------------------------------------------------------------------
bool mpc_auto_tuning_step(const VehicleState* current_state, 
                          const OptimalPathPoint* reference_path, 
                          uint16_t path_length);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取自动调参状态
// 参数说明     void
// 返回参数     mpc_tuning_state_t - 当前调参状态
// 使用示例     state = mpc_get_tuning_state();
//-------------------------------------------------------------------------------------------------------------------
mpc_tuning_state_t mpc_get_tuning_state(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取最优权重参数
// 参数说明     weights - 输出的最优权重结构指针
// 返回参数     bool - 是否有有效的最优参数
// 使用示例     mpc_get_best_weights(&best_weights);
//-------------------------------------------------------------------------------------------------------------------
bool mpc_get_best_weights(mpc_weights_t* weights);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     手动设置最优权重参数
// 参数说明     weights - 要设置的权重参数
// 返回参数     bool - 设置是否成功
// 使用示例     mpc_set_best_weights(&optimized_weights);
//-------------------------------------------------------------------------------------------------------------------
bool mpc_set_best_weights(const mpc_weights_t* weights);

//================================================= 辅助函数声明 =================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取默认MPC权重参数
// 参数说明     weights - 输出的权重参数结构指针
// 返回参数     void
// 使用示例     mpc_get_default_weights(&weights);
// 备注信息     提供经过调试的默认参数
//-------------------------------------------------------------------------------------------------------------------
void mpc_get_default_weights(mpc_weights_t* weights);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     MPC调试信息输出
// 参数说明     void
// 返回参数     void
// 使用示例     mpc_print_debug_info();
// 备注信息     输出当前MPC状态信息，用于调试
//-------------------------------------------------------------------------------------------------------------------
void mpc_print_debug_info(void);

#endif // _MPC_CONTROLLER_H_
