/*********************************************************************************************************************
* 文件名称          state_estimator.c
* 功能说明          车辆状态估计器模块 实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本              备注
* 2025-09-18        LittleMaster        v1.0              创建文件，迁移航位推算逻辑
********************************************************************************************************************/
#include "state_estimator.h"
#include "driver_sch16tk10.h"
#include "driver_encoder.h"
#include "math_utils.h" // 引入新的数学工具模块
#include <math.h>

#define DEG_TO_RAD(deg) ((deg) * (M_PI / 180.0f))

// 内部状态变量，存储最新的车辆位姿
static VehicleState s_vehicle_state;

// 估计器模式和累计数据
static estimator_mode_t s_estimator_mode = ESTIMATOR_MODE_SIMPLE;
static float s_encoder_x_accumulated = 0.0f;  // 编码器积分位置
static float s_encoder_y_accumulated = 0.0f;
static bool s_kalman_initialized = false;

void state_estimator_init(void)
{
    memset(&s_vehicle_state, 0, sizeof(VehicleState));
    // 假设初始朝向Y轴正方向
    s_vehicle_state.heading = DEG_TO_RAD(90.0f);
    
    // 重置累计数据
    s_encoder_x_accumulated = 0.0f;
    s_encoder_y_accumulated = 0.0f;
    s_kalman_initialized = false;
    
    // 初始化陀螺仪卡尔曼滤波器
    gyro_kalman_filter_init();
    
    // 如果当前是卡尔曼模式，初始化主卡尔曼滤波器
    if (s_estimator_mode == ESTIMATOR_MODE_KALMAN) {
        kf_state_t initial_state = {
            .x = s_vehicle_state.x,
            .y = s_vehicle_state.y,
            .heading = s_vehicle_state.heading,
            .vx = 0.0f,
            .vy = 0.0f,
            .omega = 0.0f
        };
        kalman_filter_init(&initial_state);
        s_kalman_initialized = true;
    }
}

void state_estimator_update(float dt)
{
    // 1. 从编码器获取轮速 (单位: m/s)
    float left_speed = encoder_get_speed(ENCODER_ID_LEFT);
    float right_speed = encoder_get_speed(ENCODER_ID_RIGHT);

    // 2. 计算车辆的线速度 (m/s)
    s_vehicle_state.linear_speed = (left_speed + right_speed) / 2.0f;
    
    // 3. 从IMU获取Z轴角速度 (单位: rad/s)
    SCH1_raw_data imu_raw;
    SCH1_result imu_data;
    SCH1_getData(&imu_raw);
    SCH1_convert_data(&imu_raw, &imu_data);
    float omega_raw = -DEG_TO_RAD(imu_data.Rate1[2]); // 原始陀螺仪数据
    
    // 4. 陀螺仪卡尔曼滤波 (总是启用)
    float omega_filtered = gyro_kalman_filter_update(omega_raw, dt);
    
    // 5. 根据估计器模式选择算法
    if (s_estimator_mode == ESTIMATOR_MODE_KALMAN && s_kalman_initialized) {
        // 卡尔曼滤波模式
        state_estimator_update_kalman(dt, omega_filtered);
    } else {
        // 简单死推算模式
        state_estimator_update_simple(dt, omega_filtered);
    }
}

//================================================= 内部函数实现 =================================================

static void state_estimator_update_simple(float dt, float omega_filtered)
{
    // 简单的死推算算法
    float speed_mm_s = s_vehicle_state.linear_speed * 1000.0f; // 转换为 mm/s
    s_vehicle_state.x += speed_mm_s * cosf(s_vehicle_state.heading) * dt;
    s_vehicle_state.y += speed_mm_s * sinf(s_vehicle_state.heading) * dt;
    s_vehicle_state.heading += omega_filtered * dt;
    s_vehicle_state.heading = normalize_angle(s_vehicle_state.heading);
}

static void state_estimator_update_kalman(float dt, float omega_filtered)
{
    // 更新编码器积分位置 (作为位置测量)
    float speed_mm_s = s_vehicle_state.linear_speed * 1000.0f;
    s_encoder_x_accumulated += speed_mm_s * cosf(s_vehicle_state.heading) * dt;
    s_encoder_y_accumulated += speed_mm_s * sinf(s_vehicle_state.heading) * dt;
    
    // 卡尔曼滤波器预测步骤
    kalman_filter_predict(dt);
    
    // 准备测量数据
    kf_measurement_t measurement = {
        .encoder_x = s_encoder_x_accumulated,
        .encoder_y = s_encoder_y_accumulated,
        .gyro_omega = omega_filtered,
        .valid = true
    };
    
    // 卡尔曼滤波器更新步骤
    if (kalman_filter_update(&measurement)) {
        // 获取滤波后的状态
        kf_state_t filtered_state;
        kalman_filter_get_state(&filtered_state);
        
        // 更新车辆状态
        s_vehicle_state.x = filtered_state.x;
        s_vehicle_state.y = filtered_state.y;
        s_vehicle_state.heading = normalize_angle(filtered_state.heading);
        // linear_speed 保持编码器计算的值
    }
}

const VehicleState* state_estimator_get_state(void)
{
    return &s_vehicle_state;
}

bool state_estimator_set_mode(estimator_mode_t mode)
{
    if (mode == s_estimator_mode) {
        return true;  // 模式没有改变
    }
    
    s_estimator_mode = mode;
    
    if (mode == ESTIMATOR_MODE_KALMAN && !s_kalman_initialized) {
        // 切换到卡尔曼模式，需要初始化
        kf_state_t initial_state = {
            .x = s_vehicle_state.x,
            .y = s_vehicle_state.y,
            .heading = s_vehicle_state.heading,
            .vx = s_vehicle_state.linear_speed * cosf(s_vehicle_state.heading) * 1000.0f,
            .vy = s_vehicle_state.linear_speed * sinf(s_vehicle_state.heading) * 1000.0f,
            .omega = 0.0f
        };
        kalman_filter_init(&initial_state);
        s_kalman_initialized = true;
        
        // 重置编码器积分
        s_encoder_x_accumulated = s_vehicle_state.x;
        s_encoder_y_accumulated = s_vehicle_state.y;
    }
    
    return true;
}

estimator_mode_t state_estimator_get_mode(void)
{
    return s_estimator_mode;
}

const kf_performance_t* state_estimator_get_kalman_performance(void)
{
    if (s_estimator_mode == ESTIMATOR_MODE_KALMAN && s_kalman_initialized) {
        return kalman_filter_get_performance();
    }
    return NULL;
}

//================================================= 内部函数声明 =================================================
static void state_estimator_update_simple(float dt, float omega_filtered);
static void state_estimator_update_kalman(float dt, float omega_filtered);
