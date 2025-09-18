/*********************************************************************************************************************
* 文件名称          state_estimator.c
* 功能说明          车辆状态估计器模块 实现文件
* 作者              AI Assistant
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本              备注
* 2024-XX-XX        AI Assistant        v1.0              创建文件，迁移航位推算逻辑
********************************************************************************************************************/
#include "state_estimator.h"
#include "driver_sch16tk10.h"
#include "driver_encoder.h"
#include "math_utils.h" // 引入新的数学工具模块
#include <math.h>

#define DEG_TO_RAD(deg) ((deg) * (M_PI / 180.0f))

// 内部状态变量，存储最新的车辆位姿
static VehicleState s_vehicle_state;

void state_estimator_init(void)
{
    memset(&s_vehicle_state, 0, sizeof(VehicleState));
    // 假设初始朝向Y轴正方向
    s_vehicle_state.heading = DEG_TO_RAD(90.0f); 
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
    float omega_imu = -DEG_TO_RAD(imu_data.Rate1[2]); // 注意方向可能需要根据安装取反

    // 4. 积分更新位置和姿态 (航位推算核心)
    float speed_mm_s = s_vehicle_state.linear_speed * 1000.0f; // 转换为 mm/s
    s_vehicle_state.x += speed_mm_s * cosf(s_vehicle_state.heading) * dt;
    s_vehicle_state.y += speed_mm_s * sinf(s_vehicle_state.heading) * dt;
    s_vehicle_state.heading += omega_imu * dt;
    s_vehicle_state.heading = normalize_angle(s_vehicle_state.heading);
}

const VehicleState* state_estimator_get_state(void)
{
    return &s_vehicle_state;
}
