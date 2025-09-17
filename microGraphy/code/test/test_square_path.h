/*********************************************************************************************************************
* 文件名称          test_square_path.h
* 功能说明          正方形路径测试程序头文件
* 作者              AI Assistant
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2024-XX-XX        AI Assistant        创建正方形路径测试
*
* 文件作用说明：
* 本文件为正方形路径测试程序的头文件，用于测试小车在1×1米正方形路径上的循环行驶
* 同时监测陀螺仪漂移情况，评估惯导系统的性能
*
* 测试功能：
* 1. 生成1×1米标准正方形路径
* 2. 高速循环跟踪（3.5m/s以上）
* 3. 陀螺仪漂移监测
* 4. 路径精度统计
* 5. 实时数据显示
*
* 使用方式：
* 1. 在主函数中调用 test_square_path() 开始测试
* 2. 观察屏幕显示的实时数据
* 3. 按键退出测试模式
********************************************************************************************************************/

#ifndef _TEST_SQUARE_PATH_H_
#define _TEST_SQUARE_PATH_H_

#include "zf_common_typedef.h"

//=================================================测试配置参数================================================
// 正方形路径配置
#define SQUARE_SIZE                 1000        // 正方形边长 (mm)
#define SQUARE_CENTER_X             500         // 正方形中心X坐标 (mm)
#define SQUARE_CENTER_Y             500         // 正方形中心Y坐标 (mm)

// 测试速度配置
#define TEST_TARGET_SPEED           3.5f        // 目标速度 (m/s)
#define TEST_MAX_SPEED              4.0f        // 最大速度限制 (m/s)
#define TEST_MIN_SPEED              3.0f        // 最小速度限制 (m/s)

// 路径点数量配置
#define SQUARE_POINTS_COUNT         40          // 正方形路径点数量
#define CORNER_POINTS_COUNT         8           // 每个转角的路径点数量

// 测试时间配置
#define TEST_DURATION_MS            300000      // 测试持续时间 (5分钟)
#define DATA_LOG_INTERVAL_MS        100         // 数据记录间隔 (100ms)
#define DISPLAY_UPDATE_INTERVAL_MS  500         // 显示更新间隔 (500ms)

// 陀螺仪漂移检测配置
#define GYRO_DRIFT_SAMPLES          100         // 陀螺仪漂移检测样本数
#define GYRO_DRIFT_THRESHOLD        0.5f        // 陀螺仪漂移阈值 (度/秒)
#define ANGLE_DRIFT_THRESHOLD       2.0f        // 角度累积漂移阈值 (度)

//=================================================数据结构定义================================================
// 正方形路径点结构
typedef struct
{
    float x;                                    // X坐标 (mm)
    float y;                                    // Y坐标 (mm)
    float angle;                                // 期望航向角 (度)
    uint8 point_type;                           // 点类型：0=直线，1=转角
} square_path_point_struct;

// 陀螺仪漂移监测结构
typedef struct
{
    float gyro_x_sum;                           // X轴陀螺仪累计值
    float gyro_y_sum;                           // Y轴陀螺仪累计值
    float gyro_z_sum;                           // Z轴陀螺仪累计值
    float gyro_x_drift;                         // X轴漂移率 (度/秒)
    float gyro_y_drift;                         // Y轴漂移率 (度/秒)
    float gyro_z_drift;                         // Z轴漂移率 (度/秒)
    uint32 sample_count;                        // 采样计数
    float angle_error_accumulate;               // 角度误差累积
    float max_angle_error;                      // 最大角度误差
    uint32 drift_warning_count;                 // 漂移警告计数
} gyro_drift_monitor_struct;

// 路径跟踪统计结构
typedef struct
{
    float max_position_error;                   // 最大位置误差 (mm)
    float avg_position_error;                   // 平均位置误差 (mm)
    float position_error_sum;                   // 位置误差累计
    uint32 error_sample_count;                  // 误差采样计数
    uint32 overspeed_count;                     // 超速次数
    uint32 underspeed_count;                    // 低速次数
    float current_speed;                        // 当前速度 (m/s)
    float avg_speed;                            // 平均速度 (m/s)
    float speed_sum;                            // 速度累计
    uint32 speed_sample_count;                  // 速度采样计数
    uint32 lap_count;                           // 圈数计数
    uint32 total_distance;                      // 总行驶距离 (mm)
} path_tracking_stats_struct;

// 正方形路径测试系统结构
typedef struct
{
    square_path_point_struct path_points[SQUARE_POINTS_COUNT];  // 路径点数组
    gyro_drift_monitor_struct gyro_monitor;                    // 陀螺仪监测
    path_tracking_stats_struct tracking_stats;                 // 跟踪统计
    
    uint8 current_point_index;                                 // 当前路径点索引
    uint8 test_running;                                        // 测试运行标志
    uint32 test_start_time;                                    // 测试开始时间
    uint32 last_log_time;                                      // 上次记录时间
    uint32 last_display_time;                                  // 上次显示时间
    
    float reference_angle_start;                               // 起始参考角度
    uint8 path_initialized;                                    // 路径初始化标志
} square_test_system_struct;

//=================================================全局变量声明================================================
extern square_test_system_struct square_test_system;

//=================================================函数声明================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     正方形路径测试主函数
// 参数说明     void
// 返回参数     void
// 使用示例     test_square_path();
// 备注信息     执行完整的正方形路径测试，包括路径生成、循环跟踪、数据监测
//-------------------------------------------------------------------------------------------------------------------
void test_square_path(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     生成正方形路径点
// 参数说明     void
// 返回参数     uint8                          生成结果 (0=成功, 1=失败)
// 使用示例     generate_square_path();
// 备注信息     在Flash中生成1×1米标准正方形路径点
//-------------------------------------------------------------------------------------------------------------------
uint8 generate_square_path(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     陀螺仪漂移监测
// 参数说明     gyro_x                         X轴陀螺仪数据 (度/秒)
// 参数说明     gyro_y                         Y轴陀螺仪数据 (度/秒)
// 参数说明     gyro_z                         Z轴陀螺仪数据 (度/秒)
// 参数说明     current_angle                  当前航向角 (度)
// 返回参数     uint8                          漂移状态 (0=正常, 1=轻微漂移, 2=严重漂移)
// 使用示例     drift_status = monitor_gyro_drift(gx, gy, gz, angle);
// 备注信息     实时监测陀螺仪漂移情况并统计
//-------------------------------------------------------------------------------------------------------------------
uint8 monitor_gyro_drift(float gyro_x, float gyro_y, float gyro_z, float current_angle);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     路径跟踪统计更新
// 参数说明     current_x                      当前X坐标 (mm)
// 参数说明     current_y                      当前Y坐标 (mm)
// 参数说明     current_speed                  当前速度 (m/s)
// 返回参数     void
// 使用示例     update_tracking_stats(x, y, speed);
// 备注信息     更新路径跟踪精度和速度统计
//-------------------------------------------------------------------------------------------------------------------
void update_tracking_stats(float current_x, float current_y, float current_speed);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     测试数据显示
// 参数说明     void
// 返回参数     void
// 使用示例     display_test_data();
// 备注信息     在屏幕上实时显示测试数据和监测结果
//-------------------------------------------------------------------------------------------------------------------
void display_test_data(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     测试系统初始化
// 参数说明     void
// 返回参数     uint8                          初始化结果 (0=成功, 1=失败)
// 使用示例     init_square_test_system();
// 备注信息     初始化测试系统各项参数和数据结构
//-------------------------------------------------------------------------------------------------------------------
uint8 init_square_test_system(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查测试退出条件
// 参数说明     void
// 返回参数     uint8                          退出标志 (0=继续, 1=退出)
// 使用示例     if (check_test_exit()) break;
// 备注信息     检查按键或其他退出条件
//-------------------------------------------------------------------------------------------------------------------
uint8 check_test_exit(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     保存测试结果
// 参数说明     void
// 返回参数     void
// 使用示例     save_test_results();
// 备注信息     将测试统计结果保存到Flash或显示汇总
//-------------------------------------------------------------------------------------------------------------------
void save_test_results(void);

#endif // _TEST_SQUARE_PATH_H_
