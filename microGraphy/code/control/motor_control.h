/*********************************************************************************************************************
* 文件名称          motor_control.h
* 功能说明          电机闭环控制系统头文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本              备注
* 2025-09-18        LittleMaster        v1.0              创建电机闭环控制系统
*
* 文件作用说明：
* 本文件为电机闭环控制系统的头文件，提供基于编码器反馈的精确电机控制
* 
* 主要功能模块：
* 1. PID控制器实现
*    - 位置式PID控制算法
*    - 增量式PID控制算法
*    - PID参数在线调整
*    - 积分饱和处理
*    - 微分先行控制
*
* 2. 速度闭环控制
*    - 基于编码器反馈的实时速度控制
*    - 速度设定值跟踪
*    - 加速度限制和平滑控制
*    - 速度滤波和稳定性保证
*
* 3. 位置闭环控制
*    - 基于编码器脉冲的精确位置控制
*    - 位置设定值跟踪
*    - 轨迹规划和运动平滑
*    - 位置误差监控和保护
*
* 4. 电流闭环控制（可选）
*    - 电流反馈控制
*    - 转矩控制
*    - 过流保护
*
* 5. 多模式控制系统
*    - 开环控制模式
*    - 速度闭环模式
*    - 位置闭环模式
*    - 级联控制模式
*
* 6. 系统诊断和保护
*    - 控制系统状态监控
*    - 异常检测和处理
*    - 参数自适应调整
*    - 性能指标统计
*
* 技术特性：
* - 控制频率：1KHz（推荐）
* - 支持双电机独立控制
* - 支持差速运动控制
* - 实时性能优化
* - 低延迟响应设计
********************************************************************************************************************/

#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include "zf_common_typedef.h"
#include "driver_motor.h"
#include "driver_encoder.h"

//=================================================系统配置参数================================================
#define MOTOR_CONTROL_FREQUENCY         (1000)     // 控制频率 1KHz
#define MOTOR_CONTROL_SAMPLE_TIME       (1.0f)     // 采样时间 1ms
#define MOTOR_CONTROL_MAX_OUTPUT        (9999)     // 最大控制输出
#define MOTOR_CONTROL_MIN_OUTPUT        (-9999)    // 最小控制输出

//=================================================PID参数配置================================================
// 速度环PID默认参数
#define PID_SPEED_KP_DEFAULT            (8.0f)     // 比例系数
#define PID_SPEED_KI_DEFAULT            (0.5f)     // 积分系数
#define PID_SPEED_KD_DEFAULT            (0.1f)     // 微分系数

// 位置环PID默认参数
#define PID_POSITION_KP_DEFAULT         (2.0f)     // 比例系数
#define PID_POSITION_KI_DEFAULT         (0.1f)     // 积分系数
#define PID_POSITION_KD_DEFAULT         (0.05f)    // 微分系数

// PID限制参数
#define PID_INTEGRAL_MAX                (5000.0f)  // 积分限幅
#define PID_INTEGRAL_MIN                (-5000.0f) // 积分限幅
#define PID_OUTPUT_MAX                  (9999.0f)  // 输出限幅
#define PID_OUTPUT_MIN                  (-9999.0f) // 输出限幅

//=================================================枚举类型定义================================================
// 控制模式枚举
typedef enum
{
    MOTOR_CTRL_MODE_OPEN_LOOP       = 0,        // 开环控制模式
    MOTOR_CTRL_MODE_SPEED_LOOP      = 1,        // 速度闭环模式
    MOTOR_CTRL_MODE_POSITION_LOOP   = 2,        // 位置闭环模式
    MOTOR_CTRL_MODE_CASCADE         = 3,        // 级联控制模式（位置外环+速度内环）
    MOTOR_CTRL_MODE_CURRENT_LOOP    = 4,        // 电流闭环模式
} motor_control_mode_enum;

// PID控制器类型枚举
typedef enum
{
    PID_TYPE_POSITIONAL     = 0,                // 位置式PID
    PID_TYPE_INCREMENTAL    = 1,                // 增量式PID
} pid_type_enum;

// 电机控制器ID枚举
typedef enum
{
    MOTOR_CTRL_LEFT         = 0,                // 左电机控制器
    MOTOR_CTRL_RIGHT        = 1,                // 右电机控制器
    MOTOR_CTRL_BOTH         = 2,                // 双电机控制器
} motor_ctrl_id_enum;

// 控制器状态枚举
typedef enum
{
    MOTOR_CTRL_STATUS_STOP      = 0,            // 停止状态
    MOTOR_CTRL_STATUS_RUNNING   = 1,            // 运行状态
    MOTOR_CTRL_STATUS_ERROR     = 2,            // 错误状态
    MOTOR_CTRL_STATUS_FAULT     = 3,            // 故障状态
} motor_ctrl_status_enum;

//=================================================数据结构定义================================================
// PID控制器结构体
typedef struct
{
    // PID参数
    float kp;                                   // 比例系数
    float ki;                                   // 积分系数
    float kd;                                   // 微分系数
    
    // PID状态变量
    float setpoint;                             // 设定值
    float feedback;                             // 反馈值
    float error;                                // 当前误差
    float last_error;                           // 上次误差
    float integral;                             // 积分累积
    float derivative;                           // 微分项
    float output;                               // 控制输出
    
    // PID限制参数
    float integral_max;                         // 积分上限
    float integral_min;                         // 积分下限
    float output_max;                           // 输出上限
    float output_min;                           // 输出下限
    
    // PID配置
    pid_type_enum type;                         // PID类型
    bool enable;                                // 使能标志
    bool integral_reset;                        // 积分清零标志
    
} pid_controller_struct;

// 电机控制器结构体
typedef struct
{
    // 控制模式和状态
    motor_control_mode_enum mode;               // 控制模式
    motor_ctrl_status_enum status;              // 控制器状态
    motor_id_enum motor_id;                     // 对应的电机ID
    encoder_id_enum encoder_id;                 // 对应的编码器ID
    
    // PID控制器
    pid_controller_struct speed_pid;            // 速度环PID
    pid_controller_struct position_pid;         // 位置环PID
    
    // 目标值和反馈值
    float target_speed;                         // 目标速度 (m/s)
    float current_speed;                        // 当前速度 (m/s)
    float target_position;                      // 目标位置 (脉冲数)
    float current_position;                     // 当前位置 (脉冲数)
    
    // 控制输出
    int16 pwm_output;                           // PWM输出值
    float speed_output;                         // 速度环输出
    float position_output;                      // 位置环输出
    
    // 运动参数
    float max_speed;                            // 最大速度限制 (m/s)
    float max_acceleration;                     // 最大加速度限制 (m/s?)
    float speed_ramp_rate;                      // 速度斜坡率
    
    // 状态监控
    float speed_error;                          // 速度误差
    float position_error;                       // 位置误差
    uint32 error_count;                         // 错误计数
    bool target_reached;                        // 目标到达标志
    
} motor_controller_struct;

// 电机控制系统结构体
typedef struct
{
    motor_controller_struct left;               // 左电机控制器
    motor_controller_struct right;              // 右电机控制器
    
    // 系统状态
    bool system_enable;                         // 系统使能
    uint32 control_cycle_count;                 // 控制周期计数
    uint32 last_update_time;                    // 上次更新时间
    
    // 差速控制参数
    float robot_linear_speed;                   // 机器人线速度设定值 (m/s)
    float robot_angular_speed;                  // 机器人角速度设定值 (rad/s)
    float wheelbase;                            // 轮距 (mm)
    
    // 系统诊断
    uint32 total_error_count;                   // 总错误计数
    float average_speed_error;                  // 平均速度误差
    float max_speed_error;                      // 最大速度误差
    
} motor_control_system_struct;

//=================================================全局变量声明================================================
extern motor_control_system_struct motor_ctrl_system;

//=================================================函数声明================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机控制系统初始化
// 参数说明     void
// 返回参数     uint8                   初始化结果 (0=成功, 1=失败)
// 使用示例     motor_control_init();
// 备注信息     初始化电机控制系统，设置默认PID参数
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_control_init(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID控制器初始化
// 参数说明     pid                     PID控制器指针
// 参数说明     kp, ki, kd              PID参数
// 参数说明     type                    PID类型
// 返回参数     uint8                   初始化结果 (0=成功, 1=失败)
// 使用示例     pid_init(&speed_pid, 8.0f, 0.5f, 0.1f, PID_TYPE_POSITIONAL);
// 备注信息     初始化PID控制器参数和状态
//-------------------------------------------------------------------------------------------------------------------
uint8 pid_init(pid_controller_struct *pid, float kp, float ki, float kd, pid_type_enum type);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID控制器计算
// 参数说明     pid                     PID控制器指针
// 参数说明     setpoint                设定值
// 参数说明     feedback                反馈值
// 返回参数     float                   PID输出
// 使用示例     output = pid_calculate(&speed_pid, target_speed, current_speed);
// 备注信息     执行PID计算，返回控制输出
//-------------------------------------------------------------------------------------------------------------------
float pid_calculate(pid_controller_struct *pid, float setpoint, float feedback);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机控制模式
// 参数说明     motor_id                电机ID
// 参数说明     mode                    控制模式
// 返回参数     uint8                   设置结果 (0=成功, 1=失败)
// 使用示例     motor_set_control_mode(MOTOR_CTRL_LEFT, MOTOR_CTRL_MODE_SPEED_LOOP);
// 备注信息     设置指定电机的控制模式
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_set_control_mode(motor_ctrl_id_enum motor_id, motor_control_mode_enum mode);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机目标速度
// 参数说明     motor_id                电机ID
// 参数说明     target_speed            目标速度 (m/s)
// 返回参数     uint8                   设置结果 (0=成功, 1=失败)
// 使用示例     motor_set_target_speed(MOTOR_CTRL_LEFT, 0.5f);
// 备注信息     设置电机目标速度，仅在速度闭环模式下有效
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_set_target_speed(motor_ctrl_id_enum motor_id, float target_speed);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机目标位置
// 参数说明     motor_id                电机ID
// 参数说明     target_position         目标位置 (脉冲数)
// 返回参数     uint8                   设置结果 (0=成功, 1=失败)
// 使用示例     motor_set_target_position(MOTOR_CTRL_LEFT, 1000);
// 备注信息     设置电机目标位置，仅在位置闭环模式下有效
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_set_target_position(motor_ctrl_id_enum motor_id, float target_position);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置机器人运动参数
// 参数说明     linear_speed            线速度 (m/s)
// 参数说明     angular_speed           角速度 (rad/s)
// 返回参数     uint8                   设置结果 (0=成功, 1=失败)
// 使用示例     motor_set_robot_motion(0.3f, 0.5f);
// 备注信息     设置机器人的线速度和角速度，自动分解为左右轮速度
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_set_robot_motion(float linear_speed, float angular_speed);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID参数设置
// 参数说明     motor_id                电机ID
// 参数说明     pid_type                PID类型 (0=速度环, 1=位置环)
// 参数说明     kp, ki, kd              PID参数
// 返回参数     uint8                   设置结果 (0=成功, 1=失败)
// 使用示例     motor_set_pid_params(MOTOR_CTRL_LEFT, 0, 8.0f, 0.5f, 0.1f);
// 备注信息     设置指定电机的PID参数
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_set_pid_params(motor_ctrl_id_enum motor_id, uint8 pid_type, float kp, float ki, float kd);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机控制系统更新（周期调用）
// 参数说明     void
// 返回参数     uint8                   更新结果 (0=成功, 1=失败)
// 使用示例     motor_control_update();  // 在1ms定时器中调用
// 备注信息     电机控制系统主循环，建议1ms调用一次
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_control_update(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机控制系统启动
// 参数说明     void
// 返回参数     uint8                   启动结果 (0=成功, 1=失败)
// 使用示例     motor_control_start();
// 备注信息     启动电机控制系统
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_control_start(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机控制系统停止
// 参数说明     void
// 返回参数     uint8                   停止结果 (0=成功, 1=失败)
// 使用示例     motor_control_stop();
// 备注信息     停止电机控制系统，所有电机停止运行
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_control_stop(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取电机状态信息
// 参数说明     motor_id                电机ID
// 参数说明     speed                   当前速度输出指针 (m/s)
// 参数说明     position                当前位置输出指针 (脉冲数)
// 参数说明     error                   当前误差输出指针
// 返回参数     uint8                   获取结果 (0=成功, 1=失败)
// 使用示例     motor_get_status(MOTOR_CTRL_LEFT, &speed, &position, &error);
// 备注信息     获取指定电机的运行状态信息
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_get_status(motor_ctrl_id_enum motor_id, float *speed, float *position, float *error);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID控制器复位
// 参数说明     motor_id                电机ID
// 参数说明     pid_type                PID类型 (0=速度环, 1=位置环)
// 返回参数     uint8                   复位结果 (0=成功, 1=失败)
// 使用示例     motor_pid_reset(MOTOR_CTRL_LEFT, 0);
// 备注信息     复位指定电机的PID控制器状态
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_pid_reset(motor_ctrl_id_enum motor_id, uint8 pid_type);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机控制系统诊断
// 参数说明     void
// 返回参数     uint8                   诊断结果 (0=正常, 1=异常)
// 使用示例     motor_control_diagnose();
// 备注信息     诊断电机控制系统状态，检查异常情况
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_control_diagnose(void);

#endif // _MOTOR_CONTROL_H_
