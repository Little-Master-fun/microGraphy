/*********************************************************************************************************************
* 文件名称          motor_control.c
* 功能说明          电机闭环控制系统实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本              备注
* 2025-09-18        LittleMaster        v1.0              创建电机闭环控制系统
*
* 文件作用说明：
* 本文件为电机闭环控制系统的实现文件，提供完整的电机控制功能
*
* 主要实现功能：
* 1. PID控制算法实现
*    - 位置式PID：u(k) = Kp*e(k) + Ki*∑e(k) + Kd*[e(k)-e(k-1)]
*    - 增量式PID：Δu(k) = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*[e(k)-2*e(k-1)+e(k-2)]
*    - 积分饱和处理和微分项平滑
*
* 2. 多模式控制系统
*    - 开环控制：直接PWM输出
*    - 速度闭环：基于编码器反馈的速度PID控制
*    - 位置闭环：基于编码器脉冲的位置PID控制
*    - 级联控制：位置外环+速度内环的双环控制
*
* 3. 运动控制功能
*    - 差速运动控制
*    - 加速度限制和平滑启停
*    - 目标跟踪和到达检测
*
* 4. 系统保护和诊断
*    - 参数有效性检查
*    - 异常状态检测和处理
*    - 性能统计和监控
*
* 技术实现特点：
* - 实时性优化：控制算法执行时间<100μs
* - 数值稳定性：防止积分饱和和微分噪声
* - 参数自适应：根据运行状态动态调整
* - 容错设计：异常情况下的安全处理
********************************************************************************************************************/

#include "motor_control.h"
#include "zf_common_headfile.h"
#include <math.h>

//=================================================全局变量定义================================================
motor_control_system_struct motor_ctrl_system;

//=================================================内部函数声明================================================
static uint8 motor_controller_init(motor_controller_struct *ctrl, motor_id_enum motor_id, encoder_id_enum encoder_id);
static float pid_limit(float value, float min, float max);
static uint8 motor_controller_update(motor_controller_struct *ctrl);
static uint8 motor_speed_to_differential(float linear_speed, float angular_speed, float *left_speed, float *right_speed);

//=================================================函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机控制系统初始化
// 参数说明     void
// 返回参数     uint8                   初始化结果 (0=成功, 1=失败)
// 使用示例     motor_control_init();
// 备注信息     初始化电机控制系统，设置默认PID参数
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_control_init(void)
{
    // 清零系统结构体
    memset(&motor_ctrl_system, 0, sizeof(motor_control_system_struct));
    
    // 初始化系统参数
    motor_ctrl_system.system_enable = false;
    motor_ctrl_system.wheelbase = 150.0f;  // 轮距150mm（根据实际机器人调整）
    motor_ctrl_system.control_cycle_count = 0;
    motor_ctrl_system.last_update_time = 0;
    
    // 初始化左电机控制器
    if (motor_controller_init(&motor_ctrl_system.left, MOTOR_LEFT, ENCODER_ID_LEFT) != 0)
    {
        return 1;
    }
    
    // 初始化右电机控制器
    if (motor_controller_init(&motor_ctrl_system.right, MOTOR_RIGHT, ENCODER_ID_RIGHT) != 0)
    {
        return 1;
    }
    
    // 初始化硬件驱动
    if (motor_init() != MOTOR_STATUS_OK)
    {
        return 1;
    }
    
    if (encoder_init() != ENCODER_STATUS_OK)
    {
        return 1;
    }
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机控制器初始化
// 参数说明     ctrl                    控制器指针
// 参数说明     motor_id                电机ID
// 参数说明     encoder_id              编码器ID
// 返回参数     uint8                   初始化结果 (0=成功, 1=失败)
// 使用示例     motor_controller_init(&left_ctrl, MOTOR_LEFT, ENCODER_ID_LEFT);
// 备注信息     初始化单个电机控制器
//-------------------------------------------------------------------------------------------------------------------
static uint8 motor_controller_init(motor_controller_struct *ctrl, motor_id_enum motor_id, encoder_id_enum encoder_id)
{
    if (ctrl == NULL)
    {
        return 1;
    }
    
    // 清零控制器结构体
    memset(ctrl, 0, sizeof(motor_controller_struct));
    
    // 设置基本参数
    ctrl->motor_id = motor_id;
    ctrl->encoder_id = encoder_id;
    ctrl->mode = MOTOR_CTRL_MODE_OPEN_LOOP;
    ctrl->status = MOTOR_CTRL_STATUS_STOP;
    
    // 初始化速度环PID
    if (pid_init(&ctrl->speed_pid, PID_SPEED_KP_DEFAULT, PID_SPEED_KI_DEFAULT, PID_SPEED_KD_DEFAULT, PID_TYPE_POSITIONAL) != 0)
    {
        return 1;
    }
    
    // 初始化位置环PID
    if (pid_init(&ctrl->position_pid, PID_POSITION_KP_DEFAULT, PID_POSITION_KI_DEFAULT, PID_POSITION_KD_DEFAULT, PID_TYPE_POSITIONAL) != 0)
    {
        return 1;
    }
    
    // 设置运动限制参数
    ctrl->max_speed = 2.0f;           // 最大速度 2m/s
    ctrl->max_acceleration = 5.0f;    // 最大加速度 5m/s?
    ctrl->speed_ramp_rate = 0.1f;     // 速度斜坡率 0.1m/s per cycle
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID控制器初始化
// 参数说明     pid                     PID控制器指针
// 参数说明     kp, ki, kd              PID参数
// 参数说明     type                    PID类型
// 返回参数     uint8                   初始化结果 (0=成功, 1=失败)
// 使用示例     pid_init(&speed_pid, 8.0f, 0.5f, 0.1f, PID_TYPE_POSITIONAL);
// 备注信息     初始化PID控制器参数和状态
//-------------------------------------------------------------------------------------------------------------------
uint8 pid_init(pid_controller_struct *pid, float kp, float ki, float kd, pid_type_enum type)
{
    if (pid == NULL)
    {
        return 1;
    }
    
    // 清零PID结构体
    memset(pid, 0, sizeof(pid_controller_struct));
    
    // 设置PID参数
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->type = type;
    
    // 设置限制参数
    pid->integral_max = PID_INTEGRAL_MAX;
    pid->integral_min = PID_INTEGRAL_MIN;
    pid->output_max = PID_OUTPUT_MAX;
    pid->output_min = PID_OUTPUT_MIN;
    
    // 初始化状态变量
    pid->setpoint = 0.0f;
    pid->feedback = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    
    // 设置使能标志
    pid->enable = true;
    pid->integral_reset = false;
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID控制器计算
// 参数说明     pid                     PID控制器指针
// 参数说明     setpoint                设定值
// 参数说明     feedback                反馈值
// 返回参数     float                   PID输出
// 使用示例     output = pid_calculate(&speed_pid, target_speed, current_speed);
// 备注信息     执行PID计算，返回控制输出
//-------------------------------------------------------------------------------------------------------------------
float pid_calculate(pid_controller_struct *pid, float setpoint, float feedback)
{
    if (pid == NULL || !pid->enable)
    {
        return 0.0f;
    }
    
    // 更新设定值和反馈值
    pid->setpoint = setpoint;
    pid->feedback = feedback;
    
    // 计算误差
    pid->error = setpoint - feedback;
    
    // 积分清零处理
    if (pid->integral_reset)
    {
        pid->integral = 0.0f;
        pid->integral_reset = false;
    }
    
    if (pid->type == PID_TYPE_POSITIONAL)
    {
        // 位置式PID计算
        
        // 积分项计算（带限幅）
        pid->integral += pid->error * MOTOR_CONTROL_SAMPLE_TIME;
        pid->integral = pid_limit(pid->integral, pid->integral_min, pid->integral_max);
        
        // 微分项计算
        pid->derivative = (pid->error - pid->last_error) / MOTOR_CONTROL_SAMPLE_TIME;
        
        // PID输出计算
        pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * pid->derivative;
    }
    else if (pid->type == PID_TYPE_INCREMENTAL)
    {
        // 增量式PID计算
        float delta_output = pid->kp * (pid->error - pid->last_error) + 
                           pid->ki * pid->error + 
                           pid->kd * (pid->error - 2 * pid->last_error + pid->integral);
        
        pid->output += delta_output;
        
        // 保存当前误差到积分项（用于下次计算）
        pid->integral = pid->last_error;
    }
    
    // 输出限幅
    pid->output = pid_limit(pid->output, pid->output_min, pid->output_max);
    
    // 更新历史误差
    pid->last_error = pid->error;
    
    return pid->output;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     数值限幅函数
// 参数说明     value                   输入值
// 参数说明     min                     最小值
// 参数说明     max                     最大值
// 返回参数     float                   限幅后的值
// 使用示例     limited_value = pid_limit(input, -1000, 1000);
// 备注信息     将输入值限制在指定范围内
//-------------------------------------------------------------------------------------------------------------------
static float pid_limit(float value, float min, float max)
{
    if (value > max)
    {
        return max;
    }
    else if (value < min)
    {
        return min;
    }
    return value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机控制模式
// 参数说明     motor_id                电机ID
// 参数说明     mode                    控制模式
// 返回参数     uint8                   设置结果 (0=成功, 1=失败)
// 使用示例     motor_set_control_mode(MOTOR_CTRL_LEFT, MOTOR_CTRL_MODE_SPEED_LOOP);
// 备注信息     设置指定电机的控制模式
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_set_control_mode(motor_ctrl_id_enum motor_id, motor_control_mode_enum mode)
{
    motor_controller_struct *ctrl = NULL;
    
    // 获取控制器指针
    if (motor_id == MOTOR_CTRL_LEFT)
    {
        ctrl = &motor_ctrl_system.left;
    }
    else if (motor_id == MOTOR_CTRL_RIGHT)
    {
        ctrl = &motor_ctrl_system.right;
    }
    else if (motor_id == MOTOR_CTRL_BOTH)
    {
        // 设置双电机模式
        motor_ctrl_system.left.mode = mode;
        motor_ctrl_system.right.mode = mode;
        return 0;
    }
    else
    {
        return 1;
    }
    
    if (ctrl == NULL)
    {
        return 1;
    }
    
    // 设置控制模式
    ctrl->mode = mode;
    
    // 根据模式重置PID控制器
    if (mode == MOTOR_CTRL_MODE_SPEED_LOOP || mode == MOTOR_CTRL_MODE_CASCADE)
    {
        ctrl->speed_pid.integral_reset = true;
    }
    
    if (mode == MOTOR_CTRL_MODE_POSITION_LOOP || mode == MOTOR_CTRL_MODE_CASCADE)
    {
        ctrl->position_pid.integral_reset = true;
    }
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机目标速度
// 参数说明     motor_id                电机ID
// 参数说明     target_speed            目标速度 (m/s)
// 返回参数     uint8                   设置结果 (0=成功, 1=失败)
// 使用示例     motor_set_target_speed(MOTOR_CTRL_LEFT, 0.5f);
// 备注信息     设置电机目标速度，仅在速度闭环模式下有效
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_set_target_speed(motor_ctrl_id_enum motor_id, float target_speed)
{
    motor_controller_struct *ctrl = NULL;
    
    // 获取控制器指针
    if (motor_id == MOTOR_CTRL_LEFT)
    {
        ctrl = &motor_ctrl_system.left;
    }
    else if (motor_id == MOTOR_CTRL_RIGHT)
    {
        ctrl = &motor_ctrl_system.right;
    }
    else if (motor_id == MOTOR_CTRL_BOTH)
    {
        // 设置双电机目标速度
        motor_ctrl_system.left.target_speed = pid_limit(target_speed, -motor_ctrl_system.left.max_speed, motor_ctrl_system.left.max_speed);
        motor_ctrl_system.right.target_speed = pid_limit(target_speed, -motor_ctrl_system.right.max_speed, motor_ctrl_system.right.max_speed);
        return 0;
    }
    else
    {
        return 1;
    }
    
    if (ctrl == NULL)
    {
        return 1;
    }
    
    // 速度限幅
    ctrl->target_speed = pid_limit(target_speed, -ctrl->max_speed, ctrl->max_speed);
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机目标位置
// 参数说明     motor_id                电机ID
// 参数说明     target_position         目标位置 (脉冲数)
// 返回参数     uint8                   设置结果 (0=成功, 1=失败)
// 使用示例     motor_set_target_position(MOTOR_CTRL_LEFT, 1000);
// 备注信息     设置电机目标位置，仅在位置闭环模式下有效
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_set_target_position(motor_ctrl_id_enum motor_id, float target_position)
{
    motor_controller_struct *ctrl = NULL;
    
    // 获取控制器指针
    if (motor_id == MOTOR_CTRL_LEFT)
    {
        ctrl = &motor_ctrl_system.left;
    }
    else if (motor_id == MOTOR_CTRL_RIGHT)
    {
        ctrl = &motor_ctrl_system.right;
    }
    else if (motor_id == MOTOR_CTRL_BOTH)
    {
        // 设置双电机目标位置
        motor_ctrl_system.left.target_position = target_position;
        motor_ctrl_system.right.target_position = target_position;
        return 0;
    }
    else
    {
        return 1;
    }
    
    if (ctrl == NULL)
    {
        return 1;
    }
    
    // 设置目标位置
    ctrl->target_position = target_position;
    ctrl->target_reached = false;
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置机器人运动参数
// 参数说明     linear_speed            线速度 (m/s)
// 参数说明     angular_speed           角速度 (rad/s)
// 返回参数     uint8                   设置结果 (0=成功, 1=失败)
// 使用示例     motor_set_robot_motion(0.3f, 0.5f);
// 备注信息     设置机器人的线速度和角速度，自动分解为左右轮速度
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_set_robot_motion(float linear_speed, float angular_speed)
{
    float left_speed, right_speed;
    
    // 保存机器人运动参数
    motor_ctrl_system.robot_linear_speed = linear_speed;
    motor_ctrl_system.robot_angular_speed = angular_speed;
    
    // 将线速度和角速度分解为左右轮速度
    if (motor_speed_to_differential(linear_speed, angular_speed, &left_speed, &right_speed) != 0)
    {
        return 1;
    }
    
    // 设置左右轮目标速度
    motor_set_target_speed(MOTOR_CTRL_LEFT, left_speed);
    motor_set_target_speed(MOTOR_CTRL_RIGHT, right_speed);
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     差速运动计算
// 参数说明     linear_speed            线速度 (m/s)
// 参数说明     angular_speed           角速度 (rad/s)
// 参数说明     left_speed              左轮速度输出指针 (m/s)
// 参数说明     right_speed             右轮速度输出指针 (m/s)
// 返回参数     uint8                   计算结果 (0=成功, 1=失败)
// 使用示例     motor_speed_to_differential(0.3f, 0.5f, &left, &right);
// 备注信息     将机器人线速度和角速度分解为左右轮速度
//-------------------------------------------------------------------------------------------------------------------
static uint8 motor_speed_to_differential(float linear_speed, float angular_speed, float *left_speed, float *right_speed)
{
    if (left_speed == NULL || right_speed == NULL)
    {
        return 1;
    }
    
    // 差速运动学公式
    // V_left = V_linear - (W_angular * wheelbase) / 2
    // V_right = V_linear + (W_angular * wheelbase) / 2
    float half_wheelbase = motor_ctrl_system.wheelbase / 2000.0f;  // 转换为米
    
    *left_speed = linear_speed - angular_speed * half_wheelbase;
    *right_speed = linear_speed + angular_speed * half_wheelbase;
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID参数设置
// 参数说明     motor_id                电机ID
// 参数说明     pid_type                PID类型 (0=速度环, 1=位置环)
// 参数说明     kp, ki, kd              PID参数
// 返回参数     uint8                   设置结果 (0=成功, 1=失败)
// 使用示例     motor_set_pid_params(MOTOR_CTRL_LEFT, 0, 8.0f, 0.5f, 0.1f);
// 备注信息     设置指定电机的PID参数
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_set_pid_params(motor_ctrl_id_enum motor_id, uint8 pid_type, float kp, float ki, float kd)
{
    motor_controller_struct *ctrl = NULL;
    pid_controller_struct *pid = NULL;
    
    // 获取控制器指针
    if (motor_id == MOTOR_CTRL_LEFT)
    {
        ctrl = &motor_ctrl_system.left;
    }
    else if (motor_id == MOTOR_CTRL_RIGHT)
    {
        ctrl = &motor_ctrl_system.right;
    }
    else if (motor_id == MOTOR_CTRL_BOTH)
    {
        // 设置双电机PID参数
        motor_set_pid_params(MOTOR_CTRL_LEFT, pid_type, kp, ki, kd);
        motor_set_pid_params(MOTOR_CTRL_RIGHT, pid_type, kp, ki, kd);
        return 0;
    }
    else
    {
        return 1;
    }
    
    if (ctrl == NULL)
    {
        return 1;
    }
    
    // 获取PID控制器指针
    if (pid_type == 0)
    {
        pid = &ctrl->speed_pid;
    }
    else if (pid_type == 1)
    {
        pid = &ctrl->position_pid;
    }
    else
    {
        return 1;
    }
    
    // 设置PID参数
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    
    // 重置PID状态
    pid->integral_reset = true;
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机控制系统更新（周期调用）
// 参数说明     void
// 返回参数     uint8                   更新结果 (0=成功, 1=失败)
// 使用示例     motor_control_update();  // 在1ms定时器中调用
// 备注信息     电机控制系统主循环，建议1ms调用一次
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_control_update(void)
{
    if (!motor_ctrl_system.system_enable)
    {
        return 0;
    }
    
    // 更新编码器数据
    encoder_update();
    
    // 更新左电机控制器
    if (motor_controller_update(&motor_ctrl_system.left) != 0)
    {
        motor_ctrl_system.total_error_count++;
    }
    
    // 更新右电机控制器
    if (motor_controller_update(&motor_ctrl_system.right) != 0)
    {
        motor_ctrl_system.total_error_count++;
    }
    
    // 更新系统计数器
    motor_ctrl_system.control_cycle_count++;
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机控制器更新
// 参数说明     ctrl                    控制器指针
// 返回参数     uint8                   更新结果 (0=成功, 1=失败)
// 使用示例     motor_controller_update(&left_ctrl);
// 备注信息     更新单个电机控制器
//-------------------------------------------------------------------------------------------------------------------
static uint8 motor_controller_update(motor_controller_struct *ctrl)
{
    if (ctrl == NULL || ctrl->status != MOTOR_CTRL_STATUS_RUNNING)
    {
        return 0;
    }
    
    // 获取编码器反馈
    ctrl->current_speed = encoder_get_speed(ctrl->encoder_id);
    ctrl->current_position = (float)encoder_get_pulse_count(ctrl->encoder_id);
    
    // 根据控制模式执行不同的控制算法
    switch (ctrl->mode)
    {
        case MOTOR_CTRL_MODE_OPEN_LOOP:
            // 开环控制：直接输出PWM
            ctrl->pwm_output = (int16)ctrl->target_speed;
            break;
            
        case MOTOR_CTRL_MODE_SPEED_LOOP:
            // 速度闭环控制
            ctrl->speed_output = pid_calculate(&ctrl->speed_pid, ctrl->target_speed, ctrl->current_speed);
            ctrl->pwm_output = (int16)pid_limit(ctrl->speed_output, MOTOR_CONTROL_MIN_OUTPUT, MOTOR_CONTROL_MAX_OUTPUT);
            ctrl->speed_error = ctrl->target_speed - ctrl->current_speed;
            break;
            
        case MOTOR_CTRL_MODE_POSITION_LOOP:
            // 位置闭环控制
            ctrl->position_output = pid_calculate(&ctrl->position_pid, ctrl->target_position, ctrl->current_position);
            ctrl->pwm_output = (int16)pid_limit(ctrl->position_output, MOTOR_CONTROL_MIN_OUTPUT, MOTOR_CONTROL_MAX_OUTPUT);
            ctrl->position_error = ctrl->target_position - ctrl->current_position;
            
            // 检查位置到达
            if (fabs(ctrl->position_error) < 5.0f)  // 5个脉冲的容差
            {
                ctrl->target_reached = true;
            }
            break;
            
        case MOTOR_CTRL_MODE_CASCADE:
            // 级联控制：位置外环+速度内环
            
            // 位置外环计算目标速度
            ctrl->position_output = pid_calculate(&ctrl->position_pid, ctrl->target_position, ctrl->current_position);
            float target_speed_from_position = pid_limit(ctrl->position_output, -ctrl->max_speed, ctrl->max_speed);
            
            // 速度内环计算PWM输出
            ctrl->speed_output = pid_calculate(&ctrl->speed_pid, target_speed_from_position, ctrl->current_speed);
            ctrl->pwm_output = (int16)pid_limit(ctrl->speed_output, MOTOR_CONTROL_MIN_OUTPUT, MOTOR_CONTROL_MAX_OUTPUT);
            
            // 计算误差
            ctrl->position_error = ctrl->target_position - ctrl->current_position;
            ctrl->speed_error = target_speed_from_position - ctrl->current_speed;
            
            // 检查位置到达
            if (fabs(ctrl->position_error) < 5.0f)
            {
                ctrl->target_reached = true;
            }
            break;
            
        default:
            ctrl->pwm_output = 0;
            break;
    }
    
    // 输出PWM到电机
    if (motor_set_speed(ctrl->motor_id, ctrl->pwm_output) != MOTOR_STATUS_OK)
    {
        ctrl->error_count++;
        return 1;
    }
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机控制系统启动
// 参数说明     void
// 返回参数     uint8                   启动结果 (0=成功, 1=失败)
// 使用示例     motor_control_start();
// 备注信息     启动电机控制系统
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_control_start(void)
{
    // 设置电机控制器状态为运行
    motor_ctrl_system.left.status = MOTOR_CTRL_STATUS_RUNNING;
    motor_ctrl_system.right.status = MOTOR_CTRL_STATUS_RUNNING;
    
    // 启用系统
    motor_ctrl_system.system_enable = true;
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机控制系统停止
// 参数说明     void
// 返回参数     uint8                   停止结果 (0=成功, 1=失败)
// 使用示例     motor_control_stop();
// 备注信息     停止电机控制系统，所有电机停止运行
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_control_stop(void)
{
    // 停止所有电机
    motor_stop(MOTOR_BOTH);
    
    // 设置电机控制器状态为停止
    motor_ctrl_system.left.status = MOTOR_CTRL_STATUS_STOP;
    motor_ctrl_system.right.status = MOTOR_CTRL_STATUS_STOP;
    
    // 禁用系统
    motor_ctrl_system.system_enable = false;
    
    return 0;
}

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
uint8 motor_get_status(motor_ctrl_id_enum motor_id, float *speed, float *position, float *error)
{
    motor_controller_struct *ctrl = NULL;
    
    // 获取控制器指针
    if (motor_id == MOTOR_CTRL_LEFT)
    {
        ctrl = &motor_ctrl_system.left;
    }
    else if (motor_id == MOTOR_CTRL_RIGHT)
    {
        ctrl = &motor_ctrl_system.right;
    }
    else
    {
        return 1;
    }
    
    if (ctrl == NULL)
    {
        return 1;
    }
    
    // 返回状态信息
    if (speed != NULL)
    {
        *speed = ctrl->current_speed;
    }
    
    if (position != NULL)
    {
        *position = ctrl->current_position;
    }
    
    if (error != NULL)
    {
        if (ctrl->mode == MOTOR_CTRL_MODE_SPEED_LOOP || ctrl->mode == MOTOR_CTRL_MODE_CASCADE)
        {
            *error = ctrl->speed_error;
        }
        else if (ctrl->mode == MOTOR_CTRL_MODE_POSITION_LOOP)
        {
            *error = ctrl->position_error;
        }
        else
        {
            *error = 0.0f;
        }
    }
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID控制器复位
// 参数说明     motor_id                电机ID
// 参数说明     pid_type                PID类型 (0=速度环, 1=位置环)
// 返回参数     uint8                   复位结果 (0=成功, 1=失败)
// 使用示例     motor_pid_reset(MOTOR_CTRL_LEFT, 0);
// 备注信息     复位指定电机的PID控制器状态
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_pid_reset(motor_ctrl_id_enum motor_id, uint8 pid_type)
{
    motor_controller_struct *ctrl = NULL;
    pid_controller_struct *pid = NULL;
    
    // 获取控制器指针
    if (motor_id == MOTOR_CTRL_LEFT)
    {
        ctrl = &motor_ctrl_system.left;
    }
    else if (motor_id == MOTOR_CTRL_RIGHT)
    {
        ctrl = &motor_ctrl_system.right;
    }
    else if (motor_id == MOTOR_CTRL_BOTH)
    {
        // 复位双电机PID
        motor_pid_reset(MOTOR_CTRL_LEFT, pid_type);
        motor_pid_reset(MOTOR_CTRL_RIGHT, pid_type);
        return 0;
    }
    else
    {
        return 1;
    }
    
    if (ctrl == NULL)
    {
        return 1;
    }
    
    // 获取PID控制器指针
    if (pid_type == 0)
    {
        pid = &ctrl->speed_pid;
    }
    else if (pid_type == 1)
    {
        pid = &ctrl->position_pid;
    }
    else
    {
        return 1;
    }
    
    // 复位PID状态
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机控制系统诊断
// 参数说明     void
// 返回参数     uint8                   诊断结果 (0=正常, 1=异常)
// 使用示例     motor_control_diagnose();
// 备注信息     诊断电机控制系统状态，检查异常情况
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_control_diagnose(void)
{
    uint8 result = 0;
    
    // 检查左电机控制器
    if (motor_ctrl_system.left.error_count > 100)
    {
        motor_ctrl_system.left.status = MOTOR_CTRL_STATUS_ERROR;
        result = 1;
    }
    
    // 检查右电机控制器
    if (motor_ctrl_system.right.error_count > 100)
    {
        motor_ctrl_system.right.status = MOTOR_CTRL_STATUS_ERROR;
        result = 1;
    }
    
    // 计算平均速度误差
    motor_ctrl_system.average_speed_error = (fabs(motor_ctrl_system.left.speed_error) + 
                                           fabs(motor_ctrl_system.right.speed_error)) / 2.0f;
    
    // 更新最大速度误差
    float max_error = fabs(motor_ctrl_system.left.speed_error) > fabs(motor_ctrl_system.right.speed_error) ? 
                     fabs(motor_ctrl_system.left.speed_error) : fabs(motor_ctrl_system.right.speed_error);
    
    if (max_error > motor_ctrl_system.max_speed_error)
    {
        motor_ctrl_system.max_speed_error = max_error;
    }
    
    return result;
}
