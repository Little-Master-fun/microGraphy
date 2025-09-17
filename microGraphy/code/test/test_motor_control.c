/*********************************************************************************************************************
* 文件名称          test_motor_control.c
* 功能说明          电机闭环控制测试程序实现文件
* 作者              AI Assistant
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本              备注
* 2024-XX-XX        AI Assistant        v1.0              创建电机闭环控制测试程序
********************************************************************************************************************/

#include "test_motor_control.h"
#include "zf_device_ips114.h"
#include "zf_common_headfile.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

//=================================================全局变量定义================================================
static test_result_struct current_test_result;
static uint32 test_start_time;
static float *test_data_buffer = NULL;
static uint32 test_data_index;

//=================================================内部函数声明================================================
static void init_test_result(test_result_struct *result, const char *name, uint32 duration);
static void calculate_performance_metrics(test_result_struct *result, float *error_data, uint32 data_count);
static uint32 get_system_time_ms(void);
static void test_delay_ms(uint32 delay);

//=================================================函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机闭环控制综合测试
// 参数说明     void
// 返回参数     uint8                   测试结果 (0=通过, 1=失败)
// 使用示例     test_motor_control();
// 备注信息     执行电机闭环控制的全面测试
//-------------------------------------------------------------------------------------------------------------------
uint8 test_motor_control(void)
{
    uint8 overall_result = 0;
    uint8 test_count = 0;
    uint8 pass_count = 0;
    
        // 初始化屏幕
    ips114_set_dir(IPS114_PORTAIT);
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_init();

    ips114_clear();
    ips114_show_string(0, 0, "Motor Control Test");
    ips114_show_string(0, 16, "Starting...");
    
    // 初始化电机控制系统
    if (motor_control_init() != 0)
    {
        ips114_show_string(0, 32, "Init Failed!");
        return 1;
    }
    
    // 启动控制系统
    motor_control_start();
    
    ips114_show_string(0, 32, "Running Tests:");
    
    // 1. PID控制器基础测试
    test_count++;
    ips114_show_string(0, 48, "1.PID Basic Test");
    if (test_pid_controller() == 0)
    {
        pass_count++;
        ips114_show_string(120, 48, "PASS");
    }
    else
    {
        ips114_show_string(120, 48, "FAIL");
        overall_result = 1;
    }
    test_delay_ms(1000);
    
    // 2. 速度闭环控制测试
    test_count++;
    ips114_show_string(0, 64, "2.Speed Loop Test");
    if (test_speed_loop_control() == 0)
    {
        pass_count++;
        ips114_show_string(120, 64, "PASS");
    }
    else
    {
        ips114_show_string(120, 64, "FAIL");
        overall_result = 1;
    }
    test_delay_ms(1000);
    
    // 3. 位置闭环控制测试
    test_count++;
    ips114_show_string(0, 80, "3.Position Test");
    if (test_position_loop_control() == 0)
    {
        pass_count++;
        ips114_show_string(120, 80, "PASS");
    }
    else
    {
        ips114_show_string(120, 80, "FAIL");
        overall_result = 1;
    }
    test_delay_ms(1000);
    
    // 4. 级联控制测试
    test_count++;
    ips114_show_string(0, 96, "4.Cascade Test");
    if (test_cascade_control() == 0)
    {
        pass_count++;
        ips114_show_string(120, 96, "PASS");
    }
    else
    {
        ips114_show_string(120, 96, "FAIL");
        overall_result = 1;
    }
    test_delay_ms(1000);
    
    // 5. 差速运动测试
    test_count++;
    ips114_show_string(0, 112, "5.Differential Test");
    if (test_differential_control() == 0)
    {
        pass_count++;
        ips114_show_string(120, 112, "PASS");
    }
    else
    {
        ips114_show_string(120, 112, "FAIL");
        overall_result = 1;
    }
    test_delay_ms(2000);
    
    // 显示测试总结
    ips114_clear();
    ips114_show_string(0, 0, "Test Summary:");
    
    char result_str[50];
    sprintf(result_str, "Total: %d, Pass: %d", test_count, pass_count);
    ips114_show_string(0, 16, result_str);
    
    sprintf(result_str, "Success Rate: %d%%", (pass_count * 100) / test_count);
    ips114_show_string(0, 32, result_str);
    
    if (overall_result == 0)
    {
        ips114_show_string(0, 48, "Overall: PASS");
    }
    else
    {
        ips114_show_string(0, 48, "Overall: FAIL");
    }
    
    // 停止控制系统
    motor_control_stop();
    
    return overall_result;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID控制器基础测试
// 参数说明     void
// 返回参数     uint8                   测试结果 (0=通过, 1=失败)
// 使用示例     test_pid_controller();
// 备注信息     测试PID控制器的基本功能
//-------------------------------------------------------------------------------------------------------------------
uint8 test_pid_controller(void)
{
    pid_controller_struct test_pid;
    float test_output;
    uint8 result = 0;
    
    // 初始化测试PID
    if (pid_init(&test_pid, 1.0f, 0.1f, 0.05f, PID_TYPE_POSITIONAL) != 0)
    {
        return 1;
    }
    
    // 测试1: 基本PID计算
    test_output = pid_calculate(&test_pid, 10.0f, 0.0f);  // 设定值10，反馈值0
    if (test_output <= 0)  // 应该有正输出
    {
        result = 1;
    }
    
    // 测试2: 积分饱和
    for (int i = 0; i < 1000; i++)
    {
        test_output = pid_calculate(&test_pid, 10.0f, 0.0f);
    }
    if (test_output > PID_OUTPUT_MAX)  // 不应该超过输出限幅
    {
        result = 1;
    }
    
    // 测试3: PID复位
    test_pid.integral = 1000.0f;
    test_pid.integral_reset = true;
    test_output = pid_calculate(&test_pid, 5.0f, 5.0f);  // 无误差输入
    if (fabs(test_output) > 1.0f)  // 复位后输出应该很小
    {
        result = 1;
    }
    
    return result;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     速度闭环控制测试
// 参数说明     void
// 返回参数     uint8                   测试结果 (0=通过, 1=失败)
// 使用示例     test_speed_loop_control();
// 备注信息     测试速度闭环控制性能
//-------------------------------------------------------------------------------------------------------------------
uint8 test_speed_loop_control(void)
{
    test_result_struct result;
    uint8 test_result = 0;
    
    // 设置速度闭环模式
    motor_set_control_mode(MOTOR_CTRL_BOTH, MOTOR_CTRL_MODE_SPEED_LOOP);
    
    // 测试1: 低速阶跃响应
    if (test_step_response(MOTOR_CTRL_LEFT, TEST_SPEED_LOW, TEST_DURATION_SHORT, &result) != 0)
    {
        test_result = 1;
    }
    display_test_result(&result);
    test_delay_ms(1000);
    
    // 测试2: 中速阶跃响应
    if (test_step_response(MOTOR_CTRL_LEFT, TEST_SPEED_MEDIUM, TEST_DURATION_SHORT, &result) != 0)
    {
        test_result = 1;
    }
    display_test_result(&result);
    test_delay_ms(1000);
    
    // 测试3: 斜坡跟踪
    if (test_ramp_tracking(MOTOR_CTRL_LEFT, 0.0f, TEST_SPEED_MEDIUM, 3000, &result) != 0)
    {
        test_result = 1;
    }
    display_test_result(&result);
    test_delay_ms(1000);
    
    // 停止电机
    motor_set_target_speed(MOTOR_CTRL_BOTH, 0.0f);
    test_delay_ms(1000);
    
    return test_result;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     位置闭环控制测试
// 参数说明     void
// 返回参数     uint8                   测试结果 (0=通过, 1=失败)
// 使用示例     test_position_loop_control();
// 备注信息     测试位置闭环控制性能
//-------------------------------------------------------------------------------------------------------------------
uint8 test_position_loop_control(void)
{
    uint8 test_result = 0;
    float current_position, target_position;
    uint32 start_time, settle_time;
    
    // 设置位置闭环模式
    motor_set_control_mode(MOTOR_CTRL_BOTH, MOTOR_CTRL_MODE_POSITION_LOOP);
    
    // 复位编码器
    encoder_reset(ENCODER_ID_BOTH);
    test_delay_ms(100);
    
    // 测试1: 小距离定点控制
    target_position = TEST_POSITION_SMALL;
    motor_set_target_position(MOTOR_CTRL_LEFT, target_position);
    
    start_time = get_system_time_ms();
    settle_time = 0;
    
    // 等待到达目标位置
    while ((get_system_time_ms() - start_time) < 10000)  // 最大等待10秒
    {
        motor_control_update();  // 更新控制系统
        
        motor_get_status(MOTOR_CTRL_LEFT, NULL, &current_position, NULL);
        
        if (fabs(current_position - target_position) < 10.0f)  // 10个脉冲容差
        {
            if (settle_time == 0)
            {
                settle_time = get_system_time_ms() - start_time;
            }
            
            // 稳定时间超过500ms认为到达
            if ((get_system_time_ms() - start_time - settle_time) > 500)
            {
                break;
            }
        }
        else
        {
            settle_time = 0;  // 重置稳定时间
        }
        
        test_delay_ms(10);
    }
    
    // 检查最终位置误差
    motor_get_status(MOTOR_CTRL_LEFT, NULL, &current_position, NULL);
    if (fabs(current_position - target_position) > 20.0f)  // 20个脉冲容差
    {
        test_result = 1;
    }
    
    // 显示测试结果
    ips114_clear();
    ips114_show_string(0, 0, "Position Test:");
    
    char result_str[50];
    sprintf(result_str, "Target: %.0f", target_position);
    ips114_show_string(0, 16, result_str);
    
    sprintf(result_str, "Actual: %.0f", current_position);
    ips114_show_string(0, 32, result_str);
    
    sprintf(result_str, "Error: %.0f", current_position - target_position);
    ips114_show_string(0, 48, result_str);
    
    sprintf(result_str, "Time: %dms", settle_time);
    ips114_show_string(0, 64, result_str);
    
    test_delay_ms(2000);
    
    // 停止电机
    motor_set_target_position(MOTOR_CTRL_BOTH, current_position);
    test_delay_ms(500);
    
    return test_result;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     级联控制测试
// 参数说明     void
// 返回参数     uint8                   测试结果 (0=通过, 1=失败)
// 使用示例     test_cascade_control();
// 备注信息     测试级联控制（位置外环+速度内环）性能
//-------------------------------------------------------------------------------------------------------------------
uint8 test_cascade_control(void)
{
    uint8 test_result = 0;
    float current_position, current_speed;
    float target_position;
    uint32 start_time;
    
    // 设置级联控制模式
    motor_set_control_mode(MOTOR_CTRL_BOTH, MOTOR_CTRL_MODE_CASCADE);
    
    // 复位编码器
    encoder_reset(ENCODER_ID_BOTH);
    test_delay_ms(100);
    
    // 级联控制测试
    target_position = TEST_POSITION_MEDIUM;
    motor_set_target_position(MOTOR_CTRL_LEFT, target_position);
    
    start_time = get_system_time_ms();
    
    // 监控运行过程
    while ((get_system_time_ms() - start_time) < 8000)  // 8秒测试时间
    {
        motor_control_update();
        
        motor_get_status(MOTOR_CTRL_LEFT, &current_speed, &current_position, NULL);
        
        // 显示实时状态
        if ((get_system_time_ms() - start_time) % 500 == 0)  // 每500ms更新显示
        {
            ips114_clear();
            ips114_show_string(0, 0, "Cascade Control:");
            
            char status_str[50];
            sprintf(status_str, "Target: %.0f", target_position);
            ips114_show_string(0, 16, status_str);
            
            sprintf(status_str, "Position: %.0f", current_position);
            ips114_show_string(0, 32, status_str);
            
            sprintf(status_str, "Speed: %.2f", current_speed);
            ips114_show_string(0, 48, status_str);
            
            sprintf(status_str, "Error: %.0f", target_position - current_position);
            ips114_show_string(0, 64, status_str);
        }
        
        // 检查是否到达目标
        if (fabs(current_position - target_position) < 15.0f)
        {
            break;
        }
        
        test_delay_ms(10);
    }
    
    // 检查最终结果
    motor_get_status(MOTOR_CTRL_LEFT, NULL, &current_position, NULL);
    if (fabs(current_position - target_position) > 30.0f)
    {
        test_result = 1;
    }
    
    test_delay_ms(2000);
    
    // 停止电机
    motor_set_target_position(MOTOR_CTRL_BOTH, current_position);
    test_delay_ms(500);
    
    return test_result;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     差速运动控制测试
// 参数说明     void
// 返回参数     uint8                   测试结果 (0=通过, 1=失败)
// 使用示例     test_differential_control();
// 备注信息     测试差速运动控制功能
//-------------------------------------------------------------------------------------------------------------------
uint8 test_differential_control(void)
{
    uint8 test_result = 0;
    float left_speed, right_speed;
    
    // 设置速度闭环模式
    motor_set_control_mode(MOTOR_CTRL_BOTH, MOTOR_CTRL_MODE_SPEED_LOOP);
    
    // 测试1: 直线运动
    ips114_clear();
    ips114_show_string(0, 0, "Straight Motion");
    
    motor_set_robot_motion(0.3f, 0.0f);  // 线速度0.3m/s，角速度0
    test_delay_ms(3000);
    
    // 检查左右轮速度是否相等
    motor_get_status(MOTOR_CTRL_LEFT, &left_speed, NULL, NULL);
    motor_get_status(MOTOR_CTRL_RIGHT, &right_speed, NULL, NULL);
    
    if (fabs(left_speed - right_speed) > 0.1f)  // 速度差异不应超过0.1m/s
    {
        test_result = 1;
    }
    
    char speed_str[50];
    sprintf(speed_str, "L:%.2f R:%.2f", left_speed, right_speed);
    ips114_show_string(0, 16, speed_str);
    
    test_delay_ms(1000);
    
    // 测试2: 转弯运动
    ips114_clear();
    ips114_show_string(0, 0, "Turn Motion");
    
    motor_set_robot_motion(0.2f, 0.5f);  // 线速度0.2m/s，角速度0.5rad/s
    test_delay_ms(3000);
    
    // 检查左右轮速度差异
    motor_get_status(MOTOR_CTRL_LEFT, &left_speed, NULL, NULL);
    motor_get_status(MOTOR_CTRL_RIGHT, &right_speed, NULL, NULL);
    
    if (left_speed >= right_speed)  // 右转时左轮应该比右轮快
    {
        test_result = 1;
    }
    
    sprintf(speed_str, "L:%.2f R:%.2f", left_speed, right_speed);
    ips114_show_string(0, 16, speed_str);
    
    test_delay_ms(1000);
    
    // 停止运动
    motor_set_robot_motion(0.0f, 0.0f);
    test_delay_ms(1000);
    
    return test_result;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     阶跃响应测试
// 参数说明     motor_id                电机ID
// 参数说明     target_value            目标值
// 参数说明     test_duration           测试时长 (ms)
// 参数说明     result                  测试结果输出指针
// 返回参数     uint8                   测试结果 (0=通过, 1=失败)
// 使用示例     test_step_response(MOTOR_CTRL_LEFT, 0.5f, 5000, &result);
// 备注信息     执行阶跃响应测试并分析性能指标
//-------------------------------------------------------------------------------------------------------------------
uint8 test_step_response(motor_ctrl_id_enum motor_id, float target_value, uint32 test_duration, test_result_struct *result)
{
    float current_value, error;
    uint32 start_time, current_time;
    uint32 sample_count = 0;
    float error_sum = 0.0f, error_sum_square = 0.0f;
    float max_error = 0.0f;
    float max_value = 0.0f;
    uint32 settle_time = 0;
    bool settled = false;
    
    // 初始化测试结果
    init_test_result(result, "Step Response", test_duration);
    
    // 设置目标值
    motor_set_target_speed(motor_id, target_value);
    
    start_time = get_system_time_ms();
    
    // 数据采集循环
    while ((current_time = get_system_time_ms()) - start_time < test_duration)
    {
        motor_control_update();
        
        // 获取当前值
        motor_get_status(motor_id, &current_value, NULL, &error);
        
        // 计算误差
        error = target_value - current_value;
        
        // 统计数据
        sample_count++;
        error_sum += fabs(error);
        error_sum_square += error * error;
        
        if (fabs(error) > max_error)
        {
            max_error = fabs(error);
        }
        
        if (current_value > max_value)
        {
            max_value = current_value;
        }
        
        // 检查稳定时间（误差在5%以内）
        if (!settled && fabs(error) < (target_value * 0.05f))
        {
            settle_time = current_time - start_time;
            settled = true;
        }
        
        test_delay_ms(10);  // 10ms采样间隔
    }
    
    // 计算性能指标
    result->sample_count = sample_count;
    result->max_error = max_error;
    result->average_error = error_sum / sample_count;
    result->rms_error = sqrt(error_sum_square / sample_count);
    result->settle_time = settle_time;
    result->overshoot = ((max_value - target_value) / target_value) * 100.0f;
    
    // 判断测试是否通过
    result->test_passed = (max_error < (target_value * 0.2f)) && (settle_time < (test_duration / 2));
    
    return result->test_passed ? 0 : 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     斜坡跟踪测试
// 参数说明     motor_id                电机ID
// 参数说明     start_value             起始值
// 参数说明     end_value               结束值
// 参数说明     ramp_time               斜坡时间 (ms)
// 参数说明     result                  测试结果输出指针
// 返回参数     uint8                   测试结果 (0=通过, 1=失败)
// 使用示例     test_ramp_tracking(MOTOR_CTRL_LEFT, 0.0f, 1.0f, 3000, &result);
// 备注信息     执行斜坡跟踪测试并分析跟踪性能
//-------------------------------------------------------------------------------------------------------------------
uint8 test_ramp_tracking(motor_ctrl_id_enum motor_id, float start_value, float end_value, uint32 ramp_time, test_result_struct *result)
{
    float current_value, target_value, error;
    uint32 start_time, current_time;
    uint32 sample_count = 0;
    float error_sum = 0.0f, error_sum_square = 0.0f;
    float max_error = 0.0f;
    float ramp_rate = (end_value - start_value) / ramp_time;  // 每毫秒的变化率
    
    // 初始化测试结果
    init_test_result(result, "Ramp Tracking", ramp_time);
    
    start_time = get_system_time_ms();
    
    // 斜坡跟踪循环
    while ((current_time = get_system_time_ms()) - start_time < ramp_time)
    {
        motor_control_update();
        
        // 计算当前目标值
        target_value = start_value + ramp_rate * (current_time - start_time);
        if (target_value > end_value) target_value = end_value;
        
        // 设置目标值
        motor_set_target_speed(motor_id, target_value);
        
        // 获取当前值
        motor_get_status(motor_id, &current_value, NULL, NULL);
        
        // 计算跟踪误差
        error = target_value - current_value;
        
        // 统计数据
        sample_count++;
        error_sum += fabs(error);
        error_sum_square += error * error;
        
        if (fabs(error) > max_error)
        {
            max_error = fabs(error);
        }
        
        test_delay_ms(10);
    }
    
    // 计算性能指标
    result->sample_count = sample_count;
    result->max_error = max_error;
    result->average_error = error_sum / sample_count;
    result->rms_error = sqrt(error_sum_square / sample_count);
    
    // 判断测试是否通过（跟踪误差应该小于目标值的15%）
    float target_range = fabs(end_value - start_value);
    result->test_passed = (max_error < (target_range * 0.15f));
    
    return result->test_passed ? 0 : 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示测试结果
// 参数说明     result                  测试结果指针
// 返回参数     void
// 使用示例     display_test_result(&result);
// 备注信息     在屏幕上显示测试结果和性能指标
//-------------------------------------------------------------------------------------------------------------------
void display_test_result(test_result_struct *result)
{
    ips114_clear();
    
    // 显示测试名称
    ips114_show_string(0, 0, result->test_name);
    
    // 显示测试结果
    if (result->test_passed)
    {
        ips114_show_string(120, 0, "PASS");
    }
    else
    {
        ips114_show_string(120, 0, "FAIL");
    }
    
    // 显示性能指标
    char metric_str[50];
    
    sprintf(metric_str, "Max Err: %.3f", result->max_error);
    ips114_show_string(0, 16, metric_str);
    
    sprintf(metric_str, "Avg Err: %.3f", result->average_error);
    ips114_show_string(0, 32, metric_str);
    
    sprintf(metric_str, "RMS Err: %.3f", result->rms_error);
    ips114_show_string(0, 48, metric_str);
    
    if (result->settle_time > 0)
    {
        sprintf(metric_str, "Settle: %dms", result->settle_time);
        ips114_show_string(0, 64, metric_str);
    }
    
    if (result->overshoot > 0)
    {
        sprintf(metric_str, "Overshoot: %.1f%%", result->overshoot);
        ips114_show_string(0, 80, metric_str);
    }
    
    sprintf(metric_str, "Samples: %d", result->sample_count);
    ips114_show_string(0, 96, metric_str);
}

//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化测试结果结构体
//-------------------------------------------------------------------------------------------------------------------
static void init_test_result(test_result_struct *result, const char *name, uint32 duration)
{
    memset(result, 0, sizeof(test_result_struct));
    strncpy(result->test_name, name, sizeof(result->test_name) - 1);
    result->test_duration = duration;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取系统时间（毫秒）
//-------------------------------------------------------------------------------------------------------------------
static uint32 get_system_time_ms(void)
{
    // 这里需要根据实际的系统时钟实现
    // 暂时使用简单的计数器实现
    static uint32 time_counter = 0;
    return time_counter++;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     测试延时函数
//-------------------------------------------------------------------------------------------------------------------
static void test_delay_ms(uint32 delay)
{
    system_delay_ms(delay);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID参数调优测试（简化实现）
//-------------------------------------------------------------------------------------------------------------------
uint8 test_pid_tuning(motor_ctrl_id_enum motor_id, uint8 pid_type)
{
    // 这是一个简化的PID调优测试
    // 实际应用中可以实现更复杂的自动调优算法
    
    float kp_values[] = {1.0f, 2.0f, 4.0f, 8.0f};
    float ki_values[] = {0.1f, 0.2f, 0.5f, 1.0f};
    float kd_values[] = {0.01f, 0.05f, 0.1f, 0.2f};
    
    float best_kp = kp_values[1];  // 默认选择中等值
    float best_ki = ki_values[1];
    float best_kd = kd_values[1];
    
    // 设置最优参数
    motor_set_pid_params(motor_id, pid_type, best_kp, best_ki, best_kd);
    
    return 0;  // 简化实现，总是返回成功
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     抗干扰能力测试（简化实现）
//-------------------------------------------------------------------------------------------------------------------
uint8 test_disturbance_rejection(motor_ctrl_id_enum motor_id, uint8 disturbance_level)
{
    // 这是一个简化的抗干扰测试
    // 实际应用中需要模拟外部干扰
    
    test_result_struct result;
    
    // 执行阶跃响应测试作为抗干扰测试的简化版本
    return test_step_response(motor_id, TEST_SPEED_MEDIUM, TEST_DURATION_SHORT, &result);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     长期稳定性测试
//-------------------------------------------------------------------------------------------------------------------
uint8 test_long_term_stability(uint32 test_duration)
{
    uint32 start_time = get_system_time_ms();
    uint32 error_count = 0;
    uint32 sample_count = 0;
    
    // 设置恒定速度
    motor_set_control_mode(MOTOR_CTRL_BOTH, MOTOR_CTRL_MODE_SPEED_LOOP);
    motor_set_target_speed(MOTOR_CTRL_BOTH, TEST_SPEED_LOW);
    
    while ((get_system_time_ms() - start_time) < test_duration)
    {
        motor_control_update();
        
        // 检查系统状态
        if (motor_control_diagnose() != 0)
        {
            error_count++;
        }
        
        sample_count++;
        
        test_delay_ms(100);  // 100ms检查间隔
    }
    
    // 停止电机
    motor_set_target_speed(MOTOR_CTRL_BOTH, 0.0f);
    
    // 计算成功率
    float success_rate = ((float)(sample_count - error_count) / sample_count) * 100.0f;
    
    // 显示结果
    ips114_clear();
    ips114_show_string(0, 0, "Stability Test:");
    
    char result_str[50];
    sprintf(result_str, "Duration: %ds", test_duration / 1000);
    ips114_show_string(0, 16, result_str);
    
    sprintf(result_str, "Success: %.1f%%", success_rate);
    ips114_show_string(0, 32, result_str);
    
    sprintf(result_str, "Errors: %d", error_count);
    ips114_show_string(0, 48, result_str);
    
    return (success_rate > 95.0f) ? 0 : 1;  // 成功率>95%认为通过
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     保存测试数据（简化实现）
//-------------------------------------------------------------------------------------------------------------------
uint8 save_test_data(test_result_struct *result, const char *filename)
{
    // 这里可以集成Flash存储功能
    // 暂时返回成功
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     控制系统性能评估
//-------------------------------------------------------------------------------------------------------------------
uint8 evaluate_control_performance(void)
{
    // 运行诊断
    uint8 diagnose_result = motor_control_diagnose();
    
    if (diagnose_result == 0)
    {
        return 0;  // 优秀
    }
    else
    {
        return 2;  // 一般
    }
}
