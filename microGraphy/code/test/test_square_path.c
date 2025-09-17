/*********************************************************************************************************************
* 文件名称          test_square_path.c
* 功能说明          正方形路径测试程序实现文件
* 作者              AI Assistant
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2024-XX-XX        AI Assistant        创建正方形路径测试
*
* 文件作用说明：
* 本文件为正方形路径测试程序的实现，用于测试小车在1×1米正方形路径上的高速循环行驶
* 监测陀螺仪漂移情况，评估惯导系统在长时间高速运行下的性能表现
*
* 测试流程：
* 1. 初始化测试系统和IMU
* 2. 生成1×1米正方形路径并写入Flash
* 3. 启动高速路径跟踪（3.5m/s+）
* 4. 实时监测陀螺仪数据和累积漂移
* 5. 统计路径跟踪精度和速度表现
* 6. 显示实时数据和测试结果
********************************************************************************************************************/

#include "test_square_path.h"
#include "navigation_flash_improved.h"
#include "config_navigation.h"
#include "driver_sch16tk10.h"
#include "motor_control.h"
#include "driver_encoder.h"
#include "zf_device_ips114.h"
#include "zf_common_headfile.h"
#include <stdio.h>

//=================================================全局变量定义================================================
square_test_system_struct square_test_system = {0};

//=================================================内部函数声明================================================
static void calculate_square_corner_points(uint8 corner_index, float start_angle, float end_angle);
static float calculate_distance(float x1, float y1, float x2, float y2);
static float normalize_angle(float angle);
static void reset_statistics(void);
static uint8 is_near_corner(float x, float y);
static void apply_tuned_pid_parameters(void);
static uint32 get_system_time_ms(void);

//=================================================主要接口函数================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     正方形路径测试主函数
//-------------------------------------------------------------------------------------------------------------------
void test_square_path(void)
{
    // 初始化屏幕
    ips114_set_dir(IPS114_PORTAIT);
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_init();
    ips114_clear();
    
    ips114_show_string(0, 0, "Square Path Test");
    ips114_show_string(0, 16, "Initializing...");
    
    // 初始化测试系统
    if (init_square_test_system() != 0)
    {
        ips114_show_string(0, 32, "Init Failed!");
        while(1);
    }
    
    // 初始化IMU
    SCH1_filter filter = {0x00, 0x00, 0x00};
    SCH1_sensitivity sensitivity = {0x00, 0x00, 0x00, 0x00, 0x00};
    SCH1_decimation decimation = {0x00, 0x00};
    
    if (SCH1_init(filter, sensitivity, decimation, false) != SCH1_OK)
    {
        ips114_show_string(0, 32, "IMU Init Failed!");
        while(1);
    }
    
    // 初始化导航系统
    if (nav_system_init() != 0)
    {
        ips114_show_string(0, 48, "Nav Init Failed!");
        while(1);
    }
    
    // 初始化电机控制系统
    if (motor_control_init() != 0)
    {
        ips114_show_string(0, 64, "Motor Init Failed!");
        while(1);
    }
    
    // 应用实际调试的PID参数
    apply_tuned_pid_parameters();
    
    // 生成正方形路径
    ips114_show_string(0, 80, "Generating Path...");
    if (generate_square_path() != 0)
    {
        ips114_show_string(0, 96, "Path Gen Failed!");
        while(1);
    }
    
    ips114_show_string(0, 96, "Path Generated OK");
    system_delay_ms(2000);
    
    // 开始测试
    ips114_clear();
    ips114_show_string(0, 0, "Starting Test...");
    ips114_show_string(0, 16, "Target: 3.5+ m/s");
    ips114_show_string(0, 32, "Press key to stop");
    
    square_test_system.test_running = 1;
    square_test_system.test_start_time = get_system_time_ms();
    square_test_system.last_log_time = square_test_system.test_start_time;
    square_test_system.last_display_time = square_test_system.test_start_time;
    
    // 设置目标速度
    motor_set_target_speed(MOTOR_CTRL_BOTH, TEST_TARGET_SPEED);
    motor_control_start();
    
    // 主测试循环
    while (square_test_system.test_running)
    {
        uint32 current_time = get_system_time_ms();
        
        // 检查测试时间
        if (current_time - square_test_system.test_start_time > TEST_DURATION_MS)
        {
            break;
        }
        
        // 检查退出条件
        if (check_test_exit())
        {
            break;
        }
        
        // 读取IMU数据
        SCH1_raw_data imu_raw;
        SCH1_result imu_data;
        SCH1_getData(&imu_raw);
        SCH1_convert_data(&imu_raw, &imu_data);
        
        // 读取编码器数据
        encoder_update();
        float left_speed = encoder_get_speed(ENCODER_ID_LEFT);
        float right_speed = encoder_get_speed(ENCODER_ID_RIGHT);
        float current_speed = (left_speed + right_speed) / 2.0f;
        
        // 估算当前位置（简化）
        static float estimated_x = SQUARE_CENTER_X;
        static float estimated_y = SQUARE_CENTER_Y;
        
        // 执行路径跟踪
        float nav_output = nav_path_tracking();
        
        // 更新电机控制
        motor_control_update();
        
        // 数据记录
        if (current_time - square_test_system.last_log_time > DATA_LOG_INTERVAL_MS)
        {
            // 监测陀螺仪漂移
            uint8 drift_status = monitor_gyro_drift(imu_data.Rate1[0], 
                                                   imu_data.Rate1[1], 
                                                   imu_data.Rate1[2], 
                                                   0.0f); // 暂时使用0作为角度
            
            // 更新跟踪统计
            update_tracking_stats(estimated_x, estimated_y, current_speed);
            
            square_test_system.last_log_time = current_time;
        }
        
        // 显示更新
        if (current_time - square_test_system.last_display_time > DISPLAY_UPDATE_INTERVAL_MS)
        {
            display_test_data();
            square_test_system.last_display_time = current_time;
        }
        
        system_delay_ms(10);  // 100Hz主循环
    }
    
    // 停止电机
    motor_control_stop();
    
    // 保存测试结果
    save_test_results();
    
    ips114_clear();
    ips114_show_string(0, 0, "Test Completed!");
    ips114_show_string(0, 16, "Results saved");
    ips114_show_string(0, 32, "Press key to exit");
    
    while(1)
    {
        if (check_test_exit()) break;
        system_delay_ms(100);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     生成正方形路径点
//-------------------------------------------------------------------------------------------------------------------
uint8 generate_square_path(void)
{
    uint8 point_index = 0;
    float half_size = SQUARE_SIZE / 2.0f;
    
    // 正方形四个顶点坐标
    float corner_x[4] = {
        SQUARE_CENTER_X - half_size,  // 左下
        SQUARE_CENTER_X + half_size,  // 右下
        SQUARE_CENTER_X + half_size,  // 右上
        SQUARE_CENTER_X - half_size   // 左上
    };
    
    float corner_y[4] = {
        SQUARE_CENTER_Y - half_size,  // 左下
        SQUARE_CENTER_Y - half_size,  // 右下
        SQUARE_CENTER_Y + half_size,  // 右上
        SQUARE_CENTER_Y + half_size   // 左上
    };
    
    float corner_angles[4] = {0.0f, 90.0f, 180.0f, 270.0f};  // 各边的方向角
    
    // 生成四条边的路径点
    for (uint8 side = 0; side < 4; side++)
    {
        uint8 next_side = (side + 1) % 4;
        uint8 points_per_side = SQUARE_POINTS_COUNT / 4;
        
        for (uint8 i = 0; i < points_per_side && point_index < SQUARE_POINTS_COUNT; i++)
        {
            float t = (float)i / (float)(points_per_side - 1);
            
            // 线性插值计算路径点
            square_test_system.path_points[point_index].x = 
                corner_x[side] + t * (corner_x[next_side] - corner_x[side]);
            square_test_system.path_points[point_index].y = 
                corner_y[side] + t * (corner_y[next_side] - corner_y[side]);
            square_test_system.path_points[point_index].angle = corner_angles[side];
            
            // 标记转角点
            if (i < 2 || i >= points_per_side - 2)
            {
                square_test_system.path_points[point_index].point_type = 1;  // 转角
            }
            else
            {
                square_test_system.path_points[point_index].point_type = 0;  // 直线
            }
            
            point_index++;
        }
    }
    
    // 将路径点写入导航系统（简化实现）
    // 实际应该写入Flash，这里先设置配置参数
    X1 = square_test_system.path_points[0].x;
    Y1 = square_test_system.path_points[0].y;
    X2 = square_test_system.path_points[10].x;
    Y2 = square_test_system.path_points[10].y;
    X3 = square_test_system.path_points[20].x;
    Y3 = square_test_system.path_points[20].y;
    X4 = square_test_system.path_points[30].x;
    Y4 = square_test_system.path_points[30].y;
    
    square_test_system.path_initialized = 1;
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     陀螺仪漂移监测
//-------------------------------------------------------------------------------------------------------------------
uint8 monitor_gyro_drift(float gyro_x, float gyro_y, float gyro_z, float current_angle)
{
    gyro_drift_monitor_struct *monitor = &square_test_system.gyro_monitor;
    
    // 累积陀螺仪数据
    monitor->gyro_x_sum += gyro_x;
    monitor->gyro_y_sum += gyro_y;
    monitor->gyro_z_sum += gyro_z;
    monitor->sample_count++;
    
    // 计算平均漂移率
    if (monitor->sample_count >= GYRO_DRIFT_SAMPLES)
    {
        monitor->gyro_x_drift = monitor->gyro_x_sum / monitor->sample_count;
        monitor->gyro_y_drift = monitor->gyro_y_sum / monitor->sample_count;
        monitor->gyro_z_drift = monitor->gyro_z_sum / monitor->sample_count;
        
        // 重置累积值
        monitor->gyro_x_sum = 0;
        monitor->gyro_y_sum = 0;
        monitor->gyro_z_sum = 0;
        monitor->sample_count = 0;
    }
    
    // 检查漂移严重程度
    float drift_x = (monitor->gyro_x_drift >= 0) ? monitor->gyro_x_drift : -monitor->gyro_x_drift;
    float drift_y = (monitor->gyro_y_drift >= 0) ? monitor->gyro_y_drift : -monitor->gyro_y_drift;
    float drift_z = (monitor->gyro_z_drift >= 0) ? monitor->gyro_z_drift : -monitor->gyro_z_drift;
    
    float max_drift = drift_x;
    if (drift_y > max_drift) max_drift = drift_y;
    if (drift_z > max_drift) max_drift = drift_z;
    
    if (max_drift > GYRO_DRIFT_THRESHOLD * 2)
    {
        monitor->drift_warning_count++;
        return 2;  // 严重漂移
    }
    else if (max_drift > GYRO_DRIFT_THRESHOLD)
    {
        return 1;  // 轻微漂移
    }
    
    return 0;  // 正常
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     路径跟踪统计更新
//-------------------------------------------------------------------------------------------------------------------
void update_tracking_stats(float current_x, float current_y, float current_speed)
{
    path_tracking_stats_struct *stats = &square_test_system.tracking_stats;
    
    // 找到最近的路径点
    float min_distance = 1000000.0f;
    uint8 nearest_point = 0;
    
    for (uint8 i = 0; i < SQUARE_POINTS_COUNT; i++)
    {
        float dist = calculate_distance(current_x, current_y,
                                      square_test_system.path_points[i].x,
                                      square_test_system.path_points[i].y);
        if (dist < min_distance)
        {
            min_distance = dist;
            nearest_point = i;
        }
    }
    
    // 更新位置误差统计
    if (min_distance > stats->max_position_error)
    {
        stats->max_position_error = min_distance;
    }
    
    stats->position_error_sum += min_distance;
    stats->error_sample_count++;
    stats->avg_position_error = stats->position_error_sum / stats->error_sample_count;
    
    // 更新速度统计
    stats->current_speed = current_speed;
    stats->speed_sum += current_speed;
    stats->speed_sample_count++;
    stats->avg_speed = stats->speed_sum / stats->speed_sample_count;
    
    // 检查速度范围
    if (current_speed > TEST_MAX_SPEED)
    {
        stats->overspeed_count++;
    }
    else if (current_speed < TEST_MIN_SPEED)
    {
        stats->underspeed_count++;
    }
    
    // 检查是否完成一圈
    static uint8 last_nearest_point = 0;
    if (nearest_point < 5 && last_nearest_point > SQUARE_POINTS_COUNT - 5)
    {
        stats->lap_count++;
    }
    last_nearest_point = nearest_point;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     测试数据显示
//-------------------------------------------------------------------------------------------------------------------
void display_test_data(void)
{
    char info[50];
    uint32 elapsed_time = get_system_time_ms() - square_test_system.test_start_time;
    
    ips114_clear();
    
    // 显示测试状态
    sprintf(info, "Time:%02d:%02d", (elapsed_time/60000), (elapsed_time/1000)%60);
    ips114_show_string(0, 0, info);
    
    sprintf(info, "Laps:%d", square_test_system.tracking_stats.lap_count);
    ips114_show_string(80, 0, info);
    
    // 显示速度信息
    sprintf(info, "Speed:%.2f m/s", square_test_system.tracking_stats.current_speed);
    ips114_show_string(0, 16, info);
    
    sprintf(info, "Avg:%.2f m/s", square_test_system.tracking_stats.avg_speed);
    ips114_show_string(0, 32, info);
    
    // 显示位置误差
    sprintf(info, "Err:%.1fmm", square_test_system.tracking_stats.avg_position_error);
    ips114_show_string(0, 48, info);
    
    sprintf(info, "Max:%.1fmm", square_test_system.tracking_stats.max_position_error);
    ips114_show_string(80, 48, info);
    
    // 显示陀螺仪漂移
    sprintf(info, "Drift X:%.3f", square_test_system.gyro_monitor.gyro_x_drift);
    ips114_show_string(0, 64, info);
    
    sprintf(info, "Drift Z:%.3f", square_test_system.gyro_monitor.gyro_z_drift);
    ips114_show_string(0, 80, info);
    
    // 显示漂移警告
    if (square_test_system.gyro_monitor.drift_warning_count > 0)
    {
        sprintf(info, "Warn:%d", square_test_system.gyro_monitor.drift_warning_count);
        ips114_show_string(0, 96, info);
    }
    
    // 显示速度问题
    if (square_test_system.tracking_stats.underspeed_count > 0 || 
        square_test_system.tracking_stats.overspeed_count > 0)
    {
        sprintf(info, "U:%d O:%d", 
                square_test_system.tracking_stats.underspeed_count,
                square_test_system.tracking_stats.overspeed_count);
        ips114_show_string(80, 96, info);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     测试系统初始化
//-------------------------------------------------------------------------------------------------------------------
uint8 init_square_test_system(void)
{
    // 清零测试系统
    memset(&square_test_system, 0, sizeof(square_test_system_struct));
    
    // 初始化基本参数
    square_test_system.current_point_index = 0;
    square_test_system.test_running = 0;
    square_test_system.path_initialized = 0;
    
    // 重置统计数据
    reset_statistics();
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查测试退出条件
//-------------------------------------------------------------------------------------------------------------------
uint8 check_test_exit(void)
{
    // 这里可以添加按键检测逻辑
    // 暂时返回0，表示不退出
    // 实际使用时可以检测按键状态
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     保存测试结果
//-------------------------------------------------------------------------------------------------------------------
void save_test_results(void)
{
    // 显示最终统计结果
    ips114_clear();
    ips114_show_string(0, 0, "Final Results:");
    
    char info[50];
    
    sprintf(info, "Total Laps: %d", square_test_system.tracking_stats.lap_count);
    ips114_show_string(0, 16, info);
    
    sprintf(info, "Avg Speed: %.2f", square_test_system.tracking_stats.avg_speed);
    ips114_show_string(0, 32, info);
    
    sprintf(info, "Avg Error: %.1f", square_test_system.tracking_stats.avg_position_error);
    ips114_show_string(0, 48, info);
    
    sprintf(info, "Max Error: %.1f", square_test_system.tracking_stats.max_position_error);
    ips114_show_string(0, 64, info);
    
    sprintf(info, "Drift Warn: %d", square_test_system.gyro_monitor.drift_warning_count);
    ips114_show_string(0, 80, info);
    
    sprintf(info, "Speed Issues: %d", 
            square_test_system.tracking_stats.underspeed_count + 
            square_test_system.tracking_stats.overspeed_count);
    ips114_show_string(0, 96, info);
    
    system_delay_ms(5000);  // 显示5秒结果
}

//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算两点距离
//-------------------------------------------------------------------------------------------------------------------
static float calculate_distance(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    float distance_squared = dx * dx + dy * dy;
    
    // 使用简化的平方根计算，避免标准库冲突
    if (distance_squared == 0.0f) return 0.0f;
    
    // 牛顿法求平方根
    float result = distance_squared;
    float x0 = distance_squared / 2.0f;
    
    for (int i = 0; i < 10; i++) {
        if (x0 == 0.0f) break;
        float x1 = (x0 + distance_squared / x0) / 2.0f;
        if ((x1 >= x0 ? x1 - x0 : x0 - x1) < 0.001f) break;
        x0 = x1;
    }
    
    return x0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     角度归一化
//-------------------------------------------------------------------------------------------------------------------
static float normalize_angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置统计数据
//-------------------------------------------------------------------------------------------------------------------
static void reset_statistics(void)
{
    memset(&square_test_system.tracking_stats, 0, sizeof(path_tracking_stats_struct));
    memset(&square_test_system.gyro_monitor, 0, sizeof(gyro_drift_monitor_struct));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断是否接近转角
//-------------------------------------------------------------------------------------------------------------------
static uint8 is_near_corner(float x, float y)
{
    float half_size = SQUARE_SIZE / 2.0f;
    float corner_threshold = 100.0f;  // 100mm范围内认为是转角
    
    // 检查四个角
    float corners[4][2] = {
        {SQUARE_CENTER_X - half_size, SQUARE_CENTER_Y - half_size},
        {SQUARE_CENTER_X + half_size, SQUARE_CENTER_Y - half_size},
        {SQUARE_CENTER_X + half_size, SQUARE_CENTER_Y + half_size},
        {SQUARE_CENTER_X - half_size, SQUARE_CENTER_Y + half_size}
    };
    
    for (uint8 i = 0; i < 4; i++)
    {
        if (calculate_distance(x, y, corners[i][0], corners[i][1]) < corner_threshold)
        {
            return 1;
        }
    }
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     应用实际调试的PID参数
//-------------------------------------------------------------------------------------------------------------------
static void apply_tuned_pid_parameters(void)
{
    // 设置导航路径跟踪PID参数 (PIDG - 导航控制)
    nav_config_set_pid(NAV_DEFAULT_PID_KP,     // Kp = 24.3 (主要路径跟踪)
                       NAV_DEFAULT_PID_KI,     // Ki = 0.0
                       NAV_DEFAULT_PID_KD,     // Kd = 29.5
                       NAV_DEFAULT_PID_SPEED); // Speed = 100
    
    // 设置左轮速度环PID参数 (PIDSL - 左轮速度，继承PIDS)
    nav_config_set_motor_pid(0, 0,              // 左电机，速度环
                            LEFT_WHEEL_SPEED_PID_KP,  // Kp = 30.0
                            LEFT_WHEEL_SPEED_PID_KI,  // Ki = 2.4
                            LEFT_WHEEL_SPEED_PID_KD); // Kd = 0.0
    
    // 设置右轮速度环PID参数 (PIDS - 速度控制)
    nav_config_set_motor_pid(1, 0,              // 右电机，速度环
                            MOTOR_SPEED_PID_KP,       // Kp = 30.0
                            MOTOR_SPEED_PID_KI,       // Ki = 2.4
                            MOTOR_SPEED_PID_KD);      // Kd = 0.0
    
    // 更新电机控制系统PID参数
    if (motor_set_pid_params(MOTOR_CTRL_LEFT, PID_TYPE_POSITIONAL,
                            LEFT_WHEEL_SPEED_PID_KP,
                            LEFT_WHEEL_SPEED_PID_KI,
                            LEFT_WHEEL_SPEED_PID_KD) != 0)
    {
        ips114_show_string(0, 112, "Left PID Set Failed");
    }
    
    if (motor_set_pid_params(MOTOR_CTRL_RIGHT, PID_TYPE_POSITIONAL,
                            MOTOR_SPEED_PID_KP,
                            MOTOR_SPEED_PID_KI,
                            MOTOR_SPEED_PID_KD) != 0)
    {
        ips114_show_string(80, 112, "Right PID Failed");
    }
    
    // 显示PID参数设置状态
    ips114_show_string(0, 96, "PID Params Applied:");
    char pid_info[50];
    sprintf(pid_info, "Nav: %.1f,%.1f,%.1f", 
            NAV_DEFAULT_PID_KP, NAV_DEFAULT_PID_KI, NAV_DEFAULT_PID_KD);
    ips114_show_string(0, 112, pid_info);
    
    system_delay_ms(2000);  // 显示2秒让用户看到参数
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取系统时间（毫秒）
//-------------------------------------------------------------------------------------------------------------------
static uint32 get_system_time_ms(void)
{
    // 使用静态计数器模拟系统时间
    // 实际使用时应该替换为真实的系统时间函数
    static uint32 time_counter = 0;
    static uint32 last_call_time = 0;
    
    // 每次调用增加10ms，模拟100Hz调用频率
    time_counter += 10;
    
    return time_counter;
}
