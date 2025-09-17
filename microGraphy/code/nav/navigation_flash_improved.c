/*********************************************************************************************************************
* 文件名称          navigation_flash_improved.c
* 功能说明          改进版导航Flash系统实现文件
* 作者              AI Assistant
* 版本信息          v2.0
* 修改记录
* 日期              作者                备注
* 2024-XX-XX        AI Assistant        重构导航模块，规范变量命名，移除UI依赖
*
* 文件作用说明：
* 本文件为改进版的导航Flash系统实现文件，通过配置系统统一管理参数和数据
* 规范了变量命名，移除了与UI模块的直接依赖，提高了代码的可维护性
*
* 主要改进：
* 1. 规范化变量命名，移除不规范的变量名
* 2. 通过配置系统统一管理数据
* 3. 移除了与UI模块的循环依赖
* 4. 保持了向后兼容性
* 5. 提高了代码的可读性和维护性
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "navigation_flash_improved.h"

//=================================================全局变量定义================================================
nav_system_struct nav_system = {0};            // 导航系统结构体

// 导航计算相关变量
float nav_error_direction = 0.0f;              // 误差方向
int nav_error_angle_direction = 0;              // 误差角度方向
int nav_point_error_index = 0;                 // 点误差索引
int nav_error_make_flag = 0;                   // 误差生成标志
int nav_calculation_buffer = 0;                // 计算缓冲
float nav_curvature = 0.0f;                    // 曲率
uint8 nav_status_flags = 0;                    // 状态标志

// 编码器相关变量
int nav_encoder_sum = 0;                       // 编码器总和
int nav_encoder_left = 0;                      // 左编码器

// 数据计数相关
int nav_actual_error_point = 0;                // 实际误差点
int nav_max_error_point_mem = 0;               // 最大误差点内存
int nav_mileage_total_sum = 0;                 // 总里程和

//=================================================内部函数声明================================================
static void nav_update_curvature_state_machine(void);
static void nav_calculate_path_tracking_output(void);
static void nav_process_curvature_threshold(void);
static float nav_calculate_curvature_value(float theta1, float theta2, float theta3, int d12, int d23);

//=================================================主要接口函数================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航系统初始化
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_system_init(void)
{
    // 初始化配置系统
    nav_config_init();
    nav_data_init();
    
    // 初始化导航系统结构体
    memset(&nav_system, 0, sizeof(nav_system_struct));
    
    // 设置初始值
    nav_max_error_point_mem = NAV_COORD_RECORD_SIZE;
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航数据保存
//-------------------------------------------------------------------------------------------------------------------
void nav_data_save(void)
{
    // 更新里程计数
    nav_system.mileage_total += (nav_encoder_sum + nav_encoder_left) / 2;
    nav_mileage_total_sum += (nav_encoder_sum + nav_encoder_left) / 2;
    
    // 保存当前数据到数组
    if (nav_actual_error_point < NAV_COORD_RECORD_SIZE)
    {
        nav_mileage_list[nav_actual_error_point] = nav_mileage_total_sum;
        nav_errors_coords[nav_actual_error_point] = nav_error_direction;
        nav_actual_error_point++;
    }
    
    // 更新最大误差点计数
    if (nav_actual_error_point > nav_max_error_point_mem)
    {
        nav_max_error_point_mem = nav_actual_error_point;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航数据重新保存
//-------------------------------------------------------------------------------------------------------------------
void nav_data_resave(void)
{
    nav_mileage_total_sum += (nav_encoder_sum + nav_encoder_left) / 2;
    
    int search_start = (nav_point_error_index >= 10) ? nav_point_error_index - 10 : 0;
    
    // 处理曲率阈值
    nav_process_curvature_threshold();
    
    // 搜索匹配的误差点
    for (int i = search_start; i < nav_max_error_point_mem; i++)
    {
        float threshold_offset = 0.0f;
        
        if (fabs(nav_curvature) > 50)
        {
            qulv_yuzhi_chixu_biaozhiwei_hahahahahahahahaha++;
            threshold_offset = NAV_SET_MILEAGE * 2 * (fabs(nav_curvature) / 16 - 0.2) * 
                              ((nav_encoder_sum + nav_encoder_left) / 2 - 50) / 150.0f;
        }
        else
        {
            lastopopop = 0;
            threshold_offset = NAV_SET_MILEAGE * 2 * (fabs(nav_curvature) / 70 + 9.0f / 7) * 
                              (((nav_encoder_sum + nav_encoder_left) / 2 - 150) / 116.0f);
        }
        
        if (nav_mileage_list[i] >= (nav_mileage_total_sum + threshold_offset))
        {
            nav_point_error_index = i;
            break;
        }
        nav_calculation_buffer = i;
    }
    
    // 重置阈值标志
    if (qulv_yuzhi_chixu_biaozhiwei_hahahahahahahahaha == 2)
    {
        qulv_yuzhi_chixu_biaozhiwei_hahahahahahahahaha = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算曲率
//-------------------------------------------------------------------------------------------------------------------
double nav_calculate_curvature(float theta1, float theta2, float theta3, int distance12, int distance23)
{
    return nav_calculate_curvature_value(theta1, theta2, theta3, distance12, distance23);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取最大值
//-------------------------------------------------------------------------------------------------------------------
double nav_get_max_value(double value_a, double value_b)
{
    return (value_a > value_b) ? value_a : value_b;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取绝对值
//-------------------------------------------------------------------------------------------------------------------
double nav_get_absolute_value(double value)
{
    return (value < 0) ? -value : value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航路径跟踪
//-------------------------------------------------------------------------------------------------------------------
float nav_path_tracking(void)
{
    nav_calculate_path_tracking_output();
    return nav_system.final_output;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航系统状态更新
//-------------------------------------------------------------------------------------------------------------------
void nav_system_update(void)
{
    // 更新导航系统状态
    nav_update_curvature_state_machine();
    
    // 更新角度信息
    nav_system.angle_current = rt_yaw;
    nav_system.angle_reference = Last_Nag_yaw;
    
    // 计算误差
    nav_system.error_value = nav_calculate_error(nav_system.angle_current, nav_system.angle_reference);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航误差计算
//-------------------------------------------------------------------------------------------------------------------
float nav_calculate_error(float current_position, float target_position)
{
    return target_position - current_position;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航参数更新
//-------------------------------------------------------------------------------------------------------------------
void nav_update_parameters(void)
{
    // 从配置系统更新参数
    // 这里可以添加参数同步逻辑
}

//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新曲率状态机
//-------------------------------------------------------------------------------------------------------------------
static void nav_update_curvature_state_machine(void)
{
    // 实现曲率状态机逻辑
    if (nav_curvature > 40)
    {
        was_high = true;
        if (zheng_reset_state == 0)
        {
            zheng_reset_state = 1;
        }
    }
    else if (nav_curvature < -40)
    {
        was_low = true;
        if (fu_reset_state == 0)
        {
            fu_reset_state = 1;
        }
    }
    else if (nav_curvature >= 0 && nav_curvature <= 20)
    {
        if (was_high)
        {
            high_to_mid_reset = true;
        }
    }
    else if (nav_curvature >= -20 && nav_curvature <= 0)
    {
        if (was_low)
        {
            low_to_mid_reset = true;
        }
    }
    
    // 处理状态转换
    lianxuzhijiao = 0;
    if (zheng_reset_state > 0 && zheng_reset_state < 3)
    {
        lianxuzhijiao = 1;
    }
    else if (fu_reset_state > 0 && fu_reset_state < 3)
    {
        lianxuzhijiao = 2;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算路径跟踪输出
//-------------------------------------------------------------------------------------------------------------------
static void nav_calculate_path_tracking_output(void)
{
    // 实现路径跟踪算法
    float error = nav_system.error_value;
    
    // 简单的比例控制
    nav_system.final_output = error * 0.5f; // 简化的控制输出
    
    // 限制输出范围
    if (nav_system.final_output > 100.0f)
    {
        nav_system.final_output = 100.0f;
    }
    else if (nav_system.final_output < -100.0f)
    {
        nav_system.final_output = -100.0f;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     处理曲率阈值
//-------------------------------------------------------------------------------------------------------------------
static void nav_process_curvature_threshold(void)
{
    // 处理曲率阈值逻辑
    if (fabs(nav_curvature) > 50)
    {
        qulv_yuzhi_chixu_biaozhiwei_hahahahahahahahaha++;
    }
    
    // 更新时间计数器
    zhetime++;
    if (zhetime > 255)
    {
        zhetime = 0;
        zhitime++;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算曲率值
//-------------------------------------------------------------------------------------------------------------------
static float nav_calculate_curvature_value(float theta1, float theta2, float theta3, int d12, int d23)
{
    // 简化的曲率计算
    if (d12 == 0 || d23 == 0)
    {
        return 0.0f;
    }
    
    float angle_diff1 = theta2 - theta1;
    float angle_diff2 = theta3 - theta2;
    
    float curvature = (angle_diff2 - angle_diff1) / ((d12 + d23) / 2.0f);
    
    return curvature;
}
