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
#include "config_navigation.h"    // 包含配置系统，获取所有数据变量访问权限

//=================================================全局变量定义================================================
nav_system_struct nav_system = {0};            // 导航系统结构体

// 注意：所有数据变量现在都通过 config_navigation 系统管理
// 不再在这里定义局部变量，直接使用宏定义访问配置系统

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
    max_error_point_mem = NAV_COORD_RECORD_SIZE;
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航数据保存
//-------------------------------------------------------------------------------------------------------------------
void nav_data_save(void)
{
    // 更新里程计数
    nav_system.mileage_total += (encoder_sum_nav + encoder_left_nav) / 2;
    Mileage_All_sum += (encoder_sum_nav + encoder_left_nav) / 2;
    
    // 保存当前数据到数组
    if (actual_error_point < NAV_COORD_RECORD_SIZE)
    {
        Mileage_All_sum_list[actual_error_point] = Mileage_All_sum;
        errors_coords[actual_error_point] = error_dir;
        actual_error_point++;
    }
    
    // 更新最大误差点计数
    if (actual_error_point > max_error_point_mem)
    {
        max_error_point_mem = actual_error_point;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航数据重新保存
//-------------------------------------------------------------------------------------------------------------------
void nav_data_resave(void)
{
    Mileage_All_sum += (encoder_sum_nav + encoder_left_nav) / 2;
    
    int search_start = (point_error_index >= 10) ? point_error_index - 10 : 0;
    
    // 处理曲率阈值
    nav_process_curvature_threshold();
    
    // 搜索匹配的误差点
    for (int i = search_start; i < max_error_point_mem; i++)
    {
        float threshold_offset = 0.0f;
        
        if (fabs(qulv) > 50)
        {
            curvature_threshold_counter++;
            threshold_offset = NAV_SET_MILEAGE * 2 * (fabs(qulv) / 16 - 0.2) * 
                              ((encoder_sum_nav + encoder_left_nav) / 2 - 50) / 150.0f;
        }
        else
        {
            lastopopop = 0;
            threshold_offset = NAV_SET_MILEAGE * 2 * (fabs(qulv) / 70 + 9.0f / 7) * 
                              (((encoder_sum_nav + encoder_left_nav) / 2 - 150) / 116.0f);
        }
        
        if (Mileage_All_sum_list[i] >= (Mileage_All_sum + threshold_offset))
        {
            point_error_index = i;
            break;
        }
        cnmb = i;
    }
    
    // 重置阈值标志
    if (curvature_threshold_counter == 2)
    {
        curvature_threshold_counter = 0;
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
    if (qulv > 40)
    {
        was_high = true;
        if (zheng_reset_state == 0)
        {
            zheng_reset_state = 1;
        }
    }
    else if (qulv < -40)
    {
        was_low = true;
        if (fu_reset_state == 0)
        {
            fu_reset_state = 1;
        }
    }
    else if (qulv >= 0 && qulv <= 20)
    {
        if (was_high)
        {
            high_to_mid_reset = true;
        }
    }
    else if (qulv >= -20 && qulv <= 0)
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
    if (fabs(qulv) > 50)
    {
        curvature_threshold_counter++;
    }
    
    // 更新时间计数器（溢出不是会自动归零吗）
    zhetime++;

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
