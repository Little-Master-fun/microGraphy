/*********************************************************************************************************************
* 文件名称          navigation_flash_improved.h
* 功能说明          改进版导航Flash系统头文件
* 作者              AI Assistant
* 版本信息          v2.0
* 修改记录
* 日期              作者                备注
* 2024-XX-XX        AI Assistant        重构导航模块，移除UI依赖
*
* 文件作用说明：
* 本文件为改进版的导航Flash系统头文件，移除了与UI模块的直接依赖
* 通过配置系统统一管理参数，实现模块间的解耦
*
* 主要改进：
* 1. 移除了与UI模块的循环依赖
* 2. 参数管理通过配置系统统一处理
* 3. 提高了代码的可维护性和可扩展性
* 4. 保持了向后兼容性
********************************************************************************************************************/

#ifndef _NAVIGATION_FLASH_IMPROVED_H_
#define _NAVIGATION_FLASH_IMPROVED_H_

#include "zf_common_typedef.h"
#include "config_navigation.h"    


//=================================================导航系统配置================================================
#define NAV_MAX_SIZE                    NAV_FLASH_MAX_SIZE          // Flash存储的最大页面
#define NAV_COORD_RECORD_SIZE           NAV_COORD_RECORD            // 坐标记录数量
#define NAV_END_PAGE                    NAV_FLASH_END_PAGE          // Flash中止页面
#define NAV_START_PAGE                  NAV_FLASH_START_PAGE        // Flash起始页面
#define NAV_MILEAGE_SETTING             NAV_SET_MILEAGE             // 里程计设置

// 数学常数和转换
#define NAV_PI                          M_PI
#define NAV_DEG_TO_RAD_CONV(angle)      NAV_DEG_TO_RAD(angle)

//=================================================导航数据结构================================================
// 导航系统主结构体
typedef struct
{
    float final_output;                         // 最终输出
    int mileage_total;                          // 总里程计数
    float angle_current;                        // 当前偏航角
    float angle_reference;                      // 参考偏航角
    float error_value;                          // 误差值
    float path_segment;                         // 路径段长度
    
    // 系统标志
    bool nav_stop_flag;                         // 导航停止标志
    uint8 flash_read_flag;                      // Flash读取标志
    
    // 计数器和索引
    uint16 array_size;                          // 数组大小
    uint16 save_counter;                        // 保存计数器
    uint16 save_index;                          // 保存索引
    uint8 end_flag;                             // 结束标志
    
    // Flash相关
    uint8 flash_page_index;                     // Flash页面索引
    uint8 flash_save_page_index;                // Flash保存页码索引
    uint8 nav_system_run_index;                 // 导航系统运行索引
    
} nav_system_struct;

//=================================================全局变量声明================================================
extern nav_system_struct nav_system;           // 导航系统结构体

// 注意：所有数据变量现在都通过 config_navigation.h 系统管理
// 请使用 #include "config_navigation.h" 来访问这些变量
// 例如：max_error_point_mem, errors_coords, Mileage_All_sum_list, sum, suml 等
// 所有不规范变量名都已通过宏定义映射到规范变量

//=================================================函数声明================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航系统初始化
// 参数说明     void
// 返回参数     uint8                          初始化结果 (0=成功, 1=失败)
// 使用示例     nav_system_init();
// 备注信息     初始化导航系统，加载配置参数
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_system_init(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航数据保存
// 参数说明     void
// 返回参数     void
// 使用示例     nav_data_save();
// 备注信息     保存当前导航数据到存储系统
//-------------------------------------------------------------------------------------------------------------------
void nav_data_save(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航数据重新保存
// 参数说明     void
// 返回参数     void
// 使用示例     nav_data_resave();
// 备注信息     重新保存导航数据，用于数据更新
//-------------------------------------------------------------------------------------------------------------------
void nav_data_resave(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算曲率
// 参数说明     theta1                         角度1 (度)
// 参数说明     theta2                         角度2 (度)
// 参数说明     theta3                         角度3 (度)
// 参数说明     distance12                     距离12
// 参数说明     distance23                     距离23
// 返回参数     double                         计算的曲率值
// 使用示例     curvature = nav_calculate_curvature(45.0f, 90.0f, 135.0f, 100, 100);
// 备注信息     根据三个点的角度和距离计算路径曲率
//-------------------------------------------------------------------------------------------------------------------
double nav_calculate_curvature(float theta1, float theta2, float theta3, int distance12, int distance23);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取最大值
// 参数说明     value_a                        值A
// 参数说明     value_b                        值B
// 返回参数     double                         最大值
// 使用示例     max_val = nav_get_max_value(10.5, 20.3);
// 备注信息     返回两个值中的最大值
//-------------------------------------------------------------------------------------------------------------------
double nav_get_max_value(double value_a, double value_b);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取绝对值
// 参数说明     value                          输入值
// 返回参数     double                         绝对值
// 使用示例     abs_val = nav_get_absolute_value(-15.6);
// 备注信息     返回输入值的绝对值
//-------------------------------------------------------------------------------------------------------------------
double nav_get_absolute_value(double value);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航路径跟踪
// 参数说明     void
// 返回参数     float                          路径跟踪输出
// 使用示例     output = nav_path_tracking();
// 备注信息     执行路径跟踪算法，返回控制输出
//-------------------------------------------------------------------------------------------------------------------
float nav_path_tracking(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航系统状态更新
// 参数说明     void
// 返回参数     void
// 使用示例     nav_system_update();
// 备注信息     更新导航系统状态，包括位置、角度等信息
//-------------------------------------------------------------------------------------------------------------------
void nav_system_update(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航误差计算
// 参数说明     current_position               当前位置
// 参数说明     target_position                目标位置
// 返回参数     float                          计算的误差值
// 使用示例     error = nav_calculate_error(current_pos, target_pos);
// 备注信息     计算当前位置与目标位置的误差
//-------------------------------------------------------------------------------------------------------------------
float nav_calculate_error(float current_position, float target_position);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航参数更新
// 参数说明     void
// 返回参数     void
// 使用示例     nav_update_parameters();
// 备注信息     从配置系统更新导航参数
//-------------------------------------------------------------------------------------------------------------------
void nav_update_parameters(void);

// 兼容性宏定义（保持向后兼容）
#define Run_Nag_Save()                  nav_data_save()
#define Run_Nag_reSave()                nav_data_resave()
#define calculate_curvature(t1,t2,t3,d1,d2)  nav_calculate_curvature(t1,t2,t3,d1,d2)
#define fmax(a,b)                       nav_get_max_value(a,b)
#define fabs(f)                         nav_get_absolute_value(f)

// 兼容性变量映射
// #define sum                             nav_encoder_sum
// #define suml                            nav_encoder_left
// #define N                               nav_system
// #define actual_error_point              nav_actual_error_point
// #define max_error_point_mem             nav_max_error_point_mem
// #define errors_coords                   nav_errors_coords
// #define Mileage_All_sum                 nav_mileage_total_sum
// #define Mileage_All_sum_list            nav_mileage_list
// #define error_dir                       nav_error_direction
// #define error_angle_dir                 nav_error_angle_direction
// #define point_error_index               nav_point_error_index
// #define error_make_flag                 nav_error_make_flag
// #define cnmb                            nav_calculation_buffer
// #define qulv                            nav_curvature
// #define opopop                          nav_status_flags

#endif
