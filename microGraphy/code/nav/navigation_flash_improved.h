/*********************************************************************************************************************
* 文件名称          navigation_flash_improved.h
* 功能说明          【重构版】导航与路径跟踪系统 头文件
* 作者              LittleMaster
* 版本信息          v3.0
* 修改记录
* 日期              作者                版本              备注
* 2025-09-18        LittleMaster        v3.0              整合状态估计与路径跟踪控制逻辑
*
* 文件作用说明：
* 本文件定义了导航系统的核心数据结构和接口。
* 它整合了状态估计、路径管理和路径跟踪控制功能，形成一个高内聚的导航模块。
*
********************************************************************************************************************/

#ifndef _NAVIGATION_FLASH_IMPROVED_H_
#define _NAVIGATION_FLASH_IMPROVED_H_

#include "zf_common_typedef.h"
#include "config_navigation.h"
#include <math.h>

//================================================= 宏定义 =================================================
#define NAV_MAX_PATH_POINTS         (200)       // 路径点最大数量
#define NAV_WHEELBASE               (150.0f)    // 车辆轮距 (mm)
#define NAV_STANLEY_LATERAL_GAIN    (2.0f)      // Stanley控制器横向误差增益
#define NAV_LOOKAHEAD_DISTANCE      (200.0f)    // 固定前瞻距离 (mm)

//================================================= 核心数据结构 =================================================

// 1. 车辆状态结构体 (State Estimator Output)
typedef struct {
    float x;                // 全局X坐标 (mm)
    float y;                // 全局Y坐标 (mm)
    float heading;          // 航向角 (rad)
    float linear_speed;     // 当前线速度 (m/s)
} VehicleState;

// 2. 优化后的路径点结构 (Path Point Definition)
typedef struct {
    float x;                // X坐标 (mm)
    float y;                // Y坐标 (mm)
    float heading;          // 期望航向角 (rad)
    float curvature;        // 路径曲率 (1/mm)
    float reference_speed;  // 参考速度 (m/s)
} OptimalPathPoint;

// 3. 运动指令结构 (Controller Output / Motor Input)
typedef struct {
    float desired_linear_speed;     // 期望线速度 (m/s)
    float desired_angular_speed;    // 期望角速度 (rad/s)
} MotionCommand;

// 4. 导航控制器类型
typedef enum {
    NAV_CONTROLLER_STANLEY = 0,     // Stanley控制器（默认）
    NAV_CONTROLLER_MPC = 1,         // 模型预测控制器
} nav_controller_type_t;

// 5. 导航系统主结构体
typedef struct {
    VehicleState state;                         // 车辆实时状态
    OptimalPathPoint path[NAV_MAX_PATH_POINTS]; // 存储的路径
    uint16_t path_point_count;                  // 当前路径的点数
    nav_controller_type_t controller_type;      // 当前控制器类型
    bool initialized;                           // 初始化标志
} NavigationSystem;


//================================================= 全局变量声明 =================================================
extern NavigationSystem g_nav_system;

//================================================= 核心功能函数声明 =================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化导航系统
//-------------------------------------------------------------------------------------------------------------------
void Navigation_Init(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置导航控制器类型
// 参数说明     controller_type - 控制器类型 (Stanley/MPC)
// 返回参数     bool - 设置是否成功
//-------------------------------------------------------------------------------------------------------------------
bool Navigation_SetControllerType(nav_controller_type_t controller_type);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取当前控制器类型
// 返回参数     nav_controller_type_t - 当前控制器类型
//-------------------------------------------------------------------------------------------------------------------
nav_controller_type_t Navigation_GetControllerType(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行路径跟踪控制计算
// 参数说明     current_state - 当前车辆的位姿状态
// 返回参数     MotionCommand - 计算出的期望运动指令
//-------------------------------------------------------------------------------------------------------------------
MotionCommand Navigation_PathTrack(const VehicleState* current_state);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     加载或设置导航路径
// 参数说明     new_path - 指向路径点数组的指针
// 参数说明     point_count - 路径点数量
//-------------------------------------------------------------------------------------------------------------------
void Navigation_SetPath(const OptimalPathPoint* new_path, uint16_t point_count);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     【辅助】生成用于测试的标准正方形路径
//-------------------------------------------------------------------------------------------------------------------
void Navigation_GenerateTestPath(float size_mm, uint16_t point_count);


#endif // _NAVIGATION_FLASH_IMPROVED_H_
