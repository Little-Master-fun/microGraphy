/*********************************************************************************************************************
* 文件名称          state_estimator.h
* 功能说明          车辆状态估计器模块 头文件
* 作者              AI Assistant
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本              备注
* 2024-XX-XX        AI Assistant        v1.0              创建文件，用于M7_1核心
*
* 文件作用说明：
* 本模块专门负责车辆位姿的高频估算（航位推算）。它融合编码器和IMU数据，
* 计算车辆的全局坐标(x, y)、航向角(heading)和速度。
* 这个模块被设计为在M7_1协处理器上独立运行。
********************************************************************************************************************/
#ifndef _STATE_ESTIMATOR_H_
#define _STATE_ESTIMATOR_H_

#include "zf_common_headfile.h"
#include "navigation_flash_improved.h" // 引用核心数据结构 VehicleState

// 定义车辆物理参数
#define VEHICLE_WHEELBASE_MM    (150.0f)    // 车辆轮距 (mm)

// 对外接口函数
void state_estimator_init(void);
void state_estimator_update(float dt);
const VehicleState* state_estimator_get_state(void);


#endif // _STATE_ESTIMATOR_H_
