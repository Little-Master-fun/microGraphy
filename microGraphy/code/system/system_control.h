/*********************************************************************************************************************
* 文件名称          system_control.h
* 功能说明          主控流程与状态机管理模块 头文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本              备注
* 2025-09-18        LittleMaster        v1.0              创建文件，定义状态机接口
*
* 文件作用说明：
* 本模块是整个应用程序的“大脑”。它通过一个显式的状态机来管理系统的核心行为，
* 负责接收UI模块的指令，并调度其他功能模块（如导航、电机）执行相应任务。
********************************************************************************************************************/
#ifndef _SYSTEM_CONTROL_H_
#define _SYSTEM_CONTROL_H_

#include "zf_common_headfile.h"
#include "ui_manager.h" // 引入UI模块定义的枚举

// 定义系统状态
typedef enum 
{
    SYS_STATE_INIT,             // 系统初始化状态
    SYS_STATE_IDLE,             // 待机状态，等待指令
    SYS_STATE_PATH_RECORDING,   // 路径记录模式
    SYS_STATE_PATH_FOLLOWING,   // 路径循迹模式
    SYS_STATE_STOPPED,          // 紧急停止或任务完成
    SYS_STATE_ERROR             // 错误状态
} system_state_enum;

// 对外接口函数
void system_control_init(void);
void system_control_update(void);
system_state_enum system_control_get_state(void);

#endif // _SYSTEM_CONTROL_H_
