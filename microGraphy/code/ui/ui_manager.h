/*********************************************************************************************************************
* 文件名称          ui_manager.h
* 功能说明          UI界面与按键管理模块 头文件
* 作者              AI Assistant
* 版本信息          v1.0
* 修改记录
* 日Dias              作者                版本              备注
* 2024-XX-XX        AI Assistant        v1.0              创建文件，定义UI接口
*
* 文件作用说明：
* 本文件负责管理用户交互，包括IPS屏幕的显示和物理按键的输入处理。
* 它将UI逻辑与主控制流程解耦，使两者可以独立开发和修改。
********************************************************************************************************************/
#ifndef _UI_MANAGER_H_
#define _UI_MANAGER_H_

#include "zf_common_headfile.h"

// 定义系统的主模式（状态）
typedef enum 
{
    SYS_MODE_IDLE,              // 待机模式，显示参数
    SYS_MODE_ADJUST_PARAMS,     // 参数调整模式
    SYS_MODE_PATH_RECORDING,    // 路径记录模式
    SYS_MODE_PATH_FOLLOWING,    // 路径循迹模式
    SYS_MODE_CONFIRM_START      // 等待确认发车
} system_mode_enum;

// 定义UI模块对外提供的函数接口
void ui_init(void);
void ui_update(void);
system_mode_enum ui_get_current_mode(void);
void ui_set_mode(system_mode_enum mode);

#endif // _UI_MANAGER_H_
