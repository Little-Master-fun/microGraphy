/*********************************************************************************************************************
* 文件名称          math_utils.c
* 功能说明          通用数学工具函数模块 实现文件
* 作者              AI Assistant
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本              备注
* 2024-XX-XX        AI Assistant        v1.0              创建文件
********************************************************************************************************************/
#include "math_utils.h"

float normalize_angle(float angle)
{
    while (angle > (float)M_PI) angle -= 2.0f * (float)M_PI;
    while (angle < -(float)M_PI) angle += 2.0f * (float)M_PI;
    return angle;
}
