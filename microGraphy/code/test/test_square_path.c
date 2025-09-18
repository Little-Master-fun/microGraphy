/*********************************************************************************************************************
* 文件名称          test_square_path.c
* 功能说明          【OBSOLETE/已弃用】正方形路径测试 实现文件
* 作者              LittleMaster
* 版本信息          v3.0
* 修改记录
* 日期              作者                版本              备注
* 2025-09-18        LittleMaster        v3.0              标记为已弃用
*
* 文件作用说明：
*  - Core M7_0: `code/system/system_control.c` (负责主循环和决策)
*  - Core M7_1: `code/estimator/state_estimator.c` & `user/main_cm7_1.c` (负责高频状态计算)
*
* 当前保留此文件并提供一个空函数体，仅为解决旧工程引用导致的编译链接错误。
* 新的程序入口在 `user/main_cm7_0.c` -> `system_control_update()`。
********************************************************************************************************************/

#include "test_square_path.h"
#include "zf_common_headfile.h" // 仅为printf
#include <stdio.h>

void test_square_path_optimized(void)
{
    // 本函数已在双核重构后被弃用。
    // 主控制逻辑现在由 'system_control' 模块驱动。
    printf("Function test_square_path_optimized() is obsolete.\n");
    printf("Please use main_cm7_0.c as the entry point.\n");
    
    // 进入死循环，防止意外执行
    while(1)
    {
    }
}
