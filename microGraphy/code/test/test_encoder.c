/*********************************************************************************************************************
* 文件名称          test_encoder.c
* 功能说明          双编码器驱动测试程序实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-09-17        LittleMaster       1.0v
* 
* 文件作用说明：
* 本文件为双编码器驱动测试程序的实现，用于验证编码器驱动的各项功能
* 
* 测试内容：
* 1. 编码器基本功能测试（初始化、脉冲计数、方向判断）
* 2. 编码器速度计算测试
* 3. 编码器里程统计测试
* 4. 综合功能测试（实时数据显示）
* 
* 使用方式：
* 在主函数中调用相应的测试函数即可开始测试
* 
* 注意事项：
* 1. 确保编码器硬件连接正确
* 2. 手动转动车轮进行测试
* 3. 观察屏幕显示的数据是否合理
********************************************************************************************************************/

#include "test_encoder.h"
#include "driver_encoder.h"
#include "zf_device_ips114.h"
#include "zf_common_headfile.h"
#include <stdio.h>

//=================================================测试参数定义================================================
#define TEST_DELAY_SHORT        (500)       // 短延时 0.5秒
#define TEST_DELAY_MEDIUM       (1000)      // 中延时 1秒
#define TEST_DELAY_LONG         (2000)      // 长延时 2秒
#define TEST_UPDATE_INTERVAL    (100)       // 数据更新间隔 100ms

//=================================================内部函数声明================================================
static void test_display_encoder_data(void);
static void test_init_display(void);
static void test_display_info(const char* info);

//=================================================外部接口实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器基本功能测试
// 参数说明     void
// 返回参数     void
// 使用示例     test_encoder_basic();
// 备注信息     测试编码器的基本功能：初始化、脉冲计数、方向判断
//-------------------------------------------------------------------------------------------------------------------
void test_encoder_basic(void)
{
    test_init_display();
    test_display_info("Encoder Basic Test");
    
    // 初始化编码器
    if (encoder_init() == ENCODER_STATUS_OK)
    {
        test_display_info("Encoder Init: OK");
    }
    else
    {
        test_display_info("Encoder Init: FAILED");
        return;
    }
    
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    test_display_info("Please rotate wheels");
    test_display_info("Testing for 10s...");
    
    // 测试10秒钟
    for (int i = 0; i < 100; i++)
    {
        encoder_update();
        
        // 每秒显示一次数据
        if (i % 10 == 0)
        {
            char buffer[50];
            sprintf(buffer, "L:%ld R:%ld", 
                   encoder_get_pulse_count(ENCODER_ID_LEFT),
                   encoder_get_pulse_count(ENCODER_ID_RIGHT));
            test_display_info(buffer);
        }
        
        system_delay_ms(TEST_UPDATE_INTERVAL);
    }
    
    test_display_info("Basic Test Complete");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器速度测试
// 参数说明     void
// 返回参数     void
// 使用示例     test_encoder_speed();
// 备注信息     测试编码器的速度计算功能
//-------------------------------------------------------------------------------------------------------------------
void test_encoder_speed(void)
{
    test_init_display();
    test_display_info("Encoder Speed Test");
    
    // 初始化编码器
    encoder_init();
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    test_display_info("Rotate wheels steadily");
    test_display_info("Speed test for 15s...");
    
    // 测试15秒钟
    for (int i = 0; i < 150; i++)
    {
        encoder_update();
        
        // 每0.5秒显示一次速度数据
        if (i % 5 == 0)
        {
            char buffer[50];
            float left_speed = encoder_get_speed(ENCODER_ID_LEFT);
            float right_speed = encoder_get_speed(ENCODER_ID_RIGHT);
            
            sprintf(buffer, "L:%.3f R:%.3f m/s", left_speed, right_speed);
            test_display_info(buffer);
        }
        
        system_delay_ms(TEST_UPDATE_INTERVAL);
    }
    
    test_display_info("Speed Test Complete");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器里程测试
// 参数说明     void
// 返回参数     void
// 使用示例     test_encoder_distance();
// 备注信息     测试编码器的里程统计功能
//-------------------------------------------------------------------------------------------------------------------
void test_encoder_distance(void)
{
    test_init_display();
    test_display_info("Encoder Distance Test");
    
    // 初始化编码器
    encoder_init();
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    test_display_info("Rotate wheels forward");
    test_display_info("Distance test...");
    
    // 持续测试
    uint32 test_count = 0;
    while (test_count < 200)  // 20秒测试
    {
        encoder_update();
        
        // 每秒显示一次里程数据
        if (test_count % 10 == 0)
        {
            char buffer[50];
            float left_dist = encoder_get_distance(ENCODER_ID_LEFT);
            float right_dist = encoder_get_distance(ENCODER_ID_RIGHT);
            
            sprintf(buffer, "L:%.1fmm R:%.1fmm", left_dist, right_dist);
            test_display_info(buffer);
        }
        
        test_count++;
        system_delay_ms(TEST_UPDATE_INTERVAL);
    }
    
    test_display_info("Distance Test Complete");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器综合测试
// 参数说明     void
// 返回参数     void
// 使用示例     test_encoder();
// 备注信息     综合测试所有编码器功能，实时显示编码器数据
//-------------------------------------------------------------------------------------------------------------------
void test_encoder(void)
{
    test_init_display();
    test_display_info("Encoder Full Test");
    
    // 初始化编码器
    if (encoder_init() != ENCODER_STATUS_OK)
    {
        test_display_info("Init Failed!");
        return;
    }
    
    test_display_info("Init Success");
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    test_display_info("Real-time Display");
    test_display_info("Rotate wheels to test");
    
    // 进入实时显示模式
    while (1)
    {
        encoder_update();
        test_display_encoder_data();
        system_delay_ms(TEST_UPDATE_INTERVAL);
    }
}

//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示编码器数据（内部函数）
// 参数说明     void
// 返回参数     void
// 备注信息     在屏幕上实时显示编码器的详细数据
//-------------------------------------------------------------------------------------------------------------------
static void test_display_encoder_data(void)
{
    char buffer[50];
    
    ips114_clear();
    
    // 显示标题
    ips114_show_string(0, 0, "Encoder Data Monitor");
    
    // 显示左编码器数据
    ips114_show_string(0, 16, "=== LEFT ENCODER ===");
    
    sprintf(buffer, "Pulse: %ld", encoder_get_pulse_count(ENCODER_ID_LEFT));
    ips114_show_string(0, 32, buffer);
    
    sprintf(buffer, "Speed: %.3f m/s", encoder_get_speed(ENCODER_ID_LEFT));
    ips114_show_string(0, 48, buffer);
    
    sprintf(buffer, "Dist: %.1f mm", encoder_get_distance(ENCODER_ID_LEFT));
    ips114_show_string(0, 64, buffer);
    
    // 显示右编码器数据
    ips114_show_string(0, 80, "=== RIGHT ENCODER ===");
    
    sprintf(buffer, "Pulse: %ld", encoder_get_pulse_count(ENCODER_ID_RIGHT));
    ips114_show_string(0, 96, buffer);
    
    sprintf(buffer, "Speed: %.3f m/s", encoder_get_speed(ENCODER_ID_RIGHT));
    ips114_show_string(0, 112, buffer);
    
    sprintf(buffer, "Dist: %.1f mm", encoder_get_distance(ENCODER_ID_RIGHT));
    ips114_show_string(0, 128, buffer);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     测试显示初始化（内部函数）
// 参数说明     void
// 返回参数     void
// 备注信息     初始化测试显示屏幕
//-------------------------------------------------------------------------------------------------------------------
static void test_init_display(void)
{
    // 初始化屏幕
    ips114_set_dir(IPS114_PORTAIT);
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_init();
    ips114_clear();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     测试信息显示（内部函数）
// 参数说明     info                测试信息
// 返回参数     void
// 备注信息     在屏幕上显示测试信息
//-------------------------------------------------------------------------------------------------------------------
static void test_display_info(const char* info)
{
    static uint8 line_count = 0;
    
    // 显示信息
    ips114_show_string(0, line_count * 16, info);
    
    // 更新行计数，超过屏幕高度时清屏重新开始
    line_count++;
    if (line_count > 7)  // IPS114屏幕大约可显示8行
    {
        line_count = 0;
        system_delay_ms(2000);  // 暂停2秒让用户看清信息
        ips114_clear();
    }
}
