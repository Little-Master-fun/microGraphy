/*********************************************************************************************************************
* 文件名称          show_ips_ljmnb_improved.c
* 功能说明          改进版IPS显示和用户界面实现文件
* 作者              AI Assistant
* 版本信息          v2.0
* 修改记录
* 日期              作者                备注
* 2024-XX-XX        AI Assistant        重构UI模块，移除参数依赖，规范变量命名
*
* 文件作用说明：
* 本文件为改进版的IPS显示和用户界面实现文件，通过配置系统统一管理参数
* 移除了与导航模块的直接依赖，提高了代码的可维护性和可扩展性
*
* 主要改进：
* 1. 移除了与导航模块的循环依赖
* 2. 参数管理通过配置系统统一处理
* 3. 规范了变量命名和代码结构
* 4. 保持了向后兼容性
* 5. 提高了代码的可读性和维护性
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "show_ips_ljmnb_improved.h"
#include "zf_device_ips114.h"

//=================================================全局变量定义================================================
ui_state_struct ui_state = {                    // UI状态结构
    .current_mode = UI_MODE0,                       // 初始化为模式0
    .current_param = UI_KEY2_PARAM_0,               // 初始化为参数0
    .key3_action = UI_KEY3_INCREASE,                // 初始化为增加动作
    .key4_action = UI_KEY4_DECREASE,                // 初始化为减少动作
    .key1_pressed = false,                          // 按键状态为false
    .key1_last_state = false,
    .key2_pressed = false,
    .key2_last_state = false,
    .key3_pressed = false,
    .key3_last_state = false,
    .key4_pressed = false,
    .key4_last_state = false,
    .display_refresh_flag = 0,                      // 显示标志为0
    .last_refresh_time = 0                          // 时间为0
};

//=================================================硬件引脚定义================================================
#define UI_LED_PIN                      (P19_0)
#define UI_KEY1_PIN                     (P00_3)
#define UI_KEY2_PIN                     (P00_2)
#define UI_KEY3_PIN                     (P01_0)
#define UI_KEY4_PIN                     (P01_1)

//=================================================内部函数声明================================================
static void ui_update_display(void);
static void ui_handle_key1(void);
static void ui_handle_key2(void);
static void ui_handle_key3(void);
static void ui_handle_key4(void);
static uint8 ui_detect_key_press(gpio_pin_enum pin, bool *last_state, bool *pressed_flag);

//=================================================主要接口函数================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UI系统初始化
//-------------------------------------------------------------------------------------------------------------------
uint8 ui_system_init(void)
{
    // 初始化GPIO
    gpio_init(UI_LED_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(UI_KEY1_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(UI_KEY2_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(UI_KEY3_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(UI_KEY4_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    
    // 初始化IPS114显示屏
    ips114_init();
    
    // 初始化配置系统
    nav_config_init();
    nav_data_init();
    
    // 初始化UI状态
    memset(&ui_state, 0, sizeof(ui_state_struct));
    ui_state.current_mode = UI_MODE0;
    ui_state.current_param = UI_KEY2_PARAM_0;
    ui_state.key3_action = UI_KEY3_INCREASE;
    ui_state.key4_action = UI_KEY4_DECREASE;
    
    // 初始化按键状态
    ui_state.key1_last_state = true;
    ui_state.key2_last_state = true;
    ui_state.key3_last_state = true;
    ui_state.key4_last_state = true;
    
    ui_state.display_refresh_flag = 1;
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按键检测和处理
//-------------------------------------------------------------------------------------------------------------------
void ui_key_process(void)
{
    // 检测按键状态
    ui_detect_key_press(UI_KEY1_PIN, &ui_state.key1_last_state, &ui_state.key1_pressed);
    ui_detect_key_press(UI_KEY2_PIN, &ui_state.key2_last_state, &ui_state.key2_pressed);
    ui_detect_key_press(UI_KEY3_PIN, &ui_state.key3_last_state, &ui_state.key3_pressed);
    ui_detect_key_press(UI_KEY4_PIN, &ui_state.key4_last_state, &ui_state.key4_pressed);
    
    // 处理按键事件
    if (ui_state.key1_pressed)
    {
        ui_handle_key1();
        ui_state.key1_pressed = false;
        ui_state.display_refresh_flag = 1;
    }
    
    if (ui_state.key2_pressed)
    {
        ui_handle_key2();
        ui_state.key2_pressed = false;
        ui_state.display_refresh_flag = 1;
    }
    
    if (ui_state.key3_pressed)
    {
        ui_handle_key3();
        ui_state.key3_pressed = false;
        ui_state.display_refresh_flag = 1;
    }
    
    if (ui_state.key4_pressed)
    {
        ui_handle_key4();
        ui_state.key4_pressed = false;
        ui_state.display_refresh_flag = 1;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示主界面
//-------------------------------------------------------------------------------------------------------------------
void ui_show_main(void)
{
    ips114_clear();
    ips114_show_string(1, 10, "MODE0: Main Menu");
    ips114_show_string(10, 30, "KEY1: Next Mode");
    ips114_show_string(10, 50, "KEY2: Enter Setting");
    
    // 显示当前配置状态
    ips114_show_string(10, 70, "Config Status:");
    if (nav_config.config_loaded)
    {
        ips114_show_string(120, 70, "LOADED");
    }
    else
    {
        ips114_show_string(120, 70, "DEFAULT");
    }
    
    // 显示当前速度设置
    ips114_show_string(10, 90, "Target Speed:");
    ips114_show_int(120, 90, finaltarget_speed, 3);
    
    // 显示辅助功能状态
    ips114_show_string(10, 110, "Fuya:");
    if (fuya == 1)
    {
        ips114_show_string(60, 110, "OPEN");
    }
    else
    {
        ips114_show_string(60, 110, "CLOSE");
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示参数设置界面
//-------------------------------------------------------------------------------------------------------------------
void ui_show_param_setting(void)
{
    ips114_clear();
    ips114_show_string(1, 10, "MODE1: Parameter List");
    ips114_show_string(10, 30, "KEY2: Select Param");
    ips114_show_string(10, 50, "KEY1: Next Mode");
    
    // 显示当前选择的参数
    ips114_show_string(10, 70, "Current Param:");
    
    switch (ui_state.current_param)
    {
        case UI_KEY2_PARAM_0:
            ips114_show_string(10, 90, "X1 Coordinate");
            break;
        case UI_KEY2_PARAM_1:
            ips114_show_string(10, 90, "Y1 Coordinate");
            break;
        case UI_KEY2_PARAM_2:
            ips114_show_string(10, 90, "X2 Coordinate");
            break;
        case UI_KEY2_PARAM_3:
            ips114_show_string(10, 90, "Y2 Coordinate");
            break;
        case UI_KEY2_PARAM_4:
            ips114_show_string(10, 90, "X3 Coordinate");
            break;
        case UI_KEY2_PARAM_5:
            ips114_show_string(10, 90, "Y3 Coordinate");
            break;
        case UI_KEY2_PARAM_6:
            ips114_show_string(10, 90, "X4 Coordinate");
            break;
        case UI_KEY2_PARAM_7:
            ips114_show_string(10, 90, "Y4 Coordinate");
            break;
        case UI_KEY2_PARAM_8:
            ips114_show_string(10, 90, "X5 Coordinate");
            break;
        case UI_KEY2_PARAM_9:
            ips114_show_string(10, 90, "Y5 Coordinate");
            break;
        case UI_KEY2_PARAM_10:
            ips114_show_string(10, 90, "X6 Coordinate");
            break;
        case UI_KEY2_PARAM_11:
            ips114_show_string(10, 90, "Y6 Coordinate");
            break;
        default:
            ips114_show_string(10, 90, "Unknown Param");
            break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示参数调整界面
//-------------------------------------------------------------------------------------------------------------------
void ui_show_param_adjust(void)
{
    ips114_clear();
    ips114_show_string(1, 10, "MODE2: Parameter Adjust");
    ips114_show_string(10, 30, "KEY3: +1  KEY4: -1");
    ips114_show_string(10, 50, "KEY1: Next Mode");
    
    // 显示当前参数值
    float current_value = 0.0f;
    
    switch (ui_state.current_param)
    {
        case UI_KEY2_PARAM_0:
            ips114_show_string(10, 70, "X1:");
            current_value = X1;
            break;
        case UI_KEY2_PARAM_1:
            ips114_show_string(10, 70, "Y1:");
            current_value = Y1;
            break;
        case UI_KEY2_PARAM_2:
            ips114_show_string(10, 70, "X2:");
            current_value = X2;
            break;
        case UI_KEY2_PARAM_3:
            ips114_show_string(10, 70, "Y2:");
            current_value = Y2;
            break;
        case UI_KEY2_PARAM_4:
            ips114_show_string(10, 70, "X3:");
            current_value = X3;
            break;
        case UI_KEY2_PARAM_5:
            ips114_show_string(10, 70, "Y3:");
            current_value = Y3;
            break;
        case UI_KEY2_PARAM_6:
            ips114_show_string(10, 70, "X4:");
            current_value = X4;
            break;
        case UI_KEY2_PARAM_7:
            ips114_show_string(10, 70, "Y4:");
            current_value = Y4;
            break;
        case UI_KEY2_PARAM_8:
            ips114_show_string(10, 70, "X5:");
            current_value = X5;
            break;
        case UI_KEY2_PARAM_9:
            ips114_show_string(10, 70, "Y5:");
            current_value = Y5;
            break;
        case UI_KEY2_PARAM_10:
            ips114_show_string(10, 70, "X6:");
            current_value = X6;
            break;
        case UI_KEY2_PARAM_11:
            ips114_show_string(10, 70, "Y6:");
            current_value = Y6;
            break;
        default:
            ips114_show_string(10, 70, "Unknown:");
            break;
    }
    
    ips114_show_float(60, 70, current_value, 2, 4);
    
    // 显示控制参数
    ips114_show_string(10, 90, "R1:");
    ips114_show_int(40, 90, R1, 3);
    ips114_show_string(80, 90, "R2:");
    ips114_show_int(110, 90, R2, 3);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示运行控制界面
//-------------------------------------------------------------------------------------------------------------------
void ui_show_run_control(void)
{
    ips114_clear();
    ips114_show_string(1, 10, "MODE3: Run Control");
    ips114_show_string(10, 30, "KEY3: Start/Stop");
    ips114_show_string(10, 50, "Go State:");
    ips114_show_int(80, 50, go_go_go, 1);
    
    if (fuya == 1)
    {
        ips114_show_string(10, 70, "Fuya: OPEN");
    }
    else
    {
        ips114_show_string(10, 70, "Fuya: CLOSE");
    }
    
    // 显示导航数据
    ips114_show_string(10, 90, "Mileage:");
    ips114_show_int(80, 90, Mileage_All_sum, 6);
    
    ips114_show_string(10, 110, "Error Pts:");
    ips114_show_int(80, 110, max_error_point_mem, 5);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UI主循环处理
//-------------------------------------------------------------------------------------------------------------------
void ui_main_loop(void)
{
    // 处理按键
    ui_key_process();
    
    // 更新显示
    if (ui_state.display_refresh_flag)
    {
        ui_update_display();
        ui_state.display_refresh_flag = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     参数值调整
//-------------------------------------------------------------------------------------------------------------------
uint8 ui_adjust_parameter(uint8 param_index, float increment)
{
    if (param_index >= UI_KEY2_PARAM_MAX)
    {
        return 1; // 参数索引无效
    }
    
    float current_value = 0.0f;
    uint8 point_index = param_index / 2;
    uint8 is_y_coord = param_index % 2;
    
    // 获取当前值
    if (nav_config_get_path_point(point_index, &current_value, &current_value) != 0)
    {
        return 1; // 获取失败
    }
    
    // 调整值
    float new_value = current_value + increment;
    
    // 设置新值
    float x, y;
    nav_config_get_path_point(point_index, &x, &y);
    
    if (is_y_coord)
    {
        y = new_value;
    }
    else
    {
        x = new_value;
    }
    
    return nav_config_set_path_point(point_index, x, y);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     保存当前参数设置
//-------------------------------------------------------------------------------------------------------------------
uint8 ui_save_parameters(void)
{
    return nav_config_save();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置参数到默认值
//-------------------------------------------------------------------------------------------------------------------
uint8 ui_reset_parameters(void)
{
    nav_config_load_default();
    return 0;
}

//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新显示
//-------------------------------------------------------------------------------------------------------------------
static void ui_update_display(void)
{
    switch (ui_state.current_mode)
    {
        case UI_MODE0:
            ui_show_main();
            break;
        case UI_MODE1:
            ui_show_param_setting();
            break;
        case UI_MODE2:
            ui_show_param_adjust();
            break;
        case UI_MODE3:
            ui_show_run_control();
            break;
        default:
            ui_state.current_mode = UI_MODE0;
            ui_show_main();
            break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     处理KEY1按键
//-------------------------------------------------------------------------------------------------------------------
static void ui_handle_key1(void)
{
    // KEY1: 切换模式
    ui_state.current_mode++;
    if (ui_state.current_mode >= UI_MODE_MAX)
    {
        ui_state.current_mode = UI_MODE0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     处理KEY2按键
//-------------------------------------------------------------------------------------------------------------------
static void ui_handle_key2(void)
{
    switch (ui_state.current_mode)
    {
        case UI_MODE0:
            // 在主界面，KEY2进入参数设置
            ui_state.current_mode = UI_MODE1;
            break;
            
        case UI_MODE1:
            // 在参数列表，KEY2选择下一个参数
            ui_state.current_param++;
            if (ui_state.current_param >= UI_KEY2_PARAM_MAX)
            {
                ui_state.current_param = UI_KEY2_PARAM_0;
            }
            break;
            
        case UI_MODE2:
            // 在参数调整，KEY2进入运行控制
            ui_state.current_mode = UI_MODE3;
            break;
            
        case UI_MODE3:
            // 在运行控制，KEY2返回主界面
            ui_state.current_mode = UI_MODE0;
            break;
            
        default:
            break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     处理KEY3按键
//-------------------------------------------------------------------------------------------------------------------
static void ui_handle_key3(void)
{
    switch (ui_state.current_mode)
    {
        case UI_MODE2:
            // 在参数调整模式，KEY3增加参数值
            ui_adjust_parameter(ui_state.current_param, 1.0f);
            break;
            
        case UI_MODE3:
            // 在运行控制模式，KEY3启动/停止
            go_go_go = !go_go_go;
            break;
            
        default:
            break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     处理KEY4按键
//-------------------------------------------------------------------------------------------------------------------
static void ui_handle_key4(void)
{
    switch (ui_state.current_mode)
    {
        case UI_MODE2:
            // 在参数调整模式，KEY4减少参数值
            ui_adjust_parameter(ui_state.current_param, -1.0f);
            break;
            
        case UI_MODE3:
            // 在运行控制模式，KEY4切换辅助功能
            fuya = !fuya;
            break;
            
        default:
            break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检测按键按下
//-------------------------------------------------------------------------------------------------------------------
static uint8 ui_detect_key_press(gpio_pin_enum pin, bool *last_state, bool *pressed_flag)
{
    bool current_state = (gpio_get_level(pin) == 0); // 低电平有效
    
    if (current_state && !(*last_state))
    {
        *pressed_flag = true;
    }
    
    *last_state = current_state;
    
    return 0;
}
