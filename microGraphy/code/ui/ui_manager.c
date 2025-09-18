/*********************************************************************************************************************
* 文件名称          ui_manager.c
* 功能说明          UI界面与按键管理模块 实现文件
* 作者              AI Assistant
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本              备注
* 2024-XX-XX        AI Assistant        v1.0              创建文件，实现UI逻辑
*
* 文件作用说明：
* 本文件实现了UI模块的具体功能，包括屏幕内容的绘制和按键事件的处理。
********************************************************************************************************************/
#include "ui_manager.h"
#include "zf_device_ips114.h"
#include "config_navigation.h" // 引入配置模块以修改参数
#include "zf_device_systick.h" // 引入systick模块以使用时间戳
#include <stdio.h>

//-------------------------------------------------------------------------------------------------------------------
// 内部变量与类型定义
//-------------------------------------------------------------------------------------------------------------------

// 按键引脚定义
#define KEY1_PIN    (P00_3)
#define KEY2_PIN    (P00_2)
#define KEY3_PIN    (P01_0)
#define KEY4_PIN    (P01_1)

// 内部状态变量
static system_mode_enum s_current_mode = SYS_MODE_IDLE;
static uint8_t s_key1_state = 0, s_key2_state = 0, s_key3_state = 0, s_key4_state = 0;
static uint8_t s_param_select_index = 0; // 参数选择索引

//-------------------------------------------------------------------------------------------------------------------
// 内部函数声明
//-------------------------------------------------------------------------------------------------------------------
static void key_scan(void);
static void ui_display(void);
static void handle_key_events(void);
static void display_idle_mode(void);
static void display_adjust_mode(void);
static void display_running_mode(void);

//-------------------------------------------------------------------------------------------------------------------
// 对外接口函数实现
//-------------------------------------------------------------------------------------------------------------------

void ui_init(void)
{
    ips114_init();
    gpio_init(KEY1_PIN, GPI, 0, GPI_PULL_UP);
    gpio_init(KEY2_PIN, GPI, 0, GPI_PULL_UP);
    gpio_init(KEY3_PIN, GPI, 0, GPI_PULL_UP);
    gpio_init(KEY4_PIN, GPI, 0, GPI_PULL_UP);
    s_current_mode = SYS_MODE_IDLE;
}

// 主更新函数，应在主循环中定期调用
void ui_update(void)
{
    key_scan();
    handle_key_events();
    ui_display();
}

system_mode_enum ui_get_current_mode(void)
{
    return s_current_mode;
}

void ui_set_mode(system_mode_enum mode)
{
    s_current_mode = mode;
}

//-------------------------------------------------------------------------------------------------------------------
// 内部功能函数实现
//-------------------------------------------------------------------------------------------------------------------

// 按键扫描函数
static void key_scan(void)
{
    // 简单的消抖和状态更新
    static uint8_t key_debounce_count[4] = {0};
    uint8_t key_pressed[4] = {0};

    if(gpio_get_level(KEY1_PIN) == 0) key_pressed[0] = 1;
    if(gpio_get_level(KEY2_PIN) == 0) key_pressed[1] = 1;
    if(gpio_get_level(KEY3_PIN) == 0) key_pressed[2] = 1;
    if(gpio_get_level(KEY4_PIN) == 0) key_pressed[3] = 1;

    for(int i=0; i<4; i++)
    {
        if(key_pressed[i])
        {
            key_debounce_count[i]++;
            if(key_debounce_count[i] > 2) // 消抖 20ms
            {
                if(i==0) s_key1_state = 1;
                if(i==1) s_key2_state = 1;
                if(i==2) s_key3_state = 1;
                if(i==3) s_key4_state = 1;
            }
        }
        else
        {
            key_debounce_count[i] = 0;
        }
    }
}

// 按键事件处理
static void handle_key_events(void)
{
    // KEY1: 切换主模式
    if(s_key1_state)
    {
        s_key1_state = 0;
        if(s_current_mode == SYS_MODE_IDLE) s_current_mode = SYS_MODE_ADJUST_PARAMS;
        else if (s_current_mode == SYS_MODE_ADJUST_PARAMS) s_current_mode = SYS_MODE_IDLE;
    }

    // 根据当前模式处理其他按键
    switch(s_current_mode)
    {
        case SYS_MODE_IDLE:
            if(s_key3_state) // KEY3: 确认发车
            {
                s_key3_state = 0;
                s_current_mode = SYS_MODE_CONFIRM_START;
            }
            break;

        case SYS_MODE_ADJUST_PARAMS:
            if(s_key2_state) // KEY2: 切换要调整的参数
            {
                s_key2_state = 0;
                s_param_select_index = (s_param_select_index + 1) % 3; // 假设有3个参数可调
            }
            if(s_key3_state) // KEY3: 增加参数值
            {
                s_key3_state = 0;
                nav_config_struct* cfg = nav_config_get();
                if(s_param_select_index == 0) cfg->nav_pid_speed += 5;
                if(s_param_select_index == 1) cfg->nav_pid_kp += 0.1f;
                if(s_param_select_index == 2) cfg->nav_pid_kd += 0.1f;
            }
            if(s_key4_state) // KEY4: 减小参数值
            {
                s_key4_state = 0;
                nav_config_struct* cfg = nav_config_get();
                if(s_param_select_index == 0) cfg->nav_pid_speed -= 5;
                if(s_param_select_index == 1) cfg->nav_pid_kp -= 0.1f;
                if(s_param_select_index == 2) cfg->nav_pid_kd -= 0.1f;
            }
            break;
        
        default:
            // 运行中或等待发车时不处理按键
            s_key2_state = s_key3_state = s_key4_state = 0;
            break;
    }
}

// UI显示主函数
static void ui_display(void)
{
    static uint32_t last_display_time = 0;
    if(systick_get_time_ms() - last_display_time < 200) return; // 200ms刷新一次
    last_display_time = systick_get_time_ms();
    
    ips114_clear();

    switch(s_current_mode)
    {
        case SYS_MODE_IDLE:
        case SYS_MODE_CONFIRM_START:
            display_idle_mode();
            break;
        case SYS_MODE_ADJUST_PARAMS:
            display_adjust_mode();
            break;
        case SYS_MODE_PATH_RECORDING:
        case SYS_MODE_PATH_FOLLOWING:
            display_running_mode();
            break;
    }
}

static void display_idle_mode(void)
{
    char buf[50];
    nav_config_struct* cfg = nav_config_get();
    nav_data_struct* data = nav_data_get();

    ips114_show_string(0, 0, "---[IDLE MODE]---");
    sprintf(buf, "Speed: %d", cfg->nav_pid_speed);
    ips114_show_string(0, 16, buf);
    sprintf(buf, "P:%.1f K:%.1f", cfg->nav_pid_kp, cfg->nav_pid_kd);
    ips114_show_string(0, 32, buf);

    if(s_current_mode == SYS_MODE_CONFIRM_START)
    {
        ips114_show_string(0, 48, "--> START? (KEY3)");
    }
    else
    {
        ips114_show_string(0, 48, "Press KEY1 to ADJ");
    }
}

static void display_adjust_mode(void)
{
    char buf[50];
    nav_config_struct* cfg = nav_config_get();
    
    ips114_show_string(0, 0, "---[ADJUST MODE]---");

    sprintf(buf, "%sSpeed: %d", (s_param_select_index==0 ? ">" : " "), cfg->nav_pid_speed);
    ips114_show_string(0, 16, buf);

    sprintf(buf, "%sP: %.1f", (s_param_select_index==1 ? ">" : " "), cfg->nav_pid_kp);
    ips114_show_string(0, 32, buf);

    sprintf(buf, "%sD: %.1f", (s_param_select_index==2 ? ">" : " "), cfg->nav_pid_kd);
    ips114_show_string(0, 48, buf);
}

static void display_running_mode(void)
{
    // 此函数可以从外部调用，或者UI模块自己获取导航数据来显示
    // 这里暂时留空，可以在主循环中调用并传入导航数据
    if(s_current_mode == SYS_MODE_PATH_RECORDING)
    {
        ips114_show_string(0, 0, "--[RECORDING]--");
    }
    else
    {
        ips114_show_string(0, 0, "--[FOLLOWING]--");
    }
    //... 显示 x, y, speed, yaw 等
}
