# 电机闭环控制系统使用指南

## ? 系统概述

电机闭环控制系统提供了完整的电机控制解决方案，支持多种控制模式和精确的 PID 控制算法。

### ? 主要特性

- **多控制模式**：开环、速度闭环、位置闭环、级联控制
- **双 PID 算法**：位置式 PID 和增量式 PID
- **双电机支持**：独立控制左右电机
- **差速运动**：支持机器人线速度和角速度控制
- **参数管理**：集成配置系统，支持参数保存和加载
- **性能测试**：完整的测试框架验证控制性能

## ? 快速开始

### 1. 初始化系统

```c
#include "motor_control.h"
#include "config_navigation.h"

int main(void)
{
    // 初始化配置系统
    nav_config_init();

    // 初始化电机控制系统
    motor_control_init();

    // 启动控制系统
    motor_control_start();

    // 主循环
    while(1)
    {
        // 1ms调用一次控制更新
        motor_control_update();
        system_delay_ms(1);
    }
}
```

### 2. 定时器集成（推荐）

```c
// 在1ms定时器中断中调用
void timer_1ms_isr(void)
{
    motor_control_update();
}
```

## ? 控制模式使用

### 开环控制

```c
// 设置开环模式
motor_set_control_mode(MOTOR_CTRL_LEFT, MOTOR_CTRL_MODE_OPEN_LOOP);

// 直接设置PWM值 (-9999 到 9999)
motor_set_target_speed(MOTOR_CTRL_LEFT, 5000);  // 50%占空比正转
```

### 速度闭环控制

```c
// 设置速度闭环模式
motor_set_control_mode(MOTOR_CTRL_BOTH, MOTOR_CTRL_MODE_SPEED_LOOP);

// 设置目标速度 (m/s)
motor_set_target_speed(MOTOR_CTRL_LEFT, 0.5f);   // 左轮0.5m/s
motor_set_target_speed(MOTOR_CTRL_RIGHT, 0.5f);  // 右轮0.5m/s

// 或者设置机器人运动参数
motor_set_robot_motion(0.3f, 0.0f);  // 直线运动 0.3m/s
motor_set_robot_motion(0.2f, 0.5f);  // 转弯运动
```

### 位置闭环控制

```c
// 设置位置闭环模式
motor_set_control_mode(MOTOR_CTRL_LEFT, MOTOR_CTRL_MODE_POSITION_LOOP);

// 设置目标位置 (编码器脉冲数)
motor_set_target_position(MOTOR_CTRL_LEFT, 1000);  // 移动1000个脉冲

// 检查是否到达目标
float current_pos, error;
motor_get_status(MOTOR_CTRL_LEFT, NULL, &current_pos, &error);
if (fabs(error) < 5.0f) {
    // 到达目标位置
}
```

### 级联控制（位置外环+速度内环）

```c
// 设置级联控制模式
motor_set_control_mode(MOTOR_CTRL_LEFT, MOTOR_CTRL_MODE_CASCADE);

// 设置目标位置，系统会自动进行位置和速度的双环控制
motor_set_target_position(MOTOR_CTRL_LEFT, 2000);
```

## ?? PID 参数调整

### 使用配置系统访问 PID 参数

```c
// 获取左电机速度环PID参数
float kp, ki, kd;
nav_config_get_motor_pid(0, 0, &kp, &ki, &kd);  // 左电机(0), 速度环(0)

// 设置右电机位置环PID参数
nav_config_set_motor_pid(1, 1, 2.0f, 0.1f, 0.05f);  // 右电机(1), 位置环(1)

// 直接使用宏访问（推荐）
motor_left_speed_kp = 8.0f;
motor_left_speed_ki = 0.5f;
motor_left_speed_kd = 0.1f;
```

### 直接设置 PID 参数

```c
// 设置左电机速度环PID参数
motor_set_pid_params(MOTOR_CTRL_LEFT, 0, 8.0f, 0.5f, 0.1f);

// 设置双电机位置环PID参数
motor_set_pid_params(MOTOR_CTRL_BOTH, 1, 2.0f, 0.1f, 0.05f);
```

### PID 参数调优指南

| 参数 | 作用     | 调优建议                 |
| ---- | -------- | ------------------------ |
| Kp   | 比例系数 | 增大响应速度，过大会震荡 |
| Ki   | 积分系数 | 消除稳态误差，过大会超调 |
| Kd   | 微分系数 | 减少超调，过大会放大噪声 |

**调优步骤：**

1. 先设置 Ki=0, Kd=0，调整 Kp 到临界震荡
2. 将 Kp 减小到临界值的 60%
3. 逐渐增加 Ki 消除稳态误差
4. 适当增加 Kd 减少超调

## ? 系统监控

### 获取运行状态

```c
float speed, position, error;

// 获取左电机状态
motor_get_status(MOTOR_CTRL_LEFT, &speed, &position, &error);

printf("速度: %.2f m/s, 位置: %.0f 脉冲, 误差: %.3f\n",
       speed, position, error);
```

### 系统诊断

```c
// 运行系统诊断
if (motor_control_diagnose() != 0) {
    printf("系统检测到异常！\n");

    // 获取错误统计
    printf("总错误次数: %d\n", motor_ctrl_system.total_error_count);
    printf("平均速度误差: %.3f\n", motor_ctrl_system.average_speed_error);
}
```

## ? 性能测试

### 运行完整测试

```c
#include "test_motor_control.h"

// 运行综合测试
if (test_motor_control() == 0) {
    printf("所有测试通过！\n");
} else {
    printf("测试失败，请检查系统配置。\n");
}
```

### 单项测试

```c
// 测试速度闭环性能
test_speed_loop_control();

// 测试位置闭环性能
test_position_loop_control();

// 测试PID控制器
test_pid_controller();
```

### 阶跃响应测试

```c
test_result_struct result;

// 执行阶跃响应测试
test_step_response(MOTOR_CTRL_LEFT, 0.5f, 5000, &result);

// 显示测试结果
display_test_result(&result);

printf("最大误差: %.3f\n", result.max_error);
printf("稳定时间: %dms\n", result.settle_time);
printf("超调量: %.1f%%\n", result.overshoot);
```

## ? 高级功能

### 电机参数配置

```c
// 设置系统参数
nav_config_set_motor_params(
    2.0f,    // 最大速度 2m/s
    5.0f,    // 最大加速度 5m/s?
    150.0f,  // 轮距 150mm
    1000     // 控制频率 1KHz
);

// 使用宏直接访问
motor_max_speed = 1.5f;
motor_wheelbase = 160.0f;
```

### PID 控制器复位

```c
// 复位左电机速度环PID
motor_pid_reset(MOTOR_CTRL_LEFT, 0);

// 复位所有PID控制器
motor_pid_reset(MOTOR_CTRL_BOTH, 0);  // 速度环
motor_pid_reset(MOTOR_CTRL_BOTH, 1);  // 位置环
```

### 差速运动控制

```c
// 直线运动
motor_set_robot_motion(0.5f, 0.0f);  // 0.5m/s直线前进

// 原地转弯
motor_set_robot_motion(0.0f, 1.0f);  // 1rad/s原地转弯

// 弧线运动
motor_set_robot_motion(0.3f, 0.5f);  // 边前进边转弯

// 停止运动
motor_set_robot_motion(0.0f, 0.0f);
```

## ?? 安全和保护

### 系统保护功能

```c
// 系统自动保护功能
// 1. PWM输出限幅：自动限制在 ±9999 范围内
// 2. 速度限幅：限制在最大速度范围内
// 3. 积分饱和保护：防止积分项过大
// 4. 异常检测：自动检测系统异常状态

// 手动停止系统
motor_control_stop();  // 紧急停止

// 重新启动系统
motor_control_start();
```

### 错误处理

```c
// 检查初始化结果
if (motor_control_init() != 0) {
    printf("电机控制系统初始化失败！\n");
    return -1;
}

// 检查控制模式设置
if (motor_set_control_mode(MOTOR_CTRL_LEFT, MOTOR_CTRL_MODE_SPEED_LOOP) != 0) {
    printf("控制模式设置失败！\n");
}
```

## ? 使用技巧

### 1. 控制频率选择

- **1KHz（推荐）**：适合大多数应用，响应快，稳定性好
- **500Hz**：适合低速精确控制
- **2KHz**：适合高速响应要求

### 2. PID 参数选择

- **速度环**：Kp=8.0, Ki=0.5, Kd=0.1（参考值）
- **位置环**：Kp=2.0, Ki=0.1, Kd=0.05（参考值）
- **级联控制**：位置环参数可以更小，依赖速度环响应

### 3. 编码器配置

- 确保编码器 PPR 设置正确（`ENCODER_PPR = 1000`）
- 检查编码器方向是否与电机方向一致
- 定期校准编码器零点

### 4. 性能优化

- 使用定时器中断调用`motor_control_update()`
- 避免在控制循环中使用阻塞函数
- 定期运行系统诊断检查性能

## ? 性能指标

### 典型性能参数

| 指标     | 速度闭环 | 位置闭环 | 级联控制 |
| -------- | -------- | -------- | -------- |
| 稳定时间 | <200ms   | <500ms   | <800ms   |
| 稳态误差 | <2%      | <5 脉冲  | <10 脉冲 |
| 超调量   | <10%     | <5%      | <8%      |
| 响应频率 | 1KHz     | 1KHz     | 1KHz     |

### 系统资源占用

- **CPU 占用率**：<5%（1KHz 控制频率）
- **内存占用**：约 2KB RAM
- **Flash 占用**：约 8KB 程序空间

## ? 故障排除

### 常见问题

1. **电机不转动**

   - 检查电机驱动初始化
   - 验证 PWM 输出是否正常
   - 确认电机电源供应

2. **控制不稳定**

   - 检查 PID 参数设置
   - 验证编码器反馈信号
   - 调整控制频率

3. **位置控制不准确**

   - 校准编码器 PPR 设置
   - 检查编码器安装
   - 验证机械间隙

4. **系统响应慢**
   - 增大 Kp 参数
   - 检查控制频率设置
   - 优化代码执行效率

### 调试方法

```c
// 启用调试输出
#define MOTOR_DEBUG_ENABLE

// 在控制循环中添加调试信息
if (debug_counter++ > 100) {  // 每100ms输出一次
    printf("L_speed: %.2f, R_speed: %.2f, Error: %.3f\n",
           left_speed, right_speed, speed_error);
    debug_counter = 0;
}
```

---

**? 技术支持**

如果在使用过程中遇到问题，请：

1. 检查本使用指南的故障排除章节
2. 运行系统诊断程序
3. 查看系统日志和错误信息
4. 联系技术支持团队

**? 最佳实践**

1. 在实际应用前先运行完整测试程序
2. 根据具体应用场景调整 PID 参数
3. 定期保存和备份配置参数
4. 建立系统性能基线进行对比
5. 实施渐进式参数调优策略
