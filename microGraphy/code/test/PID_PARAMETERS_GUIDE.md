# PID 参数调优指南 - 正方形路径测试

## ? 实际调试 PID 参数表

基于实际调试结果，以下 PID 参数已被验证并集成到正方形路径测试系统中：

| 参数名称  | 功能描述 | Kp   | Ki  | Kd   | 用途说明     |
| --------- | -------- | ---- | --- | ---- | ------------ |
| **PIDG**  | 导航控制 | 24.3 | 0   | 29.5 | 主要路径跟踪 |
| **PIDS**  | 速度控制 | 30   | 2.4 | 0    | 电机速度环   |
| **PIDD**  | 方向控制 | 12.9 | 0   | 13.6 | 转向控制     |
| **PID1**  | 直线方向 | 10   | 0   | 10   | 直线行驶     |
| **PIDSL** | 左轮速度 | 30   | 2.4 | 0    | 继承 PIDS    |

## ? 参数应用说明

### 1. 导航路径跟踪 (PIDG)

```c
#define NAV_DEFAULT_PID_KP          (24.3f)     // 主要路径跟踪
#define NAV_DEFAULT_PID_KI          (0.0f)      // 无积分项
#define NAV_DEFAULT_PID_KD          (29.5f)     // 强微分项，快速响应
```

- **用途**: 控制小车沿预定路径行驶的精度
- **特点**: 高 Kp 保证快速响应，高 Kd 提供稳定性，无 Ki 避免超调
- **适用场景**: 正方形路径的直线段和转角跟踪

### 2. 电机速度控制 (PIDS)

```c
#define MOTOR_SPEED_PID_KP          (30.0f)     // 速度环比例系数
#define MOTOR_SPEED_PID_KI          (2.4f)      // 速度环积分系数
#define MOTOR_SPEED_PID_KD          (0.0f)      // 无微分项
```

- **用途**: 控制电机转速，维持目标速度
- **特点**: 高 Kp 快速响应，适量 Ki 消除稳态误差，无 Kd 避免噪声
- **适用场景**: 3.5m/s 高速运行的速度维持

### 3. 方向控制 (PIDD)

```c
#define DIRECTION_PID_KP            (12.9f)     // 方向控制比例系数
#define DIRECTION_PID_KI            (0.0f)      // 无积分项
#define DIRECTION_PID_KD            (13.6f)     // 方向控制微分系数
```

- **用途**: 控制小车的航向角，保证方向准确
- **特点**: 中等 Kp 平衡响应速度，高 Kd 提供方向稳定性
- **适用场景**: 正方形路径的转角控制

### 4. 直线行驶 (PID1)

```c
#define STRAIGHT_LINE_PID_KP        (10.0f)     // 直线方向比例系数
#define STRAIGHT_LINE_PID_KI        (0.0f)      // 无积分项
#define STRAIGHT_LINE_PID_KD        (10.0f)     // 直线方向微分系数
```

- **用途**: 专门用于直线段的方向保持
- **特点**: 平衡的 Kp 和 Kd，确保直线行驶稳定性
- **适用场景**: 正方形路径的四条边

### 5. 左轮速度 (PIDSL)

```c
#define LEFT_WHEEL_SPEED_PID_KP     (30.0f)     // 继承PIDS参数
#define LEFT_WHEEL_SPEED_PID_KI     (2.4f)      // 继承PIDS参数
#define LEFT_WHEEL_SPEED_PID_KD     (0.0f)      // 继承PIDS参数
```

- **用途**: 单独控制左轮速度，实现差速控制
- **特点**: 与 PIDS 相同的参数，保证左右轮一致性
- **适用场景**: 差速转向和速度同步

## ? 正方形路径测试中的应用

### 自动参数设置

测试系统会在初始化时自动应用这些 PID 参数：

```c
static void apply_tuned_pid_parameters(void)
{
    // 导航路径跟踪PID (PIDG)
    nav_config_set_pid(24.3f, 0.0f, 29.5f, 100);

    // 左轮速度PID (PIDSL)
    nav_config_set_motor_pid(0, 0, 30.0f, 2.4f, 0.0f);

    // 右轮速度PID (PIDS)
    nav_config_set_motor_pid(1, 0, 30.0f, 2.4f, 0.0f);

    // 更新电机控制系统
    motor_set_pid_params(MOTOR_CTRL_ID_LEFT, PID_TYPE_SPEED, 30.0f, 2.4f, 0.0f);
    motor_set_pid_params(MOTOR_CTRL_ID_RIGHT, PID_TYPE_SPEED, 30.0f, 2.4f, 0.0f);
}
```

### 参数验证指标

在 3.5m/s 高速正方形路径测试中，这些参数应该实现：

1. **路径跟踪精度**: 平均误差 < 50mm
2. **速度控制精度**: 3.5±0.3m/s
3. **角度控制精度**: 转角误差 < 5°
4. **系统稳定性**: 无振荡，无超调

## ? 参数调优建议

### 如果路径跟踪误差过大：

- 适当增加 PIDG 的 Kp (24.3 → 30.0)
- 检查 PIDG 的 Kd 是否足够 (29.5)

### 如果速度不稳定：

- 调整 PIDS 的 Ki (2.4 → 3.0)
- 检查电机负载和电源供应

### 如果转角不准确：

- 增加 PIDD 的 Kp (12.9 → 15.0)
- 调整 PIDD 的 Kd (13.6 → 16.0)

### 如果直线行驶偏移：

- 调整 PID1 的参数平衡
- 检查机械对称性

## ? 性能监测

测试系统会实时监测以下性能指标来验证 PID 参数效果：

- **位置误差统计**: 平均误差、最大误差
- **速度性能**: 平均速度、速度稳定性
- **陀螺仪漂移**: 检测 PID 控制对 IMU 的影响
- **完成圈数**: 评估整体系统稳定性

通过这些实际调试的 PID 参数，正方形路径测试系统能够实现稳定的高速循环行驶，为陀螺仪漂移监测提供可靠的测试平台。
