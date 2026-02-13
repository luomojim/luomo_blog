---
title: STM32智能小车项目参数配置说明
published: 2026-02-07
pinned: false
description: STM32智能小车项目参数配置说明
tags: [stm32, 智能小车, 参数配置]
category: 教程
licenseName: "MIT"
author: luomo
sourceLink: ""
draft: false
date: 2026-02-07
pubDate: 2026-02-07
---

# STM32智能小车项目参数配置说明

> 本文档详细说明了STM32智能小车避障程序中所有可调整参数的配置方法、取值范围和使用场景。

---

## 目录

1. [电机控制参数](#1-电机控制参数)
2. [PWM控制参数](#2-pwm控制参数)
3. [PID控制参数](#3-pid控制参数)
4. [超声波传感器参数](#4-超声波传感器参数)
5. [红外传感器参数](#5-红外传感器参数)
6. [系统定时参数](#6-系统定时参数)
7. [典型应用场景配置示例](#7-典型应用场景配置示例)

---

## 1. 电机控制参数

### 1.1 电机方向配置

| 参数名称 | `MOTOR_DIRECTION_FORWARD` | `MOTOR_DIRECTION_REVERSE` |
|---------|--------------------------|--------------------------|
| **所在文件** | [motor.h](file:///d:\code\stm32\motor.h#L22) | [motor.h](file:///d:\code\stm32\motor.h#L23) |
| **数据类型** | 宏定义 (uint8_t) | 宏定义 (uint8_t) |
| **默认值** | 0 | 1 |
| **功能描述** | 电机正转方向配置 | 电机反转方向配置 |
| **取值范围** | 0 | 1 |
| **使用场景** | 正常行驶方向 | 需要反向行驶时 |

**用法说明：**
```c
// 方法1：运行时动态设置
Motor_SetDirection(MOTOR_DIRECTION_FORWARD);  // 设置为正转
Motor_SetDirection(MOTOR_DIRECTION_REVERSE);  // 设置为反转

// 方法2：修改默认方向（在motor.c第7行修改）
static uint8_t motor_direction_config = MOTOR_DIRECTION_FORWARD;  // 默认正转
```

**注意事项：**
- 方向切换会影响所有电机控制函数（Motor_Forward、Motor_Back、Motor_Left、Motor_Right）
- 切换方向后，前进和后退的实际方向会互换
- 建议在Motor_Init()之后立即设置方向

---

### 1.2 电机速度参数

| 参数名称 | `NORMAL_LEFT_SPEED` | `NORMAL_RIGHT_SPEED` | `TURN_SPEED` | `BACK_SPEED` |
|---------|---------------------|----------------------|--------------|--------------|
| **所在文件** | [motor.h](file:///d:\code\stm32\motor.h#L25) | [motor.h](file:///d:\code\stm32\motor.h#L26) | [motor.h](file:///d:\code\stm32\motor.h#L27) | [motor.h](file:///d:\code\stm32\motor.h#L28) |
| **数据类型** | float | float | float | float |
| **默认值** | 70.0 | 60.0 | 50.0 | 50.0 |
| **功能描述** | 左电机正常行驶速度 | 右电机正常行驶速度 | 转向时的电机速度 | 倒车时的电机速度 |
| **取值范围** | 0.0 ~ 99.0 | 0.0 ~ 99.0 | 0.0 ~ 99.0 | 0.0 ~ 99.0 |
| **单位** | PWM占空比(%) | PWM占空比(%) | PWM占空比(%) | PWM占空比(%) |

**用法说明：**
```c
// 直接修改motor.h中的宏定义
#define NORMAL_LEFT_SPEED 70.0f   // 调整左轮速度
#define NORMAL_RIGHT_SPEED 60.0f  // 调整右轮速度
#define TURN_SPEED 50.0f         // 调整转向速度
#define BACK_SPEED 50.0f         // 调整倒车速度
```

**使用场景：**
- `NORMAL_LEFT_SPEED` / `NORMAL_RIGHT_SPEED`：正常直行时的左右轮速度，可根据电机特性差异调整以保持直线行驶
- `TURN_SPEED`：转向时的速度，过快可能导致转向过度，过慢影响响应速度
- `BACK_SPEED`：倒车速度，通常设置为较低值以保证安全

**注意事项：**
- 左右轮速度差异会影响直线行驶的稳定性
- 速度值不能超过99.0（PWM最大占空比）
- 速度为0时电机停止
- 建议左右轮速度差不超过20，避免转向过大

---

### 1.3 电机引脚配置

| 参数名称 | `IN1_PIN` | `IN2_PIN` | `IN3_PIN` | `IN4_PIN` |
|---------|-----------|-----------|-----------|-----------|
| **所在文件** | [motor.h](file:///d:\code\stm32\motor.h#L7) | [motor.h](file:///d:\code\stm32\motor.h#L8) | [motor.h](file:///d:\code\stm32\motor.h#L9) | [motor.h](file:///d:\code\stm32\motor.h#L10) |
| **数据类型** | GPIO引脚宏 | GPIO引脚宏 | GPIO引脚宏 | GPIO引脚宏 |
| **默认值** | GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
| **功能描述** | 左电机方向控制1 | 左电机方向控制2 | 右电机方向控制1 | 右电机方向控制2 |
| **取值范围** | GPIO_Pin_0~15 | GPIO_Pin_0~15 | GPIO_Pin_0~15 | GPIO_Pin_0~15 |

**DRV8833电机驱动器真值表：**

| IN1 | IN2 | 左电机状态 |
|-----|-----|-----------|
| 0 | 0 | 刹车 |
| 0 | 1 | 反转 |
| 1 | 0 | 正转 |
| 1 | 1 | 刹车 |

| IN3 | IN4 | 右电机状态 |
|-----|-----|-----------|
| 0 | 0 | 刹车 |
| 0 | 1 | 反转 |
| 1 | 0 | 正转 |
| 1 | 1 | 刹车 |

**注意事项：**
- 引脚配置与硬件连接必须一致
- 修改引脚后需检查硬件连接
- 确保所选引脚未被其他外设占用

---

## 2. PWM控制参数

### 2.1 PWM基础参数

| 参数名称 | `PWM_PERIOD` | `PWM_FREQUENCY` | `PWM_UNIT_TIME` |
|---------|--------------|-----------------|-----------------|
| **所在文件** | [pwm.c](file:///d:\code\stm32\pwm.c#L4) | [pwm.c](file:///d:\code\stm32\pwm.c#L5) | [pwm.c](file:///d:\code\stm32\pwm.c#L6) |
| **数据类型** | uint16_t | uint16_t | 计算值 |
| **默认值** | 100 | 1000 | 10 |
| **功能描述** | PWM周期（单位数） | PWM频率（Hz） | 每个PWM单位的微秒数 |
| **取值范围** | 1 ~ 65535 | 1 ~ 10000 | 自动计算 |
| **单位** | - | Hz | μs |

**计算公式：**
```c
PWM_UNIT_TIME = 1000000 / PWM_FREQUENCY / PWM_PERIOD
```

**用法说明：**
```c
// 在pwm.c中修改
#define PWM_PERIOD 100      // PWM周期为100个单位
#define PWM_FREQUENCY 1000   // PWM频率为1kHz
// PWM_UNIT_TIME会自动计算为10μs
```

**使用场景：**
- `PWM_PERIOD`：决定PWM的分辨率，值越大分辨率越高，但频率会降低
- `PWM_FREQUENCY`：决定PWM的频率，频率越高电机运行越平滑，但CPU占用越高
- 典型配置：1000Hz频率，100周期（1%分辨率）

**注意事项：**
- PWM频率建议设置在500Hz~2000Hz之间
- 频率过低会导致电机抖动
- 频率过高会增加CPU负担
- PWM_UNIT_TIME是自动计算的，无需手动修改

---

### 2.2 PWM输出引脚

| 参数名称 | `PWM1引脚` | `PWM2引脚` |
|---------|-----------|-----------|
| **所在文件** | [pwm.c](file:///d:\code\stm32\pwm.c#L18) | [pwm.c](file:///d:\code\stm32\pwm.c#L18) |
| **数据类型** | GPIO引脚宏 | GPIO引脚宏 |
| **默认值** | GPIO_Pin_8 (PA8) | GPIO_Pin_9 (PA9) |
| **功能描述** | 左电机PWM输出 | 右电机PWM输出 |
| **取值范围** | GPIO_Pin_0~15 | GPIO_Pin_0~15 |

**用法说明：**
```c
// 在pwm.c的PWM_Init()函数中修改
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;  // 修改为其他引脚
```

**注意事项：**
- 引脚必须支持推挽输出模式
- 确保引脚与电机驱动器的PWM输入连接正确
- 修改引脚后需同步修改电机控制函数中的引脚引用

---

## 3. PID控制参数

### 3.1 PID系数参数

| 参数名称 | `PID_MotorLeft.Kp` | `PID_MotorLeft.Ki` | `PID_MotorLeft.Kd` | `PID_MotorRight.Kp` | `PID_MotorRight.Ki` | `PID_MotorRight.Kd` |
|---------|-------------------|-------------------|-------------------|--------------------|--------------------|--------------------|
| **所在文件** | [PID.c](file:///d:\code\stm32\PID.c#L10) | [PID.c](file:///d:\code\stm32\PID.c#L11) | [PID.c](file:///d:\code\stm32\PID.c#L12) | [PID.c](file:///d:\code\stm32\PID.c#L21) | [PID.c](file:///d:\code\stm32\PID.c#L22) | [PID.c](file:///d:\code\stm32\PID.c#L23) |
| **数据类型** | float | float | float | float | float | float |
| **默认值** | 2.5 | 0.1 | 0.05 | 2.5 | 0.1 | 0.05 |
| **功能描述** | 左电机比例系数 | 左电机积分系数 | 左电机微分系数 | 右电机比例系数 | 右电机积分系数 | 右电机微分系数 |
| **取值范围** | 1.5 ~ 3.5 | 0.05 ~ 0.2 | 0.03 ~ 0.1 | 1.5 ~ 3.5 | 0.05 ~ 0.2 | 0.03 ~ 0.1 |

**PID参数说明：**

| 参数 | 作用 | 增大影响 | 减小影响 |
|-----|------|---------|---------|
| **Kp (比例)** | 快速响应误差 | 响应更快，但可能超调 | 响应慢，但更稳定 |
| **Ki (积分)** | 消除稳态误差 | 消除误差更快，但可能振荡 | 消除误差慢，但更稳定 |
| **Kd (微分)** | 抑制超调 | 抑制超调更强，但对噪声敏感 | 抑制超调弱，但更平滑 |

**用法说明：**
```c
// 在PID.c的PID_Init()函数中修改
PID_MotorLeft.Kp = 2.5f;   // 调整左电机比例系数
PID_MotorLeft.Ki = 0.1f;   // 调整左电机积分系数
PID_MotorLeft.Kd = 0.05f;  // 调整左电机微分系数

PID_MotorRight.Kp = 2.5f;  // 调整右电机比例系数
PID_MotorRight.Ki = 0.1f;  // 调整右电机积分系数
PID_MotorRight.Kd = 0.05f; // 调整右电机微分系数
```

**使用场景：**
- **直线行驶**：适当增大Kp，减小Kd
- **转向控制**：适当减小Kp，增大Kd
- **负载变化大**：适当增大Ki
- **噪声干扰大**：适当减小Kd

**PID调参步骤：**
1. 首先设置Ki=0, Kd=0，调整Kp直到系统出现轻微振荡
2. 将Kp减小到振荡值的60%
3. 逐渐增大Ki，直到稳态误差消除
4. 逐渐增大Kd，直到超调量可接受

**注意事项：**
- 左右电机PID参数可以不同，以补偿电机特性差异
- PID参数调整后需要重新测试
- 积分项需要限幅（默认±500）防止积分饱和
- 输出需要限幅（默认0~100）匹配PWM范围

---

### 3.2 PID限幅参数

| 参数名称 | 积分上限 | 积分下限 | 输出上限 | 输出下限 |
|---------|---------|---------|---------|---------|
| **所在文件** | [PID.c](file:///d:\code\stm32\PID.c#L40) | [PID.c](file:///d:\code\stm32\PID.c#L41) | [PID.c](file:///d:\code\stm32\PID.c#L47) | [PID.c](file:///d:\code\stm32\PID.c#L48) |
| **数据类型** | float | float | float | float |
| **默认值** | 500 | -500 | 100 | 0 |
| **功能描述** | 积分项上限 | 积分项下限 | PID输出上限 | PID输出下限 |
| **取值范围** | 0 ~ 1000 | -1000 ~ 0 | 0 ~ 100 | 0 ~ 100 |

**用法说明：**
```c
// 在PID.c的PID_Calc()函数中修改
if(pid->SumError > 500) pid->SumError = 500;   // 积分上限
if(pid->SumError < -500) pid->SumError = -500;  // 积分下限

if(pid->Output > 100) pid->Output = 100;  // 输出上限
if(pid->Output < 0) pid->Output = 0;     // 输出下限
```

**注意事项：**
- 积分限幅防止积分饱和导致的系统失控
- 输出限幅必须与PWM范围一致（0~100）
- 限幅值过小会影响系统性能
- 限幅值过大会导致系统不稳定

---

## 4. 超声波传感器参数

### 4.1 超声波距离阈值参数

| 参数名称 | `ULTRASONIC_STOP_DIST` | `ULTRASONIC_SLOW_DIST` | `ULTRASONIC_INVALID_CNT` |
|---------|------------------------|------------------------|--------------------------|
| **所在文件** | [main.c](file:///d:\code\stm32\main.c#L11) | [main.c](file:///d:\code\stm32\main.c#L12) | [main.c](file:///d:\code\stm32\main.c#L13) |
| **数据类型** | float | float | uint8_t |
| **默认值** | 15.0 | 30.0 | 2 |
| **功能描述** | 停车距离阈值 | 减速距离阈值 | 超声无效计数阈值 |
| **取值范围** | 5.0 ~ 50.0 | 10.0 ~ 100.0 | 1 ~ 10 |
| **单位** | cm | cm | 次数 |

**用法说明：**
```c
// 在main.c中修改
#define ULTRASONIC_STOP_DIST 15.0f   // ≤15cm停车并后退
#define ULTRASONIC_SLOW_DIST 30.0f   // 30~15cm减速
#define ULTRASONIC_INVALID_CNT 2    // 连续2次无效触发停车
```

**使用场景：**
- `ULTRASONIC_STOP_DIST`：检测到障碍物距离≤此值时，立即停车并后退
- `ULTRASONIC_SLOW_DIST`：检测到障碍物距离在此范围内时，降低速度
- `ULTRASONIC_INVALID_CNT`：连续多次检测无效值时，触发停车保护

**参数关系：**
```
ULTRASONIC_STOP_DIST < ULTRASONIC_SLOW_DIST
```

**注意事项：**
- 停车距离应大于车辆最小转弯半径
- 减速距离应给车辆留出足够的减速空间
- 无效计数阈值不宜过大，避免响应延迟
- 超声波传感器有效测量范围通常为2cm~400cm

---

### 4.2 超声波引脚配置

| 参数名称 | `TRIG_PIN` | `ECHO_PIN` |
|---------|-----------|-----------|
| **所在文件** | [motor.h](file:///d:\code\stm32\motor.h#L40) | [motor.h](file:///d:\code\stm32\motor.h#L39) |
| **数据类型** | GPIO引脚宏 | GPIO引脚宏 |
| **默认值** | GPIO_Pin_0 (PB0) | GPIO_Pin_1 (PB1) |
| **功能描述** | 触发信号输出引脚 | 回波信号输入引脚 |
| **取值范围** | GPIO_Pin_0~15 | GPIO_Pin_0~15 |

**超声波工作原理：**
1. 主机向TRIG引脚发送至少10μs的高电平脉冲
2. 超声波模块发送8个40kHz的超声波脉冲
3. 超声波遇到障碍物反射回来
4. 模块向ECHO引脚输出高电平，高电平持续时间与距离成正比
5. 距离计算：`距离(cm) = 高电平时间(μs) / 58`

**注意事项：**
- TRIG引脚必须配置为推挽输出模式
- ECHO引脚必须配置为上拉输入模式
- 确保引脚与超声波模块连接正确
- 超声波测量周期建议≥60ms，避免回波干扰

---

## 5. 红外传感器参数

### 5.1 红外传感器引脚配置

| 参数名称 | `RED1_PIN` | `RED2_PIN` | `RED3_PIN` | `RED4_PIN` | `RED5_PIN` | `RED6_PIN` |
|---------|-----------|-----------|-----------|-----------|-----------|-----------|
| **所在文件** | [motor.h](file:///d:\code\stm32\motor.h#L31) | [motor.h](file:///d:\code\stm32\motor.h#L32) | [motor.h](file:///d:\code\stm32\motor.h#L33) | [motor.h](file:///d:\code\stm32\motor.h#L34) | [motor.h](file:///d:\code\stm32\motor.h#L35) | [motor.h](file:///d:\code\stm32\motor.h#L36) |
| **数据类型** | GPIO引脚宏 | GPIO引脚宏 | GPIO引脚宏 | GPIO引脚宏 | GPIO引脚宏 | GPIO引脚宏 |
| **默认值** | GPIO_Pin_4 (PA4) | GPIO_Pin_5 (PA5) | GPIO_Pin_8 (PA8) | GPIO_Pin_9 (PA9) | GPIO_Pin_11 (PA11) | GPIO_Pin_12 (PA12) |
| **功能描述** | 前方左侧红外 | 前方右侧红外 | 左侧前红外 | 右侧前红外 | 左侧后红外 | 右侧后红外 |
| **取值范围** | GPIO_Pin_0~15 | GPIO_Pin_0~15 | GPIO_Pin_0~15 | GPIO_Pin_0~15 | GPIO_Pin_0~15 | GPIO_Pin_0~15 |

**红外传感器布局：**
```
        前方
    [RED1][RED2]
    
[RED3]          [RED4]
 左侧            右侧
    
[RED5]          [RED6]
 左侧            右侧
```

**用法说明：**
```c
// 在motor.h中修改引脚定义
#define RED1_PIN GPIO_Pin_4   // 修改为其他引脚
#define RED2_PIN GPIO_Pin_5
// ... 其他引脚
```

**注意事项：**
- 所有红外引脚必须配置为上拉输入模式
- 红外传感器输出逻辑：1=检测到障碍，0=无障碍
- 确保引脚与红外模块连接正确
- 红外传感器检测距离通常为2cm~30cm

---

### 5.2 红外传感器避障逻辑

**避障优先级（从高到低）：**

| 优先级 | 触发条件 | 动作 |
|-------|---------|------|
| 1 | 超声停车触发（≤15cm或无效） | 停车并后退10cm |
| 2 | 超声减速+RED1+RED2同触 | 后退3cm，右转90°，前进10cm，右转90° |
| 3 | 超声减速+RED1单触 | 后退3cm，右转90°，前进14cm，左转90°，前进14cm，左转90°，前进14cm，右转90° |
| 4 | 超声减速+RED2单触 | 后退3cm，左转90°，前进14cm，右转90°，前进14cm，右转90°，前进14cm，左转90° |
| 5 | RED3/RED5任一触 | 调整左右轮速度 |
| 6 | RED4/RED6任一触 | 调整左右轮速度 |
| 7 | 超声减速+无红外触发 | 后退10cm |
| 8 | 无任何触发 | 正常直行 |

**速度调整逻辑：**

| 触发条件 | 左轮速度 | 右轮速度 | 说明 |
|---------|---------|---------|------|
| RED3+RED5双触 | NORMAL_LEFT_SPEED+10 | NORMAL_RIGHT_SPEED | 远离左墙 |
| RED3单触 | NORMAL_LEFT_SPEED | NORMAL_RIGHT_SPEED+10 | 微调回正 |
| RED5单触 | NORMAL_LEFT_SPEED+10 | NORMAL_RIGHT_SPEED | 远离左墙 |
| RED4+RED6双触 | NORMAL_LEFT_SPEED | NORMAL_RIGHT_SPEED+10 | 远离右墙 |
| RED4单触 | NORMAL_LEFT_SPEED+10 | NORMAL_RIGHT_SPEED | 微调回正 |
| RED6单触 | NORMAL_LEFT_SPEED | NORMAL_RIGHT_SPEED+10 | 远离右墙 |

**注意事项：**
- 避障逻辑是互斥的，同一时间只触发最高优先级的动作
- 速度调整值（+10）可以根据实际情况调整
- 转向延时（800ms）和移动距离（3cm、10cm、14cm）可以根据实际情况调整

---

## 6. 系统定时参数

### 6.1 TIM2定时器配置（超声波测距）

| 参数名称 | `TIM_Period` | `TIM_Prescaler` | `TIM_CounterMode` |
|---------|--------------|-----------------|-------------------|
| **所在文件** | [main.c](file:///d:\code\stm32\main.c#L22) | [main.c](file:///d:\code\stm32\main.c#L23) | [main.c](file:///d:\code\stm32\main.c#L25) |
| **数据类型** | uint16_t | uint16_t | 枚举类型 |
| **默认值** | 0xFFFF | 71 | TIM_CounterMode_Up |
| **功能描述** | 定时器周期 | 预分频系数 | 计数模式 |
| **取值范围** | 0 ~ 0xFFFF | 0 ~ 65535 | TIM_CounterMode_Up/Down |

**计算公式：**
```c
计数频率 = 系统时钟 / (预分频系数 + 1)
计数精度 = 1 / 计数频率

默认配置：
计数频率 = 72MHz / (71 + 1) = 1MHz
计数精度 = 1 / 1MHz = 1μs
```

**用法说明：**
```c
// 在main.c的main()函数中修改
TIM_TimeBaseStruct.TIM_Period = 0xFFFF;      // 最大计数范围
TIM_TimeBaseStruct.TIM_Prescaler = 71;       // 72MHz/72=1MHz（1μs计数）
TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
```

**注意事项：**
- 预分频系数为71时，计数精度为1μs，适合超声波测距
- 修改预分频系数会影响超声波测距精度
- 周期0xFFFF可测量最大时间：65535μs ≈ 65ms

---

### 6.2 延时参数

| 参数名称 | `Delay_ms()` | `Delay_us()` | `move_delay()` |
|---------|-------------|-------------|----------------|
| **所在文件** | delay.c | delay.c | [motor.c](file:///d:\code\stm32\motor.c#L51-57) |
| **数据类型** | 函数 | 函数 | 函数 |
| **功能描述** | 毫秒级延时 | 微秒级延时 | 根据距离和速度计算延时 |
| **取值范围** | 0 ~ 65535 | 0 ~ 65535 | 自动计算 |

**move_delay计算公式：**
```c
time_ms = (cm * 60) / (speed / 10)
```

**用法说明：**
```c
// 毫秒延时
Delay_ms(100);  // 延时100ms

// 微秒延时
Delay_us(10);   // 延时10μs

// 距离延时（在motor.c中调整计算公式）
void move_delay(float cm, float speed)
{
    float time_ms = (cm * 60) / (speed / 10);  // 适配系数60可调整
    if (speed <= 0)
        time_ms = 0;
    Delay_ms((uint16_t)time_ms);
}
```

**注意事项：**
- 延时函数精度受系统时钟影响
- move_delay中的适配系数60需要根据实际车辆速度调整
- 主循环中的Delay_us(10)用于消抖，不宜过小或过大

---

### 6.3 主循环参数

| 参数名称 | `PWM_Task()`调用频率 | `Delay_us(10)` |
|---------|---------------------|----------------|
| **所在文件** | [main.c](file:///d:\code\stm32\main.c#L169) | [main.c](file:///d:\code\stm32\main.c#L170) |
| **数据类型** | 函数调用 | 函数调用 |
| **功能描述** | 生成PWM波形 | 消抖延时 |
| **默认值** | 每次循环调用 | 10μs |

**用法说明：**
```c
while (1)
{
    // 避障逻辑...
    
    PWM_Task();   // 必须调用，生成软件PWM波形
    Delay_us(10); // 消抖，避免频繁触发
}
```

**注意事项：**
- PWM_Task()必须在主循环中调用，否则PWM无法工作
- Delay_us(10)的值不宜过小（<5μs）或过大（>50μs）
- 主循环总周期 = 避障逻辑执行时间 + Delay_us(10)

---

## 7. 典型应用场景配置示例

### 7.1 场景1：室内平坦地面快速行驶

**配置目标：** 提高行驶速度，保持稳定性

```c
// motor.h - 速度配置
#define NORMAL_LEFT_SPEED 85.0f   // 提高左轮速度
#define NORMAL_RIGHT_SPEED 80.0f  // 提高右轮速度
#define TURN_SPEED 60.0f          // 提高转向速度
#define BACK_SPEED 60.0f          // 提高倒车速度

// motor.c - 默认方向
static uint8_t motor_direction_config = MOTOR_DIRECTION_FORWARD;

// PID.c - PID参数
PID_MotorLeft.Kp = 2.0f;   // 减小Kp，避免超调
PID_MotorLeft.Ki = 0.15f;  // 增大Ki，消除稳态误差
PID_MotorLeft.Kd = 0.08f;  // 增大Kd，抑制超调

PID_MotorRight.Kp = 2.0f;
PID_MotorRight.Ki = 0.15f;
PID_MotorRight.Kd = 0.08f;

// main.c - 超声波参数
#define ULTRASONIC_STOP_DIST 20.0f   // 增大停车距离，提高安全性
#define ULTRASONIC_SLOW_DIST 40.0f   // 增大减速距离
#define ULTRASONIC_INVALID_CNT 2
```

---

### 7.2 场景2：室外复杂地形慢速行驶

**配置目标：** 降低速度，增强避障能力

```c
// motor.h - 速度配置
#define NORMAL_LEFT_SPEED 50.0f   // 降低左轮速度
#define NORMAL_RIGHT_SPEED 45.0f  // 降低右轮速度
#define TURN_SPEED 35.0f          // 降低转向速度
#define BACK_SPEED 35.0f          // 降低倒车速度

// motor.c - 默认方向
static uint8_t motor_direction_config = MOTOR_DIRECTION_FORWARD;

// PID.c - PID参数
PID_MotorLeft.Kp = 3.0f;   // 增大Kp，提高响应速度
PID_MotorLeft.Ki = 0.05f;  // 减小Ki，避免积分饱和
PID_MotorLeft.Kd = 0.03f;  // 减小Kd，避免对噪声敏感

PID_MotorRight.Kp = 3.0f;
PID_MotorRight.Ki = 0.05f;
PID_MotorRight.Kd = 0.03f;

// main.c - 超声波参数
#define ULTRASONIC_STOP_DIST 25.0f   // 增大停车距离
#define ULTRASONIC_SLOW_DIST 50.0f   // 增大减速距离
#define ULTRASONIC_INVALID_CNT 1     // 减小无效计数阈值，提高灵敏度
```

---

### 7.3 场景3：狭窄通道精确控制

**配置目标：** 精确控制，低速稳定

```c
// motor.h - 速度配置
#define NORMAL_LEFT_SPEED 40.0f   // 低速行驶
#define NORMAL_RIGHT_SPEED 40.0f  // 左右轮速度一致
#define TURN_SPEED 25.0f          // 低速转向
#define BACK_SPEED 25.0f          // 低速倒车

// motor.c - 默认方向
static uint8_t motor_direction_config = MOTOR_DIRECTION_FORWARD;

// PID.c - PID参数
PID_MotorLeft.Kp = 1.5f;   // 减小Kp，避免超调
PID_MotorLeft.Ki = 0.2f;   // 增大Ki，消除稳态误差
PID_MotorLeft.Kd = 0.1f;   // 增大Kd，抑制超调

PID_MotorRight.Kp = 1.5f;
PID_MotorRight.Ki = 0.2f;
PID_MotorRight.Kd = 0.1f;

// main.c - 超声波参数
#define ULTRASONIC_STOP_DIST 10.0f   // 减小停车距离
#define ULTRASONIC_SLOW_DIST 20.0f   // 减小减速距离
#define ULTRASONIC_INVALID_CNT 3     // 增大无效计数阈值，避免误触发

// motor.c - 调整速度调整值
// 在main.c的避障逻辑中修改速度调整值
if (red3 && red5)
    left_speed += 5;  // 减小速度调整值，避免转向过大
```

---

### 7.4 场景4：需要反向行驶

**配置目标：** 全局反转电机方向

```c
// 方法1：运行时动态设置（在main.c的main()函数中）
int main(void)
{
    // 初始化...
    Motor_Init();
    
    // 设置为反向
    Motor_SetDirection(MOTOR_DIRECTION_REVERSE);
    
    // 主循环...
}

// 方法2：修改默认方向（在motor.c第7行修改）
static uint8_t motor_direction_config = MOTOR_DIRECTION_REVERSE;
```

**注意事项：**
- 方向切换后，前进和后退的实际方向会互换
- 左转和右转的方向也会相应改变
- 建议在Motor_Init()之后立即设置方向

---

### 7.5 场景6：电机特性差异补偿

**配置目标：** 补偿左右电机速度差异

```c
// 方法1：调整基础速度（motor.h）
#define NORMAL_LEFT_SPEED 70.0f   // 左轮较快，降低速度
#define NORMAL_RIGHT_SPEED 75.0f  // 右轮较慢，提高速度

// 方法2：调整PID参数（PID.c）
PID_MotorLeft.Kp = 2.0f;   // 左轮响应较快，减小Kp
PID_MotorLeft.Ki = 0.1f;
PID_MotorLeft.Kd = 0.05f;

PID_MotorRight.Kp = 3.0f;  // 右轮响应较慢，增大Kp
PID_MotorRight.Ki = 0.1f;
PID_MotorRight.Kd = 0.05f;

// 方法3：调整速度调整值（main.c）
// 在避障逻辑中，根据左右轮特性调整速度调整值
if (red3 && red5)
    left_speed += 8;   // 左轮调整值减小
    right_speed += 12; // 右轮调整值增大
```

---

## 8. 参数调试建议

### 8.1 调试步骤

1. **基础功能测试**
   - 测试电机正转、反转、停止
   - 测试PWM输出是否正常
   - 测试传感器数据读取

2. **速度参数调整**
   - 从低速开始测试（30~40）
   - 逐步提高速度，观察稳定性
   - 调整左右轮速度差，保持直线行驶

3. **PID参数调整**
   - 先调整Kp，直到出现轻微振荡
   - 将Kp减小到振荡值的60%
   - 逐步调整Ki，消除稳态误差
   - 逐步调整Kd，抑制超调

4. **避障参数调整**
   - 测试超声波测距精度
   - 调整停车和减速距离
   - 测试红外传感器检测范围
   - 调整避障逻辑的触发条件

5. **综合测试**
   - 在实际环境中测试
   - 记录问题并调整参数
   - 反复迭代优化

### 8.2 常见问题及解决方案

| 问题 | 可能原因 | 解决方案 |
|-----|---------|---------|
| 车辆无法直线行驶 | 左右轮速度不一致 | 调整NORMAL_LEFT_SPEED和NORMAL_RIGHT_SPEED |
| 车辆转向过度 | TURN_SPEED过大 | 减小TURN_SPEED |
| 车辆响应迟钝 | PID参数Kp过小 | 增大Kp |
| 车辆振荡 | PID参数Kp过大或Kd过小 | 减小Kp或增大Kd |
| 避障不灵敏 | 超声波距离阈值过大 | 减小ULTRASONIC_STOP_DIST |
| 频繁误触发 | 超声波无效计数阈值过小 | 增大ULTRASONIC_INVALID_CNT |
| 电机抖动 | PWM频率过低 | 增大PWM_FREQUENCY |
| CPU占用过高 | PWM频率过高 | 减小PWM_FREQUENCY |

---

## 9. 附录

### 9.1 文件结构

```
d:\code\stm32\
├── main.c              # 主程序文件（超声波参数、系统定时参数）
├── motor.h             # 电机控制头文件（引脚定义、速度参数、方向配置）
├── motor.c             # 电机控制实现文件（方向控制、电机驱动函数）
├── pwm.h               # PWM控制头文件
├── pwm.c               # PWM控制实现文件（PWM参数、PWM函数）
├── PID.h               # PID控制头文件
├── PID.c               # PID控制实现文件（PID参数、PID函数）
├── Ultrasound.h        # 超声波传感器头文件
├── Ultrasound.c        # 超声波传感器实现文件
├── IRSensor.h          # 红外传感器头文件
├── IRSensor.c          # 红外传感器实现文件
└── delay.h/c           # 延时函数文件
```

### 9.2 参数速查表

| 参数类别 | 参数名称 | 所在文件 | 行号 |
|---------|---------|---------|------|
| 电机方向 | MOTOR_DIRECTION_FORWARD | motor.h | 22 |
| 电机方向 | MOTOR_DIRECTION_REVERSE | motor.h | 23 |
| 左轮速度 | NORMAL_LEFT_SPEED | motor.h | 25 |
| 右轮速度 | NORMAL_RIGHT_SPEED | motor.h | 26 |
| 转向速度 | TURN_SPEED | motor.h | 27 |
| 倒车速度 | BACK_SPEED | motor.h | 28 |
| PWM周期 | PWM_PERIOD | pwm.c | 4 |
| PWM频率 | PWM_FREQUENCY | pwm.c | 5 |
| 左电机Kp | PID_MotorLeft.Kp | PID.c | 10 |
| 左电机Ki | PID_MotorLeft.Ki | PID.c | 11 |
| 左电机Kd | PID_MotorLeft.Kd | PID.c | 12 |
| 右电机Kp | PID_MotorRight.Kp | PID.c | 21 |
| 右电机Ki | PID_MotorRight.Ki | PID.c | 22 |
| 右电机Kd | PID_MotorRight.Kd | PID.c | 23 |
| 停车距离 | ULTRASONIC_STOP_DIST | main.c | 11 |
| 减速距离 | ULTRASONIC_SLOW_DIST | main.c | 12 |
| 无效计数 | ULTRASONIC_INVALID_CNT | main.c | 13 |

---

## 10. 版本历史

| 版本 | 日期 | 修改内容 |
|-----|------|---------|
| 1.0 | 2026-02-07 | 初始版本，完成所有参数配置说明 |

---

**文档结束**

如有疑问或需要进一步说明，请参考源代码注释或联系开发人员。
