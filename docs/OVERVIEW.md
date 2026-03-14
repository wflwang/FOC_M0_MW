# HK32M070 FOC VESC风格电机控制系统 - 技术概览

## 项目完成状态

本项目已完成VESC风格的FOC电机控制系统核心代码开发，针对HK32M070 QFN32 (Cortex-M0, 64MHz)进行了定点数优化。

---

## 已实现模块

### 1. 定点数数学库 (`fixed_point.h/c`)

**功能：**
- Q15、Q31、Q8.8、Q12.20、Q16.16等多种定点格式
- 优化的算术运算（乘法、除法、饱和运算）
- CORDIC atan2算法（仅用移位和加法）
- 快速sin/cos查表插值
- 向量运算（Park/Clarke变换、归一化）
- 低通滤波器、移动平均滤波器

**关键优化：**
```c
// Q15乘法（避免浮点）
static inline q15_t q15_mul(q15_t a, q15_t b) {
    q31_t product = (q31_t)a * b;
    return (q15_t)((product + 0x4000) >> 15);
}

// CORDIC atan2（16次迭代，精度约0.1°）
q16_16_t fast_atan2(q15_t y, q15_t x);
```

---

### 2. MXLemming观测器 (`foc_observer.h/c`)

**核心算法：**
```
ψ_α = ∫(v_α - R·i_α)dt - L·i_α
ψ_β = ∫(v_β - R·i_β)dt - L·i_β
θ = atan2(ψ_β, ψ_α)
```

**特性：**
- 完全定点数实现，适合M0内核
- PLL速度跟踪器（可配置带宽）
- 磁饱和补偿
- 温度补偿
- 跟踪质量评估

**关键数据结构：**
```c
typedef struct {
    q12_20_t psi_alpha;      // α轴磁链（Q12.20格式）
    q12_20_t psi_beta;       // β轴磁链
    q12_20_t flux_magnitude; // 磁链幅值
    q16_16_t angle;          // 估计角度
    q31_t pll_speed;         // PLL速度
    uint8_t confidence;      // 置信度（0-255）
} observer_runtime_t;
```

---

### 3. 霍尔传感器处理 (`foc_hall.h/c`)

**功能：**
- 霍尔状态检测与去抖动
- 角度插值（低速直接使用霍尔，高速插值）
- 速度估计
- 方向检测
- 自动霍尔表学习

**角度插值策略：**
- 低于 `hall_interp_min_rpm`: 直接使用霍尔扇区角度
- 高于阈值: 线性插值，平滑过渡

---

### 4. 角度融合模块 (`foc_angle_fusion.h/c`)

**VESC风格融合策略：**

```
速度 (ERPM)
    │
    │         观测器模式
    │    ┌───────────────────────
    │    │   融合过渡区
    │    │   (加权融合)
    │────┤───────────────────────  foc_sl_erpm
    │    │   霍尔主导
    │────┤───────────────────────  foc_sl_erpm_start
    │    │   霍尔模式
    │    │
    └────┴───────────────────────> 速度
```

**核心函数：**
```c
q16_16_t angle_fusion_update(angle_fusion_state_t *state,
                               const angle_fusion_config_t *config,
                               const hall_state_t *hall_state,
                               const observer_runtime_t *observer_state,
                               q12_4_t speed_rpm,
                               q16_16_t dt);
```

---

### 5. 电机参数自学习 (`foc_motor_detect.h/c`)

**自动检测项目：**

| 参数 | 方法 | 原理 |
|------|------|------|
| 电阻 R | DC注入 | R = V/I |
| 电感 L | 高频脉冲 | L = V·dt/di |
| Ld-Lq差 | 多位置测量 | IPM电机凸极效应 |
| 磁链 λ | 反电动势积分 | λ = V_bemf/ω |
| 霍尔表 | 开环旋转记录 | 记录霍尔跳变角度 |

**检测流程：**
```c
// 完整检测序列
detect_start_full(&sm, &config, &results);

// 在控制循环中更新
detect_update(&sm, dt, v_bus, i_a, i_b, i_c, hall_state, &duty_a, &duty_b, &duty_c);
```

---

### 6. 电流控制器 (`foc_current_ctrl.h/c`)

**PI控制器 + 解耦：**

```
// dq轴电压方程
v_d = R·i_d + L_d·(di_d/dt) - ω·L_q·i_q
v_q = R·i_q + L_q·(di_q/dt) + ω·L_d·i_d + ω·λ

// 解耦补偿
v_d_comp = v_d_pi - ω·L_q·i_q
v_q_comp = v_q_pi + ω·L_d·i_d + ω·λ
```

**解耦模式：**
- `DECOUPLE_NONE`: 无解耦
- `DECOUPLE_CROSS`: 交叉解耦
- `DECOUPLE_BEMF`: 反电动势补偿
- `DECOUPLE_CROSS_BEMF`: 完全解耦

---

### 7. SVPWM生成器 (`foc_svpwm.h/c`)

**空间矢量调制：**

```
     β
     │
  2  │  1
     │
─────┼───── α
     │
  3  │  6
     │
  4  │  5
```

**特性：**
- 6扇区确定
- 占空比计算
- 过调制支持
- 向量饱和

---

### 8. 主控制循环 (`foc_control.h/c`)

**完整控制架构：**

```
┌─────────────────────────────────────────────────────────────┐
│                    应用层控制接口                             │
│     电流控制 | 速度控制 | 位置控制 | 占空比控制               │
├─────────────────────────────────────────────────────────────┤
│                    FOC核心引擎                               │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
│  │   霍尔      │  │  观测器     │  │    角度融合         │  │
│  │   传感器    │  │  MXLemming  │  │    (速度加权)       │  │
│  └─────────────┘  └─────────────┘  └─────────────────────┘  │
│  ┌─────────────────────────────────────────────────────────┐│
│  │           电流控制器 (PI + 解耦 + 弱磁)                  ││
│  └─────────────────────────────────────────────────────────┘│
│  ┌─────────────────────────────────────────────────────────┐│
│  │           SVPWM生成器 (空间矢量调制)                     ││
│  └─────────────────────────────────────────────────────────┘│
├─────────────────────────────────────────────────────────────┤
│                    保护与平滑控制                            │
│  防抖动 | 换向处理 | 再生制动管理 | 弱磁控制                 │
└─────────────────────────────────────────────────────────────┘
```

---

## 特殊功能实现

### 1. 平滑控制（防抖动）

**原理：**
- 极低速时对角度和速度进行强滤波
- 防止齿槽效应导致电流尖峰
- 平滑换向

```c
void foc_anti_jitter_update(foc_state_t *s) {
    if (speed_abs < jitter_threshold) {
        // 使用更强的低通滤波
        s->angle = lpf(s->angle, s->jitter.angle_filtered, strong_lpf);
    }
}
```

### 2. 正反转快速切换

**流程：**
1. 检测方向变化请求
2. 施加制动电流减速
3. 等待速度接近零
4. 反向启动

```c
void foc_direction_change_update(foc_state_t *s) {
    // 1. 检测方向变化
    // 2. 施加制动电流
    s->current_ctrl.iq_ref = -brake_current * current_direction;
    
    // 3. 等待低速
    if (speed_abs < threshold) {
        // 4. 完成换向
        s->dir_change.current = new_direction;
    }
}
```

### 3. 再生制动管理

**目的：** 防止减速时母线电压过高

**策略：**
- 实时监测母线电压
- 电压超限时按比例减小再生电流
- 软限幅到硬限幅平滑过渡

```c
void foc_regen_update(foc_state_t *s) {
    if (v_bus > v_soft_limit) {
        // 计算再生电流缩放因子
        regen_scale = 1.0 - (v_bus - v_soft) / (v_max - v_soft);
    }
}

q8_8_t foc_calc_regen_limit(foc_state_t *s, q8_8_t requested) {
    return requested * s->regen.regen_scale;
}
```

### 4. 弱磁控制

**目的：** 扩展电机高速范围

**原理：**
- 占空比接近上限时注入负的Id电流
- 减弱气隙磁场，降低反电动势
- 允许更高转速

```c
void foc_field_weakening_update(foc_state_t *s) {
    if (duty > duty_start) {
        // 计算弱磁电流
        fw_current = map(duty, duty_start, duty_max, 0, fw_max);
        s->current_ctrl.id_ref = -fw_current;
    }
}
```

---

## 文件结构

```
D:\project\HK32M070_FOC_VESC\
├── inc/
│   ├── types.h              # 数据类型和定点格式定义
│   ├── fixed_point.h        # 定点数数学库接口
│   ├── foc_observer.h       # MXLemming观测器接口
│   ├── foc_hall.h           # 霍尔传感器处理接口
│   ├── foc_angle_fusion.h   # 角度融合接口
│   ├── foc_current_ctrl.h   # 电流控制器接口
│   ├── foc_svpwm.h          # SVPWM生成器接口
│   ├── foc_motor_detect.h   # 电机参数检测接口
│   └── foc_control.h        # 主控制循环接口
├── src/
│   ├── utils/
│   │   └── fixed_point.c    # 定点数数学实现
│   └── foc/
│       ├── foc_observer.c   # 观测器实现
│       ├── foc_hall.c       # 霍尔处理实现
│       ├── foc_angle_fusion.c # 角度融合实现
│       ├── foc_svpwm.c      # SVPWM实现
│       ├── foc_motor_detect.c # 参数检测实现
│       └── foc_control.c    # 主控制实现
└── README.md
```

---

## 使用示例

### 初始化

```c
#include "foc_control.h"

foc_state_t motor;
foc_config_t config;

// 设置默认配置
foc_init(&motor, &config);

// 配置电机参数（或使用自学习）
config.observer.resistance = float_to_q8_8(0.015f);    // 15mΩ
config.observer.inductance = float_to_q12_20(7e-6f);   // 7µH
config.observer.flux_linkage = float_to_q12_20(0.00245f); // 2.45mWb
```

### 运行电机

```c
// 在20kHz PWM中断中调用
void PWM_ISR(void) {
    // 读取ADC
    q8_8_t i_a = read_current_a();
    q8_8_t i_b = read_current_b();
    q8_8_t i_c = read_current_c();
    q8_8_t v_bus = read_bus_voltage();
    uint8_t hall = read_hall_sensors();
    
    // 运行FOC
    foc_run(&motor, i_a, i_b, i_c, v_bus, hall);
    
    // 更新PWM
    set_pwm_duty(motor.duty_a, motor.duty_b, motor.duty_c);
}
```

### 控制命令

```c
// 电流控制（转矩模式）
foc_set_current(&motor, float_to_q8_8(10.0f));  // 10A

// 速度控制
foc_set_speed(&motor, float_to_q12_4(1000.0f)); // 1000 RPM

// 位置控制
foc_set_position(&motor, angle);  // 目标角度

// 停止
foc_stop_motor(&motor);
```

### 参数自学习

```c
detect_state_machine_t detect_sm;
detect_results_t detect_results;
detect_config_t detect_config;

detect_get_default_config(&detect_config);
detect_start_full(&detect_sm, &detect_config, &detect_results);

while (!detect_is_complete(&detect_sm)) {
    // 等待检测完成
}

// 应用检测结果
motor.config->observer.resistance = detect_results.resistance;
motor.config->observer.inductance = detect_results.inductance;
motor.config->observer.flux_linkage = detect_results.flux_linkage;
```

---

## 性能优化说明

### 针对Cortex-M0的优化

1. **无浮点运算**
   - 所有计算使用Q15/Q31定点格式
   - 避免软浮点库调用

2. **CORDIC算法**
   - atan2仅用移位和加减法
   - 16次迭代精度约0.1°

3. **查表插值**
   - sin/cos使用256项查表
   - 线性插值提高精度

4. **内联函数**
   - 关键路径使用static inline
   - 减少函数调用开销

5. **避免除法**
   - 使用移位和乘法代替
   - 预计算常数倒数

---

## 后续工作建议

1. **HAL层适配**
   - 实现ADC、PWM、GPIO驱动
   - 适配HK32M070外设

2. **通信协议**
   - 实现CAN/UART通信
   - VESC Tool兼容协议

3. **保护功能**
   - 过流保护
   - 过温保护
   - 欠压/过压保护

4. **调试接口**
   - 实时数据上传
   - 波形显示

---

## 许可证

本代码基于GPL v3许可证（遵循VESC许可证）。
MXLemming观测器基于BSD 3条款许可证，原作者David Molony。
