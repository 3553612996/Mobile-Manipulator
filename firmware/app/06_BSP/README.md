# 06_BSP 设计说明

具体器件的**驱动实现**。只依赖平台层（`platform_mcu_*` / `platform_os`），
不包含 HAL、不包含 `stm32f1xx_hal.h`、不知道引脚号。

## 与上下层的关系

```text
03_Platform/platform_bsp/*.h        设备契约（上层只看到这里）
        |
04_Impl/impl_bsp/*.c                设备表 + 注册 + 分发 + ops 适配
        |
06_BSP/*.c  <-- 本目录               具体驱动：时序、状态机、换算
        |
03_Platform/platform_mcu/*          GPIO / PWM / ADC / 编码器 / 定时器
```

每个驱动是一个**句柄结构体 + 一组函数**，句柄里有 `cfg`（接线）和私有状态。
`impl_bsp` 把句柄指针作为 `ops` 的 `context` 传进来，因此同一份驱动代码可以
同时服务四个电机，差别只在各自的 `cfg`。

## 驱动清单

| 文件 | 器件 | 用到的平台接口 | 关键点 |
| --- | --- | --- | --- |
| `bsp_motor_dual_pwm.c` | 双 PWM H 桥 + 正交编码器 | `platform_mcu_pwm`、`platform_mcu_encoder` | 一侧满占空、一侧斩波；编码器 16 位，软件累加成 32 位 |
| `bsp_servo_softpwm.c` | 六路模拟舵机 | `platform_mcu_gpio`、`platform_mcu_timer` | TIM7 单次定时器链，20 ms 帧 |
| `bsp_key_gpio.c` | 按键 | `platform_mcu_gpio` | 按时间计的去抖，`poll()` 由调用方给 `elapsed_ms` |
| `bsp_led_gpio.c` | 状态灯 | `platform_mcu_gpio` | `active_low` 在驱动内转换 |
| `bsp_buzzer_gpio.c` | 有源蜂鸣器 | `platform_mcu_gpio` | 只做开关，节奏留给应用 |
| `bsp_battery_adc.c` | 电池分压 | `platform_mcu_adc` | 分压比 11，输出 mV |

## 电机：为什么是双 PWM

厂家板子没有方向引脚，H 桥两个输入各接一路 PWM，靠两路占空比之差决定方向：

```text
duty > 0 :  forward = 1000 - duty,  reverse = 1000
duty < 0 :  forward = 1000,         reverse = 1000 + duty
duty = 0 :  两路都 1000（同时导通 = 刹车，与原厂 MOTOR_x_SetSpeed 一致）
```

`enabled == false` 时两路都拉低（滑行）并停止输出，避免残留比较值。
占空比单位是千分比，范围 `-1000..+1000`，符号表示方向。

编码器只用 `read_delta()`（读后清零），驱动内部累加为 `int32_t`，
所以 16 位计数器回绕不会丢位置。`invert_direction` 同时翻转驱动方向和
编码器计数，保证“正占空比 = 计数增加”。

## 舵机：软件 PWM 的时序

硬件没有舵机控制器，六路都是普通 GPIO，波形在定时器中断里生成：

```text
20 ms 帧 = 6 × 2.5 ms 槽 + 5 ms 间隙
每个槽：先高 pulse_us（500..2500），再低 (2500 - pulse_us)
```

中断里每一步都重新编程下一次间隔，所以用的是**单次定时器**
（`platform_mcu_timer`），不是固定周期的定时器。每帧 12 次中断
（6 次上升 + 6 次下降），20 ms 内完成，CPU 占用可以忽略。

角度单位是 0.01°，0 对应中位脉冲 1500 µs，默认 ±90° 对应 1500±1000 µs。
`move_time_ms` 按帧做整数定点插值（Q8），没有浮点，M3 上不需要 FPU。

中断只读写 `int32_t`/`uint16_t` 数组，Cortex-M3 上对齐访问是原子的；
任务侧写入新目标可能晚一帧生效，但不会读到半个数据。

## 按键：为什么由调用方给时间

`poll(elapsed_ms)` 不读时钟，驱动不依赖 OS 时基，PC 上可以直接单测；
采样周期由状态任务决定（建议 5~20 ms）。判定规则：原始电平连续保持
`debounce_ms` 才被接受，抖动会重置计时。

## 加一个新驱动

1. 在 `03_Platform/platform_bsp/` 已有契约能覆盖时直接实现，否则先加契约头；
2. 本目录新增 `bsp_xxx.c/.h`，只包含 `platform_mcu_*` 头；
3. `04_Impl/impl_bsp/impl_bsp_xxx.c` 提供 ops 适配与注册函数；
4. `04_Impl/impl_board/board_init.c` 填 `cfg` 并注册；
5. `MDK-ARM/app.uvprojx` 的 `BSP/Drivers`、`Impl/BSP` 分组加入新 `.c`。

## 不在本层的内容

- 寄存器、HAL 调用、引脚号 → `04_Impl/impl_mcu`、`04_Impl/impl_board`
- PID、轨迹插补、限位检查 → 控制/运动层
- 故障分级、上报策略 → 安全/协议层
