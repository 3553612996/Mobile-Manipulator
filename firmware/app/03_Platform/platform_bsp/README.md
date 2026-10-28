# platform_bsp 设计说明

具体器件的**设备级契约**。这里定义“上层需要电机/舵机/Flash/温湿度做什么”，不定义“W25Q64 的读命令是 0x03 还是 0x0B”。

## 落地文件

| 文件 | 职责 |
| --- | --- |
| `platform_bsp_motor.h` | 直流电机 + 编码器：使能、带符号占空比、编码器/状态读取 |
| `platform_bsp_servo.h` | 六轴舵机：六关节角度下发、状态回读 |
| `platform_bsp_key.h` | 按键：去抖后的按下状态与按住时长 |
| `platform_bsp_led.h` | 状态灯：按视觉语义开关，极性由驱动决定 |
| `platform_bsp_buzzer.h` | 无源/有源蜂鸣器：开关，节奏由上层决定 |
| `platform_bsp_battery.h` | 电池监测：分压还原后的电压、原始 ADC 值 |
| `platform_bsp_flash.h` | 外部 Flash：读/写/擦除、容量与几何信息、忙状态（本板未使用） |
| `platform_bsp_temphumi.h` | 温湿度传感器：初始化、带超时的读取（本板未使用） |
| `platform_bsp.h` | 聚合头 |

## 三层关系

```text
01_APP / 02_Service
        |
        v
03_Platform/platform_bsp/*.h            <-- 本目录：设备契约 + 注册入口
        |
        v
04_Impl/impl_bsp/*.c                    <-- 实现：设备表 + 分发 + ops 适配 + 注册
        |
        v
06_BSP/*.c                              <-- 具体驱动：寄存器、时序、协议
        |
        v
03_Platform/platform_mcu + platform_os  <-- 适配层向下使用总线/OS 接口
```

对应关系：

| 平台契约 | 04_Impl 适配器 | 06_BSP 具体驱动 |
| --- | --- | --- |
| `platform_motor_ops_t` | `impl_bsp_motor.c` | `bsp_motor_dual_pwm.c`：双 PWM H 桥 + 定时器编码器 |
| `platform_servo_ops_t` | `impl_bsp_servo.c` | `bsp_servo_softpwm.c`：六路 GPIO 软件 PWM |
| `platform_key_ops_t` | `impl_bsp_key.c` | `bsp_key_gpio.c`：GPIO 按键 + 时间去抖 |
| `platform_led_ops_t` | `impl_bsp_led.c` | `bsp_led_gpio.c`：GPIO 状态灯 |
| `platform_buzzer_ops_t` | `impl_bsp_buzzer.c` | `bsp_buzzer_gpio.c`：GPIO 蜂鸣器 |
| `platform_battery_ops_t` | `impl_bsp_battery.c` | `bsp_battery_adc.c`：ADC + 11 倍分压还原 |
| `platform_flash_ops_t` | 未实现 | W25Q64 SPI 驱动（预留） |
| `platform_temphumi_ops_t` | 未实现 | AHT21 I2C 驱动（预留） |

## 注册与生命周期

`04_Impl` 在板级初始化阶段把具体驱动注册进来：

```c
/* 04_Impl/impl_board/board_init.c */
platform_motor_register(PLATFORM_MOTOR_ID_0, &motor_left_ops, &motor_left_ctx);
platform_motor_register(PLATFORM_MOTOR_ID_1, &motor_right_ops, &motor_right_ctx);
platform_servo_register(&servo_bus_ops, &servo_bus_ctx);
platform_flash_register(&w25q64_ops, &w25q64_ctx);
platform_temphumi_register(&aht21_ops, &aht21_ctx);
```

顺序要求：

1. `platform_mcu_*_init()` 完成总线/引脚；
2. `platform_*_register()` 绑定 ops 与驱动实例；
3. `platform_*_init()` 执行器件初始化；
4. 业务任务开始运行。

未注册就调用 `platform_*_init/set/get` 返回 `PLATFORM_ERR_NOT_INITIALIZED`。重复注册同一个 id 返回 `PLATFORM_ERR_ALREADY_INIT`。

## 设备与单位

| 设备 | 关键字段 | 单位 |
| --- | --- | --- |
| 电机 | `duty_permille` | 千分比，`-1000..+1000`，符号表示方向 |
| 电机 | `encoder_count` | 有符号累计计数 |
| 电机 | `speed_mm_s` | mm/s，驱动能测才填 |
| 舵机 | `joint_centi_deg` | 0.01°，与串口协议一致 |
| 舵机 | `move_time_ms` | 毫秒，0 表示尽快 |
| Flash | `address` / `length` | 字节，地址从 0 开始 |
| 温湿度 | `temperature_centi_c` | 0.01 °C |
| 温湿度 | `humidity_centi_rh` | 0.01 %RH |

无法测量的字段用 `PLATFORM_MOTOR_VALUE_INVALID` / `PLATFORM_MOTOR_TEMPERATURE_INVALID` / `PLATFORM_SERVO_VALUE_INVALID` 标记，不要填 0 冒充有效值。

## 为什么电机只暴露 duty，不暴露 speed

速度环 PID 属于控制层（`control_task`），不属于 BSP。BSP 只负责：

- 把带符号占空比变成 PWM + 方向 GPIO；
- 把编码器计数读回来；
- 报告使能/故障状态。

这样 PID 可以在 PC 上用录制的编码器数据单测，换电机/驱动板时也不用改控制算法。如果以后某个驱动板自带闭环，可以新增一个 `platform_motor_set_speed()` 的可选 op，而不是修改现有契约。

## 并发与中断

- 电机 `set_duty` / `get_encoder` 会被 1 ms 控制任务高频调用，Impl 必须保证无阻塞、无动态内存。
- `platform_motor_stop_all()` 是任务级安全兜底；**急停 ISR 仍然要直接关 PWM**，不能依赖这个函数，因为 ISR 不能等调度。
- Flash/温湿度操作可能阻塞数十到数百毫秒，禁止在控制任务或 ISR 中调用。
- 同一器件的 ops 不保证可重入；多任务访问由 Impl 用总线锁或 OS 互斥锁保护。

## 错误码语义

| 场景 | 返回 |
| --- | --- |
| 未注册 / 未初始化 | `PLATFORM_ERR_NOT_INITIALIZED` |
| 重复注册 | `PLATFORM_ERR_ALREADY_INIT` |
| 占空比/角度/地址越界 | `PLATFORM_ERR_PARAM` |
| Flash 未擦除就写、跨页写 | `PLATFORM_ERR_PARAM` |
| 传感器转换超时 | `PLATFORM_ERR_TIMEOUT` |
| 器件无应答/校验失败 | `PLATFORM_ERR_FAIL` |
| 器件忙 | `PLATFORM_ERR_BUSY` |

## 不在本层的内容

- 芯片寄存器、命令字、时序、片选延时 → `06_BSP`
- 引脚/总线号映射 → `04_Impl/impl_board`
- PID、轨迹插补、限位与奇异检查 → 控制/运动层
- 故障码分级与上报策略 → 安全/协议层

## 契约冻结

本目录的头文件是上层依赖的稳定契约。新增设备应新增头文件；修改已有 ops 结构必须同步升级版本号，并检查所有注册方。
