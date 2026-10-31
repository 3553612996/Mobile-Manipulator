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
| `bsp_servo_bus.c` | 六轴总线舵机（YH ASCII，UART5 半双工） | `platform_mcu_uart` | 100 ms 帧间隔、PRAD/PRTV 轮询、ID/偏置/零点配置、离线计数 |
| `bsp_servo_softpwm.c` | 六路模拟舵机（备用驱动，当前未注册） | `platform_mcu_gpio`、`platform_mcu_timer` | TIM7 单次定时器链，20 ms 帧 |
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

## 舵机：单线总线与轮询

机械臂用的是带控制板的串行总线舵机，六颗串在一条线上，接线是
**UART5 半双工单线**（PC12，经 R22 到舵机信号线），115200 8N1。协议是
供应商的 YH ASCII v2.0：

```text
移动       #IDPpwmTtime!        pwm 0500..2500，time 0000..9999 ms
多关节     {#000P1500T0800!#001P...!}
读角度     #IDPRAD!      ->     #IDPpppp!
读温度电压 #IDPRTV!      ->     #IDT25V07!   （回包格式随固件版本略有不同）
改 ID      #IDPIDnew!    ->     #OK!
偏置       #IDPSCK±xxx!         范围 ±100
力矩       #IDPULK!（松）/ #IDPULR!（上力）
停止       #255PDST!
```

供应商明确要求**两条命令至少间隔 100 ms**，所以驱动不是"调用即发送"，
而是一个由 `appServo` 任务每 5 ms 推进一格的状态机：一个 slot 最多一帧，
运动帧优先，但两次运动之间至少插一次查询，回读不会被连续目标饿死。
角度按 6 颗一轮 `PRAD` 扫描（约 0.8~1 s 一圈），之后用一颗 `PRTV` 轮流补
温度/电压（整圈约 5 s）。连续 3 次无应答判该关节离线。

`bsp_servo_softpwm.c` 是上一版的模拟舵机驱动（六路 GPIO 软件 PWM，20 ms
帧在 TIM7 单次定时器中断里生成），代码保留但**未注册**：当前板子接的是
总线舵机。换回模拟舵机时在 `board_init.c` 改回注册它即可。

保护逻辑不在本层：堵转、过温、过压的判定在 `service_arm`，驱动只负责
如实回读角度、温度、电压和离线/回包错误位。

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
