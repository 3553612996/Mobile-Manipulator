# 04_Impl 设计说明

平台层与具体芯片之间的适配层。分为 **wrapper** 和 **实现层** 两级，职责严格分离。

## 调用链

```text
03_Platform/platform_mcu/platform_mcu_gpio.c
        |  platform_gpio_init()
        v
04_Impl/impl_wrapper/impl_gpio_wrapper.c
        |  impl_gpio_init_wrap()      日志、插桩、转发
        v
04_Impl/impl_mcu/impl_mcu_gpio.c
        |  impl_gpio_init()           参数/单位转换 + HAL 调用
        v
STM32 HAL / LL / 寄存器
```

命名规则固定为：

```c
platform_gpio_init()        /* 平台契约，永不修改 */
    -> impl_gpio_init_wrap()    /* wrapper，只做日志/插桩 + 转发 */
        -> impl_gpio_init()     /* 实现层，只做适配 */
```

## 目录

| 目录 | 职责 | 允许包含 |
| --- | --- | --- |
| `impl_wrapper/` | 调用日志、耗时统计、错误码观测；不校验参数、不转换单位 | 只包含 `platform_*` 与 `impl_mcu/*` 头 |
| `impl_mcu/` | 逻辑资源映射、参数/单位转换、HAL/LL/寄存器调用 | 可以包含 `stm32f1xx_hal.h` |
| `impl_bsp/` | 设备表、`platform_bsp_*.h` 的注册与分发、ops 适配 | `platform_bsp/*` 与 `05_BSP/*` 头，不包含 HAL |
| `impl_board/` | 编译器基础类型、板级实例与注册（组合根） | 不包含上层业务头 |

`impl_board/board_init.c` 是唯一的装配点：它填好每个 `bsp_*_t` 实例的配置
（引脚、通道、极性），注册到平台契约，再初始化必须在任务启动前工作的设备。
上层只看到 `platform_bsp_*.h`，看不到具体驱动，也看不到 HAL。

## 规则

- **wrapper 是 impl_mcu 的唯一调用方。** 平台层和其他层不得直接调用 `impl_*`。
- **wrapper 不写业务适配。** 参数检查、7 位地址转 8 位、permille 转 CCR、
  raw 转 mV、超时转 tick 全部放在实现层。wrapper 只允许“记录 → 调用 → 记录”。
- **实现层只关注适配。** 不打印日志、不做策略选择、不访问上层全局状态。
- **平台层只转发。** `platform_*.c` 每个函数体只有一行 `return impl_*_wrap(...)`。
- 新增平台接口时，三处同时增加：平台 `.c`、wrapper `.c/.h`、实现层 `.c/.h`；
  三层函数签名保持一致。
- `platform_mcu` 下的 GPIO/PWM/ADC/编码器/定时器已由 `impl_mcu` 实现；
  I2C/SPI/UART/WDG 仍是骨架，返回 `PLATFORM_ERR_NOT_SUPPORTED`。替换函数体时
  不要修改签名，wrapper 依赖它们。

## 日志插桩

`impl_wrapper/impl_trace.h` 默认把钩子展开为空：

```c
IMPL_TRACE_ENTER(name)
IMPL_TRACE_EXIT(name, err)
IMPL_TRACE_ISR_ENTER(name)
IMPL_TRACE_ISR_EXIT(name, err)
```

需要打开日志时：

1. 定义 `IMPL_TRACE_ENABLE`；
2. 在本目录新增 `impl_trace_backend.h`，实现上述四个宏；
3. `*_ISR_*` 版本必须可在中断上下文调用（只置标志或写环形缓冲区）。

不允许在 wrapper 里直接调用 `printf`。

## 编译接入

Keil Include Paths 已需要 `04_Impl`，因此：

- 平台层包含 `impl_wrapper/impl_mcu_wrapper.h`；
- wrapper 包含 `impl_mcu/impl_mcu_*.h`。

新增的 `.c` 文件需要手动加入 Keil 工程的对应分组：

```text
03_Platform/platform_mcu/platform_mcu_*.c
04_Impl/impl_wrapper/impl_*_wrapper.c
04_Impl/impl_mcu/impl_mcu_*.c
04_Impl/impl_bsp/impl_bsp_*.c
04_Impl/impl_board/board_init.c
05_BSP/bsp_*.c
```

Include Paths 需要 `03_Platform/platform_common`、`03_Platform/platform_mcu`、
`03_Platform/platform_bsp`、`03_Platform/platform_os`、`03_Platform/platform_middleware`、
`04_Impl`、`05_BSP`。`main.c` 在 `USER CODE BEGIN 2` 中调用 `board_init()`，
在 `USER CODE BEGIN Callback 1` 中把 TIM7 更新中断转发给
`impl_timer_period_elapsed()`（全局 `HAL_TIM_PeriodElapsedCallback` 由 CubeMX
生成，TIM6 时基也在里面）。

PC 侧语法/链接检查：

```bash
gcc -std=c99 -Wall -Wextra -Wshadow -Wconversion -Werror -fsyntax-only \
    -I03_Platform/platform_common -I03_Platform/platform_mcu \
    -I03_Platform/platform_os -I03_Platform/platform_middleware \
    -I03_Platform/platform_bsp -I04_Impl \
    03_Platform/platform_mcu/*.c 04_Impl/impl_wrapper/*.c 04_Impl/impl_mcu/*.c
```
