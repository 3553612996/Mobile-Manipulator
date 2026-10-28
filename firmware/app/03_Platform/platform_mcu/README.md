# platform_mcu 设计说明

MCU 外设能力的统一出口。上层只认识 `platform_uart_*` / `platform_i2c_*` 这类函数，不认识 `USART_HandleTypeDef`、`HAL_UART_Transmit()`，也不关心底层是 HAL、标准库还是寄存器。

## 落地文件

| 文件 | 职责 |
| --- | --- |
| `platform_mcu_gpio.h` | 引脚初始化、读写、翻转、中断绑定 |
| `platform_mcu_uart.h` | 阻塞/异步收发、空闲中断、半双工总线 |
| `platform_mcu_spi.h` | 全双工传输、DMA 异步发送、软件/硬件片选、总线锁 |
| `platform_mcu_i2c.h` | 原始读写、寄存器读写、设备探测、总线锁 |
| `platform_mcu_pwm.h` | PWM 频率与占空比（千分比） |
| `platform_mcu_adc.h` | 原始值/毫伏值读取 |
| `platform_mcu_wdg.h` | 看门狗启动、喂狗、复位原因 |
| `platform_mcu.h` | 聚合头，上层只包含这一个 |

## 依赖方向

```text
01_APP / 02_Service
        |
        v
03_Platform/platform_mcu/*.h        <-- 本目录，冻结契约
        |
        v
04_Impl/impl_mcu/*.c                <-- 适配 HAL / 标准库 / 寄存器
        |
        v
05_BSP / 07_Vender (STM32 HAL, CMSIS)
```

**规则：**

- 本目录只放头文件，不放任何 `stm32f4xx_hal.h`。
- 上层不得直接包含 `stm32f4xx_hal.h`、`cmsis_os.h` 或具体芯片头。
- 参数、句柄、状态码的转换全部在 `04_Impl` 完成。

## 逻辑资源编号

平台层使用逻辑编号，不暴露物理外设名：

| 类型 | 编号 | 说明 |
| --- | --- | --- |
| UART | `PLATFORM_UART_BUS_0..3` | 0 = 与 RK3566 通信，1 = 六轴舵机总线（示例） |
| SPI | `PLATFORM_SPI_BUS_0..3` | 0 = 外部 Flash（示例） |
| I2C | `PLATFORM_I2C_BUS_0..3` | 0 = IMU（示例） |
| PWM | `PLATFORM_PWM_CHANNEL_0..7` | 每个通道对应一路电机/舵机 |
| ADC | `PLATFORM_ADC_CHANNEL_0..7` | 电源、电流采样 |
| GPIO | `platform_gpio_t { port, pin }` | `port` 是逻辑端口号，不是 `GPIOA` |

映射表放在 `04_Impl/impl_board/board_map.h`，例如：

```c
#define BOARD_UART_HOST_BUS     PLATFORM_UART_BUS_0
#define BOARD_UART_SERVO_BUS    PLATFORM_UART_BUS_1
#define BOARD_SPI_FLASH_BUS     PLATFORM_SPI_BUS_0
#define BOARD_I2C_IMU_BUS       PLATFORM_I2C_BUS_0
```

换板子时只改 `board_map.h` 和 `04_Impl` 的实现，本目录不动。

## 参数适配示例

这些正是 `04_Impl` 存在的理由：

| 平台接口 | 底层差异 | Impl 中的适配 |
| --- | --- | --- |
| `platform_i2c_write(bus, dev_addr, ...)` | HAL 需要 8 位地址 | `dev_addr << 1` |
| `platform_i2c_mem_read(..., mem_addr_size_bytes, ...)` | HAL 只支持 1/2 字节内存地址 | 校验后映射到 `I2C_MEMADD_SIZE_8BIT/16BIT` |
| `platform_uart_read(..., timeout_ms)` | HAL 使用 tick | `timeout_ms` 转 tick，`HAL_MAX_DELAY` 映射为 `PLATFORM_OS_WAIT_FOREVER` |
| `platform_pwm_set_duty(ch, permille)` | HAL 使用比较寄存器 CCR | `duty * (ARR + 1) / 1000`，并做边界钳位 |
| `platform_adc_read_mv(ch, &mv)` | 原始值 0..4095 | `raw * reference_mv / 4095` |
| `platform_gpio_init({port, pin})` | 引脚分散在 GPIOA.. | `board_map` 查表得到 `GPIO_TypeDef*` 与 `GPIO_PIN_x` |
| `platform_uart_write` 半双工 | HAL 需要 `HAL_HalfDuplex_Init` | 根据 `duplex` 选择初始化分支，发送前切换方向 |

## 并发与中断规则

- `platform_gpio_read/write/toggle` 可以在 ISR 中调用；初始化/中断绑定不行。
- UART/SPI 的异步完成回调运行在 ISR 上下文，只允许置标志、发通知、写环形缓冲区。
- `platform_*_lock/unlock` 可能使用 OS 互斥锁，**禁止在 ISR 中调用**。
- 同一个总线被多个任务访问时，由 Impl 或调用方用总线锁串行化；平台接口不假定线程安全。

## 错误码语义

| 错误码 | 含义 |
| --- | --- |
| `PLATFORM_ERR_PARAM` | 参数非法（总线号、长度、地址、占空比越界） |
| `PLATFORM_ERR_NOT_INITIALIZED` | 未调用对应的 `init` |
| `PLATFORM_ERR_ALREADY_INIT` | 重复初始化 |
| `PLATFORM_ERR_TIMEOUT` | 阻塞传输超时 |
| `PLATFORM_ERR_BUSY` | 总线/外设正被占用，或零超时下无法开始传输 |
| `PLATFORM_ERR_NOT_SUPPORTED` | 当前芯片/配置不支持该模式 |
| `PLATFORM_ERR_FAIL` | 底层返回错误（HAL_ERROR、总线错误等） |

## 不在本层的内容

以下内容属于 `04_Impl` / `05_BSP`，不要放进平台接口：

- 时钟树、引脚复用、DMA 通道分配、NVIC 优先级；
- 具体芯片的寄存器时序、片选延时、上电时序；
- 具体器件的初始化命令序列（例如 W25Q64 的读 ID 指令）。

## Keil Include Paths

平台层第一次接入 Keil 工程时，需要把下面这些目录加入 Include Paths：

```text
03_Platform/platform_common
03_Platform/platform_mcu
03_Platform/platform_os
03_Platform/platform_middleware
03_Platform/platform_bsp
04_Impl
```

`04_Impl` 这一项是给 `platform_common/platform_type.h` 里的
`#include "impl_board/board_types.h"` 用的，不是让上层直接包含 Impl。

## 契约冻结

本目录的头文件是上层依赖的稳定契约。新增能力应通过**新增头文件**完成；修改已有函数签名必须同步升级版本号，并在提交说明中列出受影响的调用方。
