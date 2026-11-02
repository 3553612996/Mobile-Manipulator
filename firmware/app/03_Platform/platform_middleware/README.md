# platform_middleware 设计说明

与硬件、RTOS 无关的通用算法和数据结构。这里的代码不包含任何 MCU 头、HAL 头或 FreeRTOS 头，可以在 PC 上直接用 gcc 编译和单测。

## 落地文件

| 文件 | 职责 |
| --- | --- |
| `platform_ringbuf.h/.c` | 单生产者/单消费者字节环形缓冲区 |
| `platform_crc.h/.c` | CRC-16/CCITT-FALSE |
| `platform_filter.h/.c` | 滑动平均、一阶低通滤波 |
| `platform_pid.h/.c` | 位置式 PID，微分测量值、条件积分抗饱和 |
| `platform_middleware.h` | 聚合头 |

## 依赖方向

```text
01_APP / 02_Service / 03_Platform 其他模块
        |
        v
03_Platform/platform_middleware/*.h
        |
        v
platform_common (platform_type.h / platform_def.h / platform_error.h)
```

中间件不依赖 `04_Impl`、`06_BSP`、`07_Vender`。

## 环形缓冲区

```c
static uint8_t uart_rx_storage[512];
static platform_ringbuf_t uart_rx;

platform_ringbuf_init(&uart_rx, uart_rx_storage, sizeof(uart_rx_storage));

/* ISR：空闲中断里把 DMA 收到的字节写入 */
platform_ringbuf_write(&uart_rx, dma_buf, rx_len);

/* 任务：取出并解析 */
uint8_t chunk[64];
uint32_t n = platform_ringbuf_read(&uart_rx, chunk, sizeof(chunk));
```

设计约束：

- **容量必须是 2 的幂**，内部用 `index & (size - 1)` 代替取模；
- 只保证**单生产者 + 单消费者**无锁安全。多个生产者/消费者必须自己加锁；
- `write`/`read` 返回实际处理字节数，短读/短写不是错误，调用方决定重试还是丢弃；
- 数据拷贝与索引更新之间插入了编译器屏障，防止优化器把索引写提前到数据写之前；
- 索引是 32 位无符号数，靠无符号回绕保持正确，前提是任意时刻未处理数据量小于 2^31。

## CRC-16/CCITT-FALSE

参数与串口协议文档一致：

| 参数 | 值 |
| --- | --- |
| 多项式 | `0x1021` |
| 初值 | `0xFFFF` |
| 输入/输出反射 | 否 |
| 结果异或 | `0x0000` |
| 校验值 | `"123456789"` → `0x29B1` |

```c
uint16_t crc = platform_crc16_ccitt(frame, frame_len);
assert(crc == expected);
```

增量接口用于逐字节解析：

```c
uint16_t crc = PLATFORM_CRC16_CCITT_INIT_VALUE;
crc = platform_crc16_ccitt_update(crc, &byte, 1);
```

**联调前必须先用 `"123456789"` 验证 `0x29B1`**，否则 CRC 参数一定有一边不对。

## 滤波器

### 滑动平均

```c
static int32_t storage[16];
static platform_filter_moving_average_t filter;

platform_filter_moving_average_init(&filter, storage, 16);
int32_t smoothed = platform_filter_moving_average_update(&filter, raw);
```

- 存储由调用方提供，窗口大小不做 2 的幂限制；
- 内部用 `int64_t` 累加，长窗口 + 大样本不会溢出；
- 第一次 `update` 之前 `get()` 返回 0。

### 一阶低通

```c
static platform_filter_lowpass_t filter;

platform_filter_lowpass_init(&filter, 0.1f);
float_t smoothed = platform_filter_lowpass_update(&filter, raw);
```

- `alpha ∈ (0, 1]`，越小越平滑、滞后越大；
- 第一个样本直接作为初值，避免从 0 爬升的启动瞬态；
- 状态不是线程安全的，一个实例只归一个任务所有。

## PID

```c
static platform_pid_t speed_pid;

platform_pid_cfg_t cfg = {
    .kp = 1.0f, .ki = 0.6f, .kd = 0.0f,
    .output_min = -1000.0f, .output_max = 1000.0f
};
platform_pid_init(&speed_pid, &cfg);

float_t duty = platform_pid_update(&speed_pid, target_mm_s,
                                   measured_mm_s, 0.01f);
```

- 位置式 PID：`u = Kp*e + Ki*∫e dt + Kd*d(measurement)/dt`；
- 微分作用在**测量值**上，设定值阶跃不会产生微分冲击；
- 条件积分抗饱和：输出顶到限幅且误差继续往同方向推时，本拍不累加积分；
- 第一个 `update` 没有历史测量值，跳过微分；
- 输入 `dt_s <= 0`、未初始化或空句柄时返回 0，不改变状态；
- 增益必须 ≥ 0，输出上限必须大于下限；
- 状态由调用方持有，不做内存分配，也不是线程安全的。

## PC 单测编译

```bash
gcc -std=c99 -Wall -Wextra -Werror \
    -I firmware/app/03_Platform/platform_common \
    -I firmware/app/04_Impl \
    -I firmware/app/03_Platform/platform_middleware \
    firmware/app/03_Platform/platform_middleware/platform_ringbuf.c \
    firmware/app/03_Platform/platform_middleware/platform_crc.c \
    firmware/app/03_Platform/platform_middleware/platform_filter.c \
    firmware/app/03_Platform/platform_middleware/platform_pid.c \
    tests/unit/platform/test_platform_middleware.c \
    -o build/test_platform_middleware

./build/test_platform_middleware
```

`04_Impl` 只用于提供 `impl_board/board_types.h`；中间件本身不包含任何 Impl 实现。

## 契约冻结

本目录的头文件是上层依赖的稳定契约。新增算法应通过新增头文件完成；修改已有函数签名必须同步升级版本号。
