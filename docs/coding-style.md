# 编码规范

> 目标不是"好看"，而是**减少出错的可能**。每条规则背后都有具体的坑。

---

## 1. 命名

| 对象 | 风格 | 示例 |
| --- | --- | --- |
| 函数 | `模块_动作`，小写下划线 | `motor_set_pwm`、`protocol_feed_byte` |
| 局部变量 | 小写下划线 | `wheel_speed`、`frame_len` |
| 全局变量 | `g_` 前缀 | `g_fault_code` |
| 宏 / 常量 | 全大写下划线 | `PROTO_HEAD1`、`MAX_PAYLOAD` |
| 类型 | 小写 + `_t` 后缀 | `proto_frame_t`、`pid_t` |
| 枚举值 | 大写下划线 | `STATE_IDLE` |

**函数名要带模块前缀**：`motor_` / `servo_` / `protocol_`。C 没有命名空间，前缀是唯一的隔离手段，也方便全局搜索。

---

## 2. 单位必须写进名字

```c
uint16_t timeout_ms;      /* ✅ */
uint16_t timeout;         /* ❌ 毫秒还是秒？ */
int16_t speed_mm_s;       /* ✅ */
int16_t speed;            /* ❌ 整数还是浮点？什么单位？ */
float angle_deg;          /* ✅ */
float angle_rad;          /* ✅ */
```

这条规则的收益在调试时最明显：对着串口抓到的 `0x012C`，如果变量名是 `speed_mm_s`，立刻知道是 300 mm/s；如果是 `speed`，还得翻文档。

---

## 3. 禁止魔法数

```c
/* ❌ */
if (len > 64) return;
HAL_Delay(200);

/* ✅ */
#define PROTO_MAX_PAYLOAD  64
#define CMD_TIMEOUT_MS     200
if (len > PROTO_MAX_PAYLOAD) return;
HAL_Delay(CMD_TIMEOUT_MS);
```

例外：`0`、`1`、`-1` 这类显而易见的边界值。

---

## 4. 错误处理

**返回错误码，不要静默失败。**

```c
typedef enum {
    PROTO_OK = 0,
    PROTO_ERR_CRC,
    PROTO_ERR_LEN,
    PROTO_ERR_UNKNOWN_CMD,
} proto_result_t;

proto_result_t protocol_feed_byte(uint8_t byte, proto_frame_t *out);
```

| 场景 | 做法 |
| --- | --- |
| 参数非法 | 返回错误码，**不静默截断** |
| 资源不足 | 返回错误码 + 置故障码 |
| 不可恢复 | 置 FATAL 故障码 + 安全停机 |
| 可恢复 | 记录计数，超过阈值再上报 |

**为什么不能静默截断**：调参时"设了 500 结果生效的是 255"，排查起来要命。宁可返回错误让人看见。

**不要用 `assert` 做运行时检查**：`NDEBUG` 一开就全部消失，而嵌入式发布版必然定义 `NDEBUG`。断言只在开发期用于"绝不可能发生"的内部不变量。

---

## 5. 中断服务程序

**三条铁律**：

1. **不阻塞**——不调用 `HAL_Delay`、不轮询等待标志位；
2. **不动态分配**——不用 `pvPortMalloc`；
3. **不打印**——`printf` 底层阻塞，会把中断拖到毫秒级。

耗时逻辑（协议解析、Flash 写入、复杂计算）交给任务处理，用状态机推进。

**ISR 中允许的操作**：读寄存器、置标志位、更新计数器、写 DMA 缓冲区、调用纯计算函数、调用 `xxxFromISR()` 版本的 FreeRTOS 接口。

### 5.1 FreeRTOS 专项

**中断优先级必须 ≥ `configMAX_SYSCALL_INTERRUPT_PRIORITY` 才能调用 `FromISR` 接口。**

Cortex-M 的优先级是**数值越小优先级越高**，正好和直觉相反。CubeMX 里 `configMAX_SYSCALL_INTERRUPT_PRIORITY` 默认是 5（即 NVIC 优先级 5）。把串口中断配成 0~4，它就能在 FreeRTOS 关中断的临界区里插进来，破坏内核链表——现象是随机 HardFault，断点打哪都不对。

| 用途 | 允许调用 RTOS API | 优先级数值 |
| --- | --- | --- |
| 急停 EXTI | 否 | 0（最高，不碰内核） |
| 串口 DMA / 编码器 | 是 | ≥ 5 |
| SysTick | 是 | 15（最低） |

**中断里只通知，不干活**：

```c
/* 好：只唤醒任务 */
void USART1_IRQHandler(void)
{
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(g_proto_task, &woken);
    portYIELD_FROM_ISR(woken);
}

/* 坏：在中断里解析一整帧，还顺便打个日志 */
```

`portYIELD_FROM_ISR` 不能漏——漏了的话任务要等到下一个时基才切换，延迟可能到 1 ms。

**任务侧**：用 `vTaskDelayUntil` 保证周期，不用 `vTaskDelay`（后者会累积漂移）。

```c
TickType_t last = xTaskGetTickCount();
for (;;) {
    control_step();                 /* 1 ms 一次的真实工作 */
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
}
```

**时间预算**：任何 ISR 的执行时间都应控制在 **10 µs 量级**。用 GPIO 翻转 + 示波器实测，不要估算。

---

## 6. `volatile`

用在**会被中断或硬件修改的变量**上：

```c
static volatile uint8_t g_rx_ready;    /* ISR 写，任务读 */
volatile uint32_t *reg = (volatile uint32_t *)0x40021000;  /* 硬件寄存器 */
```

**`volatile` 不保证原子性**。`volatile int n; n++;` 是读-改-写三步，多上下文共享依然会丢更新。需要原子性用临界区或 `std::atomic`。

---

## 7. 结构体与协议

**不要用结构体直接 `memcpy` 收发**：

1. 填充字节内容未定义，会把栈上残留数据发出去；
2. 两端对齐规则可能不同；
3. 端序可能不同。

**显式序列化**：

```c
static void put_u16_le(uint8_t *buf, uint16_t v)
{
    buf[0] = (uint8_t)(v & 0xFF);
    buf[1] = (uint8_t)(v >> 8);
}

static uint16_t get_u16_le(const uint8_t *buf)
{
    return (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8));
}
```

端序、对齐、平台全都无关了，而且能在 PC 上跑测试。

---

## 8. 动态内存

**STM32 侧禁止使用 `malloc/free`，运行期也不调用 `pvPortMalloc`。**

理由：碎片不可预测、失败点难处理、实时性无法保证。所有缓冲区在编译期静态分配。

FreeRTOS 的对象也一样：

- 任务、队列、信号量**在 `MX_FREERTOS_Init()` 里一次性创建完**，之后不再增删；
- 有条件就开 `configSUPPORT_STATIC_ALLOCATION = 1`，用 `xTaskCreateStatic` / `xQueueCreateStatic`，连堆都不需要；
- 必须用堆时选 `heap_4`（有合并，碎片最少），并核对 `configTOTAL_HEAP_SIZE` 与实际用量。

**唯一的例外**：bootloader 也不需要——它就是最该简单可靠的那部分。

---

## 9. 注释

**写"为什么"，不写"是什么"。**

```c
/* ❌ 读编码器 */
cnt = get_encoder();

/* ✅ 读完立即清零，避免 16 位计数器溢出丢计数 */
cnt = get_encoder();

/* ✅ 用中值积分而非欧拉积分：转弯时末端误差可降到 1/3 */
x += v * dt * cosf(theta + dtheta / 2.0f);
```

**必须写注释的地方**：

- 魔数或特殊取值的来历（如"200ms 取自最大失控距离 10cm / 最大速度 0.5m/s"）
- 与直觉相反的写法（如"这里必须先关中断再复位外设"）
- 已知限制和 TODO
- 协议字段的物理含义

---

## 10. 头文件

```c
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>        /* 只包含真正需要的 */

/* ... */

#endif /* PROTOCOL_H */
```

**`shared/` 下禁止 `#include "stm32f4xx_hal.h"`**——这是能在 PC 上跑测试的前提。

**头文件里不放变量定义**（只放 `extern` 声明），否则多文件包含会重复定义。

---

## 11. ROS 2 / C++

| 对象 | 风格 | 示例 |
| --- | --- | --- |
| 类名 | 大驼峰 | `BaseBridge`、`VisionDetector` |
| 成员变量 | 下划线后缀 | `cmd_sub_`、`tf_buffer_` |
| 成员函数 | 小写下划线 | `on_cmd_vel()` |
| 文件 | 小写下划线 | `base_bridge.cpp` |

**强制项**：

- 用 `rclcpp` 的智能指针（`create_subscription` 返回的对象由节点管理）
- 不在构造函数里做可能失败的重活，用生命周期节点的 `on_configure`
- 定时器回调里不做阻塞操作

---

## 12. Python

- 遵循 PEP 8，用 `black` 格式化、`flake8` 检查
- 类型注解写上：`def parse(cls: str) -> Optional[str]:`
- 不用可变对象作默认参数（`def f(a, b=[])` 是经典错误）
- 异常不要裸 `except:`

---

## 13. 格式化与静态检查

```bash
# C / C++
clang-format -i --style=file firmware/**/*.c firmware/**/*.h

# Python
black ros2_ws/src tools
flake8 ros2_ws/src tools --max-line-length=100

# 编译告警全开（这是最有效的静态检查）
gcc -std=c99 -Wall -Wextra -Wshadow -Wconversion -Werror
```

**`-Werror` 的理由**：告警不处理就会越积越多，最后没人看。一开始就要求零告警，成本最低。

---

## 14. 提交前自检

- [ ] 能编译，零告警
- [ ] 没有注释掉的调试代码
- [ ] 没有 `printf` / `HAL_Delay` 留在 ISR 里
- [ ] 没有硬编码的密码、IP、路径
- [ ] 新增的魔数已经定义成宏
- [ ] 单位已经写进变量名
- [ ] 改了协议的话，`docs/protocol.md` 同步更新
