# platform_os 设计说明

OS 抽象层（OSAL）。业务代码只调用 `platform_os_task_create()`、`platform_os_queue_send()`、`platform_os_mutex_lock()`，不直接包含 `FreeRTOS.h` 或 `cmsis_os.h`。

## 落地文件

| 文件 | 职责 |
| --- | --- |
| `platform_os_common.h` | 超时常量、优先级枚举 |
| `platform_os_task.h` | 任务创建/删除、延时、任务通知 |
| `platform_os_time.h` | tick、毫秒/tick 转换、ISR 判断、内核启动 |
| `platform_os_mutex.h` | 普通/递归互斥锁 |
| `platform_os_semaphore.h` | 二值/计数信号量 |
| `platform_os_queue.h` | 消息队列 |
| `platform_os_event.h` | 事件组（ALL/ANY 等待） |
| `platform_os_timer.h` | 软件定时器 |
| `platform_os_critical.h` | 临界区进入/退出 |
| `platform_os.h` | 聚合头，上层只包含这一个 |

## 依赖方向

```text
01_APP / 02_Service / 04_Impl
        |
        v
03_Platform/platform_os/*.h         <-- 本目录，冻结契约
        |
        v
04_Impl/impl_os/*.c                 <-- 适配 FreeRTOS / CMSIS-RTOS v2
        |
        v
07_Vender/FreeRTOS
```

## 不透明句柄

```c
typedef struct platform_os_task *platform_os_task_t;
```

句柄的布局、静态池、生命周期全部由 `04_Impl` 决定。上层不能解引用句柄，也不能假设它等于 `TaskHandle_t`。这样以后把 FreeRTOS 换成 ThreadX 或裸机调度器时，上层代码不用改。

## 优先级映射

平台只暴露 6 个相对优先级，避免上层写死 `configMAX_PRIORITIES` 相关的数字：

| 平台优先级 | FreeRTOS/CMSIS 建议映射 |
| --- | --- |
| `PLATFORM_OS_PRIORITY_IDLE` | `osPriorityIdle` |
| `PLATFORM_OS_PRIORITY_LOW` | `osPriorityLow` |
| `PLATFORM_OS_PRIORITY_NORMAL` | `osPriorityNormal` |
| `PLATFORM_OS_PRIORITY_ABOVE_NORMAL` | `osPriorityAboveNormal` |
| `PLATFORM_OS_PRIORITY_HIGH` | `osPriorityHigh` |
| `PLATFORM_OS_PRIORITY_REALTIME` | `osPriorityRealtime` |

具体数值由 `04_Impl` 决定。控制任务的 1 ms 周期要求决定了 `REALTIME` 必须映射到足够高的抢占优先级，这个决策不应散落在业务代码里。

## 时间单位

- API 参数一律用**毫秒**（`_ms` 后缀）。
- `platform_os_tick_get()` 返回内核 tick，不是毫秒。
- `platform_os_task_delay_until()` 的 `last_wake_tick` 用 `platform_os_tick_get()` 初始化，函数内部按 tick 累加，避免 `delay_ms()` 的累积漂移。
- `PLATFORM_OS_WAIT_FOREVER` 表示永久等待，`PLATFORM_OS_NO_WAIT` 表示不等待。

## 超时与错误语义

| 场景 | 返回 |
| --- | --- |
| 等待锁/信号量/队列超时 | `PLATFORM_ERR_TIMEOUT` |
| 队列满且零等待发送 | `PLATFORM_ERR_BUSY` |
| 队列空且零等待接收 | `PLATFORM_ERR_BUSY` |
| 静态池耗尽 | `PLATFORM_ERR_NO_RESOURCE` |
| 句柄为 NULL / 参数非法 | `PLATFORM_ERR_PARAM` |
| 在 ISR 中调用非 ISR 安全接口 | `PLATFORM_ERR_FAIL`（Impl 可选择断言） |

`*_from_isr` 系列只允许在中断里调用。Impl 内部负责 `portYIELD_FROM_ISR()`，调用方不需要、也不应该自己判断是否需要让出。

## ISR 规则

| 接口 | ISR 可用 |
| --- | --- |
| `platform_os_task_notify_from_isr` | 是 |
| `platform_os_sem_post_from_isr` | 是 |
| `platform_os_queue_send_from_isr` / `receive_from_isr` | 是 |
| `platform_os_event_set_from_isr` | 是 |
| 其余带阻塞/超时的接口 | 否 |
| `platform_os_critical_enter/exit` | 是（保持极短） |

## 静态分配

项目禁止运行期 `malloc/free`。`04_Impl` 应在编译期分配：

- 任务控制块与栈：`xTaskCreateStatic` + 静态数组；
- 队列：`xQueueCreateStatic` + 静态存储；
- 信号量/互斥锁：`xSemaphoreCreateBinaryStatic` 等。

平台句柄返回 NULL 表示静态池耗尽，上层应按 `PLATFORM_ERR_NO_RESOURCE` 处理，不允许在运行期重试分配。

## 契约冻结

本目录的头文件是上层依赖的稳定契约。新增同步原语应通过新增头文件完成；修改已有函数签名必须同步升级版本号。
