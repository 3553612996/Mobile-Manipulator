# 软件结构

模块级视角：包怎么划分、接口怎么定、状态机怎么转、并发怎么管、错误怎么处理。

相关文档：[项目框架](overview.md) · [串口协议](protocol.md) · [编码规范](../conventions/coding-style.md)

---

## 1. 仓库分层

```
firmware/
├── bootloader/   独立工程，只做三件事：判断跳转、接收升级、写 Flash
├── app/          主固件
└── shared/       两端共用：协议、CRC、故障码定义

ros2_ws/src/
├── robot_interfaces/ 自定义 msg / srv / action
├── robot_bringup/   launch + 参数 + 地图 + URDF
├── robot_base/      串口桥接、里程计
├── robot_arm/       关节控制、逆解、轨迹
├── robot_vision/    YOLO/RKNN 推理、目标定位
├── robot_speech/    Vosk 识别、意图解析
└── robot_task/      任务状态机（纯逻辑，可在 x86 单测）
```

**`shared/` 存在的理由**：bootloader 和 app 都要做 CRC 校验和帧解析。复制两份的话，改了一边忘了另一边，升级时校验失败的概率会变成必然。宁可多一层目录，也不要有两份实现。

**`robot_task` 独立成包的理由**：状态机是纯逻辑，输入是事件、输出是动作，不依赖硬件。这样它能在 PC 上跑单元测试——这是整个项目唯一能做完整单测的部分，价值很高。

---

## 2. ROS 2 包划分

| 包 | 职责 | 依赖 |
| --- | --- | --- |
| `robot_interfaces` | 自定义消息、服务、动作 | 仅 rosidl |
| `robot_bringup` | 启动编排、参数、地图 | 全部 |
| `robot_base` | `/cmd_vel` ↔ 串口；串口 → `/odom` | 无（只依赖 rclcpp + serial） |
| `robot_arm` | 关节目标下发、逆解、抓取序列 | KDL / TRAC-IK |
| `robot_vision` | 图像 → 检测框 → 3D 坐标 | OpenCV、RKNN runtime |
| `robot_speech` | 音频 → 文本 → 意图 | Vosk |
| `robot_task` | 任务编排、状态机 | 仅接口，不依赖实现 |

**依赖方向必须是单向的**：`robot_task` 依赖其他包的接口，其他包不依赖 `robot_task`。否则会形成环，编译顺序和测试都会出问题。

---

## 3. 接口定义

### 3.1 话题

| 话题 | 类型 | 频率 | QoS |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | 50 Hz | RELIABLE |
| `/odom` | `nav_msgs/Odometry` | 50 Hz | RELIABLE |
| `/scan` | `sensor_msgs/LaserScan` | 10 Hz | BEST_EFFORT |
| `/camera/color/image_raw` | `sensor_msgs/Image` | 30 Hz | BEST_EFFORT |
| `/joint_states` | `sensor_msgs/JointState` | 20 Hz | RELIABLE |
| `/detections` | `vision_msgs/Detection2DArray` | 10 Hz | BEST_EFFORT |
| `/robot_status` | `robot_interfaces/RobotStatus` | 10 Hz | RELIABLE |
| `/system_event` | `robot_interfaces/SystemEvent` | 事件触发 | RELIABLE |

**QoS 的选择理由**：

- 传感器数据用 `BEST_EFFORT`——宁可丢帧也不要堆积延迟，堆积会导致"看着正常但实际延迟 2 秒"。
- 控制与状态用 `RELIABLE`——丢一条速度指令可能让底盘动作不连续。
- 注意 QoS 兼容性是**有方向**的：发布 `BEST_EFFORT` + 订阅 `RELIABLE` 收不到任何数据，且 `ros2 topic list` 照常显示，非常难发现。

### 3.2 服务

| 服务 | 类型 | 用途 |
| --- | --- | --- |
| `/set_enable` | `std_srvs/SetBool` | 使能 / 失能底盘与机械臂 |
| `/clear_fault` | `robot_interfaces/ClearFault` | 清除故障码（条件未消失时拒绝） |
| `/get_version` | `robot_interfaces/GetVersion` | 返回 app / bootloader 版本与 git hash |

### 3.3 动作

| 动作 | goal | feedback | result |
| --- | --- | --- | --- |
| `/grasp`（`robot_interfaces/action/Grasp`） | 目标类别、可选指定位姿 | 阶段、进度百分比 | 成功 / 失败原因码 |

导航直接用 Nav2 的 `NavigateToPose`，不重新定义。

**为什么抓取用动作而不是服务**：抓取是长任务（几秒到十几秒），需要反馈阶段让上层显示进度，需要能中途取消（比如用户喊停）。服务是阻塞的一问一答，做不了这两件事。

---

## 4. STM32 模块划分

| 模块 | 文件 | 职责 | 依赖 |
| --- | --- | --- | --- |
| 启动 | `main.c` | HAL 初始化、外设初始化、`osKernelStart()` | 全部 |
| 任务 | `app_freertos.c`（CubeMX 生成的 `freertos.c`） | 创建五个任务与队列、挂接任务函数 | FreeRTOS |
| 协议 | `shared/protocol.c` | 帧解析、组帧、CRC | 无（纯 C） |
| 电机 | `motor.c` | PWM 输出、编码器读取、PID | bsp |
| 关节 | `servo.c` | 舵机总线指令、插补 | bsp |
| 安全 | `safety.c` | 超时、过流、限位、急停 | motor, servo |
| 故障 | `fault.c` | 故障码置位/清除/上报 | 无 |
| BSP | `bsp_*.c` | UART / TIM / ADC / EXTI 封装 | HAL |

**约定**：`protocol.c`、`fault.c` 和整个 `shared/` 不允许 `#include "stm32f4xx_hal.h"`。这条约束保证了它们能在 PC 上编译和测试。

---

## 5. 状态机

### 5.1 任务状态机（`robot_task`）

```
        ┌────────┐
        │  IDLE  │◄─────────────────────────┐
        └───┬────┘                          │
            │ 收到抓取任务                   │
        ┌───▼─────┐  失败(可重试)            │
        │NAVIGATE │──────────┐              │
        └───┬─────┘          │              │
            │ 到达            │              │
        ┌───▼─────┐          │              │
        │ DETECT  │──────────┤              │
        └───┬─────┘          │              │
            │ 找到目标         │              │
        ┌───▼─────┐          │              │
        │  PLAN   │──────────┤              │
        └───┬─────┘          │              │
            │ 有解             │              │
        ┌───▼─────┐          │              │
        │APPROACH │──────────┤              │
        └───┬─────┘          │              │
            │ 到预抓取点       │              │
        ┌───▼─────┐          │              │
        │ DESCEND │──────────┤              │
        └───┬─────┘          │              │
            │                 │              │
        ┌───▼─────┐          │              │
        │  CLOSE  │──────────┤              │
        └───┬─────┘  没夹住   │              │
            │ 确认夹持         │              │
        ┌───▼─────┐          │              │
        │  LIFT   │──────────┤              │
        └───┬─────┘          │              │
        ┌───▼─────┐          ▼              │
        │  PLACE  │────► 重试(≤N次) ────────┤
        └───┬─────┘          │              │
            │                 ▼              │
            └──────────► FAILED ─────────────┘
```

**每个状态必须有三样东西**：

1. **超时**：如 `NAVIGATE` 超过 60 s 判失败；
2. **失败原因码**：不是布尔值，要能区分 `no_target` / `ik_fail` / `collision` / `timeout` / `not_grasped`；
3. **重试策略**：不同类型次数不同（没检测到可重拍 2 次；IK 无解要换站位，不原地重试）。

### 5.2 通信状态机（`protocol.c`）

见[串口协议 · 第 9 节](protocol.md#9-接收解析状态机)。

### 5.3 升级状态机（bootloader）

```
IDLE ──OTA_BEGIN──► RECEIVING ──OTA_DATA──► RECEIVING
                        │                       │
                        │ OTA_ABORT             │ OTA_END
                        ▼                       ▼
                     ABORTED ◄──校验失败──── VERIFYING
                                                │ 成功
                                                ▼
                                             JUMP
```

**关键约束**：`OTA_END` 时先校验整包 CRC，通过才写"有效标志"。校验失败保留旧镜像并上报，**任何时候不能出现两块 flash 都无效的状态**。

---

## 6. 并发与实时性

### 6.1 STM32 侧

FreeRTOS 上跑五个任务，按优先级从高到低：

| 任务 | 优先级 | 周期 | 栈 | 职责 |
| --- | --- | --- | --- | --- |
| `control_task` | `osPriorityRealtime` | 1 ms | 512 B | 读编码器 → PID → 更新 PWM |
| `motion_task` | `osPriorityHigh` | 10 ms | 512 B | 六轴插补、限位与奇异检查 |
| `proto_task` | `osPriorityAboveNormal` | 事件（任务通知） | 512 B | 帧解析、CRC、序号去重 |
| `report_task` | `osPriorityNormal` | 20 ms | 512 B | 组帧上报里程计/关节/状态 |
| `app_task` | `osPriorityLow` | 事件（队列） | 768 B | 状态机、参数读写、故障处理 |

**为什么是这五层而不是更多**：每个任务的节奏差一个数量级，再细分只会增加同步复杂度。通信超时检查放在 `control_task` 里（1 ms 检查一次，比单独开 10 ms 任务更灵敏）；Flash 写入这类耗时为百毫秒级的操作单独放 `app_task` 的低优先级里慢慢做，不会抢占控制环。

#### 中断与任务的边界

| 中断 | 抢占优先级 | 做什么 |
| --- | --- | --- |
| 急停 EXTI | 最高（0） | **直接关闭 PWM**，置标志位；不调用任何 FreeRTOS API |
| 串口 DMA 空闲 | 6 | `vTaskNotifyGiveFromISR(proto_task)`，仅此 |
| 编码器溢出 | 6 | 计数修正，< 5 µs |
| SysTick | 15（最低） | FreeRTOS 时基 |

**中断优先级铁律**：凡是调用 `xxxFromISR()` 的中断，优先级数值**必须 ≥ `configMAX_SYSCALL_INTERRUPT_PRIORITY`**（CubeMX 默认 5，数值越大优先级越低）。把串口中断配成 0 或 1，它会在内核临界区里插进来，破坏队列的链表操作，表现是随机 HardFault —— 这类问题用调试器几乎定位不到，只能靠事前遵守规则。

急停中断之所以配成最高且不碰 RTOS API：它是唯一必须无条件立即执行的路径。

#### ISR 三条铁律

1. **不阻塞**：不调用 `HAL_Delay`、不等待标志位、不用阻塞版 FreeRTOS API（必须用 `FromISR` 版本）；
2. **不分配内存**：不用 `pvPortMalloc`；
3. **不打印**：`printf` 底层是阻塞的，会把中断拖到毫秒级。

耗时逻辑（协议解析、Flash 写入）一律丢给任务，中断里只做"通知"。

#### 任务实现约束

- 周期任务用 `vTaskDelayUntil`，不用 `vTaskDelay`——后者按"执行完之后再等 N ms"计时，会累积漂移；
- 任务间共享的变量（如目标速度）用临界区或原子操作保护，不要靠"反正写入是原子的"赌；
- 开 `configCHECK_FOR_STACK_OVERFLOW = 2`，在 `vApplicationStackOverflowHook` 里置故障码并停电机；调试期用 `uxTaskGetStackHighWaterMark()` 核对余量；
- 任务、队列、信号量**在初始化阶段一次性创建**，运行期不再增删。

### 6.2 RK3566 侧

| 节点 | 执行器 | 回调组 | 理由 |
| --- | --- | --- | --- |
| `base_bridge` | 单线程 | 默认 | 50Hz，耗时短 |
| `vision_detector` | 多线程 | `ReentrantCallbackGroup` | 推理耗时，不能阻塞其他回调 |
| `task_manager` | 多线程 | `ReentrantCallbackGroup` | 会同步调用其他服务 |

**已知陷阱**：在 `MutuallyExclusiveCallbackGroup` 里同步调用服务，会自锁——客户端占住组等响应，服务端回调永远进不来。表现为节点卡死但 CPU 不高，`ros2 node info` 看不出问题。

### 6.3 时间预算（RK3566）

| 环节 | 目标 |
| --- | --- |
| 推理（单帧） | ≤ 50 ms |
| 检测 → 抓取指令下发 | ≤ 100 ms |
| `/cmd_vel` → 串口发出 | ≤ 5 ms |

---

## 7. 错误处理与降级

### 7.1 分层

| 层 | 检测方式 | 处理 |
| --- | --- | --- |
| 硬件 | 过流、过压、编码器异常 | 该轴停止，置故障码 |
| 通信 | CRC 错误率、超时计数、心跳 | 重发 → 降频 → 停车 |
| 应用 | 节点心跳、资源水位、业务校验 | 告警、重启节点、中止任务 |
| 系统 | dmesg、coredump、磁盘 | 记录、自动重启、告警 |

### 7.2 降级策略

| 故障 | 降级行为 |
| --- | --- |
| 视觉节点失效 | 禁止抓取任务，导航继续可用 |
| 语音节点失效 | 界面/遥控接管，不影响运动 |
| 定位丢失 | 停止导航，原地等待或请求人工 |
| 通信超时 | 底盘停车，机械臂保持位置 |
| 电池低压 | 禁止新任务，引导返回充电位 |

**原则：局部失效不扩散。** 视觉挂了不该影响机器人走路，这是分层带来的直接好处。

### 7.3 故障码

定义见[串口协议 · 第 8 节](protocol.md#8-故障码)。分 WARN / ERROR / FATAL 三级：

- WARN：记录上报，继续运行但受限
- ERROR：停止对应子系统，其他部分继续
- FATAL：全部停止，必须人工清除

---

## 8. 参数管理

| 参数类型 | 存放位置 | 生效方式 |
| --- | --- | --- |
| ROS 2 节点参数 | `robot_bringup/config/*.yaml` | 启动时加载，`ros2 param set` 可热改 |
| 标定结果 | `config/calibration/*.yaml` | 启动加载，含设备序列号与日期 |
| MCU 运行参数 | STM32 Flash（SET_PARAM 写入） | 立即生效，掉电保留 |
| 编译期常量 | `shared/config.h` | 重新编译 |

**规则**：任何会影响行为的数字都不许硬编码在业务代码里。标定文件必须写明设备序列号和标定日期——换了相机还用旧标定，是视觉抓取项目里最隐蔽的错误来源。
