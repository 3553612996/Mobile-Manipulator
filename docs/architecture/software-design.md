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
| `robot_base` | `/cmd_vel` ↔ 串口；串口 → `/odom` | rclcpp、geometry_msgs、nav_msgs、std_srvs、robot_interfaces |
| `robot_arm` | 关节目标下发、逆解、抓取序列 | rclcpp、rclcpp_action、geometry_msgs、sensor_msgs、trajectory_msgs、robot_interfaces；KDL / TRAC-IK（M8） |
| `robot_vision` | 图像 → 检测框 → 3D 坐标 | rclcpp、sensor_msgs、geometry_msgs、vision_msgs、cv_bridge、image_transport、tf2_ros、robot_interfaces；OpenCV / RKNN（M7） |
| `robot_speech` | 音频 → 文本 → 意图 | rclcpp、std_msgs、robot_interfaces；Vosk（M9） |
| `robot_task` | 任务编排、状态机 | rclcpp、rclcpp_action、geometry_msgs、nav2_msgs、std_srvs、robot_interfaces |

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

固件按 `01_APP → 02_Service → 03_Platform → 04_Impl → 06_BSP` 分层，依赖只能向下，不能反向引用。

| 层 | 目录 | 职责 | 约束 |
| --- | --- | --- | --- |
| 应用 | `01_APP/app_robot` | 任务表、队列、状态机、故障降级、命令处理、上报调度 | 只编排，不含运动学、协议字节、寄存器操作 |
| 服务 | `02_Service/service_*` | 麦轮运动学与四轮 PID、关节限位、协议实现、按键策略 | 不读时钟、不建任务、不直接碰寄存器 |
| 平台 | `03_Platform/platform_*` | 提供给上层的冻结接口：MCU 外设、OS、BSP 设备、中间件算法 | 接口不随下层实现变化 |
| 实现 | `04_Impl/impl_*` | 把 HAL / FreeRTOS / 具体芯片适配成平台接口；`impl_wrapper` 做日志插桩 | 只做适配，不含业务 |
| 驱动 | `06_BSP/bsp_*.c` | 电机与编码器、舵机、按键、LED、蜂鸣器、电池 ADC | 允许 `#include` HAL |
| 中间件 | `05_Middlewares/protocol` | 帧编解码、CRC、故障码与级别表 | 纯 C，可在 PC 编译 |

**`01_APP` 内部按职责拆文件**，`app_robot.c` 只剩组合根：

| 文件 | 职责 |
| --- | --- |
| `app_robot.c` | `app_robot_init()` / `app_robot_start()` 转发给 `app_user_init` |
| `app_user_init.c` | 常量任务表 + 常量队列表，初始化时循环创建；注册 UART 事件 |
| `app_tasks.c` | `appComm` / `appCtrl` 两个任务的主循环 |
| `app_handlers.c` | 协议命令处理表、按键手势、PID 参数入队 |
| `app_state.c` | 主状态机（INIT / IDLE / MANUAL / AUTO / ESTOP / FAULT） |
| `app_fault.c` | 故障仲裁与降级策略 |
| `app_report.c` | 上报节奏（ODOM / WHEEL_SPEED / ARM / STATUS / HEARTBEAT） |
| `app_battery.c` | 电池电压读取与低压判定 |
| `app_robot_config.h` | 全部需要实测/标定的数字 |

**约定**：`protocol`、`app_state`、`app_fault` 不允许 `#include "stm32f1xx_hal.h"` 或 `FreeRTOS.h`。这条约束保证了它们能在 PC 上编译和测试。

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

### 5.4 应用状态机（`app_state.c`）

```text
INIT ──ready──► IDLE ◄──SET_ENABLE(0)──► MANUAL
                 ▲          ──SET_ENABLE(1)──►

任意状态 ──ERROR──► FAULT ──CLEAR_FAULT──► IDLE
任意状态 ──FATAL──► ESTOP ──CLEAR_FAULT──► IDLE
```

| 迁移 | 触发 | 动作 |
| --- | --- | --- |
| INIT → IDLE | 初始化完成（`app_state_ready()`） | 允许接受使能 |
| IDLE ↔ MANUAL | `SET_ENABLE` | 使能/失能底盘与机械臂 |
| → FAULT | ERROR 级故障 | 停对应子系统，拒绝运动指令，仍接受 `CLEAR_FAULT` |
| → ESTOP | FATAL 级故障、`ESTOP`、按键长按 | 四路桥臂 coast，拒绝使能 |
| FAULT / ESTOP → IDLE | `CLEAR_FAULT` 且条件消失 | **不自动使能**，Host 必须重新 `SET_ENABLE` |

AUTO 在协议中预留，当前没有内部自主运动任务，固件不会主动进入。状态由 `app_state_protocol()` 转成协议状态字上报。故障级别的判定与降级动作集中在 `app_fault.c`，状态机本身不做 I/O：它不认识电机、串口和故障码。

---

## 6. 并发与实时性

### 6.1 STM32 侧

FreeRTOS 上跑两个任务，定义在 `app_user_init.c` 的常量表里：

| 任务 | 优先级 | 周期 | 栈 | 职责 |
| --- | --- | --- | --- | --- |
| `appComm` | HIGH | 10 ms（UART 事件唤醒） | 见 `APP_COMM_TASK_STACK_BYTES` | 协议处理与 ACK、故障轮询、全部上报帧 |
| `appCtrl` | ABOVE_NORMAL | 10 ms | 见 `APP_CONTROL_TASK_STACK_BYTES` | 按键、PID 参数出队、四轮速度环、里程计、蜂鸣器 |

**为什么是两个而不是五个**：上报本身不耗时（四帧合计不到 40 字节 / 周期），独立成任务只会增加同步点。通信超时在 `appComm` 里以 10 ms 粒度轮询，控制环仍由 `appCtrl` 独占——这样 PID 的浮点状态只有控制任务写，不存在并发修改。机械臂插补当前由 `service_arm` 在 `appCtrl` 中推进，等真实舵机总线需要更长耗时时再拆任务。

**只有一个发送者**：串口 TX 只在 `appComm` 里调用。控制任务要上报（按键长按急停）时置 `s_pending_estop` 标志，由通信任务变成事件帧，避免两个任务同时进入阻塞发送。PID 调参走 `appPid` 队列从通信任务传给控制任务，队列按值拷贝，参数在控制任务里才写入 PID。

速度环取 10 ms 而不是 1 ms：当前 13 线编码器经 4 倍频、1:90 减速后约 24828 counts/mm，1 ms 内一个计数相当于约 40 mm/s，PID 反馈会被量化噪声主导。10 ms 下仍有 100 Hz 的控制频率，且四个浮点环与里程计都在控制任务预算内。若后续换更高线数编码器或加装速度滤波，再评估提高控制频率。

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
| MCU 运行参数 | RAM（`SET_PARAM` 写入） | 立即生效，掉电恢复编译期初值（Flash 保留待标定完成后设计） |
| 编译期常量 | `shared/config.h` | 重新编译 |

**规则**：任何会影响行为的数字都不许硬编码在业务代码里。标定文件必须写明设备序列号和标定日期——换了相机还用旧标定，是视觉抓取项目里最隐蔽的错误来源。
