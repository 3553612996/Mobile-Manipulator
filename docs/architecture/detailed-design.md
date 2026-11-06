# 详细设计说明书：STM32 固件与 ROS 2 上位机

> 文档状态：**v1.0 / 设计基线**
> 日期：2026-11-02
> 代码基线：固件 M3；ROS 2 `robot_base`、`robot_lidar` 已实现，其余节点为设计阶段
> 适用范围：RK3566（Linux + ROS 2 Humble）与 STM32F103RCT6（FreeRTOS）两端
> 相关文档：[项目框架](overview.md) · [软件结构](software-design.md) · [串口协议](protocol.md) · [底盘桥接](robot-base-design.md) · [Flash 布局](flash-layout.md) · [存储实现](storage-design.md)

---

## 1 文档说明

### 1.1 目的

本文把系统设计落到**可实现、可测试、可评审**的粒度：每个模块的职责边界、接口、关键数据结构、时序预算、异常路径和验证方法。

现有文档各管一段：`overview.md` 讲双控分工与硬件拓扑，`software-design.md` 讲包划分与状态机，`protocol.md` 讲线上字节，`robot-base-design.md` 讲桥接实现，`flash-layout.md` / `storage-design.md` 讲存储介质与记录格式。本文不重复字节级协议表和分区表，而是把它们组织成一条从需求到验证的链路，并补充：

- 两端模块之间的责任划分与调用顺序；
- 控制环、通信环、存储环的并发与单写者约束；
- 跨端单位、坐标、时间戳的映射；
- 每个模块的失败模式与降级动作；
- 当前实现状态与设计目标的差距。

### 1.2 读者

| 读者 | 关注章节 |
| --- | --- |
| 固件开发者 | 第 3、4、6、7 章 |
| ROS 2 上位机开发者 | 第 3、5、6、7 章 |
| 项目评审 / 面试 | 第 2、6、8、9 章 |
| 后续接手者 | 全文，重点是 1.3 的实现状态与第 9 章的限制 |

### 1.3 实现状态

**状态标记**：✅ 已实现并验证；🚧 部分实现；📐 设计完成、未实现；⬜ 未开始。

| 子系统 | 状态 | 说明 | 证据 |
| --- | --- | --- | --- |
| 串口协议 v1 | ✅ | 帧解析、CRC16、ACK、SEQ 去重、上报组帧两端实现 | `protocol.md`、`firmware/app/05_Middlewares/protocol`、`robot_base/src/protocol.cpp` |
| 底盘速度环 | ✅ | 四轮 PID、死区补偿、18 kHz PWM、10 ms 周期 | `service_chassis.c`、M3 测试报告 |
| 里程计与超时停车 | ✅ | 编码器积分、50 Hz 上报、200 ms 断流停车 | `service_chassis.c`、`app_robot_config.h` |
| 六轴舵机（当前） | ✅ | 六路 GPIO 软件 PWM，20 ms 帧，±90°，800 ms 插值 | `bsp_servo_softpwm.c`、`service_arm.c` |
| 舵机堵转 / 过温保护 | 📐 | 平台接口预留负载与温度字段，当前返回无效值 | `platform_bsp_servo.h` |
| 参数与标定持久化 | ✅ | A/B 副本、CRC32、递增 seq、脏标志 | `service_storage.c`、`app_params.c` |
| 日志环 / 故障环 | ✅ | 顺序追加、环满覆盖、20 Hz 导出 | `platform_logstore`、`app_storage.c` |
| HardFault 现场 | 🚧 | 存储服务支持 16 槽与提交 CRC；HardFault 入口尚未接线 | `service_storage.c`、`stm32f1xx_it.c` |
| bootloader / OTA | 📐 | 双镜像槽、回滚策略已设计，工程为空 | `flash-layout.md`、`firmware/bootloader/.gitkeep` |
| `robot_base` | ✅ | PC 直连验证：`/odom` 45.6 Hz、断链重连、服务 ACK | 50 个 gtest、`robot-base-design.md` |
| `robot_lidar` | ✅ | X2 实机 6.11 Hz，`/scan` 720 bin | 15 个 gtest、`robot_lidar/README.md` |
| `robot_interfaces` | 🚧 | `RobotStatus` / `SystemEvent` / `ClearFault` / `GetVersion` / `Grasp` 已定义，字段待 M8 冻结 | `robot_interfaces/` |
| `robot_bringup` | 🚧 | base / lidar / mapping / board / navigation 的 launch 与参数、URDF 骨架；Nav2 参数已落地 | `robot_bringup/` |
| `robot_arm` / `robot_vision` / `robot_speech` / `robot_task` | 📐 | 目录与接口规划完成，源码为空 | 第 5.5～5.8 节 |
| SLAM / Nav2 | 🚧 | SLAM 已接入（M4）；Nav2 + AMCL 离线闭环通过（M5）；实机 `0x0202` 已确认低电量并修复，精度待补 | M4/M5 节点报告 |
| 总线舵机升级 | 📐 | UART5 半双工 + YH ASCII 的目标设计，当前硬件用软件 PWM | 第 4.5.2 节 |

> **写法约定**：本文描述"设计应当如何"，实现状态以上表为准。未实现的模块不写成已完成；未实测的指标保持"待测"。

### 1.4 术语

| 术语 | 含义 |
| --- | --- |
| Host | RK3566，运行 Linux 与 ROS 2，负责感知、导航、视觉、语音与任务编排 |
| MCU | STM32F103RCT6，运行 FreeRTOS，负责底盘、机械臂、安全与存储 |
| 下行 | Host → MCU 的命令帧 |
| 上行 / 上报 | MCU → Host 的周期帧或事件帧 |
| ACK | 对需要确认的下行命令的应答，命令字为原命令 OR `0x80` |
| SEQ | 单字节序号，0～255 循环，用于 ACK 匹配与去重 |
| 硬实时 | 微秒级确定性，错过截止时间即视为失效；由 MCU 承担 |
| 软实时 | 毫秒级抖动可接受，失效可重试或降级；由 Linux 承担 |
| L1 / L3 | L1 为 PC 上无硬件的单元测试；L3 为台架或实车测试 |
| HIL | Hardware-in-the-Loop，上位机跑真实 ROS 2 节点、下位机为真实 MCU |

---

## 2 系统总体设计

### 2.1 需求与验收

**功能需求**：

| ID | 需求 | 验收方式 |
| --- | --- | --- |
| F1 | 接收 `/cmd_vel`，四轮独立闭环，50 Hz 上报里程计 | 台架：给定速度，轮速与位移在误差内 |
| F2 | 接收六关节目标角，限位检查后驱动机械臂 | 台架：逐关节给角，实际角跟随且不越限 |
| F3 | 通信断流 200 ms 内停车；急停无条件关闭 PWM | 故障注入：拔线、按急停 |
| F4 | 激光建图定位、Nav2 导航、视觉识别、语音指令、抓取闭环 | 场景测试：导航到点、识别并抓取 |
| F5 | 参数在线修改与掉电保存；日志、故障、崩溃现场可导出 | 调参工具回读、断电重启、导出解析 |
| F6 | 故障分级、状态上报、诊断与自动重连 | 注入 CRC 错、断链、低压、越限 |
| F7 | 固件在线升级，失败可回滚 | 升级中断电、校验失败、回滚 |

**非功能需求**：

| 维度 | 目标 | 设计手段 |
| --- | --- | --- |
| 控制实时性 | 控制环 10 ms，单次预算 < 1 ms | MCU 独占控制任务，最高业务优先级 |
| 安全响应 | 断流到停车 < 200 ms；急停不等调度 | 超时在控制任务内判定；急停走 EXTI 直接关 PWM |
| 通信可靠性 | 关键命令不丢；错误帧不影响后续帧 | CRC16、ACK 重发、同 SEQ 去重、单字节重同步 |
| 可观测性 | 状态、故障、日志、崩溃现场可回读 | 周期上报 + 事件帧 + Flash 环 + 诊断话题 |
| 可维护性 | 协议与业务分离，核心逻辑可在 PC 单测 | 五层单向依赖；协议层不依赖 HAL/RTOS |
| 可移植性 | 换 MCU 只改实现层 | `platform_*` 冻结接口，`04_Impl` 适配 |
| 成本与体积 | 单板双控，不增加实时协处理器 | RK3566 + STM32F103 最小系统 |

### 2.2 双控制器职责划分

| 维度 | RK3566 | STM32F103RCT6 |
| --- | --- | --- |
| 任务性质 | 计算密集、可容忍几十至几百毫秒延迟 | 简单但要求微秒至毫秒级确定性 |
| 调度 | Linux 分时调度，抖动毫秒级 | FreeRTOS 抢占式，抖动微秒级 |
| 失效后果 | 功能降级，可重启恢复 | 电机失控 |
| 算力 | 4×A55 @1.8 GHz，0.8 TOPS NPU | 72 MHz，48 KB RAM，256 KB Flash |
| 主要职责 | SLAM、Nav2、视觉推理、语音、任务状态机、日志监控 | 电机 PID、编码器、舵机、安全保护、协议、存储 |
| 不做什么 | 不直接控制电机 PWM | 不做视觉、导航、语音 |

**切分依据是实时性等级，不是算力强弱。** Linux 无法保证控制环的截止时间；而 SLAM、视觉推理、语音又只有 Linux 能承载。MCU 同时承担最后一道保护——通信断流 200 ms 主动停车——因为 Host 正是可能出故障的那一端。

### 2.3 硬件拓扑与接线约束

```
7.4 V 电池（2S，5C）
 ├──► 降压 5 V / 3 A ──► RK3566
 ├──► 降压 5 V / 10 A ─► 六轴舵机（独立一路）
 └──► 电机驱动板 ──────► 四路直流减速电机（含编码器）
                          │ PWM + 编码器
                       STM32F103RCT6
                          ├── USART3 + DMA ◄──► RK3566
                          ├── 六路 GPIO ──────► 六轴 PWM 舵机（当前软件 PWM）
                          ├── ADC ────────────► 电池分压监测
                          ├── SPI2 ───────────► W25Q64（8 MB）
                          ├── EXTI ───────────► 急停按钮
                          └── I2C ────────────► IMU（规划）

RK3566
 ├── USB ──► 深度相机
 ├── USB ──► 麦克风阵列
 └── USB / UART ──► 2D 激光雷达（X2）
```

**接线约束**：

1. **所有设备必须共地**。不共地时串口会出现随机乱码，这是板间通信第一大坑。
2. **舵机独立供电**。六轴同时动作瞬时电流可达 10 A，与主控共用一路会把 RK3566 拉欠压重启。
3. **串口线远离电机动力线**。必要时用屏蔽线或双绞线，降低 CRC 错误率。
4. **急停是常闭触点接 EXTI**。断线即触发，符合失效安全原则。

### 2.4 软件分层

**STM32 侧**：

```
┌──────────────────────────────────────────────────────────┐
│ 01_APP      app_robot：任务编排、命令处理、状态机、故障策略 │
├──────────────────────────────────────────────────────────┤
│ 02_Service  chassis / arm / proto / storage / key         │
├──────────────────────────────────────────────────────────┤
│ 03_Platform platform_mcu / os / bsp / middleware 冻结接口  │
├──────────────────────────────────────────────────────────┤
│ 04_Impl     HAL、FreeRTOS、具体芯片的适配与实例化          │
├──────────────────────────────────────────────────────────┤
│ 06_BSP      双 PWM 电机、软件 PWM 舵机、Flash、ADC、GPIO    │
├──────────────────────────────────────────────────────────┤
│ 05_Middlewares  protocol：纯 C 的帧编解码与协议表          │
└──────────────────────────────────────────────────────────┘
```

依赖只向下：`01_APP → 02_Service → 03_Platform → 04_Impl → 06_BSP`。`01_APP` 内不允许出现 `HAL_*`、`FreeRTOS.h` 或寄存器名；`02_Service` 不允许包含板级头文件。协议表放在 `05_Middlewares`，只依赖 `platform_def` 与 CRC，可在 PC 上用 gcc 编译。

**一处结构性例外**：`06_BSP` 通过 `03_Platform/platform_mcu` 的 `platform_*` 接口访问外设，而 `platform_mcu` 只是 `04_Impl/impl_*_wrap` 的薄转发。物理调用链在实现层与驱动层之间折返一次：

```
impl_bsp_motor → bsp_motor_dual_pwm → platform_pwm_set_duty
               → impl_pwm_set_duty_wrap → HAL_TIM_PWM_Start/Stop
```

这样 BSP 不直接持有 HAL 句柄，换 MCU 时只改 `impl_mcu`。

**RK3566 侧**：

```
应用层      robot_task
功能层      robot_vision   robot_arm   robot_speech
传感器层    robot_lidar
中间件      Nav2   SLAM Toolbox   TF2
桥接层      robot_base（唯一的底盘硬件边界）
系统层      systemd   日志轮转   资源监控
```

只有底层驱动包（`robot_base`、`robot_lidar`、未来的 `robot_arm` 传输层）知道设备路径。上层节点只依赖 ROS 2 接口，因此可以在 PC 上仿真、录包回放。

### 2.5 设计原则

| 原则 | 落实方式 | 解决的问题 |
| --- | --- | --- |
| 唯一硬件边界 | `robot_base` 独占底盘串口；`appStorage` 独占 Flash；`appComm` 独占 UART TX | 多写者导致帧交错、总线争用、状态分叉 |
| 单向依赖 | 层间只向下；ROS 包依赖接口包，不反向依赖任务包 | 编译顺序、循环依赖、单测困难 |
| 安全动作不排队 | 急停 EXTI 直接关 PWM；断流在控制任务内判定 | 任务卡死时安全动作仍能执行 |
| 状态权威、事件通知 | `REPORT_STATUS` 是状态权威，`REPORT_EVENT` 只通知 | 事件帧丢失不会导致状态误判 |
| 失败可观测 | 每个错误码、故障码、ACK 结果都有唯一含义 | 现场排障不靠猜 |
| 参数不硬编码 | 影响行为的数字进参数区或 yaml；标定带设备序列号与日期 | 换硬件后旧参数悄悄生效 |

---

## 3 跨端接口设计

### 3.1 物理层与帧格式

| 项目 | 当前值 | 目标值 | 说明 |
| --- | --- | --- | --- |
| 接口 | USART3 ↔ RK3566 UART | 同左 | 3.3 V TTL，TX/RX 交叉，必须共地 |
| 帧格式 | 8N1 | 8N1 | 无校验、1 停止位、无流控 |
| 波特率 | 115200 | 921600 | 先调通协议，再提速，分离协议 bug 与信号完整性问题 |
| 角色 | RK3566 = Host，STM32 = MCU | 同左 | Host 主导指令，MCU 周期上报 |

```
偏移   0      1      2      3      4      5 .. 2+LEN   3+LEN  4+LEN
     ┌──────┬──────┬──────┬──────┬──────┬───────────┬──────┬──────┐
     │ 0xAA │ 0x55 │ LEN  │ CMD  │ SEQ  │  PAYLOAD  │ CRC_L│ CRC_H│
     └──────┴──────┴──────┴──────┴──────┴───────────┴──────┴──────┘
```

- `LEN = 2 + N`，范围 `[2, 66]`；总帧长 `5 + LEN`，最短 7 字节（ESTOP），最长 71 字节。
- CRC 为 CRC-16/CCITT-FALSE：多项式 `0x1021`、初值 `0xFFFF`、不反射、结果不异或，对 `"123456789"` 的结果为 `0x29B1`。
- CRC 覆盖 `LEN + CMD + SEQ + PAYLOAD`，小端存放，不含帧头和 CRC 自身。
- 数值统一小端、定点编码，不直接内存拷贝；显式序列化保证 PC 单测与 MCU 行为一致。

### 3.2 命令与上报总表

**下行命令**：

| CMD | 名称 | 负载 | 周期 | ACK |
| --- | --- | --- | --- | --- |
| `0x01` | SET_VELOCITY | `int16 v` mm/s，`int16 w` mrad/s | 20 ms | 否 |
| `0x02` | SET_ARM_JOINTS | 6 × `int16`，0.01° | 按需 | 否 |
| `0x03` | SET_GRIPPER | `uint8 action`，`uint8 force` | 按需 | 是 |
| `0x04` | SET_ENABLE | `uint8` 0/1 | 按需 | 是 |
| `0x05` | SET_PARAM | `uint8 id`，`int32 value` | 按需 | 是 |
| `0x06` | ESTOP | 无 | 事件 | 是 |
| `0x07` | CLEAR_FAULT | `uint16 code`，`0xFFFF` 清全部 | 事件 | 是 |
| `0x08` | PARAM_SAVE | `uint8 scope` | 按需 | 是 |
| `0x09` | PARAM_RESET | `uint8 scope`，`uint8 confirm=0xA5` | 事件 | 是 |
| `0x0A` | STORAGE_QUERY | 无 | 按需 | 是 |
| `0x0B` | LOG_EXPORT | `source`，`action`，`uint32 from_seq` | 事件 | 是 |
| `0x0C` | CRASH_READ | `slot`，`flags` | 事件 | 是 |
| `0x0D` | LOG_CLEAR | `source`，`confirm=0xA5` | 事件 | 是 |

**上行上报**：

| CMD | 名称 | 负载长度 | 周期 | 用途 |
| --- | --- | --- | --- | --- |
| `0xC1` | REPORT_ODOM | 14 | 50 Hz | `x/y/theta/vl/vr` |
| `0xC2` | REPORT_ARM | 15 | 20 Hz | 六关节实际角、夹爪位置与状态 |
| `0xC3` | REPORT_STATUS | 8 | 10 Hz | 状态字、故障码、电池、温度、最大抖动 |
| `0xC4` | REPORT_EVENT | 6 | 事件 | 故障产生、清除、模式切换 |
| `0xC5` | REPORT_HEARTBEAT | 3 | 10 Hz | 状态 + 运行计数 |
| `0xC6` | REPORT_WHEEL_SPEED | 8 | 50 Hz | 四轮实测速度，调参用 |
| `0xC7` | REPORT_WHEEL_DUTY | 8 | 50 Hz | 四轮实际占空比，调参用 |
| `0xC8` | REPORT_STORAGE | 24 | 查询/保存结束 | 存储状态与寿命计数 |
| `0xC9` | REPORT_LOG | 15+N | 导出 ≤ 20 Hz | 日志或故障记录 |
| `0xCA` | REPORT_CRASH | 7+N | 读取时连续 | 崩溃现场分片 |

**设计要点**：

- `SET_VELOCITY` 不要 ACK。50 Hz 周期发送，下一帧就是重传；可靠性由断流停车保证，而不是由 ACK 保证。
- `SET_ENABLE`、`SET_PARAM`、`ESTOP`、存储类命令要 ACK。状态变更丢失会出问题。
- 存储类 ACK 只表示"已受理"。擦除一个 4 KB 扇区最长 400 ms，等写完再 ACK 会阻塞通信任务并误触发 200 ms 超时。落盘结果由 `REPORT_STORAGE` 的 bit0（无脏数据）与 bit3（保存失败）表达。
- `REPORT_WHEEL_SPEED` 与 `REPORT_WHEEL_DUTY` 同拍发送。只有速度没有占空比时，"被限幅"与"电机不动"在主机看来完全一样。

### 3.3 时序与可靠性

| 项目 | 值 | 说明 |
| --- | --- | --- |
| 速度帧周期 | 20 ms | `robot_base` 定时器持续发送 |
| MCU 指令超时 | 200 ms | 超时主动停车并锁存 `0x0202` |
| 关键命令 ACK 超时 | 200 ms | 未收到应答则重发 |
| 重发次数 | 3 次 | 共 4 次尝试，沿用同一 SEQ 与同一帧字节 |
| SEQ 去重窗口 | 1 s | 重复 SEQ 回缓存结果，不重复执行 |
| 心跳周期 | 100 ms | MCU → Host |
| Host 链路超时 | 500 ms | 无合法帧判定失联，停止下发 |
| CRC 错误告警阈值 | 连续 10 帧 | 上报 `0x0201`（WARN），不断链 |
| 日志导出速率 | ≤ 20 Hz | 防止挤掉速度帧与心跳 |

**超时停车的理由**：Host 崩溃、进程卡死、线断了、节点被 kill，这几种情况下机器人都不应该沿最后一条速度指令继续跑。停车必须由 MCU 判定，因为 Host 正是可能出问题的那一端。200 ms 是折中：50 ms 会因一次丢帧误触发；更长则失控距离过大（0.5 m/s × 0.2 s = 10 cm）。

### 3.4 单位、坐标与所有权

| 量 | 线上编码 | ROS 2 单位 | 唯一所有者 |
| --- | --- | --- | --- |
| 线速度 | `int16` mm/s | m/s | MCU 执行；Host 下发 |
| 角速度 | `int16` mrad/s | rad/s | MCU 执行；Host 下发 |
| 关节角 | `int16` 0.01° | rad | MCU 执行；Host 解算 |
| 位置 | `int32` mm | m | MCU 积分；Host 只换算 |
| 航向 | `int16` 0.01°，归一到 `(-π, π]` | rad | MCU 积分；Host 解卷绕 |
| 电池 | `uint16` mV | — | MCU 采样 |
| 温度 | `int8` °C | — | MCU 采样 |
| 时间戳 | — | ROS 到达时间 | Host 打时间戳，不用固定周期推算 |

**坐标约定**：REP-103 风格，平面运动；`+x` 前进，`+y` 向左，`+w` 从上往下看逆时针为正。四轮顺序固定为 FL、FR、RL、RR，所有数组、上报、参数 id 都按这个顺序。

**为什么位姿由 MCU 积分**：MCU 已经在控制环里拿到四轮编码器增量，积分成本低且与轮速同源。Host 再积一遍会引入第二套误差，并与固件分叉。Host 只做单位换算、航向解卷绕和 twist 反算。

### 3.5 状态与故障

**状态字**（`REPORT_STATUS` / `REPORT_HEARTBEAT` 第 0 字节）：

| 值 | 状态 | 含义 |
| --- | --- | --- |
| 0 | INIT | 上电初始化，尚未进入控制 |
| 1 | IDLE | 已就绪、未使能，电机不出力 |
| 2 | MANUAL | 已使能，接受 Host / 按键运动指令 |
| 3 | AUTO | 预留：MCU 内部自主运动 |
| 4 | ESTOP | 急停锁定，需清故障并重新使能 |
| 5 | FAULT | 有故障码锁存 |

**状态与故障码分开**：状态表示"现在能不能动"，故障码表示"为什么不能动"。Host 不需要从状态字反推故障类型。

**故障级别**：

| 级别 | 行为 | 例子 |
| --- | --- | --- |
| WARN | 记录、上报，继续运行但受限 | CRC 错误率超限、电池低压、参数区无效 |
| ERROR | 停止对应子系统，其他部分继续 | 电机过流、关节越限、存储写失败 |
| FATAL | 全部停止，必须人工清除 | 通信超时、急停 |

**仲裁规则**：`REPORT_STATUS` 只带一个故障码。多个条件同时成立时，级别高的顶替级别低的；同级保留先锁存的；急停显式强制覆盖。被顶替的条件若仍存在，会在下一次 `CLEAR_FAULT` 后重新上报。故障码只置位不清零，直到 Host 发 `CLEAR_FAULT` 且条件已消失；条件未消失时固件返回 `0x02`，避免"清了又立刻报"的抖动。

### 3.6 版本与兼容

- 当前协议版本为 **v1**，没有版本协商命令。
- 命令字分段：`0x01～0x3F` 下行，`0x80～0xBF` 应答，`0xC0～0xFF` 主动上报。
- 固件对每条命令做严格负载长度校验；长度不符或未知命令不进入业务处理。
- v1 没有 `GET_VERSION`，因此 `robot_base` 不注册 `/get_version`；启动日志明确打印"固件版本未知"。
- v1 没有 `GET_PARAM`，固件里的轮距、轮径等标定值读不回来；改标定后必须人工同步 `base.yaml`。
- 任何协议改动必须同时修改固件、ROS 2、`protocol.md` 和测试向量，并在同一个提交里完成。新增命令时优先复用现有语义，不改变已有字段布局。

---

## 4 STM32 详细设计

### 4.1 分层与模块清单

| 层 | 目录 | 模块 | 职责 | 约束 |
| --- | --- | --- | --- | --- |
| 应用 | `01_APP/app_robot` | `app_robot` / `app_user_init` | 组合根：初始化顺序、任务表、队列表 | 只编排 |
|  |  | `app_tasks` | `appComm` / `appCtrl` 主循环 | 不写业务算法 |
|  |  | `app_storage` | `appStorage` 主循环、Flash 单写者 | 不碰 UART |
|  |  | `app_params` | 参数与标定的 RAM 副本、默认值、序列化、脏标志 | 不直接读写 Flash |
|  |  | `app_handlers` | 13 条命令的处理函数、按键手势、PID 入队 | 只做校验与翻译 |
|  |  | `app_state` | 主状态机 | 不做 I/O |
|  |  | `app_fault` | 故障仲裁与降级策略 | 不做 I/O |
|  |  | `app_report` | 上报节奏 | 只决定何时发 |
|  |  | `app_battery` | 电池采样、中值滤波与低压去抖 | 只读 ADC；连续 5 次滤波低压才置位 |
| 服务 | `02_Service` | `service_chassis` | 麦轮逆解、四轮 PID、里程计、超时、急停锁存 | 不读时钟，时间由调用方注入 |
|  |  | `service_arm` | 六关节限位、使能、目标保持 | 不实现总线协议 |
|  |  | `service_proto` | 帧解析、命令分发、ACK、SEQ 去重、上报组帧 | 唯一 UART 发送者 |
|  |  | `service_storage` | A/B blob、环、崩溃槽、磨损统计 | 单线程写 |
|  |  | `service_key` | 按键消抖、短按 / 长按手势 | 不直接执行动作 |
| 平台 | `03_Platform` | `platform_mcu` / `platform_os` / `platform_bsp` / `platform_middleware` | 外设、OS、BSP 设备、通用算法的冻结接口 | 不随下层实现变化 |
| 实现 | `04_Impl` | `impl_mcu` / `impl_os` / `impl_bsp` / `impl_board` / `impl_wrapper` | HAL、FreeRTOS、板级实例化、日志插桩 | 只做适配 |
| 驱动 | `06_BSP` | `bsp_motor_dual_pwm` / `bsp_servo_softpwm` / `bsp_flash_w25q64` / `bsp_battery_adc` / `bsp_key_gpio` / `bsp_led_gpio` / `bsp_buzzer_gpio` | 具体芯片与模块驱动 | 允许包含 HAL |
| 中间件 | `05_Middlewares/protocol` | `protocol_def` / `protocol_codec` | 协议表、组帧 / 解帧、CRC | 纯 C，可在 PC 编译 |

**`shared/` 的用途**：bootloader 与 app 都需要 CRC 与帧解析。复制两份会导致改一边忘一边，升级校验失败从"偶发"变成"必然"。因此共享实现，不共享状态。

### 4.2 启动与初始化

```
上电
 ├─► HAL_Init / 时钟 / 外设
 ├─► board_init：BSP 注册（电机、舵机、Flash、ADC、按键、LED、蜂鸣器）
 ├─► app_robot_init → app_user_init_init
 │    1. platform_uart_init：USART3 + DMA 循环接收，115200 8N1
 │    2. 创建三个队列：appPid、appStore、appExport
 │    3. 挂载 W25Q64，加载参数区与标定区 blob
 │    4. service_chassis_init：几何、限速、四轮 PID、编码器比例
 │    5. service_arm_init：六关节限位、回零、默认运动时间
 │    6. service_key_init：消抖与长按阈值
 │    7. service_proto_init：绑定 UART，准备解析器
 │    8. app_state_init / app_fault_init
 │    9. 注册 13 条命令处理函数
 │   10. app_storage_init：绑定请求队列与导出队列
 └─► app_robot_start：按任务表创建三个任务，注册 UART 事件回调
      └─► app_state_ready：INIT → IDLE
      └─► osKernelStart
```

**关键顺序**：存储先于底盘初始化。底盘需要从参数区读取几何、增益、编码器比例；如果先初始化底盘再挂载 Flash，第一次上电会用编译期默认值跑，直到下一次重启才生效。

**存储失败不阻止启动**：W25Q64 缺失或分区表无效时，`app_user_init_storage()` 返回能力标志而不是错误。系统用编译期默认值运行，存储类命令返回 `0x06`（存储不可用），参数区无效时上报 `0x0601`。机器人首先必须能停下来，其次才是能保存参数。

### 4.3 任务、队列与中断

**任务表**（`app_user_init.c` 的常量表）：

| 任务 | 优先级 | 周期 / 唤醒 | 栈 | 职责 |
| --- | --- | --- | --- | --- |
| `appComm` | HIGH | 10 ms 或 UART RX 事件 | 1024 B | 协议解析与分发、ACK、故障轮询、全部上报、导出队列出栈 |
| `appCtrl` | ABOVE_NORMAL | 10 ms | 1024 B | 按键、PID 参数出队、四轮速度环、里程计、蜂鸣器 |
| `appStorage` | LOW | 10 ms 轮询 | 1024 B | Flash 单写者：保存、日志 / 故障入环、清除、导出产生 |

另有 CubeMX 生成的 `defaultTask`（1 s 空转），不参与业务。三个业务任务足以覆盖四类节奏：10 ms 控制、事件驱动通信、20/50/100 ms 上报、按需存储。上报本身每周期不到 40 字节，独立成任务只会增加同步点。

**周期调度**：平台层提供 `platform_os_task_delay_until()`（封装 `vTaskDelayUntil`），用于消除累积漂移。当前 `appCtrl` 使用 10 ms 延时并以 `elapsed_ms` 计算实际间隔，控制算法对间隔做了补偿；后续切换到 `delay_until` 可进一步降低残余抖动。

**队列表**：

| 队列 | 元素 | 长度 | 生产者 → 消费者 | 说明 |
| --- | --- | --- | --- | --- |
| `appPid` | `app_pid_param_msg_t` | 8 | `appComm` → `appCtrl` | 按值拷贝；PID 只在控制任务里写入 |
| `appStore` | `app_storage_req_t`（描述符） | 8 | 任意任务 → `appStorage` | 不传 4 KB 负载，只传描述符与缓冲区借用信息 |
| `appExport` | `app_storage_export_t` | 4 | `appStorage` → `appComm` | 日志 / 崩溃 / 存储状态待发记录 |

**单写者约束**：

| 资源 | 唯一所有者 | 其他任务如何访问 |
| --- | --- | --- |
| UART TX | `appComm` | 控制任务置 `s_pending_estop` 标志；存储任务投递导出记录 |
| 外部 Flash | `appStorage` | 其他任务投递 `appStore` 描述符 |
| 四轮 PID 状态 | `appCtrl` | `appComm` 只投递参数消息 |
| 里程计状态 | `appCtrl` | `appComm` 通过 `service_chassis_get_odometry()` 只读快照 |
| 主状态机 | `appComm`（命令处理） | 控制任务只上报事件，不直接改状态 |

**中断与优先级**：

| 中断 | 抢占优先级 | 做什么 | 不做什么 |
| --- | --- | --- | --- |
| 急停 EXTI | 0（最高） | 直接关闭 PWM，置标志 | 不调用任何 FreeRTOS API |
| USART3 DMA HT / TC / IDLE | 6 | 搬运字节到环形缓冲，`vTaskNotifyGiveFromISR` | 不解析协议 |
| 编码器溢出 | 6 | 计数修正，< 5 µs | 不做浮点运算 |
| 舵机软件 PWM 定时器 | 由 BSP 配置 | 翻转 GPIO，推进 2.5 ms 时隙 | 不调用阻塞 API |
| SysTick | 15（最低） | FreeRTOS 时基 | — |

**中断铁律**：凡是调用 `xxxFromISR()` 的中断，优先级数值必须 ≥ `configMAX_SYSCALL_INTERRUPT_PRIORITY`（CubeMX 默认 5，数值越大优先级越低）。把串口中断配成 0 或 1，它会在内核临界区里插进来，破坏队列链表，表现为随机 HardFault。急停中断是唯一例外：它配成最高优先级且完全不碰 RTOS API。

**ISR 三条约束**：不阻塞（不用 `HAL_Delay`、不等标志位）、不分配内存、不打印。耗时逻辑一律丢给任务，中断只做"通知"。

### 4.4 底盘控制

#### 4.4.1 坐标与麦轮逆运动学

平面坐标：`+x` 前进，`+y` 向左，`+w` 逆时针。四轮 FL / FR / RL / RR。设 `k = half_length + half_width`（默认 `150 + 120 = 270 mm`）：

```
FL = vx - vy - k·w/1000
FR = vx + vy + k·w/1000
RL = vx + vy - k·w/1000
RR = vx - vy + k·w/1000
```

其中 `vx/vy` 单位 mm/s，`w` 单位 mrad/s，`k·w/1000` 单位 mm/s。协议 v1 的 `SET_VELOCITY` 只有 `v` 与 `w`，因此 `vy = 0`；服务层仍保留 `vy` 入口，方便台架脚本和未来协议扩展。

每个轮目标先按 `max_wheel_mm_s`（默认 500 mm/s）限幅，再进入该轮 PID。整车限幅为 `max_vx = 500 mm/s`、`max_vy = 500 mm/s`、`max_wz = 1500 mrad/s`；越界命令返回 `0x01`，不静默截断。

#### 4.4.2 10 ms 控制环

`service_chassis_apply(now_ms, dt_ms)` 的执行顺序：

1. **读编码器**：读取四个 32 位计数，用无符号减法计算增量，兼容计数器回绕。
2. **换算位移**：`d_mm = counts × counts_per_mm_inv × dir`；每轮有独立的 `counts_per_mm` 与方向。
3. **计算实测轮速**：`d_mm / dt_s`，用于 PID 反馈与上报。
4. **检查测试模式**：若开环测试未过期，直接输出测试占空比，跳过速度环。
5. **检查指令超时**：`now_ms - last_cmd_ms > cmd_timeout_ms` 时，目标清零、`platform_motor_stop_all()`、重置 PID、锁存 `0x0202`。
6. **逆运动学**：目标速度 → 四轮目标速度。
7. **四轮 PID**：浮点位置式 / 增量式速度环，输出占空比（‰）。
8. **死区补偿**：轮子静止且目标非零时，把输出抬到该轮该方向的可起转占空比。
9. **限幅与输出**：占空比限制在 `[-1000, 1000]‰`，写入 18 kHz PWM。
10. **里程计**：同一任务内更新位姿，保证与轮速同源。

#### 4.4.3 PID 与死区补偿

| 项 | 默认值 | 说明 |
| --- | --- | --- |
| Kp / Ki / Kd | 5.0 / 6.0 / 0.0 | 台架整定起点；Kd 保持 0，直到速度反馈加滤波 |
| 参数编码 | ×1000 的 `int32` | 线上与 Flash 都用整数，避免浮点布局差异 |
| 死区补偿（前向） | A 75、B 67、C 82、D 80 ‰ | 7.81 V 台架实测 |
| 死区补偿（反向） | A 67、B 75、C 80、D 100 ‰ | 正反向死区不对称，分别标定 |
| PID 增益范围 | Kp/Ki 0～10.000，Kd 0～5.000 | 越界返回 `0x01`，不静默截断 |

**为什么 10 ms 而不是 1 ms**：13 线编码器 × 4 倍频 × 90 减速比 ≈ 24828 counts/mm。1 ms 内一个计数约等于 40 mm/s，PID 反馈会被量化噪声主导。10 ms 下仍有 100 Hz 控制频率，四个浮点环与里程计都在预算内。

#### 4.4.4 指令超时与恢复

- 超时判定在控制任务内完成，不依赖通信任务是否被调度。
- 超时后：目标速度清零、四轮占空比清零、PID 积分与微分状态复位、锁存 `0x0202`（FATAL）、上报事件。
- 恢复流程：Host 发 `CLEAR_FAULT`（条件已消失）→ 状态回 IDLE → Host 重新 `SET_ENABLE`。**清故障不自动使能**，避免机器人无预期地重新运动。
- 超时期间收到新的 `SET_VELOCITY` 只清除超时标志，不直接恢复输出；下一次控制节拍从新的编码器基线重新开始，防止等待期间累积的计数被当成速度尖峰。

#### 4.4.5 里程计

```
d_mm[i] = counts[i] × (1 / counts_per_mm[i]) × dir[i]

dx_body = (FL + FR + RL + RR) / 4
dy_body = (-FL + FR + RL - RR) / 4
d_theta = (-FL + FR - RL + RR) / (4 × (half_length + half_width))

h_mid = heading + d_theta / 2
x += dx_body·cos(h_mid) - dy_body·sin(h_mid)
y += dx_body·sin(h_mid) + dy_body·cos(h_mid)
heading += d_theta，并归一到 (-π, π]

vl = (FL + RL) / (2·dt)
vr = (FR + RR) / (2·dt)
```

- 中点航向是二阶精度，计算量小，直线与转弯都能接受。
- `counts_per_mm` 默认 24828（60 mm 轮径、4680 counts/rev）；轮径写错表现为固定比例的距离误差，不是随机漂移，便于标定。
- 航向归一到 `(-π, π]` 是为了 int16 的 0.01° 编码；Host 负责解卷绕，固件不维护连续航向。
- 里程计在控制任务中更新，上报在通信任务中读取快照，避免并发写。

#### 4.4.6 台架开环测试

| 项 | 规定 |
| --- | --- |
| 参数 id | `0x30～0x33`，对应 FL / FR / RL / RR |
| 取值 | `-400～+400 ‰`，越界返回 `0x01` |
| 前置 | 底盘已使能且未急停，否则返回 `0x02` |
| 失效 | 每轮单独计时，300 ms 未刷新按 0 处理，该轮滑行 |
| 退出 | 任何 `SET_VELOCITY`、`STOP`、`SET_ENABLE(0)`、`ESTOP` 立即退出 |
| 超时 | 测试模式期间 200 ms 指令超时不生效，使用自己的 300 ms 过期 |

这组 id 是台架工具，不是产品接口。上限 400‰ 高于减速电机死区、低于满转，脚本写错也不会让轮子满扭矩冲出去。写入后固件在同一 20 ms 节拍回报 `0xC7`，主机据此确认"写进去了"。

#### 4.4.7 参数生效路径

```
Host SET_PARAM
  → appComm 校验范围、写 RAM 副本、置脏标志、回 ACK 0x00
  → 控制类参数投递 appPid 队列
  → appCtrl 在下一个 10 ms 节拍出队
  → service_chassis_set_pid_gain / set_geometry / set_limits
  → 下一个控制周期生效
```

只有控制任务可以调用会改变 PID 状态的 setter，避免"通信任务正在写增益、控制任务正在算 PID"的并发窗口。标定类参数（轮距、轮径、编码器比例、方向）要求底盘失能；限速与超时允许在使能状态修改，但只重载限幅，不重置 PID 积分。

### 4.5 六轴机械臂

#### 4.5.1 当前实现：GPIO 软件 PWM

平台契约 `platform_servo_ops_t` 隐藏具体总线，角度统一用 0.01°，避免线上浮点。当前实现 `bsp_servo_softpwm`：

| 项 | 值 | 说明 |
| --- | --- | --- |
| 通道数 | 6 | 每关节一个 GPIO |
| 帧长 | 20 ms | 50 Hz |
| 时隙 | 2.5 ms / 关节 | 6 × 2.5 ms + 5 ms 间隙 |
| 脉宽 | 500～2500 µs | 中心 1500 µs，±1000 µs 对应 ±90° |
| 角度单位 | 0.01° | `±9000` 对应 `±90°` |
| 插值 | 每帧一次，定点整数 | 无浮点、无 FPU 依赖 |
| 默认运动时间 | 800 ms | `SET_ARM_JOINTS` 未指定时使用 |
| 使能 | 支持 | 失能时停止输出脉冲 |

`service_arm` 的职责：

1. 校验六个关节是否都在配置限位内；任一越限返回参数错误，**保持上一个目标**，不部分执行。
2. 校验机械臂是否已使能；未使能返回失败。
3. 记录最后接受的目标，转发给平台层。
4. 提供目标角与实测状态读取接口。

**限制**：当前 `get_states()` 的负载与温度返回 `PLATFORM_SERVO_VALUE_INVALID`，因为软件 PWM 没有电流 / 温度反馈；堵转、过温、欠压 / 过压保护尚未实现。`SET_GRIPPER` 由于没有夹爪硬件，处理函数直接返回失败，不伪装成功。

#### 4.5.2 目标设计：UART5 半双工总线舵机（未实现）

BOM 选用的是支持 YH ASCII v2.0 的总线舵机。目标设计如下，平台接口保持不变，应用与 `service_arm` 不需要改：

| 项 | 设计值 |
| --- | --- |
| 总线 | UART5 半双工，115200 8N1，菊花链 |
| 协议 | YH ASCII v2.0 |
| 脉宽 | 500～2500 µs，对应 0～180°（按舵机型号标定） |
| 周期 | 5 ms 状态机轮询，不阻塞 10 ms 控制环 |
| 读状态 | PRAD（位置）/ PRTV（速度 / 负载）轮询 |
| 写运动 | 六关节运动帧，带时间参数 |
| 保护 | 限位、目标斜坡、堵转、过温、欠压 / 过压 |
| 唯一性 | 关节 ID 与零点在总线上全局唯一，标定结果入标定区 |

**迁移步骤**：

1. 新建 `bsp_servo_bus_uart5`，实现 `platform_servo_ops_t` 的五个回调。
2. 在 `impl_bsp_servo` 中按编译开关选择软件 PWM 或总线实现。
3. 在 5 ms 状态机中完成"扫描 → 读状态 → 写目标"的轮转，避免在控制任务里阻塞等回复。
4. 把负载、温度、故障位映射到 `platform_servo_state_t`，由 `service_arm` 升级为真实保护。
5. 增加 ID / 零点唯一性校验和总线超时恢复。

**为什么先做软件 PWM**：六路 GPIO 方案没有总线时序与舵机固件差异，能把"关节限位、目标插值、使能逻辑"先验证掉。总线方案的主要风险在通信与舵机个体差异，分层后可以单独替换，不推倒上层。

### 4.6 协议服务

**接收路径**：

```
USART3 RX → DMA 循环缓冲（256 B）
  → HT / TC / IDLE 中断：把新字节搬进环形缓冲，只 notify appComm
  → appComm：service_proto_process(now_ms)
      → 每次取最多 32 B 喂给协议解析器
      → 解析器输出完整帧
      → 查命令表 → 调用 app_handlers 注册的处理函数
      → 需要 ACK 的命令：组 ACK，写入 UART
```

**解析器行为**：

- 71 字节重组缓冲。每次 `feed()` 追加新字节，然后从头扫描。
- 找到 `AA 55` 且 `LEN ∈ [2, 66]` 时，按 `LEN` 取整帧算 CRC；通过则输出，失败则**只丢 1 个字节再重新扫描**。
- 不维护"当前处于哪个字段"的状态机，因此不会被负载里的伪帧头带偏；错误路径只有一条。
- 两个字节间隔超过 50 ms 时丢弃半帧、重新同步，用于处理"收到一半线断了"。
- 解析器不读时钟，时间由 `service_proto_process()` 传入，便于 PC 单测。

**分发与应答**：

- 命令表 16 个槽位，当前注册 13 条命令；重复注册返回错误。
- 每条命令校验固定负载长度；长度不符不进入业务处理。
- ACK payload 为 `[被应答 SEQ, result]`；结果码 `0x00` 成功、`0x01` 范围、`0x02` 状态不允许、`0x03` 未知命令、`0x04` 执行失败、`0x05` 忙、`0x06` 存储不可用。
- 需要去重的命令按命令字各自保存 `last_seq` 与时间戳；1 s 内重复 SEQ 直接回缓存结果，不重复执行。`SET_VELOCITY` 不去重，重复执行无害。

**上报调度**：

| 上报 | 周期 | 触发 |
| --- | --- | --- |
| ODOM | 20 ms | `app_report_tick` |
| WHEEL_SPEED / WHEEL_DUTY | 20 ms | 同一节拍，先速度后占空比 |
| ARM | 50 ms | 周期 |
| STATUS | 100 ms | 周期 |
| HEARTBEAT | 100 ms | 周期 |
| EVENT | 事件 | 故障产生 / 清除 / 模式切换 |
| STORAGE / LOG / CRASH | 按需 | 查询、保存结束、导出 |

**链路统计**：`frames_ok`、`crc_errors`、`len_errors`、`resyncs`、`unknown_cmds`、`acks_sent`、`tx_failures`。连续 10 帧 CRC 错误上报 `0x0201`，但不断链——偶发干扰不应让机器人失联。

### 4.7 应用状态机与故障

```
INIT ──ready──► IDLE ◄──SET_ENABLE(0)──► MANUAL
                 ▲          ──SET_ENABLE(1)──►

任意状态 ──ERROR──► FAULT ──CLEAR_FAULT──► IDLE
任意状态 ──FATAL──► ESTOP ──CLEAR_FAULT──► IDLE
```

| 迁移 | 触发 | 动作 |
| --- | --- | --- |
| INIT → IDLE | 初始化完成 | 允许接受使能 |
| IDLE ↔ MANUAL | `SET_ENABLE` | 使能 / 失能底盘与机械臂 |
| → FAULT | ERROR 级故障 | 停对应子系统，拒绝运动指令，仍接受 `CLEAR_FAULT` |
| → ESTOP | FATAL 级故障、`ESTOP`、按键长按 | 四路桥臂 coast，拒绝使能 |
| FAULT / ESTOP → IDLE | `CLEAR_FAULT` 且条件消失 | **不自动使能**，Host 必须重新 `SET_ENABLE` |

AUTO 在协议中预留，当前没有内部自主运动任务，固件不会主动进入。状态机本身不做 I/O：它不认识电机、串口和故障码，只接收事件并输出状态。

**故障处理分层**：

| 层 | 检测 | 处理 |
| --- | --- | --- |
| 硬件 | 过流、编码器异常、电池低压、温度 | 停对应轴 / 降额，置故障码 |
| 通信 | CRC 错误率、超时、心跳 | 重发 → 告警 → 停车 |
| 应用 | 参数越界、状态不允许、关节超限 | 拒绝命令，返回结果码 |
| 存储 | 擦除 / 编程 / 回读校验失败 | 保留旧副本，上报 `0x0603` |
| 系统 | 栈溢出、HardFault | 停电机、记录现场、复位 |

**局部失效不扩散**：机械臂故障不影响底盘移动；视觉节点失效禁止抓取但导航继续；语音失效由界面或遥控接管。

### 4.8 参数与标定管理

| 数据类型 | 存放 | 生效 | 掉电 |
| --- | --- | --- | --- |
| 编译期默认值 | `app_robot_config.h` | 编译时 | 固化在 Flash |
| 运行参数（PID、限速、超时、日志等级） | RAM + 参数区 | `SET_PARAM` 立即生效 | `PARAM_SAVE(scope=0)` |
| 标定（轮距、轮径、编码器比例、方向、起转占空比） | RAM + 标定区 | 失能状态下写入，下一控制周期生效 | `PARAM_SAVE(scope=1)` |
| ROS 2 节点参数 | `robot_bringup/config/*.yaml` | 启动加载，`ros2 param set` 热改 | 版本库 |

**写入与固化分离**：

- `SET_PARAM` 只写 RAM 并置脏，不擦 Flash。台架调参一秒能改十几次，每次都擦会把参数区写坏，而且 400 ms 擦除会夹在控制流里触发超时。
- `PARAM_SAVE` 才落盘。Host 保存后轮询 `STORAGE_QUERY`，直到 `REPORT_STORAGE` 的 bit0（无脏数据）置位再断电。
- `PARAM_RESET` 恢复编译期默认值，只改 RAM 并置脏，不自动落盘；误按恢复在断电前不会造成损失。
- **恢复出厂只清参数区，不清标定区**。标定需要重新上标定台，不能和"调乱了 PID"一起被抹掉。

**范围校验**：PID 增益 Kp/Ki `0～10000`（×1000），Kd `0～5000`；轮距 / 轴距 50～2000 mm；轮径 10～500 mm；`counts_per_mm ×1000` 1000～100000；起转占空比 0～1000‰；方向只能 ±1；超时 10～5000 ms；最大线速度 10～500 mm/s；最大角速度 10～1500 mrad/s。越界返回 `0x01`，不静默截断。

**轮径与编码器比例的关系**：写 `0x02`（轮径）时，固件按 `counts_per_mm = 4680 / (π·d)` 重算四轮比例。因此必须先写轮径、再写 `0x50～0x53` 的单轮微调；反过来会丢失微调。

**Blob 格式**：当前 `APP_PARAMS_VERSION = 2`，参数区 60 B、标定区 80 B；头部为 `uint16 version + uint16 length`，其余字段按小端整数编码。加载时也接受 v1（参数 56 B、标定 28 B），便于升级兼容。

### 4.9 存储子系统

**分区地图**（W25Q64，8 MB）：

| 区域 | 起始 | 大小 | 写入者 | 内容 |
| --- | --- | --- | --- | --- |
| 引导元数据 | `0x000000` | 64 KB | bootloader | 分区表 A/B、引导控制块 A/B、启动计数 |
| 镜像槽 A | `0x010000` | 128 KB | bootloader | 完整 `.bin` + 镜像头 |
| 镜像槽 B | `0x030000` | 128 KB | bootloader | 完整 `.bin` + 镜像头 |
| 参数区 | `0x050000` | 64 KB | `appStorage` | A/B 各 32 KB，各 8 扇区 |
| 标定区 | `0x060000` | 64 KB | `appStorage` | A/B 各 32 KB |
| 崩溃现场 | `0x070000` | 64 KB | `appStorage` / HardFault | 16 槽 × 4 KB |
| 故障事件环 | `0x080000` | 128 KB | `appStorage` | 32 扇区 |
| 运行日志环 | `0x0A0000` | 896 KB | `appStorage` | 224 扇区 |
| 寿命统计 | `0x180000` | 64 KB | `appStorage` | A/B 两份 |
| 出厂信息 | `0x7F0000` | 64 KB | 产线工具 | 序列号、板号、生产日期 |

**通用记录头（32 B）**：

| 偏移 | 字段 | 说明 |
| --- | --- | --- |
| 0 | `magic` | 每类记录固定值（`PARA` / `CALI` / `CRAS` / `WEAR` 等） |
| 4 | `layout_ver` | 记录格式版本，当前 1 |
| 6 | `flags` | bit0 有效，其余按区域定义 |
| 8 | `seq` | 同一副本每次写 +1，A/B 比较用 |
| 12 | `payload_len` | 负载长度 |
| 16 | `crc32` | 覆盖 `magic..payload` |
| 20 | `ts` | 写入时刻，无 RTC 时为开机秒 |
| 24 | `writer` | 0=bootloader，1=app，2=产线工具 |
| 28 | 保留 | `0xFFFF` / `0xFFFFFFFF` |

**A/B 保存流程**：

1. 选择 `seq` 较小的非活动副本；
2. 擦除该副本的目标扇区（活动副本不动）；
3. 写记录头 + 负载；
4. 回读并重算 CRC32；不一致则丢弃本次写入，旧副本继续有效，上报 `0x0603`；
5. 成功后才更新 RAM 中的"当前活动副本"。

CRC32 覆盖 `seq` 与 `payload_len`，半写记录必然校验失败，不需要额外的"提交标志"。这是掉电安全的基础。

**环结构**：故障环 32 扇区、日志环 224 扇区，顺序追加，写满后覆盖最旧扇区。每条记录带 CRC32；迭代时 CRC 失败视为该条损坏。日志正文是文本，导出后可直接 `grep`；故障记录用故障码而不是文本。

**崩溃现场**：16 槽，每槽 4 KB。写顺序为擦除 → 写头 → 写负载 → **最后写提交 CRC**；读取不清除记录，主机解析确认后才发 `LOG_CLEAR`。HardFault 入口当前尚未接线，这是存储子系统最后一段待补的集成工作。

**单写者与队列**：

- 只有 `appStorage` 调用 `service_storage_*`。一次扇区擦除会占住 SPI 总线 45 ms～1 s，直接调用会卡住通信任务（触发 200 ms 超时）或控制任务（保持上一次占空比）。
- 队列传 24 B 描述符，不传 4 KB 负载。小于 `APP_STORAGE_STAGING_BYTES`（240 B）的记录复制到两个 staging 槽；更大的 blob 借用调用方缓冲区，同一时间只允许一笔借用，第二笔返回 BUSY。
- 完成回调在存储任务中执行，最多一次；入队时被拒绝则一次也不执行。回调不得阻塞、不得回调本模块。

**导出节流**：`REPORT_LOG` / `REPORT_CRASH` 以 20 Hz（50 ms）上限发送，避免挤掉 50 Hz 轮速帧与 100 ms 心跳；Host 静默 500 ms 自动停止导出。`REPORT_STORAGE` 不做周期上报，只在查询与保存结束时发。

**磨损管理**：启动计数追加写，1024 槽写满才擦一次；寿命记录每累计 16 次环擦除写一次；环擦除次数与日志已用字节通过 `REPORT_STORAGE` 回读。

### 4.10 安全设计

| 机制 | 实现 | 失效时的行为 |
| --- | --- | --- |
| 硬件急停 | 常闭触点接 EXTI，最高优先级，ISR 直接关闭 PWM | 断线即触发；不依赖任务调度 |
| 通信断流 | 控制任务判断 200 ms 无有效速度帧 | 主动停车，锁存 `0x0202` |
| 使能门控 | IDLE 下忽略速度与关节指令 | 未使能时电机不出力 |
| 急停锁存 | `ESTOP` 后拒绝使能，直到 `CLEAR_FAULT` | 防止自动恢复运动 |
| 输出限幅 | 整车速度、单轮速度、占空比、测试占空比均有限制 | 参数写错不会满扭矩 |
| 标定互锁 | 写几何 / 比例 / 方向要求底盘失能 | 行进中不改"尺子" |
| 栈溢出检测 | `configCHECK_FOR_STACK_OVERFLOW = 2` | 钩子置故障码、停电机 |
| 独立看门狗 | 平台接口已预留，IWDG 尚未启用 | 设计目标：任务卡死时复位 |
| 低压保护 | ADC 分压，5 点中值滤波，连续 5 次低于 6800 mV 告警 | 启动瞬时跌压倒不动判定；真低压约 0.7 s 后触发，禁止新任务 |

**急停为什么必须走中断**：安全动作不能排队。如果急停依赖任务调度，一旦控制任务被高优先级中断或错误配置卡住，PWM 就不会关闭。EXTI 路径只做寄存器写，不调用任何 RTOS API，因而不受内核临界区影响。

### 4.11 资源与时间预算

| 资源 | 预算 | 当前占用 / 说明 |
| --- | --- | --- |
| RAM | 48 KB | 三个任务栈各 1 KB；`appStore` 队列静态池 512 B；参数缓冲 2 × 80 B；协议重组 71 B；其余为 BSS 与 FreeRTOS 堆 |
| 内部 Flash | 256 KB，App ≤ 128 KB | 镜像槽按 128 KB 设计，构建脚本应拒绝超限 |
| 控制环单次 | < 1 ms | 编码器读取 + 四轮 PID + 里程计 |
| 协议单帧 | < 20 µs | 解析 + CRC + 分发 |
| 上报组帧 | < 200 µs / 周期 | 四帧合计不到 40 B |
| 存储任务 | 低优先级，可被抢占 | 擦除 45 ms 典型 / 400 ms 最大 |

**测量方法**：DWT 周期计数器 + GPIO 翻转。进入关键段拉高 GPIO，退出拉低，用逻辑分析仪或示波器测脉宽；同时在 `REPORT_STATUS` 中上报最大调度抖动（µs），Host 侧长期观察。预算值是设计目标，实测值以 `project-intro.md` 的指标表为准，未测的保持"待测"。

### 4.12 升级设计（bootloader，未实现）

```
上电
 └─► bootloader
      ├─ 读 RTC 备份寄存器 / 引导控制块，判断是否请求升级
      ├─ 校验分区表与镜像头
      ├─ 选择 active_slot，校验镜像 CRC
      ├─ 校验 App 栈顶指针是否落在 RAM 范围
      └─ 合法 → 关中断、复位外设、设 MSP、跳转 App
```

**镜像槽**：外部 W25Q64 两个 128 KB 槽，内部单 App 分区。写顺序为擦除目标槽 → 写载荷 → 回读整段校验 CRC → **最后写镜像头**。头在即载荷完整；头未写或 CRC 错，槽视为空。因此升级过程中任意时刻断电，两个槽至少有一个有效。

**回滚**：新镜像安装后 `boot_status=0`、`boot_attempts+1`；App 稳定运行 30 s 后写 `boot_status=1`。连续 3 次启动仍未确认，bootloader 从 `previous_slot` 重装上一版。**任何时刻不允许两个槽都无效**；装新镜像前先确认另一槽有效，否则拒绝升级。

---

## 5 ROS 2 详细设计

### 5.1 包划分与依赖

| 包 | 状态 | 职责 | 依赖 |
| --- | --- | --- | --- |
| `robot_interfaces` | 🚧 | 自定义 msg / srv / action | 仅 rosidl |
| `robot_bringup` | 🚧 | launch、参数、地图、URDF | 全部 |
| `robot_base` | ✅ | `/cmd_vel` ↔ 串口；串口 → `/odom`、状态、诊断 | rclcpp、geometry_msgs、nav_msgs、std_srvs、robot_interfaces |
| `robot_lidar` | ✅ | X2 驱动；`/scan`、`/lidar/points` | rclcpp、sensor_msgs |
| `robot_arm` | 📐 | 关节控制、逆解、抓取序列 | rclcpp、rclcpp_action、KDL / TRAC-IK、robot_interfaces |
| `robot_vision` | 📐 | 图像 → 检测框 → 3D 位姿 | rclcpp、cv_bridge、image_transport、tf2_ros、vision_msgs、OpenCV / RKNN |
| `robot_speech` | 📐 | 音频 → 文本 → 意图 | rclcpp、std_msgs、robot_interfaces、Vosk |
| `robot_task` | 📐 | 任务编排、状态机 | rclcpp、rclcpp_action、nav2_msgs、robot_interfaces |

**依赖方向**：`robot_task` 依赖其他包暴露的接口，其他包不依赖 `robot_task`。否则形成环，编译顺序和单测都会出问题。`robot_task` 是纯逻辑包，输入事件、输出动作，可以在 x86 上完整单测。

### 5.2 接口契约

**话题**：

| 话题 | 类型 | 方向 | 频率 | QoS |
| --- | --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | 订阅 | 由上层决定 | RELIABLE，depth 10 |
| `/odom` | `nav_msgs/Odometry` | 发布 | 实测 45.6 Hz | RELIABLE，depth 10 |
| `/scan` | `sensor_msgs/LaserScan` | 发布 | X2 约 6 Hz | BEST_EFFORT |
| `/lidar/points` | `sensor_msgs/PointCloud2` | 发布 | 随雷达 | BEST_EFFORT |
| `/camera/color/image_raw` | `sensor_msgs/Image` | 发布 | 30 Hz | BEST_EFFORT |
| `/joint_states` | `sensor_msgs/JointState` | 发布 | 20 Hz | RELIABLE |
| `/detections` | `vision_msgs/Detection2DArray` | 发布 | 10 Hz | BEST_EFFORT |
| `/robot_status` | `robot_interfaces/RobotStatus` | 发布 | 10 Hz | RELIABLE |
| `/system_event` | `robot_interfaces/SystemEvent` | 发布 | 事件 | RELIABLE |

**QoS 理由**：传感器数据用 `BEST_EFFORT`，宁可丢帧也不要堆积延迟；控制与状态用 `RELIABLE`，丢一条速度指令可能让底盘动作不连续。注意 QoS 兼容性有方向：发布 `BEST_EFFORT`、订阅 `RELIABLE` 收不到任何数据，但 `ros2 topic list` 照常显示，非常难发现。

**服务**：

| 服务 | 类型 | 语义 |
| --- | --- | --- |
| `/set_enable` | `std_srvs/SetBool` | ACK OK 表示已受理；等到 `REPORT_STATUS` 变 MANUAL 才放行速度 |
| `/clear_fault` | `robot_interfaces/ClearFault` | `fault_code=0xFFFF` 清全部；条件未消失时固件返回 `0x02` |
| `/get_version` | `robot_interfaces/GetVersion` | 接口已定义；protocol v1 无对应命令，当前不注册 |

**动作**：

| 动作 | 类型 | goal | feedback | result |
| --- | --- | --- | --- | --- |
| `/grasp` | `robot_interfaces/action/Grasp` | 目标类别、可选指定位姿 | 阶段、进度 0～100 | 成功 / 失败原因码 |

导航直接使用 Nav2 的 `NavigateToPose`，不重复定义。抓取用动作而不是服务：抓取是数秒到十几秒的长任务，需要反馈阶段、需要能中途取消。

**TF 树**：

```
odom ──► base_link ──► laser_link
                    ├─► camera_link ──► camera_color_optical_frame
                    └─► arm_base ──► joint1 ... joint6 ──► gripper
```

`robot_base` 发布 `odom → base_link`；`robot_lidar` 不发布 TF，URDF 提供 `base_link → laser_link` 静态变换；手眼标定提供 `camera_link → arm_base` 或等价的末端变换。

### 5.3 robot_base（已实现）

#### 5.3.1 分层

| 层 | 文件 | 职责 | 约束 |
| --- | --- | --- | --- |
| 协议编解码 | `protocol.hpp` / `protocol.cpp` | 帧格式、CRC、全部上下行 payload、名称映射 | 纯 C++，不依赖 rclcpp / 串口 |
| 链路 | `stm32_link.hpp` / `stm32_link.cpp` | 独占 fd、RX 线程、帧解析、ACK 匹配、同 SEQ 重发、链路健康 | 只有它能 `open()` 串口 |
| 串口适配 | `serial_io.hpp` / `termios_serial_io.cpp` | termios 115200 8N1、非阻塞读、写超时 | 通过接口注入，测试用 Fake |
| 节点 | `base_bridge.cpp` | 定时器、话题、服务、单位换算、诊断 | 只处理底盘命令与上报 |
| 辅助 | `heading_unwrapper.hpp` / `cmd_vel_monitor.*` | 航向解卷绕、不支持分量限频告警 | 无 ROS 依赖，便于单测 |

拆成四层而不是一个节点文件，是为了机械臂接入时把 `Stm32Link` 提升为独立 transport 节点，`robot_base` 与 `robot_arm` 共用一条串口，避免两个包争抢 fd。

#### 5.3.2 线程与执行器

| 执行体 | 数量 | 职责 | 共享数据保护 |
| --- | --- | --- | --- |
| RX 线程 | 1 | `poll()` 读字节、50 ms 半帧复位、解析、分发 ACK / 上报 | `parser_` 仅该线程访问；最后帧时间与 `has_frame_` 为原子量 |
| 默认回调组 | 1 组 | `/cmd_vel` 订阅、20 ms 速度定时器、1 s 链路定时器 | `cmd_mutex_` 保护最新命令；状态与故障为原子量 |
| 服务回调组 | Reentrant | `/set_enable`、`/clear_fault`，内部阻塞等 ACK | ACK 等待表由互斥量 + 条件变量保护 |
| `MultiThreadedExecutor` | 4 线程 | 服务等 ACK 时不饿死速度定时器 | — |

一次需要 ACK 的命令最坏等待 4 × 200 ms = 800 ms。单线程执行器下，服务回调会把 20 ms 速度定时器堵住，超过固件 200 ms 超时，底盘会自己停车并锁存 `0x0202`。4 线程保证即使两个服务同时在等 ACK，仍有线程跑定时器。

#### 5.3.3 下行速度流

```
/cmd_vel ──► on_cmd_vel
              ├─ 检查 NaN / Inf，非法值整条丢弃
              ├─ CmdVelMonitor：linear.y / z、angular.x / y 超阈值时限频告警
              └─ 保存 linear.x、angular.z、到达时间

每 20 ms ──► on_velocity_tick
              ├─ 端口未打开或链路失联？不发
              ├─ 未使能或命令超过 0.5 s？目标 = 0
              ├─ v = linear.x × 1000 (mm/s)，w = angular.z × 1000 (mrad/s)
              └─ SET_VELOCITY（无 ACK）──► UART
```

**持续发零速而不是停发**：停发会让固件在 200 ms 后锁存 `0x0202`，恢复时必须先清故障再重新使能；持续发零速不会触发故障，恢复无缝。未使能时也持续发零速，保证"使能后第一帧"不会因链路空闲而延迟。

#### 5.3.4 上行、解卷绕与 twist

```
UART ──► RX 线程 ──► FrameParser ──► on_frame
                                      ├─ 0xC1 ODOM   → 解卷绕 → /odom + TF
                                      ├─ 0xC3 STATUS → /robot_status
                                      ├─ 0xC4 EVENT  → /system_event（只通知）
                                      └─ 其他         → 忽略（arm / wheel / storage 不归本包）
```

**航向解卷绕**：

```
raw = theta_centi_deg × 0.01° → rad
首帧：continuous = raw
后续：delta = atan2(sin(raw - last_raw), cos(raw - last_raw))
      continuous += delta
      last_raw = raw
```

固件把航向归一到 `(-π, π]`，直接转 TF 会在 ±180° 处跳 358°。`/odom` 的 pose 与 TF 使用连续值，原始值只进日志。**只在 MCU 进入 INIT（重启）时重置**；串口断开 / 恢复不重置，因为 MCU 一直在积分，重置会造成 yaw 跳变。

**twist 反算**：`REPORT_ODOM` 没有角速度，只有左右侧速度：

```
linear.x  = (vl + vr) / 2000.0                    [m/s]
angular.z = (vr - vl) / 1000.0 / wheel_separation_m  [rad/s]
```

`wheel_separation_m` 默认 0.165 m，对应固件轮距 165 mm。v1 没有 `GET_PARAM`，改标定后必须人工同步 yaml，否则位姿（固件积分）与 twist（ROS 计算）会互相矛盾，EKF 缓慢漂移。

**协方差**：`REPORT_ODOM` 观测不到的自由度必须填 `1e6`，不能留 0。`robot_localization` 把 0 读作"该量测完全准确"，会让轮式里程计的 z / roll / pitch 和 `linear.y` 盖过 IMU 的真实估计，且不报任何错。x、y、yaw 保持小幅占位值，接入定位前按实测重标。

#### 5.3.5 服务与 ACK

```
服务客户端 ──► on_set_enable / on_clear_fault
                 ├─ 链路不可用 → 立即失败并发布诊断
                 ├─ 分配 SEQ、组帧、注册等待项
                 └─ 最多 4 次（首发 + 3 次重发，同 SEQ）：
                      发送 ──► RX 线程收到 ACK（cmd OR 0x80，[seq, result]）
                                └─ 条件变量唤醒 → 返回 result
                      200 ms 超时 → 重发
                 超限 → success=false + /diagnostics ERROR
```

`SET_VELOCITY` 不进入这条路径，没有等待队列。固件按命令字保存 `last_seq` 与 1 s 窗口，重复 SEQ 回缓存结果，不重复执行。

**使能门控**：

```
REPORT_STATUS.state == MANUAL (2)  → enabled_ = true
其他状态（INIT / IDLE / AUTO / ESTOP / FAULT） → enabled_ = false
```

`/set_enable true` 的 ACK 只表示受理；节点等下一帧状态变 MANUAL 才放行 `/cmd_vel`（实测约 27 ms）。`/set_enable false` 成功后立即置 `enabled_=false`，不等状态帧。`CLEAR_FAULT` 成功后固件回 IDLE，不自动使能，节点也不替上层重新使能。

#### 5.3.6 链路监控与重连

| 状态 | 判定 | 行为 |
| --- | --- | --- |
| 端口未打开 | `is_open()==false` | 每 2 s 尝试打开，失败限频告警 |
| 打开但从未收到帧 | `has_frame()==false` | 不算在线；超时后主动关闭重开 |
| 收到过帧但超时 | 当前时间 − 最后帧 > 0.5 s | 诊断 ERROR，停止速度下发 |
| USB 拔出 | `poll()` 返回 `POLLHUP` / `POLLERR` 或 `read()<0` | RX 线程退出，进入重开流程 |
| 重新插入 | udev 重建 `/dev/robot_stm32` | 自动重开，收到第一帧后恢复 |

不能只靠 `is_open()`：USB 拔出后 fd 可能仍有效，`read()` 一直返回 0；只看 open 标志会永远不重连。实测遇到后增加了 `POLLHUP` / `POLLERR` 检测和"僵尸 fd 超时关闭"两条路径。

#### 5.3.7 诊断、日志与参数

`/diagnostics` 以 1 Hz 发布：链路等级（OK / ERROR）、端口、波特率、`rx_frames`、`crc_errors`、`tx_frames`、`ack_timeouts`、`last_frame_age_ms`、状态、故障码、电池、温度、调度抖动。ACK 超限时额外发布一条 ERROR。

日志只记录状态迁移、故障变化、链路上下线，不记录 50 Hz 数据，避免刷屏。

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `port` | `/dev/robot_stm32` | udev 固定名；未安装规则时为 `/dev/ttyUSB0` |
| `baud` | `115200` | 8N1，无流控 |
| `wheel_separation_m` | `0.165` | 必须与固件轮距标定一致 |
| `velocity_period_ms` | `20` | 速度帧周期 |
| `cmd_vel_timeout_s` | `0.5` | 超时后发零速 |
| `link_timeout_s` | `0.5` | 无帧判定失联 |
| `ack_timeout_ms` | `200` | 需 ACK 命令 |
| `ack_retries` | `3` | 重发次数，同 SEQ |
| `publish_tf` | `true` | 是否发布 `odom → base_link` |
| `odom_frame_id` / `base_frame_id` | `odom` / `base_link` | 坐标系名 |
| `warn_unsupported_cmd_vel` | `true` | 不支持分量告警开关 |
| `unsupported_cmd_vel_warn_period_s` | `5.0` | 告警限频 |
| `odom_pose_covariance_xy` | `0.05` | 占位，待标定 |
| `odom_pose_covariance_yaw` | `0.02` | 占位，待标定 |
| `odom_twist_covariance_xy` | `0.05` | 占位，待标定 |
| `odom_twist_covariance_yaw` | `0.05` | 占位，待标定 |

参数非法时节点拒绝启动，不做静默截断。

#### 5.3.8 生命周期

`BaseBridge` 析构函数第一件事是 `link_->close()`：先停 RX 线程，再销毁发布器与 TF 广播器。否则退出瞬间 RX 线程可能通过已析构的发布器发布，表现为段错误。实测遇到过，已修。

#### 5.3.9 测试与实测

| 套件 | 用例 | 覆盖 |
| --- | --- | --- |
| `test_protocol` | 35 | CRC 自检、协议测试向量、任意分块、坏 CRC / LEN 重同步、全部 payload 编解码、帧长边界 |
| `test_heading_unwrapper` | 5 | 跨 ±180°、负向跨接、连续绕三圈、reset |
| `test_cmd_vel_monitor` | 5 | 不支持分量、阈值、5 s 限频、关闭开关 |
| `test_stm32_link` | 5 | ACK 匹配、同 SEQ 重发 4 次、断流拒发、CRC 计数、50 ms 半帧复位 |

合计 4 个测试目标、50 个 gtest 用例。PC 直连实测：`/odom` 45.6 Hz（中位间隔 20.3 ms，p95 29.2 ms），`/robot_status` 9.9 Hz；前进 0.1 m/s × 1 s 得到 x +0.145 m；转向 +0.5 rad/s × 0.6 s 得到 yaw +0.465 rad；断链约 0.7 s 后诊断 ERROR，重连自动恢复。

### 5.4 robot_lidar（已实现）

**X2 协议要点**：

| 项 | 值 |
| --- | --- |
| 串口 | 115200 8N1，3.3 V TTL，无流控 |
| 帧头 | `AA 55` |
| 包结构 | `AA 55 | CT | LSN | FSA(2) | LSA(2) | CS(2) | S1(2)..Sn(2)` |
| 包长 | `10 + 2 × LSN` |
| 距离 | `Si / 4` mm |
| 一级角度 | `(raw >> 1) / 64` 度；bit0 是固定校验位 |
| 校验 | 16 位字异或，不含 CS 自身 |
| 起始包 | `CT bit0 = 1`，`F = CT[7:1] / 10` Hz |
| 二级角度修正 | `atan(21.8 × (155.3 − d) / (155.3 × d))`，d 单位 mm |
| 电机 | USB 转接板用 DTR 控制，上电待机，DTR 拉高才转 |

**扫描组装**：

- 以起始包为分圈依据；跨圈包（`LSA < FSA` 且 `FSA > 270°`、`LSA < 90°`）按 `+360°` 计算间隔。
- 每圈按 `scan_bins = 720` 分箱，0.5°/bin；缺失 bin 填 `inf`，不填 0。
- `angle_min = -π`，`angle_max = -π + (N−1)×increment`，符合 REP-117。
- `time_increment = scan_time / scan_bins`；`header.stamp` 打在本圈中点，把最大时间误差减半。
- 角度方向按 REP-103 取反（X2 手册顺时针为正），`invert_angle` 可关闭。

**参数**：`port`、`baud`、`frame_id`、`angle_offset_deg`、`invert_angle`、`range_min_m` / `range_max_m`、`scan_bins`、`publish_scan`、`publish_pointcloud`、`motor_dtr`、`scan_frequency_hz`、`timer_period_ms`、`reconnect_period_s`。全部在 `robot_bringup/config/lidar.yaml`。

**测试与实测**：15 个 gtest，覆盖 16 位字校验、帧头、角度修正、跨圈、坏包重同步、任意分块。PC 实机 8 s 收到 49 帧，6.11 Hz；`/scan` 720 bin、312 个有限点、范围 0.164～4.077 m。

**已知限制**：未做强度 / 干扰位过滤；未做 `ignore_array` 与角度裁剪；不控制电机转速；安装方向与 `angle_offset_deg` 待现场标定。

### 5.5 robot_vision（M6 / M7 设计）

**目标**：从深度相机图像中得到目标类别、2D 框与相机坐标系下的 3D 位姿，再经 TF2 转到机械臂基座。

```
/camera/color/image_raw ──► vision_detector ──► /detections
/camera/depth/image_raw ──►        │
                                   ▼
                           pose_estimator
                           ├─ 深度图取目标区域中值 / 点云聚类
                           ├─ 相机内参反投影
                           ├─ TF2: camera_link → arm_base
                           └─ 输出抓取位姿（PoseStamped）
```

**节点结构**：

| 节点 | 执行器 | 回调组 | 说明 |
| --- | --- | --- | --- |
| `vision_detector` | 多线程 | `ReentrantCallbackGroup` | 推理耗时，不能阻塞图像回调 |
| `pose_estimator` | 单线程 | 默认 | 轻量几何计算，可与检测合并为组件 |

**模型链路**：数据采集 → 标注 → YOLO 训练 → 导出 ONNX → RKNN INT8 量化 → RK3566 NPU 推理。输入尺寸、类别表、置信度阈值、NMS 阈值全部进参数，不硬编码。目标 ≥ 15 FPS，单帧推理 ≤ 50 ms。

**标定**：

- 相机内参与畸变系数：`config/calibration/camera_*.yaml`，带设备序列号与日期。
- 手眼标定：`AX = XB`，输出 `camera_link → arm_base` 或末端变换；结果写 `config/calibration/hand_eye_*.yaml`。
- 换相机或换安装位置必须重新标定；旧标定文件带序列号，加载时校验。

**失败处理**：无检测 → 返回 `no_target`；置信度低 → 可重拍；深度无效 → 返回 `depth_invalid`；TF 不可用 → 返回 `tf_unavailable`；视觉节点失效只禁止抓取，导航继续可用。

### 5.6 robot_arm（M8 设计）

**职责**：把抓取位姿转成六关节目标，做逆解、限位、奇异与碰撞检查，再通过串口下发；同时发布 `/joint_states`，提供 `/grasp` 动作。

```
/grasp goal ──► arm_action_server
                  ├─ 目标位姿 → 逆解（KDL / TRAC-IK）
                  ├─ 多解筛选：关节限位、奇异、碰撞、行程最短
                  ├─ 轨迹插值（关节空间，梯形 / S 曲线）
                  ├─ SET_ARM_JOINTS ──► MCU
                  ├─ 20 Hz REPORT_ARM → /joint_states
                  └─ feedback：阶段 + 进度；支持 cancel
```

**安全约束**：

- 逆解无解或超限 → 返回 `ik_fail` / `limit`，不发送部分关节。
- 运动过程中收到 ESTOP / FAULT → 立即停止下发，保持当前位置。
- 通信超时 → 机械臂保持位置（底盘停车，机械臂不跟随运动）。
- 速度与加速度按关节限幅，不做阶跃目标。

**与 MCU 的边界**：当前 `robot_base` 不处理 arm 命令与上报。接入时把 `Stm32Link` 提升为独立 transport 节点，`robot_base` 与 `robot_arm` 通过它收发；一个 UART 仍然只有一个写者。

### 5.7 robot_speech（M9 设计）

```
麦克风阵列 ──► Vosk 流式识别 ──► 文本
                                  ├─ 唤醒词检测
                                  ├─ 命令词 / 语法解析
                                  └─ 意图 JSON
                                       ├─ {"intent":"navigate","place":"A"}
                                       └─ {"intent":"grasp","object":"cup"}
```

- Vosk 完全离线，中文小模型 40～50 MB，流式输出；唤醒词与命令词用语法文件约束，降低误识别。
- 意图解析输出结构化命令，由 `robot_task` 转成 `NavigateToPose` 或 `/grasp`。
- 识别失败 / 置信度低 → 请求重复；语音节点失效 → 界面或遥控接管，不影响运动。
- 目标识别率 ≥ 90%（安静环境），待实测。

### 5.8 robot_task（设计）

**状态机**：

```
IDLE ──收到抓取任务──► NAVIGATE ──到达──► DETECT ──找到──► PLAN
  ▲                       │失败              │未找到           │无解
  │                       ▼                  ▼                ▼
  └──────────────────── FAILED ◄──── CLOSE / LIFT / PLACE 各阶段失败
```

完整状态：`IDLE`、`NAVIGATE`、`DETECT`、`PLAN`、`APPROACH`、`DESCEND`、`CLOSE`、`LIFT`、`PLACE`、`FAILED`。

**每个状态必须有三样东西**：

| 项 | 要求 | 例子 |
| --- | --- | --- |
| 超时 | 到点判失败 | NAVIGATE 60 s、DETECT 10 s、GRASP 15 s |
| 失败原因码 | 不是布尔值 | `no_target` / `ik_fail` / `collision` / `timeout` / `not_grasped` |
| 重试策略 | 按类型区分 | 没检测到可重拍 2 次；IK 无解换站位，不原地重试 |

**实现要点**：

- 动作客户端异步调用，`MultiThreadedExecutor` + `ReentrantCallbackGroup`，避免同步调用服务自锁。
- 支持取消：用户喊停或上层取消时，取消当前动作并回到 IDLE。
- 状态迁移与失败原因写入 `/system_event` 与日志，便于复盘。
- 纯逻辑包，PC 上可用假动作服务器做完整单测。

### 5.9 执行器与并发

| 节点 | 执行器 | 回调组 | 理由 |
| --- | --- | --- | --- |
| `base_bridge` | 单线程 | 默认 + 服务 Reentrant | 50 Hz 耗时短；服务等 ACK 需要并发 |
| `vision_detector` | 多线程 | Reentrant | 推理耗时，不能阻塞其他回调 |
| `task_manager` | 多线程 | Reentrant | 会同步调用其他服务 / 动作 |
| `lidar` | 单线程 | 默认 | 10 ms 读串口，轻量 |

**已知陷阱**：在 `MutuallyExclusiveCallbackGroup` 里同步调用服务会自锁——客户端占住组等响应，服务端回调永远进不来。表现为节点卡死但 CPU 不高，`ros2 node info` 看不出问题。

### 5.10 参数、标定与配置

| 类型 | 位置 | 生效 | 版本管理 |
| --- | --- | --- | --- |
| 节点参数 | `robot_bringup/config/*.yaml` | 启动加载，`ros2 param set` 热改 | 入版本库 |
| 标定结果 | `config/calibration/*.yaml` | 启动加载 | 带设备序列号与日期 |
| 地图 | `robot_bringup/maps/` | 启动加载 | 入版本库 |
| 固件参数 | MCU 参数区 / 标定区 | `SET_PARAM` 立即生效，`PARAM_SAVE` 固化 | 通过工具导出备份 |

**规则**：任何影响行为的数字都不许硬编码在业务代码里。标定文件必须写设备序列号和标定日期——换了相机还用旧标定，是视觉抓取项目里最隐蔽的错误来源。

### 5.11 部署与运维

| 项目 | 做法 |
| --- | --- |
| 进程托管 | systemd，`Restart=always` + `WatchdogSec=10` |
| 启动顺序 | `After=` 控制依赖，避免节点起来找不到串口 / 地图 |
| 设备名 | udev 规则 `99-robot-stm32.rules`、`99-robot-lidar.rules` 建立 `/dev/robot_stm32`、`/dev/robot_lidar` |
| 权限 | 用户加入 `dialout` 组；停用并 mask `brltty-udev.service`，避免它抢占 CH340 |
| 日志 | 分级写文件 + logrotate 轮转，防止写满 SD 卡 |
| 版本追溯 | `git describe` 编进二进制，启动时打印 |
| 单实例 | 启动脚本保证只拉起一个 `base_bridge`；同名节点不互斥 |

**交叉编译**：sysroot 必须来自目标板本身（拷贝板上的 `/usr/lib`、`/usr/include`），否则编译通过、上板报 `GLIBC_2.xx not found`。

### 5.12 测试策略

| 层级 | 内容 | 工具 |
| --- | --- | --- |
| L1 单元测试 | 协议编解码、解卷绕、链路 ACK、雷达协议、任务状态机 | gtest / 纯 C 测试 |
| L2 集成测试 | 节点间话题 / 服务 / 动作联调，录包回放 | launch_testing、rosbag2 |
| L3 台架测试 | 真实 MCU + 真实电机 / 舵机，PC 或板端跑 ROS | `tools/bench/proto_loop.py` |
| L4 实车测试 | 建图、导航、抓取、语音全链路 | 场景脚本 |

当前已覆盖：`robot_base` 50 个 gtest、`robot_lidar` 15 个 gtest、固件 12 个 L1 测试文件（协议、服务、存储、应用）。待补：ROS 2 节点级集成测试、板端 HIL、长时稳定性。

---

## 6 关键流程时序

### 6.1 速度闭环

```
Nav2 / 遥控        robot_base                  STM32                      电机
    │                  │                          │                          │
    │ /cmd_vel 50Hz    │                          │                          │
    ├─────────────────►│ 保存最新命令             │                          │
    │                  │                          │                          │
    │          20ms 定时器                          │                          │
    │                  ├─ SET_VELOCITY ──────────►│ 解析、校验、更新目标      │
    │                  │                          │ 10ms 控制环：读编码器     │
    │                  │                          ├─ 四轮 PID ──────────────►│ PWM
    │                  │                          │ 积分里程计               │
    │                  │◄──── REPORT_ODOM 50Hz ───┤                          │
    │                  ├─ 解卷绕、换算              │                          │
    │                  ├─ /odom + TF               │                          │
    │                  │                          │                          │
    │                  │  200ms 无有效速度帧        │                          │
    │                  │                          ├─ 目标清零、PWM 关闭       │
    │                  │◄──── REPORT_EVENT ───────┤  锁存 0x0202             │
```

**时延预算**：`/cmd_vel` 回调到串口发出 ≤ 5 ms；MCU 收到到下一个控制节拍 ≤ 10 ms；控制计算 < 1 ms；上报到 `/odom` 发布 ≤ 2 ms。端到端命令到轮速响应目标 ≤ 20 ms。

### 6.2 使能与失能

```
上层           robot_base                 STM32
 │ /set_enable(true) │                     │
 ├──────────────────►│ SET_ENABLE(seq=n)   │
 │                   ├────────────────────►│ 校验状态，置 MANUAL
 │                   │◄──── ACK(n, 0) ─────┤
 │                   │ 等待 REPORT_STATUS   │
 │                   │◄──── STATUS(MANUAL) ─┤ 约 27 ms
 │                   │ enabled_ = true      │
 │                   │ 开始转发 /cmd_vel    │
```

失能：ACK 成功后立即 `enabled_=false`，不等状态帧。清故障：`CLEAR_FAULT` → IDLE，**不自动使能**。

### 6.3 一次抓取

```
语音 / 界面 ──► robot_task
                  ├─ action: NavigateToPose ──► Nav2 ──► 到达
                  ├─ 相机取图 ──► YOLO / RKNN ──► 目标框 + 深度
                  ├─ TF2: camera_link → arm_base（手眼标定）
                  ├─ 逆解 ──► 六关节角（限位 / 奇异 / 碰撞检查）
                  ├─ SET_ARM_JOINTS ──► MCU 插补执行
                  ├─ 夹爪闭合 ──► 位置 + 电流双判据（当前无夹爪硬件）
                  └─ 失败 ──► 按原因码重试或上报
```

每个阶段的超时、原因码、重试次数见 5.8 节。

### 6.4 故障与恢复

```
故障发生
 ├─ WARN：记录 + 上报，继续运行
 ├─ ERROR：停对应子系统，进入 FAULT，拒绝运动指令
 └─ FATAL：全停，进入 ESTOP
        │
Host 读取 REPORT_STATUS / REPORT_EVENT
        │
Host 发 CLEAR_FAULT
        ├─ 条件仍存在 → 0x02，保持故障
        └─ 条件消失 → IDLE
                │
        Host 重新 SET_ENABLE → MANUAL
```

**状态是权威，事件只是通知**：`REPORT_EVENT` 丢失不影响 Host 判断，因为 `REPORT_STATUS` 每 100 ms 带当前故障码。

### 6.5 时间预算汇总

| 环节 | 目标 | 说明 |
| --- | --- | --- |
| MCU 控制环 | 10 ms 周期，< 1 ms 计算 | 四轮 PID + 里程计 |
| MCU 协议单帧 | < 20 µs | 解析 + CRC + 分发 |
| MCU 上报 | < 200 µs / 周期 | 四帧合计 < 40 B |
| 固件指令超时 | 200 ms | 断流停车 |
| ROS `/cmd_vel` → 串口 | ≤ 5 ms | 回调 + 定时器 |
| ROS 视觉单帧 | ≤ 50 ms | RKNN 推理 |
| 检测 → 抓取指令 | ≤ 100 ms | 位姿 + 逆解 + 下发 |
| 整机导航到点 | 待测 | 受地图、速度、避障影响 |

---

## 7 可靠性与安全设计

### 7.1 失效模式与对策

| 失效 | 检测 | 对策 | 用户可见 |
| --- | --- | --- | --- |
| 串口线断开 | 200 ms 无有效帧 | MCU 停车，锁存 `0x0202` | 底盘停止；Host 诊断 ERROR |
| USB 拔出 / tty 漂移 | `POLLHUP`、无帧、udev 符号链接 | Host 自动关 fd、重开；udev 固定设备名 | 自动恢复，日志记录 |
| 帧损坏 | CRC16 | 丢该帧，继续解析；10 帧告警 | `0x0201` WARN |
| 关键命令丢失 | ACK 超时 | 同 SEQ 重发 3 次 | 服务失败 + 诊断 |
| Host 崩溃 | MCU 200 ms 超时 | 主动停车 | 底盘停止 |
| 急停按下 | EXTI | 直接关 PWM，锁存 `0x0501` | 全停，需清故障 |
| 电机过流 | 驱动反馈（规划） | 该轮停止，上报 `0x0101` | 单轮失效，其他继续 |
| 编码器异常 | 跳变 / 长时间无变化（规划） | 该轮停止，上报 `0x0102` | 单轮失效 |
| 关节越限 | 目标角校验 | 拒绝命令，保持上一目标 | ACK `0x01` |
| 电池低压 | 5 点中值 < 6800 mV，连续 5 次（10 Hz 采样） | WARN，禁止新任务 | 状态上报；电机启动跌落不误报 |
| 参数区损坏 | magic / CRC / 版本 | 用编译期默认值，上报 `0x0601` | 参数未固化 |
| 标定区损坏 | magic / CRC | 用编译期标定，上报 `0x0602` | 里程计精度不保证 |
| Flash 写失败 | 回读 CRC | 保留旧副本，上报 `0x0603` | 保存失败 |
| 栈溢出 | FreeRTOS 钩子 | 置故障、停电机 | 复位 / 故障 |
| HardFault | 异常入口 | 写崩溃现场、复位 | 重启后可导出 |

### 7.2 降级矩阵

| 故障 | 底盘 | 机械臂 | 导航 | 抓取 | 语音 |
| --- | --- | --- | --- | --- | --- |
| 视觉失效 | 可用 | 可用 | 可用 | 禁止 | 可用 |
| 语音失效 | 可用 | 可用 | 可用 | 界面 / 遥控 | 不可用 |
| 定位丢失 | 可用 | 保持 | 停止，原地等待 | 禁止 | 可用 |
| 通信超时 | 停车 | 保持位置 | 停止 | 中止 | 可用 |
| 机械臂故障 | 可用 | 停止 | 可用 | 禁止 | 可用 |
| 电池低压 | 可用（限速） | 可用 | 返航 | 禁止新任务 | 可用 |
| 存储失效 | 可用 | 可用 | 可用 | 可用 | 可用（参数不保存） |

**原则：局部失效不扩散。** 视觉挂了不该影响机器人走路，这是分层带来的直接好处。

### 7.3 数据完整性

| 数据 | 保护 | 掉电后果 |
| --- | --- | --- |
| 参数 / 标定 | A/B 双副本 + CRC32 + 递增 seq | 旧副本仍有效，最多丢失本次保存 |
| 日志 / 故障环 | 每条记录 CRC32，顺序追加 | 最多丢失最后一条未写完的记录 |
| 崩溃现场 | 提交 CRC 最后写 | 未提交的记录视为无效 |
| 固件镜像 | 载荷 CRC + 镜像头最后写 | 两槽至少一个有效 |
| 协议帧 | CRC16 + 长度校验 | 坏帧丢弃，不影响后续帧 |

### 7.4 可观测性

| 数据 | 通道 | 频率 | 用途 |
| --- | --- | --- | --- |
| 实时状态 | `REPORT_STATUS` / `/robot_status` | 10 Hz | 状态、故障、电池、温度、抖动 |
| 里程计 | `REPORT_ODOM` / `/odom` | 50 Hz | 定位、导航 |
| 轮速 / 占空比 | `0xC6` / `0xC7` | 50 Hz | 调参、判断限幅 |
| 事件 | `REPORT_EVENT` / `/system_event` | 事件 | 故障产生 / 清除 |
| 诊断 | `/diagnostics` | 1 Hz | 链路、CRC、ACK 超时 |
| 日志 | Flash 日志环 + `REPORT_LOG` | 持续 / 导出 | 现场排障 |
| 崩溃现场 | Flash 16 槽 + `REPORT_CRASH` | 崩溃时 / 读取 | 定位 HardFault |

---

## 8 测试与验证

### 8.1 测试分层

| 层级 | 环境 | 目标 | 通过标准 |
| --- | --- | --- | --- |
| L1 单元 | PC，无硬件 | 纯逻辑：协议、PID、状态机、存储 | 退出码 0，用例全过 |
| L2 集成 | PC + 仿真 / 录包 | 节点接口、QoS、TF、动作 | 话题频率与数据正确 |
| L3 台架 | 真实 MCU + 电机 / 舵机 | 驱动、时序、保护、标定 | 指标在误差内 |
| L4 实车 | 完整机器人 | 建图、导航、抓取、语音 | 场景任务完成 |

### 8.2 协议一致性

联调前必须先用协议测试向量跑通，避免"协议 bug"与"硬件 bug"混在一起：

1. CRC 对 `"123456789"` 结果为 `0x29B1`。
2. `SET_VELOCITY(v=300, w=500, seq=1)` → `AA 55 06 01 01 2C 01 F4 01 72 3F`。
3. `SET_ENABLE(1, seq=2)` → `AA 55 03 04 02 01 9F B5`。
4. `ESTOP(seq=3)` → `AA 55 02 06 03 39 38`，`LEN=2` 是最短合法帧。
5. 坏 CRC、`LEN=1`、`LEN=255` 必须丢弃且不越界。
6. 逐字节喂入与一次性喂入结果一致；随机字节流 10 分钟不崩溃。
7. 半帧中断 100 ms 后能自动恢复。

### 8.3 单元测试清单

**固件 L1**（`tests/unit`）：

| 测试 | 覆盖 |
| --- | --- |
| `test_protocol_codec` | 帧编解码、CRC、测试向量、坏帧重同步 |
| `test_platform_middleware` | 环形缓冲、CRC、滤波、PID |
| `test_service_proto` | 解析、分发、ACK、SEQ 去重（内存管道） |
| `test_service_chassis` | 逆解、超时、急停、PID、里程计（电机桩） |
| `test_service_key` | 消抖、短按 / 长按 |
| `test_platform_storage` | 磨损均衡、日志环（RAM NOR 仿真） |
| `test_service_storage` | A/B blob、环、崩溃槽（8 MiB 仿真） |
| `test_bsp_flash_w25q64` | 命令、CS、BUSY、WEL、JEDEC |
| `test_app_state` | 状态迁移 |
| `test_app_fault` | 故障仲裁与降级 |
| `test_app_params` | 参数序列化、范围、默认值 |
| `test_app_storage` | 请求队列、导出、单写者 |

**ROS 2**：`robot_base` 50 个 gtest、`robot_lidar` 15 个 gtest；`tools/host_gui/test_protocol.py` 覆盖上位机 Python 编解码。

### 8.4 台架与实车测试

| 项 | 方法 | 判据 |
| --- | --- | --- |
| 轮速闭环 | 给定 200 mm/s，测四轮速度 | 稳态误差 < 5%，无持续振荡 |
| 死区标定 | 开环占空比扫描，记录起转点 | 正反向分别记录 |
| 里程计 | 直行 1 m、原地转 360° | 距离误差 < 2%，角度误差 < 2° |
| 超时停车 | 拔串口，测停车时间 | < 200 ms |
| 急停 | 按下急停，测 PWM 关闭 | 不等任务调度 |
| 关节限位 | 逐关节给越限角 | 拒绝且保持上一目标 |
| 舵机跟随 | 阶跃 / 斜坡目标 | 无过冲、无堵转 |
| 断链重连 | unbind / rebind USB | 自动恢复，无段错误 |
| 参数掉电 | 保存后断电重启 | 值保持，CRC 正确 |
| 日志导出 | 导出 1000 条 | 20 Hz 封顶，无通信超时 |

### 8.5 验收指标

| 指标 | 目标 | 实测 |
| --- | --- | --- |
| 建图闭合误差 | < 10 cm | 待测 |
| 导航到达精度 | < 10 cm | 待测 |
| 视觉推理帧率 | ≥ 15 FPS | 待测 |
| 检测 mAP50 | ≥ 0.85 | 待测 |
| 抓取成功率 | ≥ 80%（20 次） | 待测 |
| 手眼标定误差 | < 10 mm | 待测 |
| 语音识别率 | ≥ 90%（安静） | 待测 |
| 通信超时停车 | < 200 ms | 待测 |
| `/odom` 频率 | 50 Hz 设计 | 45.6 Hz（PC 直连） |
| `/scan` 频率 | 10 Hz 设计 | 6.11 Hz（X2 实机） |

> 未实测的指标保持"待测"，不填估计值。实测数据统一回填 `project-intro.md`。

---

## 9 已知限制与演进路线

| 项 | 现状 | 影响 | 计划 |
| --- | --- | --- | --- |
| 机械臂总线舵机 | 当前为 GPIO 软件 PWM，总线方案未实现 | 无负载 / 温度反馈，堵转与过温保护缺失 | 新增 UART5 半双工 BSP，保持平台接口不变 |
| 夹爪 | 无硬件 | `SET_GRIPPER` 返回失败 | 接入后实现位置 + 电流双判据 |
| HardFault 现场 | 存储服务已支持，入口未接线 | 崩溃后无法自动留现场 | 在 `HardFault_Handler` 中写最小上下文 |
| bootloader / OTA | 设计完成，工程为空 | 无法在线升级，只能烧录 | 按 `flash-layout.md` 实现双槽与回滚 |
| 独立看门狗 | 平台接口预留，IWDG 未启用 | 任务卡死时只能靠外部断电 | 启用 IWDG，控制任务喂狗 |
| `/get_version` | protocol v1 无命令 | Host 无法读固件版本 | 协议加命令后两端同提交 |
| `/get_param` | 无读取命令 | 轮距等标定只能人工同步 yaml | 扩展协议或增加参数回读 |
| 横向速度 | `SET_VELOCITY` 只有 v/w | `linear.y` 被丢弃，麦轮横移能力未用 | 扩展协议或新增带 vy 的命令 |
| 视觉 / 语音 / 任务 | 仅设计 | 抓取闭环未打通 | M6～M9 按第 5 章实现 |
| Nav2 / SLAM | SLAM 已接入；Nav2 离线闭环通过；实机通信问题已修复 | 实机导航精度与避障未验收 | M5 实机测试 |
| 视觉协方差 | 占位值 | EKF 融合权重不准 | 定位接入前实测重标 |
| 板端实时性 | ROS 2 默认调度，20 ms 定时器有毫秒级抖动 | 极端负载下速度帧延迟 | 需要时用 `SCHED_FIFO` / `ros2_control`，MCU 200 ms 兜底 |
| 多实例 | 同名节点不互斥 | 两个 `base_bridge` 争抢串口 | 启动脚本保证单实例；后续加锁文件 |
| RK3566 板端验证 | 仅在 PC 上直连 | udev、权限、systemd 未实测 | 上板后补测 |
| 长时稳定性 | 未做 24 h 连续运行 | 内存泄漏、句柄泄漏未知 | 增加长时跑机与资源监控 |

**演进顺序**：先补 HardFault 与 IWDG（安全兜底）→ 总线舵机（机械臂可用）→ 视觉 / 手眼 / 抓取（M6～M8）→ 语音与任务（M9）→ bootloader / OTA（M10）→ 工程化与长时测试（M11～M12）。

---

## 附录 A 关键常量速查

| 常量 | 值 | 位置 |
| --- | --- | --- |
| 控制周期 | 10 ms | `APP_CONTROL_PERIOD_MS` |
| 通信周期 | 10 ms | `APP_COMM_PERIOD_MS` |
| 里程计上报 | 20 ms | `APP_ODOM_PERIOD_MS` |
| 轮速 / 占空比上报 | 20 ms | `APP_WHEEL_SPEED_PERIOD_MS` |
| 机械臂上报 | 50 ms | `APP_ARM_PERIOD_MS` |
| 状态 / 心跳 | 100 ms | `APP_STATUS_PERIOD_MS` |
| 指令超时 | 200 ms | `APP_CHASSIS_CMD_TIMEOUT_MS` |
| 轮距 / 轴距 | 165 / 170 mm | 实测；`half_width 82`、`half_length 85` |
| 轮径 | 60 mm | `APP_CHASSIS_WHEEL_DIAMETER_MM` |
| counts/rev | 4680 | `APP_CHASSIS_COUNTS_PER_REV` |
| counts/mm ×1000 | 24828 | `APP_CHASSIS_COUNTS_PER_MM_X1000` |
| 最大线速度 | 500 mm/s | `APP_CHASSIS_MAX_VX_MM_S` |
| 最大角速度 | 1500 mrad/s | `APP_CHASSIS_MAX_WZ_MRAD_S` |
| 单轮最大速度 | 500 mm/s | `APP_CHASSIS_MAX_WHEEL_MM_S` |
| PID 默认 | 5.0 / 6.0 / 0.0 | `APP_CHASSIS_LEFT_PID_*` |
| 测试占空比上限 | 400‰ | `SERVICE_CHASSIS_TEST_DUTY_MAX` |
| 测试占空比过期 | 300 ms | `SERVICE_CHASSIS_TEST_DUTY_TIMEOUT_MS` |
| 舵机帧 | 20 ms | `BSP_SERVO_FRAME_US` |
| 舵机时隙 | 2.5 ms | `BSP_SERVO_SLOT_US` |
| 舵机脉宽 | 500～2500 µs | `BSP_SERVO_PULSE_*` |
| 舵机默认运动时间 | 800 ms | `APP_ARM_MOVE_TIME_MS` |
| 日志导出节拍 | 50 ms | `APP_STORAGE_EXPORT_PERIOD_MS` |
| 导出空闲停止 | 500 ms | `APP_STORAGE_EXPORT_IDLE_MS` |
| 参数 blob | 60 B | `APP_PARAMS_BLOB_PARAM_SIZE` |
| 标定 blob | 80 B | `APP_PARAMS_BLOB_CALIB_SIZE` |
| Flash 扇区 | 4 KB | `SERVICE_STORAGE_SECTOR_SIZE` |
| 故障环 | 32 扇区 | `SERVICE_STORAGE_FAULT_RING_SECTORS` |
| 日志环 | 224 扇区 | `SERVICE_STORAGE_LOG_RING_SECTORS` |
| 崩溃槽 | 16 × 4 KB | `SERVICE_STORAGE_CRASH_SLOTS` |
| 电池低压 | 6800 mV | `APP_BATTERY_LOW_MV` |
| 电池采样周期 | 100 ms | `APP_BATTERY_SAMPLE_PERIOD_MS` |
| 电池滤波窗口 | 5 点中值 | `APP_BATTERY_FILTER_WINDOW` |
| 低压去抖 | 连续 5 次 | `APP_BATTERY_LOW_COUNT` |
| 长按急停 | 1500 ms | `APP_KEY_LONG_PRESS_MS` |

## 附录 B 代码与文档映射

| 设计内容 | 代码入口 | 详细文档 |
| --- | --- | --- |
| 协议帧与命令 | `firmware/app/05_Middlewares/protocol/` | [protocol.md](protocol.md) |
| 协议服务 | `firmware/app/02_Service/service_proto/` | protocol.md 第 9 节 |
| 底盘 | `firmware/app/02_Service/service_chassis/` | [software-design.md](software-design.md) 第 4/6 节 |
| 机械臂 | `firmware/app/02_Service/service_arm/`、`06_BSP/bsp_servo_softpwm.c` | 本文 4.5 |
| 存储 | `firmware/app/02_Service/service_storage/`、`01_APP/app_storage.c` | [flash-layout.md](flash-layout.md)、[storage-design.md](storage-design.md) |
| 应用状态机 | `01_APP/app_state.c`、`app_fault.c` | software-design.md 第 5 节 |
| ROS 桥接 | `mobile_manipulator_ws/src/robot_base/` | [robot-base-design.md](robot-base-design.md) |
| 雷达 | `mobile_manipulator_ws/src/robot_lidar/` | `robot_lidar/README.md` |
| 接口定义 | `mobile_manipulator_ws/src/robot_interfaces/` | software-design.md 第 3 节 |
| 启动配置 | `mobile_manipulator_ws/src/robot_bringup/` | 本文 5.2 / 5.10 |
| 上位机工具 | `tools/host_gui/`、`tools/bench/` | `tools/README.md` |

---

> 变更规则：架构、接口、时序或故障策略调整时，同步修改本文与对应代码；协议变更必须同时修改 `protocol.md`、两端代码和测试向量，并在同一个提交里完成。
