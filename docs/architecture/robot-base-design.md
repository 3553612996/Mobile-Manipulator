# 底盘串口桥接设计（robot_base）

> 状态：**已实现，PC + 实车直连验证通过；RK3566 板端待补。**
> 日期：2026-10-29
> 代码：`develop` @ `b8bf81a`
> 协议：`docs/architecture/protocol.md` v1
> 固件：M3 底盘闭环（115200 8N1）
> 相关：[串口协议](protocol.md) · [软件结构](software-design.md) · [包 README](../../mobile_manipulator_ws/src/robot_base/README.md)

---

## 1. 目标与边界

`robot_base` 是 ROS 2 与 STM32 之间**唯一的底盘硬件边界**。上层只跟话题、服务打交道，不接触帧格式、CRC、串口设备路径。

**做**：

| 方向 | 内容 |
| --- | --- |
| 下行 | `/cmd_vel` → `SET_VELOCITY`；`/set_enable` → `SET_ENABLE`；`/clear_fault` → `CLEAR_FAULT` |
| 上行 | `REPORT_ODOM` → `/odom` + TF；`REPORT_STATUS` → `/robot_status`；`REPORT_EVENT` → `/system_event` |
| 运维 | `/diagnostics`、链路断连检测、自动重连、ACK 重发 |

**不做**：

| 项 | 原因 |
| --- | --- |
| `SET_ARM_JOINTS` / `SET_GRIPPER` / `REPORT_ARM` | 属于 `robot_arm`；本包不处理，避免两个包争抢同一串口 |
| `SET_PARAM` / `PARAM_SAVE` / 日志 / 崩溃读取 | 属于台架与维护工具，当前不暴露为 ROS 接口 |
| `/get_version` | protocol v1 没有 `GET_VERSION` 命令；不注册服务，启动时明确打印未知 |
| 横向速度 | `SET_VELOCITY` 只有 `v/w`，`linear.y` 无处可发 |
| Host 侧重新积分里程计 | MCU 已经积好，重复积分只会让两份里程计分叉 |

**上游**：Nav2、遥控、`robot_task`。
**下游**：STM32 固件，按 `protocol.md` v1 实现。

---

## 2. 设计约束

这些约束直接决定了后面的实现，不是可选项。

| 约束 | 后果 |
| --- | --- |
| 固件 200 ms 收不到有效 `SET_VELOCITY` 就停车并锁存 `0x0202` | 速度帧必须由**定时器**持续发，不能只在 `/cmd_vel` 回调里发 |
| `/cmd_vel` 可能长期没有消息 | Host 侧要有自己的看门狗；超时后发**零速帧**而不是停发 |
| 上报周期是设计值，实际受 10 ms 通信节拍影响 | 时间戳一律用**到达时间**，不假设固定周期 |
| 固件把航向归一到 `(-π, π]` | 直接转 TF 会在 ±180° 处跳 358°，必须解卷绕 |
| `REPORT_ODOM` 只有 `vl/vr`，没有 `wz` | 角速度要在 ROS 侧按轮距计算 |
| v1 没有 `GET_PARAM` | `wheel_separation_m` 只能写进 yaml，标定变更时人工同步 |
| v1 没有 `GET_VERSION` | `/get_version` 不实现；探测未知命令会被严格长度校验静默丢弃 |
| `SET_VELOCITY` 不需要 ACK | 不为它建等待队列，40~50 Hz 下必然堆积 |
| 需要 ACK 的命令重发沿用同一 SEQ | 固件按 SEQ 去重，重复执行会被缓存结果挡掉 |
| 一个 UART 只能有一个写者 | `robot_base` 独占 `open()`；台架脚本与节点不能同时跑 |

---

## 3. 总体结构

```
┌──────────────────────────── robot_base 节点进程 ────────────────────────────┐
│                                                                            │
│  /cmd_vel ──► BaseBridge ──► Stm32Link ──► protocol ──► termios ──► UART    │
│  服务调用  ──►   (ROS)    │   (UART 所有权)  (帧/CRC)     (fd)      /dev/robot_stm32
│                           │                                  ▲             │
│  /odom ◄── 解卷绕/换算 ◄──┘        RX 线程 ◄── FrameParser ◄──┘             │
│  /robot_status ◄── REPORT_STATUS                                          │
│  /system_event ◄── REPORT_EVENT                                           │
│  /diagnostics  ◄── 链路统计                                                │
└────────────────────────────────────────────────────────────────────────────┘
```

| 层 | 文件 | 职责 | 约束 |
| --- | --- | --- | --- |
| 协议编解码 | `include/robot_base/protocol.hpp`、`src/protocol.cpp` | 帧格式、CRC、全部上下行 payload、名称映射 | 纯 C++，不依赖 rclcpp / 串口，可在 PC 单测 |
| 链路 | `include/robot_base/stm32_link.hpp`、`src/stm32_link.cpp` | 独占 fd、RX 线程、帧解析、ACK 匹配、同 SEQ 重发、链路健康 | 只有它能 `open()` 串口 |
| 串口适配 | `include/robot_base/serial_io.hpp`、`src/termios_serial_io.cpp` | termios 115200 8N1、非阻塞读、写超时 | 通过 `SerialIo` 接口注入，测试用 Fake |
| 节点 | `src/base_bridge.cpp` | 定时器、话题、服务、坐标/单位换算、诊断 | 只处理底盘命令与上报 |
| 启动配置 | `robot_bringup/config/base.yaml`、`launch/base.launch.py` | 参数与编排 | 不硬编码设备路径和轮距 |
| 辅助 | `heading_unwrapper.hpp`、`cmd_vel_monitor.*` | 航向解卷绕、不支持分量限频告警 | 无 ROS 依赖，便于单测 |

**为什么拆成三层而不是一个节点文件**：机械臂接入时，`Stm32Link` 可以提升为独立 transport 节点，`robot_base` 与 `robot_arm` 都通过它收发；现在先把边界划好，后面不用把 arm 逻辑塞回 base。

---

## 4. 线程与并发

| 执行体 | 数量 | 职责 | 共享数据保护 |
| --- | --- | --- | --- |
| RX 线程（`Stm32Link`） | 1 | `poll()` 读字节、50 ms 半帧复位、解析、分发 ACK/上报、调用 `on_frame` | `parser_` 只由该线程访问；`last_frame_ns_`、`has_frame_` 为原子量 |
| 默认回调组（MutuallyExclusive） | 1 组 | `/cmd_vel` 订阅、20 ms 速度定时器、1 s 链路定时器 | `cmd_mutex_` 保护最新命令；状态/故障为原子量 |
| 服务回调组（Reentrant） | 1 组 | `/set_enable`、`/clear_fault`，内部阻塞等 ACK | ACK 等待表由 `ack_mutex_` + 条件变量保护 |
| `MultiThreadedExecutor` | 4 线程 | 让服务等待 ACK 时不饿死速度定时器 | — |

**为什么服务用 Reentrant + 4 线程**：一次需要 ACK 的命令最坏等待 4 × 200 ms = 800 ms。如果单线程执行器，服务回调会把 20 ms 速度定时器堵住，超过固件 200 ms 超时，底盘会自己停车并锁存 `0x0202`。4 线程保证即使两个服务同时在等 ACK，仍有线程跑定时器。

**为什么速度定时器不和 `/cmd_vel` 回调绑在一起**：话题可能长时间没有消息，而固件要求持续收到速度帧。定时器每 20 ms 发一帧，值来自"最新命令或零"。

**生命周期**：`BaseBridge` 析构函数第一件事是 `link_->close()`，先停 RX 线程，再销毁发布器/TF 广播器。否则退出瞬间 RX 线程可能通过已析构的发布器发布，表现为 `[ros2run]: Segmentation fault`（实测遇到过，已修）。

---

## 5. 数据流

### 5.1 下行：速度

```
/cmd_vel ──► on_cmd_vel
              ├─ 检查 NaN/Inf，丢弃非法值
              ├─ CmdVelMonitor：linear.y/z、angular.x/y 超阈值时限频告警
              └─ 保存 linear.x / angular.z / 到达时间
                        │
每 20 ms ──► on_velocity_tick
              ├─ 端口未打开或链路失联？不发
              ├─ enabled_==false 或命令超过 0.5 s？目标 = 0
              ├─ 换算：v = linear.x×1000 (mm/s)，w = angular.z×1000 (mrad/s)
              └─ SET_VELOCITY（无 ACK）──► UART
```

### 5.2 上行：上报

```
UART ──► RX 线程 ──► FrameParser ──► on_frame
                                      ├─ 0xC1 ODOM   → 解卷绕 → /odom + TF odom→base_link
                                      ├─ 0xC3 STATUS → /robot_status；更新 state/fault/电池/温度
                                      ├─ 0xC4 EVENT  → /system_event（只通知，不驱动状态机）
                                      └─ 其他         → 忽略（arm / wheel / storage 不归本包）
```

### 5.3 服务：ACK

```
服务客户端 ──► on_set_enable / on_clear_fault
                 ├─ 链路不可用 → 立即返回失败并发布 ACK 诊断
                 ├─ 分配 SEQ、组帧、注册等待项
                 └─ 最多 4 次（首发 + 3 次重发，同 SEQ）：
                      发送 ──► RX 线程收到 ACK（cmd|0x80, [seq, result]）
                                └─ 条件变量唤醒 → 返回 result
                      200 ms 超时 → 重发
                 超限 → success=false + /diagnostics ERROR
```

`SET_VELOCITY` **不进入**这条路径，没有等待队列。

---

## 6. 关键机制

### 6.1 速度帧持续发送与 cmd_vel 看门狗

| 条件 | 发出的速度 |
| --- | --- |
| 链路正常、已使能、`/cmd_vel` 新鲜（≤ `cmd_vel_timeout_s`） | 最新命令 |
| 链路正常、已使能、`/cmd_vel` 超时 | 零速 |
| 链路正常、未使能 | 零速 |
| 链路失联 / 端口未打开 | 不发 |

**看门狗超时后发零速而不是停发**：停发会让固件在 200 ms 后锁存 `0x0202`，恢复时必须先 `CLEAR_FAULT` 再重新使能；持续发零速则不会触发故障，恢复时无缝。

**未使能也持续发零速**：固件在 IDLE 下忽略速度指令，但这让"使能后第一帧"不会因为链路空闲而延迟。

### 6.2 使能门控

```
REPORT_STATUS.state == MANUAL (2)  ──► enabled_ = true
其他状态（INIT/IDLE/AUTO/ESTOP/FAULT）──► enabled_ = false
```

- `/set_enable true` 的 ACK 只表示"受理"；节点要等下一帧 `REPORT_STATUS` 变成 `MANUAL` 才放行 `/cmd_vel`（实测约 27 ms）。
- `/set_enable false` 成功后立即 `enabled_=false`，不等状态帧。
- `CLEAR_FAULT` 成功后固件回到 IDLE，**不自动使能**；节点也不会替上层重新使能。
- FAULT / ESTOP 下速度目标恒为零。

### 6.3 航向解卷绕

固件把 `heading_rad` 归一到 `(-π, π]` 再转成 int16 的 0.01° 上报。桥接层维护连续航向：

```
raw  = theta_centi_deg × 0.01° → rad
首帧：continuous = raw
后续：delta = atan2(sin(raw - last_raw), cos(raw - last_raw))
      continuous += delta
      last_raw = raw
```

- `/odom` 的 pose 与 TF 使用 `continuous`；原始值只进日志。
- **重置策略**：只在 MCU 进入 `INIT`（重启）时重置。串口断开/恢复不重置——MCU 一直在积分，重置会造成 yaw 跳变。
- 单测覆盖：`[179°, −179°] → 181°`、负向跨接、连续绕三圈。

### 6.4 twist 计算与轮距

`REPORT_ODOM` 没有角速度，只有左右侧速度：

```
linear.x  = (vl + vr) / 2000.0          [m/s]
angular.z = (vr - vl) / 1000.0 / wheel_separation_m   [rad/s]
```

- `wheel_separation_m` 默认 0.24 m，对应固件 `SET_PARAM 0x01` 的默认轮距 240 mm。
- v1 没有 `GET_PARAM`，固件里的标定值读不回来；改标定后必须人工同步 yaml，否则位姿（固件积分）与 twist（ROS 计算）会互相矛盾，EKF 缓慢漂移。
- 台架上发 `w = +500 mrad/s`，yaw 应增大；本次实测符号正确。若反向，只在 ROS 侧取负，不改固件。
- 协方差是占位值（`0.05 m² / 0.02 rad²` 等），定位/EKF 接入前按实测重标。
- `REPORT_ODOM` 观测不到的自由度必须填 `1e6`（`kUnobservableCovariance`），不能留 0：`robot_localization` 把 0 读作"该量测完全准确"，会让轮式里程计的 z/roll/pitch 和 `linear.y` 盖过 IMU 的真实估计，且不报任何错。固件对 x、y、yaw 都有观测（麦轮正解积分两轴），所以这三个保持小幅值。

### 6.5 ACK、SEQ 与重发

| 项 | 做法 |
| --- | --- |
| 需要 ACK 的命令 | 协议表从 `0x03` 起；本包只用到 `SET_ENABLE`、`CLEAR_FAULT` |
| ACK 格式 | `cmd \| 0x80`，payload `[被应答 SEQ, result]` |
| 超时 | 200 ms |
| 重发 | 最多 3 次（共 4 次尝试），**沿用同一 SEQ 和同一帧字节** |
| 固件去重 | 按命令字各自保存 `last_seq` + 1 s 窗口；重复 SEQ 回缓存结果，不重复执行 |
| 超限 | 服务返回失败，发布 `/diagnostics` ERROR；`ack_timeouts` 计数递增 |

### 6.6 链路看门狗与重连

| 状态 | 判定 | 行为 |
| --- | --- | --- |
| 端口未打开 | `is_open()==false` | 每 2 s 尝试 `open()`，失败限频告警 |
| 打开但从未收到帧 | `has_frame()==false` | 不算在线；超过 `link_timeout+2 s` 主动关闭重开 |
| 收到过帧但超时 | 当前时间 − 最后帧 > `link_timeout_s`（默认 0.5 s） | 诊断 ERROR，停止速度下发 |
| USB 拔出 | `poll()` 返回 `POLLHUP/POLLERR` 或 `read()<0` | RX 线程退出，进入重开流程 |
| 重新插入 | udev 重建 `/dev/robot_stm32` | 自动重开，收到第一帧后恢复 |

**为什么不是只靠 `is_open()`**：USB 拔出后 fd 可能仍然有效，`read()` 一直返回 0；只看 open 标志会永远不重连。实测遇到后，增加了 `POLLHUP/POLLERR` 检测和"僵尸 fd 超时关闭"两条路径。

### 6.7 诊断与日志

`/diagnostics`（1 Hz）包含：

- 链路等级：OK / ERROR（端口未打开、无帧、断链）
- 值：端口、波特率、`rx_frames`、`crc_errors`、`tx_frames`、`ack_timeouts`、`last_frame_age_ms`、状态、故障码、电池、温度、调度抖动
- ACK 超限时额外发布一条 `robot_base: command ack` 的 ERROR

日志只记录状态迁移、故障变化、链路上下线，避免 40~50 Hz 上报刷屏。

### 6.8 时间与单位

| 量 | 线上单位 | ROS 单位 | 说明 |
| --- | --- | --- | --- |
| 线速度 | mm/s | m/s | ×1000 |
| 角速度 | mrad/s | rad/s | ×1000；twist 的 `wz` 例外，由轮速反算 |
| 关节角 | 0.01° | rad | `robot_arm` 负责，本包不处理 |
| 位置 | mm | m | ×1000 |
| 航向 | 0.01° | rad | 解卷绕后转四元数 |
| 电池 | mV | — | `RobotStatus.battery_mv` 原样 |
| 温度 | °C | — | `RobotStatus.temperature_c` 原样 |
| 时间戳 | — | ROS 到达时间 | 不使用固定周期推算 |

### 6.9 不支持的分量

`linear.y`、`linear.z`、`angular.x`、`angular.y` 阈值 0.01，超过时每 5 s 打一条 WARN 并带数值；`warn_unsupported_cmd_vel=false` 可关闭。不做"先转再走"合成横移——会引入额外状态和时延。

---

## 7. 接口

### 7.1 话题

| 话题 | 类型 | 方向 | QoS | 频率 |
| --- | --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | 订阅 | RELIABLE, depth 10 | 由上层决定 |
| `/odom` | `nav_msgs/Odometry` | 发布 | RELIABLE, depth 10 | 实测 45.6 Hz |
| `/robot_status` | `robot_interfaces/RobotStatus` | 发布 | RELIABLE, depth 10 | 9.9 Hz |
| `/system_event` | `robot_interfaces/SystemEvent` | 发布 | RELIABLE, depth 10 | 事件触发 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 发布 | 默认, depth 10 | 1 Hz + ACK 失败即时 |
| TF `odom → base_link` | `geometry_msgs/TransformStamped` | 发布 | — | 随 `/odom` |

### 7.2 服务

| 服务 | 类型 | 语义 |
| --- | --- | --- |
| `/set_enable` | `std_srvs/SetBool` | ACK OK = 已受理；使能后等状态变 MANUAL 才放行速度 |
| `/clear_fault` | `robot_interfaces/ClearFault` | `fault_code=0xFFFF` 清全部；条件未消失时固件返回 `0x02`，`success=false` |

`/get_version` **不注册**。启动日志：`firmware version unknown (protocol v1 has no GET_VERSION)`。

### 7.3 参数

| 参数 | 默认 | 单位 | 约束 / 说明 |
| --- | --- | --- | --- |
| `port` | `/dev/robot_stm32` | — | udev 固定名；未安装规则时为 `/dev/ttyUSB0` |
| `baud` | `115200` | bps | 8N1，无流控 |
| `wheel_separation_m` | `0.24` | m | 必须与固件 `SET_PARAM 0x01` 一致 |
| `velocity_period_ms` | `20` | ms | 速度帧周期 |
| `cmd_vel_timeout_s` | `0.5` | s | 超时后发零速 |
| `link_timeout_s` | `0.5` | s | 无帧判定失联 |
| `ack_timeout_ms` | `200` | ms | 需 ACK 命令 |
| `ack_retries` | `3` | 次 | 重发次数，同 SEQ |
| `publish_tf` | `true` | — | 是否发布 `odom → base_link` |
| `odom_frame_id` | `odom` | — | |
| `base_frame_id` | `base_link` | — | |
| `warn_unsupported_cmd_vel` | `true` | — | 关闭不支持分量告警 |
| `unsupported_cmd_vel_warn_period_s` | `5.0` | s | 告警限频 |
| `odom_pose_covariance_xy` | `0.05` | m² | 占位，待标定 |
| `odom_pose_covariance_yaw` | `0.02` | rad² | 占位，待标定 |
| `odom_twist_covariance_xy` | `0.05` | (m/s)² | 占位，待标定 |
| `odom_twist_covariance_yaw` | `0.05` | (rad/s)² | 占位，待标定 |

参数非法时节点拒绝启动（`std::invalid_argument`），不做静默截断。

---

## 8. 错误处理与降级

| 事件 | 检测方式 | 处理 | 用户可见 |
| --- | --- | --- | --- |
| 串口打不开 | `open()` 失败 | 保持节点运行，每 2 s 重试 | WARN 限频、诊断 ERROR |
| 无帧 | `has_frame` / 最后帧时间 | 停止速度下发 | 诊断 ERROR |
| 断链 | `POLLHUP/POLLERR` 或长时间无帧 | RX 线程退出，关闭僵尸 fd | ERROR 一次，诊断 ERROR |
| 重连 | udev 重建设备 | 自动重开，等第一帧 | INFO `serial link up` |
| CRC 错 | `FrameParser` 计数 | 丢该帧继续；诊断累计 | 诊断 `crc_errors` |
| ACK 超时 | 4 次尝试仍无 ACK | 服务失败，诊断 ERROR | 服务 `success=false` |
| 固件拒绝 | ACK result `0x01~0x06` | 原样返回结果码与中文名 | 服务 message |
| 固件故障 | `REPORT_STATUS.fault` | 记录、发布、状态不自动恢复 | `/robot_status`、`/system_event` |
| 电池低压 | `0x0401`（WARN） | 只上报，不禁使能 | `/robot_status` |
| ESC/FAULT | 状态字 4/5 | `enabled_=false`，速度归零 | `/robot_status` |
| 非法 cmd_vel | NaN/Inf | 整条丢弃 | WARN 限频 |
| 不支持分量 | 阈值判断 | 丢弃该分量，其余生效 | WARN 每 5 s |
| 节点退出 | SIGINT/SIGTERM | 先关串口线程，再析构发布器 | 正常退出码 0 |

**状态与故障分开**：状态字表示"现在能不能动"，故障码表示"为什么不能动"。`REPORT_STATUS` 是权威，`REPORT_EVENT` 只用于通知，事件帧丢失不影响状态判断。

---

## 9. 设计决策记录

### D1 串口用 termios + 独立 RX 线程，不引入 libserial/asio

- **理由**：目标板与 PC 都不需要额外运行时依赖；非阻塞 `poll()` + 200 ms 写超时足够；`SerialIo` 接口让测试用 Fake 替换，不必开真实端口。
- **代价**：自己处理 `EINTR`、`EAGAIN`、`POLLHUP`；代码量比库多一点。
- **放弃的方案**：`libserial`（依赖多、行为不可控）、Boost.Asio（引入大依赖，收益不匹配）。

### D2 速度帧由 20 ms 定时器发，不绑 `/cmd_vel`

- **理由**：话题可能长时间没有消息，而固件 200 ms 断流就停车。
- **后果**：即使命令不变也持续占带宽；40~50 Hz × 11 字节 ≈ 550 B/s，可忽略。

### D3 cmd_vel 超时发零速，不停发

- **理由**：停发会触发固件 `0x0202` 锁存，恢复要清故障再使能；零速帧不会。
- **后果**：看门狗期间机器人会按最后速度再走 `cmd_vel_timeout_s`（默认 0.5 s），这是有意的"软停"。

### D4 位姿直接用 MCU 积分，Host 不二次积分

- **理由**：MCU 已经用编码器积分好 `x/y/theta`；Host 再积一遍会引入第二套误差，且与固件分叉。
- **后果**：Host 只做单位换算、航向解卷绕和 twist 反算。

### D5 服务同步等 ACK + 4 线程执行器

- **理由**：实现简单，服务语义直观（返回时已有结果）；4 线程保证速度定时器不被阻塞。
- **代价**：最坏 800 ms 占用一个执行器线程；如果未来服务数量增多，需要改成异步命令队列。
- **放弃的方案**：单线程执行器（会触发固件超时）、异步状态机（当前规模不值得）。

### D6 航向只在 MCU 重启时重置

- **理由**：串口断开不代表 MCU 重启，MCU 仍在积分；断链后重置会让连续 yaw 跳变。
- **实现**：`REPORT_STATUS.state == INIT` 时置 `resync_odom_`，下一个 ODOM 重新播种。

### D7 只有 base 处理 base 自己的命令

- **理由**：一个 UART 只能有一个写者；arm 命令和上报以后走 `Stm32Link` 的复用路径，不塞回 base。
- **后果**：当前 `REPORT_ARM`、`SET_ARM_JOINTS` 在本包被忽略；机械臂接入时需要 transport 层。

### D8 不实现 `/get_version`

- **理由**：protocol v1 没有该命令；用未知命令探测时，`protocol_cmd_payload_ok()` 严格等长校验会静默丢弃，观察不到任何结果。
- **后续**：协议增加 `GET_VERSION` 后，固件、ROS、文档、测试向量同提交。

### D9 控制与状态用 RELIABLE

- **理由**：丢一条速度指令会让底盘动作不连续；丢状态会让上层误判故障。
- **后果**：慢订阅者可能造成发布端背压；控制链路的订阅者应按实时节奏消费。

---

## 10. 测试

### 10.1 单元测试（无硬件）

| 套件 | 用例数 | 覆盖 |
| --- | --- | --- |
| `test_protocol` | 35 | CRC 自检、protocol.md §10 全部向量、任意分块、坏 CRC/LEN 重同步、全部 payload 编解码、帧长边界 |
| `test_heading_unwrapper` | 5 | 跨 ±180°、负向跨接、连续绕三圈、reset |
| `test_cmd_vel_monitor` | 5 | 不支持分量、阈值、5 s 限频、关闭开关 |
| `test_stm32_link` | 5 | ACK 匹配、同 SEQ 重发 4 次、断流拒发、CRC 计数、50 ms 半帧复位 |
| **合计** | **4 个可执行目标 / 50 个 gtest 用例** | 全部通过 |

> `colcon test-result` 的 `Summary: 54 tests` 把 4 个测试目标本身也算进去了，实际 gtest 用例是 50 个。

运行：

```bash
cd mobile_manipulator_ws
colcon build --paths src/robot_interfaces src/robot_base --symlink-install
colcon test --paths src/robot_base
colcon test-result --verbose
```

### 10.2 实车直连测试

**条件**：2026-10-29，PC（Ubuntu 22.04 / VMware），CH340 @ 115200，`/dev/robot_stm32`，电池 7.44~7.50 V，代码 `develop @ b8bf81a`。底盘是否架空未记录；位移/twist 均来自 MCU 编码器里程计。

| 项 | 结果 |
| --- | --- |
| `/odom` | 45.6 Hz（8 s，n=366，中位间隔 20.3 ms，p95 29.2 ms） |
| `/robot_status` | 9.9 Hz，中位间隔 100.2 ms |
| 初始状态 | `IDLE`、`fault=0`、电池 7436~7513 mV |
| 前进 0.1 m/s × 1 s | odom x +0.145 m（理论 0.15 m，含 0.5 s 看门狗） |
| 转向 +0.5 rad/s × 0.6 s | yaw +0.465 rad，方向为正 |
| 使能 / 失能 | 服务 `success=True`，`IDLE ↔ MANUAL` |
| 未使能门控 | IDLE 下持续发 0.1 m/s，位置零变化 |
| 不支持分量 | `linear.y=0.2` 触发一次 WARN，带数值 |
| 断链 | unbind USB 后约 0.7 s 诊断 ERROR、停止下发 |
| 重连 | rebind 后自动重开、`serial link up`、`/odom` 恢复 |
| 退出 | 连续 3 次 SIGINT，exit 0，无段错误 |

### 10.3 测试中发现并修复的缺陷

| 现象 | 根因 | 修复 |
| --- | --- | --- |
| 设备枚举但无 `/dev/ttyUSB*` | brltty 以 usbfs 抢占 CH340 | 停用并 mask `brltty-udev.service`；安装 udev 固定名规则 |
| USB 重插后 tty 编号漂移 | 内核重新分配 `ttyUSB0/1` | `/dev/robot_stm32` 符号链接 |
| 拔插后节点不重连 | fd 仍有效、`read()` 返回 0，`open_` 一直为真 | `POLLHUP/POLLERR` 返回 -1；长时间无帧主动 close/reopen |
| 退出时段错误 | RX 线程在发布器析构后仍发布 | `BaseBridge` 析构先 `link_->close()` |
| 重连后 yaw 跳变 | 断链无条件重置解卷绕 | 只在 MCU `INIT` 时重置 |
| 同名节点服务响应不确定 | 旧 `base_bridge` 未退出，与新节点同名 | 测试脚本清理进程；README 明确只能跑一个实例 |

### 10.4 未覆盖

- RK3566 板端（udev、串口权限、systemd 拉起）未测。
- 地面/负载下的里程计标定、`wheel_separation_m` 一致性未测（本次只确认转向符号）。
- 长距离直行、原地 360°、打滑、电池低压阈值、堵转/过流未测。
- 多节点并发调用服务、执行器饱和未做压力测试。
- 真实硬件上的 ±180° 连续绕圈未做；解卷绕靠单测覆盖。

---

## 11. 已知限制与后续

| 项 | 说明 | 计划 |
| --- | --- | --- |
| `/odom` 频率 | 实测 45.6 Hz，介于设计 50 Hz 与文档"约 40 Hz"之间；中位间隔 20.3 ms，p95 29.2 ms | 板端复测后在 `project-intro.md` 指标表填实测值 |
| `/get_version` | protocol v1 无命令 | 协议加命令后同时改固件、ROS、文档、向量 |
| `/get_param` | 无读取命令 | 轮距等标定参数只能 yaml 人工同步 |
| 横向速度 | 线上帧只有 v/w | 需要扩展协议或改成带 vy 的新命令 |
| 机械臂 | 本包忽略 arm 命令与上报 | `robot_arm` 接入时把 `Stm32Link` 提升为 transport 节点 |
| 协方差 | 占位值 | 定位/EKF 接入前实测重标 |
| 实时性 | ROS 2 默认调度，20 ms 定时器有毫秒级抖动 | 需要时可加 `SCHED_FIFO` / `ros2_control`；当前由 MCU 200 ms 超时兜底 |
| 多实例 | 同名节点不互斥 | 启动脚本保证只拉起一个 |

---

## 12. 参考

| 文档 | 内容 |
| --- | --- |
| [protocol.md](protocol.md) | 帧格式、命令字、CRC、超时、故障码、测试向量 |
| [software-design.md](software-design.md) | 包划分、ROS 接口与 QoS、并发与错误处理 |
| [overview.md](overview.md) | 双控分工、数据流、部署 |
| [robot_base README](../../mobile_manipulator_ws/src/robot_base/README.md) | 使用说明、参数、实测记录 |
| `tools/host_gui/protocol.py` | 已验证的 Python 编解码参考实现 |
| `tools/bench/proto_loop.py` | 真机串口闭环测试脚本 |
