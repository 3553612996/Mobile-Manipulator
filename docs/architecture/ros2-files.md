# ROS 2 侧文件说明

按"从上层到下层"的顺序，逐个说明此刻代码库里每个文件**做什么、对外给什么接口**。

- 想先看模块怎么划分、为什么这么分，读 [software-design.md](software-design.md)；
- 想看底盘桥接的设计决策与实测数据，读 [robot-base-design.md](robot-base-design.md)；
- 想看帧格式本身，读 [protocol.md](protocol.md)。

本文只覆盖**已经实现的四个包**：`robot_bringup`、`robot_base`、`robot_lidar`、`robot_interfaces`。`robot_arm` / `robot_vision` / `robot_speech` / `robot_task` 仍为空壳，见第 5 节。

---

## 0. 分层总览

```text
第 1 层  启动层   robot_bringup      launch / yaml / rviz / urdf —— 只负责"按什么顺序起什么"
第 2 层  节点层   base_bridge        ROS 接口 ↔ 串口；x2_lidar 串口 ↔ /scan
第 3 层  库层     protocol / stm32_link / serial_io / …… —— 不含 ROS，可单测
第 4 层  接口层   robot_interfaces   自定义 msg / srv / action
```

依赖只向下：`robot_bringup` 依赖节点，节点依赖库，库不依赖任何人。

---

## 1. 启动层：`robot_bringup`

这个包**没有可执行文件**，只有被安装到 `share/robot_bringup/` 下的配置与模型。

### 1.1 `launch/base.launch.py`

启动底盘。`robot_state_publisher` 把 URDF 变成静态 TF，`base_bridge` 独占串口。

| 启动项 | 类型 | 作用 |
| --- | --- | --- |
| `robot_state_publisher` | 节点 | 解析 xacro，发布 `/robot_description` 与 `base_link → wheel_* / deck2_link / arm_base_link / laser_link` 的静态 TF |
| `base_bridge` | 节点 | 参数文件 `config/base.yaml`；独占 `/dev/robot_stm32` |
| `rviz2` | 节点（可选） | 配置文件 `config/robot.rviz` |

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `rviz` | `false` | 是否同时开 RViz2 |

用法：`ros2 launch robot_bringup base.launch.py rviz:=true`

> 没接 STM32 时 `base_bridge` 打不开端口会退出，`robot_state_publisher` 与 RViz 不受影响——可以单独看模型。

### 1.2 `launch/lidar.launch.py`

启动雷达，可选 RViz2。

| 启动项 | 参数 | 作用 |
| --- | --- | --- |
| `x2_lidar` | `config/lidar.yaml` | 发 `/scan` 与 `/lidar/points` |
| `rviz2` | `config/lidar.rviz` | 可选显示 |

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `port` | `/dev/robot_lidar` | 覆盖 yaml 里的设备名，台架上常用 `port:=/dev/ttyUSB0` |
| `rviz` | `false` | 是否开 RViz2 |

### 1.3 `config/base.yaml`

`base_bridge` 的全部参数。参数含义在 yaml 里逐条注释，这里只列分类：

| 类别 | 参数 |
| --- | --- |
| 串口 | `port`、`baud` |
| 运动学 | `wheel_separation_m`（必须等于固件 `SET_PARAM 0x01`） |
| 时序 | `velocity_period_ms`、`cmd_vel_timeout_s`、`link_timeout_s`、`ack_timeout_ms`、`ack_retries` |
| 坐标系 | `publish_tf`、`odom_frame_id`、`base_frame_id` |
| 告警 | `warn_unsupported_cmd_vel`、`unsupported_cmd_vel_warn_period_s` |
| 协方差 | `odom_pose_covariance_xy/yaw`、`odom_twist_covariance_xy/yaw` |

### 1.4 `config/lidar.yaml`

`x2_lidar` 的全部参数：

| 类别 | 参数 |
| --- | --- |
| 串口 | `port`、`baud`、`motor_dtr`（USB 转接板靠 DTR 转电机） |
| 坐标系 | `frame_id`、`angle_offset_deg`、`invert_angle` |
| 量程与分辨率 | `range_min_m`、`range_max_m`、`scan_bins` |
| 输出开关 | `publish_scan`、`publish_pointcloud` |
| 时序 | `scan_frequency_hz`、`timer_period_ms`、`reconnect_period_s` |

### 1.5 `config/robot.rviz`、`config/lidar.rviz`

RViz2 的显示配置，不含逻辑：

- `robot.rviz`：模型 + 雷达 + 里程计 + TF；Fixed Frame 为 `base_link`（没有硬件、没有 `odom` 时也能看模型）。
- `lidar.rviz`：只看雷达；Fixed Frame 为 `laser_link`。

### 1.6 `urdf/robot.urdf.xacro`

只描述坐标系，没有 inertial / collision / mesh。

| 坐标系 | 含义 | 关节 |
| --- | --- | --- |
| `base_link` | 底盘几何中心，z=0 在地面 | 根 |
| `wheel_fl/fr/rl/rr` | 四轮中心 | `fixed`（固件不报轮转角） |
| `deck2_link` | 第二层板上表面 | `fixed` |
| `arm_base_link` | 机械臂旋转轴中心 | `fixed` |
| `laser_link` | 雷达旋转中心 | `fixed` |

所有尺寸以 xacro 属性写在文件顶部（`base_length`、`deck2_top_z`、`laser_z` ……），改尺寸只改属性，不动下面的 link 定义。

### 1.7 `package.xml`、`CMakeLists.txt`

- `package.xml`：声明 `exec_depend` 拉起的节点包（`robot_base`、`robot_lidar`、`robot_state_publisher`、`xacro`、`rviz2`……）。
- `CMakeLists.txt`：本质上只有若干条 `install(DIRECTORY …)`，把 launch / config / urdf 装进 `share/`。

---

## 2. 节点层

### 2.1 `robot_base/src/base_bridge.cpp`

节点名 `base_bridge`。**唯一打开 STM32 串口的进程**。

#### 对外接口

| 方向 | 名称 | 类型 / QoS | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/cmd_vel` | `geometry_msgs/Twist`，RELIABLE | 只用 `linear.x` 与 `angular.z`，其余分量丢弃并告警 |
| 发布 | `/odom` | `nav_msgs/Odometry`，RELIABLE | 位姿来自 MCU 积分，yaw 解卷绕 |
| 发布 | `/robot_status` | `robot_interfaces/RobotStatus`，RELIABLE | 10 Hz |
| 发布 | `/system_event` | `robot_interfaces/SystemEvent`，RELIABLE | 事件触发 |
| 发布 | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 链路状态、CRC 错误数、ACK 超时数 |
| TF | `odom → base_link` | — | 与 `/odom` 同时间戳，`publish_tf:=false` 可关 |
| 服务 | `/set_enable` | `std_srvs/SetBool` | 使能/失能，等 ACK |
| 服务 | `/clear_fault` | `robot_interfaces/ClearFault` | 清故障码，等 ACK |

执行器：`MultiThreadedExecutor(4)`，服务放在 `ReentrantCallbackGroup` 里——服务回调会阻塞等 ACK（最坏 800 ms），不能让 20 ms 速度定时器跟着停。

#### 内部函数

| 函数 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `declare_parameters()` | — | — | 声明 17 个参数并给默认值 |
| `validate_parameters()` | — | — | 物理上不合理的值直接抛异常，启动失败 |
| `on_cmd_vel(msg)` | `Twist::SharedPtr` | — | 过滤非有限值、告警不支持分量、记录最近命令与时间戳 |
| `on_velocity_tick()` | — | — | 20 ms 定时器。链路死了就不发；否则**无论有没有新命令都发一帧**，过期/未使能时发零速 |
| `on_frame(frame)` | `protocol::Frame` | — | RX 线程回调。按 `cmd` 分发到 `ODOM/STATUS/EVENT`，其余报告明确忽略 |
| `handle_odom(frame)` | 同上 | — | 解出位姿与 `vl/vr` → 填 `/odom`、发布 TF |
| `handle_status(frame)` | 同上 | — | 更新状态/故障/电池/温度；`INIT` 时重置解卷绕；`MANUAL` 才置 `enabled_` |
| `handle_event(frame)` | 同上 | — | 转成 `/system_event` |
| `on_set_enable(req, res)` | `SetBool::Request/Response` | — | `send_with_ack(SET_ENABLE)`，按 ACK 结果填 `success/message` |
| `on_clear_fault(req, res)` | `ClearFault::Request/Response` | — | 同上，额外回 `result_code` |
| `send_acked_command(cmd, payload, error)` | 命令字、负载、错误出参 | `AckResult` | 统一封装超时与重发参数 |
| `on_link_timer()` | — | — | 1 s 定时器：发现假死 fd 就关，关着就每 2 s 重连，并发布诊断 |
| `publish_diagnostics(open, alive)` | 两个 bool | — | 组装 `/diagnostics`，带 10 个键值对 |
| `publish_ack_failure(op, cmd, msg)` | 操作名、命令字、消息 | — | 服务失败时补一条 ERROR 级诊断 |

### 2.2 `robot_lidar/src/x2_node.cpp`

节点名 `x2_lidar`。10 ms 定时器非阻塞地排空串口，攒满一圈发一次。

#### 对外接口

| 方向 | 名称 | 类型 / QoS | 说明 |
| --- | --- | --- | --- |
| 发布 | `/scan` | `sensor_msgs/LaserScan`，BEST_EFFORT | 一整圈的 720 个 bin，缺的点填 `+inf` |
| 发布 | `/lidar/points` | `sensor_msgs/PointCloud2`，BEST_EFFORT | 同一圈的点云，z 恒为 0 |

参数 14 个，见 1.4。执行器是单线程 `rclcpp::spin`——回调都很短。

#### 内部函数

| 函数 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `declare_parameters()` / `validate_parameters()` | — | — | 同 `base_bridge`，参数非法直接退出 |
| `try_open()` | — | — | 打开串口、重置解析器、按 `motor_dtr` 拉 DTR 让雷达转起来；失败则退避 2 s |
| `on_timer()` | — | — | 没打开就重连；已打开就循环读到空，把字节喂给 `PacketParser` |
| `handle_packet(packet)` | `x2::Packet` | — | 起始包 → 发布上一圈并复位；普通包 → 累加 |
| `accumulate(packet)` | `x2::Packet` | — | 过滤零距离与量程外；角度取反 + 偏移后落 bin；同时记点云 |
| `publish_scan()` | — | — | 填 LaserScan 全部元数据（含中点时间戳）与点云，然后复位 |
| `reset_scan()` | — | — | 所有 bin 重置为 `inf`，清空点云与本圈计数 |

---

## 3. 库层：`robot_base`

### 3.1 `include/robot_base/protocol.hpp` + `src/protocol.cpp`

帧编解码，**不依赖 rclcpp**。整个工作区里只有这里知道字节顺序。

| 接口 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `crc16_update(crc, byte)` | 当前 CRC、一个字节 | 新 CRC | 逐位 CRC-16/CCITT-FALSE |
| `crc16_ccitt(data, len)` | 缓冲区、长度 | CRC | 完整计算，初值 `0xFFFF` |
| `build_frame(cmd, seq, payload, len)` | 命令字、序号、负载、长度 | `vector<uint8_t>` | 加头、算 CRC，输出整帧；命令为 0 或负载超 64 B 抛异常 |
| `FrameParser::feed(data, len)` | 任意分块的字节 | `vector<Frame>` | 增量解析，一次可返回 0～N 帧 |
| `FrameParser::reset()` | — | — | 丢弃半帧，**不清计数器** |
| `frames_ok()` / `crc_errors()` | — | `uint64_t` | 累计计数，供诊断 |
| `encode_set_velocity(v_mm_s, w_mrad_s)` | 两个 `int16_t` | 4 B 负载 | 速度命令 |
| `encode_set_arm_joints(angles[6])` | 6 个 `int16_t`（0.01°） | 12 B 负载 | 关节目标（归 `robot_arm` 用） |
| `encode_set_gripper(action, force)` | 动作、力度 0～100 | 2 B 负载 | 夹爪；力度越界抛异常 |
| `encode_set_enable(enable)` | `bool` | 1 B 负载 | 使能 |
| `encode_set_param(id, value)` | 参数号、`int32_t` | 5 B 负载 | 改固件参数（如轮距） |
| `encode_clear_fault(code=0xFFFF)` | 故障码 | 2 B 负载 | 清故障，默认全清 |
| `encode_param_save/reset(scope)` | 范围 | 1 / 2 B 负载 | 参数固化 / 恢复默认 |
| `encode_log_export(source, action, from_seq)` | 日志源、动作、起始序号 | 6 B 负载 | 导出日志 |
| `encode_crash_read(slot, mark_read)` | 槽位、是否标记已读 | 2 B 负载 | 读崩溃信息 |
| `encode_log_clear(source)` | 日志源 | 2 B 负载 | 清日志 |
| `decode_odom/status/event/...` | 负载 | `optional<T>` | 长度不符返回 `nullopt`，**不抛异常** |
| `state_name/fault_name/result_name(code)` | 枚举值 | `string` | 转中文或 `0xNN`，未知值原样显示 |

### 3.2 `include/robot_base/stm32_link.hpp` + `src/stm32_link.cpp`

串口所有权与会话层：一个写者、一个 RX 线程、一个 SEQ 计数器。

| 接口 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| 构造 | `unique_ptr<SerialIo>`、帧回调 | — | 注入串口实现，便于测试 |
| `open(port, baud, error)` | 设备名、波特率、错误出参 | `bool` | 先停旧线程再打开，启动 RX 线程 |
| `close()` | — | — | 停线程、关 fd |
| `is_open()` | — | `bool` | fd 状态 |
| `set_link_timeout(ms)` | 时长 | — | 设置失联判定阈值 |
| `link_alive()` | — | `bool` | 打开 + 收到过帧 + 未超时 |
| `has_frame()` / `last_frame_age_ms()` | — | `bool` / `int64` | 诊断用 |
| `send(cmd, payload, error)` | 命令字、负载、错误出参 | `bool` | 发了不管；链路不活时**拒绝发送** |
| `send_with_ack(cmd, payload, timeout, retries, error)` | 同上 + 超时、重发次数 | `AckResult{received, result}` | 先登记再发；超时用同一 SEQ 重发 |
| `rx_frames/crc_errors/tx_frames/ack_timeouts()` | — | `uint64_t` | 链路统计 |

### 3.3 `include/robot_base/serial_io.hpp` + `src/termios_serial_io.cpp`

| 接口 | 签名 | 作用 |
| --- | --- | --- |
| `SerialIo` | `open/close/is_open/read/write` 纯虚函数 | 让 `Stm32Link` 不直接依赖 termios |
| `SerialIo::read` | `(buffer, max_len, timeout_ms)` → 字节数 / 0 超时 / -1 错误 | 非阻塞读 |
| `make_termios_serial_io()` | — → `unique_ptr<SerialIo>` | 生产实现：8N1、无流控、`poll` + `O_NONBLOCK`、识别拔线 |

### 3.4 `include/robot_base/heading_unwrapper.hpp`

| 接口 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `update(theta_centi_deg)` | 固件上报的、已卷绕的航向 | 连续 yaw（rad） | 只累加最短角差，跨 ±180° 不跳变 |
| `reset()` | — | — | 重新起头（MCU 报 `INIT` 时） |
| `has_value()` / `radians()` | — | `bool` / `double` | 当前连续值 |

### 3.5 `include/robot_base/cmd_vel_monitor.hpp` + `src/cmd_vel_monitor.cpp`

| 接口 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| 构造 | 开关、告警周期、阈值（默认 0.01） | — | — |
| `update(vx, vy, vz, wx, wy, wz, now_s)` | 六个分量 + 单调时间戳 | `optional<string>` | 有该报的就返回告警文本，并按周期限频 |

---

## 4. 库层：`robot_lidar`

### 4.1 `include/robot_lidar/x2_protocol.hpp` + `src/x2_protocol.cpp`

X2 的抓包与解包，不依赖 ROS。

| 接口 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `packet_checksum(data, len)` | 整包 | `uint16_t` | 逐字异或，用于校验 |
| `decode_packet(data, len, state)` | 一整包、可选的跨包状态 | `optional<Packet>` | 解一包；长度/LSN/校验位/校验和任一不符 → `nullopt` |
| `PacketParser::feed(data, len)` | 任意分块 | `vector<Packet>` | 增量解析，坏包丢 1 字节重扫，不级联 |
| `PacketParser::reset()` | — | — | 丢半包，不清计数 |
| `packets_ok/checksum_errors/dropped_bytes()` | — | `uint64_t` | 统计 |
| `angle_correction_deg(distance_mm)` | 距离（mm） | 角度修正（度） | 厂家公式的二级修正 |
| `normalize_deg_360` / `normalize_rad_pi` | 角度 | 归一化角度 | 折到 `[0,360)` / `(-π,π]` |

### 4.2 `include/robot_lidar/x2_serial.hpp` + `src/x2_serial.cpp`

| 接口 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `open(port, baud, error)` | 设备名、波特率、错误出参 | `bool` | 打开并配置 8N1 |
| `close()` / `is_open()` | — | — / `bool` | 关闭 / 状态 |
| `read(buffer, max_len)` | 缓冲区 | 字节数 / 0 无数据 / -1 错误 | 完全非阻塞 |
| `set_dtr(on, error)` | 电平、错误出参 | `bool` | 拉高 DTR 让雷达电机转起来 |

---

## 5. 接口层：`robot_interfaces`

目前都是 **v0 草案**，在 M8 抓取闭环前允许改字段。

| 文件 | 内容 | 主要字段 |
| --- | --- | --- |
| `msg/RobotStatus.msg` | 整体状态，对应 `REPORT_STATUS` | `state`（含 6 个常量）、`fault_code`、`battery_mv`、`temperature_c`、`max_sched_jitter_us` |
| `msg/SystemEvent.msg` | 事件，对应 `REPORT_EVENT` | `fault_code`、`mcu_uptime_ms` |
| `srv/ClearFault.srv` | 清故障 | 请求 `fault_code`（`0xFFFF` 全清）；响应 `success`、`result_code`、`message` |
| `srv/GetVersion.srv` | 查版本 | 响应 `app_version`、`bootloader_version`、`git_hash`、`protocol_version` |
| `action/Grasp.action` | 抓取长任务 | goal：`object_class`、`target_pose`；feedback：`stage`、`progress`；result：`success`、`reason_code`、`message` |

> `GetVersion` 目前**没有对应命令**：协议 v1 里没有 `GET_VERSION`，所以 `base_bridge` 不注册这个服务，启动时打印 `firmware version unknown`。

---

## 6. 尚未实现的包

| 包 | 现状 | 入口位置 |
| --- | --- | --- |
| `robot_arm` | 只有 `package.xml` / `CMakeLists.txt` / README | 等六自由度分配与臂长确定 |
| `robot_vision` | 同上 | M7 |
| `robot_speech` | 同上 | M9 |
| `robot_task` | 同上 | M8 |

---

## 7. 两条数据流

**控制下行**（约 20 ms 一次）

```text
/cmd_vel ─▶ on_cmd_vel（存最近命令）
              │
              ▼  20 ms 定时器
          on_velocity_tick ─▶ encode_set_velocity ─▶ build_frame ─▶ Stm32Link::send ─▶ UART
```

**状态上行**（MCU 主动上报）

```text
UART ─▶ Stm32Link RX 线程 ─▶ FrameParser::feed ─▶ on_frame
                                                    ├─ ODOM   ─▶ /odom + TF
                                                    ├─ STATUS ─▶ /robot_status
                                                    └─ EVENT  ─▶ /system_event
```

**雷达**

```text
UART ─▶ on_timer（10 ms 排空）─▶ PacketParser ─▶ accumulate ─▶ 起始包触发 publish_scan
                                                                   ├─▶ /scan
                                                                   └─▶ /lidar/points
```

---

## 8. 测试文件

| 文件 | 覆盖 | 需要硬件 |
| --- | --- | --- |
| `robot_base/test/test_protocol.cpp` | 协议测试向量、CRC、任意分块、坏帧重同步、全部 payload | 否 |
| `robot_base/test/test_heading_unwrapper.cpp` | 跨 ±180°、连续多圈、reset | 否 |
| `robot_base/test/test_cmd_vel_monitor.cpp` | 阈值、限频、开关 | 否 |
| `robot_base/test/test_stm32_link.cpp` | ACK 匹配、同 SEQ 重发、断流拒发、CRC 计数、半帧复位 | 否（用假串口） |
| `robot_lidar/test/test_x2_protocol.cpp` | 抓包、校验和、角度修正、分块喂入 | 否 |
