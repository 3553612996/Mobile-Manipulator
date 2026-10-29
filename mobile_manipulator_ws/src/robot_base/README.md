# robot_base

底盘桥接：把 ROS 2 话题转换成串口帧，把 MCU 上报转换回 ROS 2 话题。**本包和 `robot_arm` 是仅有的两个知道串口存在的包**，而串口本身只由本包打开。

## 节点

`base_bridge`（M4）：

| 方向 | 接口 | 说明 |
| --- | --- | --- |
| 订阅 | `/cmd_vel` (`geometry_msgs/Twist`) | RELIABLE；只用 `linear.x` 与 `angular.z` |
| 发布 | `/odom` (`nav_msgs/Odometry`) | 由 `REPORT_ODOM` 转换；2026-10-29 PC 直连实测 45.6 Hz（中位间隔 20.3 ms） |
| 发布 | TF `odom → base_link` | 与 `/odom` 同时间戳，可用 `publish_tf` 关闭 |
| 发布 | `/robot_status` (`robot_interfaces/RobotStatus`) | 由 `REPORT_STATUS` 转换，10 Hz |
| 发布 | `/system_event` (`robot_interfaces/SystemEvent`) | 由 `REPORT_EVENT` 转换，事件触发 |
| 发布 | `/diagnostics` | 链路状态、CRC 错误数、ACK 超时数、状态与故障 |
| 服务 | `/set_enable` (`std_srvs/SetBool`) | 使能 / 失能，等 ACK |
| 服务 | `/clear_fault` (`robot_interfaces/ClearFault`) | 清除故障码，等 ACK |

`/get_version` **未注册**：protocol v1 没有 `GET_VERSION` 命令。启动时会打印 `firmware version unknown (protocol v1 has no GET_VERSION)`。不要用"发一条不存在的命令看回什么"来探测版本——长度校验失败的帧会被静默丢弃。

## 必须遵守的三条时序

1. **速度帧持续发送**：`SET_VELOCITY` 由 20 ms 定时器发出，不绑定 `/cmd_vel` 回调。值不变也照发；固件 200 ms 收不到就停车并锁存 `0x0202`。
2. **看门狗超时发零速，不是停发**：`/cmd_vel` 超过 `cmd_vel_timeout_s`（默认 0.5 s）没有更新时，继续每 20 ms 发零速帧。这样恢复时不会触发固件的通信超时。
3. **未使能不透传**：只有 `REPORT_STATUS` 报 `MANUAL` 时 `/cmd_vel` 才生效；否则定时器发零速。`/set_enable true` 之后要等下一帧状态（≤100 ms）才放行。

## 里程计约定

- 位姿直接转发 MCU 积分结果（`x/y` 单位 mm），ROS 侧**不二次积分**。
- 固件把航向归一到 `(-π, π]`，桥接层用最短角差解卷绕成连续 yaw；`/odom` 与 TF 都用连续值。
- `REPORT_ODOM` 只有 `vl/vr`，角速度由 `(vr - vl) / wheel_separation_m` 计算。`wheel_separation_m` 必须与固件 `SET_PARAM 0x01` 的标定值一致，否则位姿与 twist 自相矛盾，EKF 会缓慢漂移。
- 台架上发 `w = +500 mrad/s`，确认 yaw 增大；若符号相反，只在 ROS 侧取负，不改固件。

## 不支持的分量

`SET_VELOCITY` 只有 `v` 与 `w`。`linear.y`、`linear.z`、`angular.x`、`angular.y` 直接丢弃；超过 0.01 时每 5 s 打一条 WARN 并带数值（`warn_unsupported_cmd_vel`、`unsupported_cmd_vel_warn_period_s`）。不做"先转再走"合成横移。

## 参数

全部在 `robot_bringup/config/base.yaml`：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `port` | `/dev/robot_stm32` | udev 固定名；未安装规则时为 `/dev/ttyUSB0` |
| `baud` | `115200` | 8N1，无硬件流控 |
| `wheel_separation_m` | `0.24` | 必须与固件轮距标定一致 |
| `velocity_period_ms` | `20` | 速度帧周期 |
| `cmd_vel_timeout_s` | `0.5` | ROS 侧 cmd_vel 看门狗 |
| `link_timeout_s` | `0.5` | 无任何合法帧判定失联 |
| `ack_timeout_ms` | `200` | 需 ACK 命令的超时 |
| `ack_retries` | `3` | 重发次数，重发沿用同一 SEQ |
| `publish_tf` | `true` | 是否发布 `odom → base_link` |
| `odom_*_covariance_*` | 见 yaml | 占位协方差，定位接入前实测重标 |

## 端口所有权

一个 UART 只能有一个写者。`base_bridge` 独占打开串口：

- 有硬件时跑 `tools/bench/proto_loop.py` / `storage_bench.py` / `wheel_tune.py`，**先停掉 `base_bridge`**，反之亦然。两个进程抢同一个口的现象是 CRC 全崩。
- **同时只能运行一个 `base_bridge`**。ROS 2 不会阻止同名节点，但 `/set_enable`、`/clear_fault` 会随机连到其中一个实例；测试中曾出现旧实例抢答、新实例实际执行的情况。
- `robot_arm` 以后接入时，复用 `Stm32Link` 或把它提升为独立 transport 节点，不要自己 `open()` 第二个口。

## 结构

```text
include/robot_base/
├── protocol.hpp         帧编解码、CRC、payload 上下行（不依赖 rclcpp）
├── stm32_link.hpp       UART 所有权、RX 线程、ACK 匹配与重发
├── serial_io.hpp        SerialIo 接口，测试用 Fake 替换
├── heading_unwrapper.hpp
└── cmd_vel_monitor.hpp  不支持分量的限频告警
src/
├── protocol.cpp
├── stm32_link.cpp
├── termios_serial_io.cpp
├── cmd_vel_monitor.cpp
└── base_bridge.cpp      节点：定时器、话题、服务、诊断
```

## 测试

```bash
cd mobile_manipulator_ws
colcon build --symlink-install --packages-select robot_interfaces robot_base
colcon test --packages-select robot_base
colcon test-result --verbose
```

| 套件 | 覆盖 |
| --- | --- |
| `test_protocol` | protocol.md §10 全部测试向量、CRC、任意分块、坏帧重同步、全部 payload 编解码 |
| `test_heading_unwrapper` | 跨 ±180°、连续绕三圈、reset |
| `test_cmd_vel_monitor` | 不支持分量的 5 s 限频告警、阈值、关闭开关 |
| `test_stm32_link` | ACK 匹配、同 SEQ 重发 3 次、断流后拒发、CRC 计数、50 ms 半帧复位 |

无硬件即可全部通过。有硬件时先跑 `tools/bench/proto_loop.py --monitor` 确认链路，再启动本节点。

## 实测记录

**测于 2026-10-29，PC（Ubuntu 22.04 / VMware）直连 CH340 @ 115200，`/dev/robot_stm32`，电池 7.44~7.50 V，代码 `develop` + 本包 M4 实现。**

| 项 | 结果 | 条件 / 说明 |
| --- | --- | --- |
| 链路 | 正常 | 启动后 `serial link up`，诊断 CRC 错误 0、ACK 超时 0 |
| `/robot_status` | 9.9 Hz | 8 s 统计，中位间隔 100.2 ms；`state=IDLE`、`fault=0` |
| `/odom` | 45.6 Hz | 8 s 统计，n=366，中位间隔 20.3 ms，p95 29.2 ms |
| 前进 | odom x +0.145 m | 下发 0.1 m/s 1 s + 0.5 s cmd_vel 看门狗；理论 0.15 m。**这是 MCU 编码器里程计增量**，底盘是否架空未记录 |
| 转向 | yaw +0.465 rad | 下发 +0.5 rad/s 0.6 s + 0.5 s 看门狗；方向为正，符合 REP-103 |
| 使能/失能 | `success=True` | `state IDLE ↔ MANUAL`，`fault=0` |
| 未使能门控 | odom 不变 | IDLE 下持续发 0.1 m/s，位置零变化 |
| 不支持分量 | 告警一次 | `linear.y=0.2` → WARN 含数值，5 s 限频 |
| 断链 | ~0.7 s 判定 | unbind USB 后诊断 ERROR"serial port is not open"，停止下发 |
| 重连 | 自动恢复 | rebind 后重新 open、`serial link up`，`/odom` 恢复 |
| 退出 | exit 0 | 连续 3 次 SIGINT，无段错误 |

> `/odom` 的 45.6 Hz 介于文档设计值 50 Hz 与 M3 报告"约 40 Hz"之间：本次中位间隔 20.3 ms，但 p95 到 29.2 ms（10 ms 通信节拍导致的偶发错拍）。时间戳按到达时间打，不假设固定周期。

## 当前状态

M4 底盘桥接已实现并在 PC + 实车直连上验证。遗留：

- 板端（RK3566）实测待补；
- 地面/负载下的里程计标定与 `wheel_separation_m` 一致性仍需在落地台架复核（本次只确认了转向符号）；
- `/get_version` 依赖协议扩展。
