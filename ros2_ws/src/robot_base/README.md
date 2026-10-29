# robot_base

底盘桥接：把 ROS 2 话题转换成串口帧，把 MCU 上报转换回 ROS 2 话题。**本包和 `robot_arm` 是仅有的两个知道串口存在的包。**

## 规划中的节点

`base_bridge`：

| 方向 | 接口 | 说明 |
| --- | --- | --- |
| 订阅 | `/cmd_vel` (`geometry_msgs/Twist`) | 50 Hz，差分/麦轮运动学 → `SET_VELOCITY` |
| 发布 | `/odom` (`nav_msgs/Odometry`) | 由 `REPORT_ODOM` 转换，50 Hz |
| 发布 | `/robot_status` (`robot_interfaces/RobotStatus`) | 由 `REPORT_STATUS` 转换，10 Hz |
| 发布 | `/system_event` (`robot_interfaces/SystemEvent`) | 由 `REPORT_EVENT` 转换，事件触发 |
| 服务 | `/set_enable` (`std_srvs/SetBool`) | 使能 / 失能 |
| 服务 | `/clear_fault` (`robot_interfaces/ClearFault`) | 清除故障码 |

## 约束

- 串口设备路径、波特率由 `robot_bringup/config/` 参数传入，代码里不写死 `/dev/ttyS*`；
- `/cmd_vel` 与 `/odom` 用 `RELIABLE`，传感器类话题按 `docs/architecture/software-design.md` 的 QoS 表；
- 协议帧格式以 `docs/architecture/protocol.md` 为唯一契约，不在本包另定义一套；
- 通信超时由 MCU 兜底停车，Host 侧只负责上报和重连。

## 当前状态

M1 只有包骨架，没有节点。`base_bridge` 在 M3 底盘闭环时实现。
