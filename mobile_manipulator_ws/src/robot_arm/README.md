# robot_arm

六轴机械臂的关节控制、逆运动学与抓取序列。

## 规划中的接口

| 方向 | 接口 | 说明 |
| --- | --- | --- |
| 订阅 | `/joint_states` (`sensor_msgs/JointState`) | 20 Hz，六关节实际角与夹爪状态 |
| 下发 | `SET_ARM_JOINTS` | 经串口到 MCU，角度单位 0.01° |
| 动作 | `/grasp` (`robot_interfaces/action/Grasp`) | 目标类别、可选指定位姿；反馈阶段与进度 |

导航不重新定义，直接使用 Nav2 的 `NavigateToPose`。

## 规划

1. 正运动学与逆解（KDL / TRAC-IK 或自写解析解）；
2. 关节限位、奇异点、碰撞检查；
3. 抓取序列：预抓取 → 下降 → 闭合 → 抬起 → 放置；
4. 夹爪状态与失败原因码映射到 `Grasp.result`。

## 当前状态

M1 只有包骨架，没有节点。关节控制与抓取动作在 M8 实现；节点名在实现时确定，避免过早冻结。
