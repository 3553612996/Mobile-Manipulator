# robot_task

任务状态机：接收语音/界面意图，编排导航、视觉、机械臂与夹爪。**纯逻辑，尽量不依赖硬件，可在 x86 上单测。**

## 规划中的节点

`task_manager`：

| 依赖 | 接口 |
| --- | --- |
| 导航 | Nav2 `NavigateToPose` 动作 |
| 视觉 | `/detections` 或视觉服务 |
| 机械臂 | `/grasp` (`robot_interfaces/action/Grasp`) |
| 语音 | 意图事件 |
| 状态 | `/robot_status`、`/system_event` |

## 状态机

`IDLE → NAVIGATE → DETECT → PLAN → APPROACH → DESCEND → CLOSE → LIFT → PLACE`，失败按类型重试或进入 `FAILED`。

每个状态必须定义：

1. 超时；
2. 失败原因码（`no_target` / `ik_fail` / `collision` / `timeout` / `not_grasped`）；
3. 重试策略（没检测到可重拍，IK 无解要换站位，不原地重试）。

完整状态图见 `docs/architecture/software-design.md` 第 5 节。

## 依赖方向

`robot_task` 依赖其他包的**接口**，其他包不依赖 `robot_task`，避免形成环。

## 当前状态

M1 只有包骨架。状态机在 M8 前实现，纯逻辑部分优先进 L1 单测。
