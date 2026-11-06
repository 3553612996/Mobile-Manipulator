# M5 Nav2 导航测试报告

> 日期：2026-11-05
> 代码：`feat/m5-nav2`（基于 `develop` 的 `e560789`；本文对应的改动尚未提交）
> 环境：PC Ubuntu 22.04.5 LTS（VMware 虚拟机），ROS 2 Humble，Nav2 1.1.20
> 硬件：本机未接 STM32 / X2（`/dev/robot_*`、`/dev/ttyUSB*` 均不存在），**没有实机导航数据**

## 1. 范围

M5 的目标是让机器人在已建好的 `room2` 地图上接收 `NavigateToPose`，完成定位、全局规划、局部避障与到达。

| 属于 M5 | 不属于 M5 |
| --- | --- |
| AMCL 定位、Nav2 规划/控制/恢复行为、PC 端 launch 与参数 | `robot_task` 任务状态机、视觉/机械臂联动 |
| `/cmd_vel` 交给板端 `base_bridge` | Nav2 上板（2+8G 板卡仍只做传感器与串口桥接） |
| 离线闭环验证与实机验收清单 | IMU / EKF 融合 |

## 2. 实现内容

| 文件 | 作用 |
| --- | --- |
| `robot_bringup/launch/navigation.launch.py` | 包含 `nav2_bringup/bringup_launch.py`：map_server + AMCL + planner/controller/behavior/BT + velocity_smoother，另起 RViz |
| `robot_bringup/config/nav2_params.yaml` | 全部 Nav2 节点参数，按本车尺寸/速度/串口时序调整 |
| `robot_bringup/config/nav.rviz` | 导航布局：地图、AMCL 粒子、全局/局部代价地图、路径、目标点工具 |
| `robot_bringup/package.xml` | 增加 `nav2_bringup`、`nav2_amcl`、`nav2_map_server`、`nav2_*` 插件依赖 |
| `tools/bench/nav_goal.py` | 命令行发 `NavigateToPose`；可先发 `/initialpose`、调用 `/set_enable` |
| `tools/bench/nav_sim.py` | 离线假底盘：按地图射线生成 `/scan`，用 `/cmd_vel` 积分 `/odom` 与 TF |
| `tools/bench/nav_smoke.sh` | 一键跑离线闭环（nav2 + nav_sim + nav_goal），退出码 0 表示 `SUCCEEDED` |

`/cmd_vel` 链路（与 `base.yaml` 的 20 ms 速度流约束一致）：

```text
controller_server ──/cmd_vel_nav──► velocity_smoother ──/cmd_vel──► board base_bridge ──UART──► STM32
```

启动导航前必须先使能底盘，否则 `base_bridge` 会把速度替换成零速：

```bash
ros2 service call /set_enable std_srvs/srv/SetBool "{data: true}"
```

## 3. 关键参数与取舍

| 项 | 值 | 理由 |
| --- | --- | --- |
| 底盘模型 | 差速（`v` / `w`） | 固件 `SET_VELOCITY` 只有线速度与角速度，没有横移；麦轮也不发 `linear.y` |
| 最大线速度 | 0.25 m/s | 固件上限 0.5 m/s；室内留制动距离 |
| 最大角速度 | 1.0 rad/s | 固件上限 1.5 rad/s |
| 加速度 | 0.5 m/s²、1.5 rad/s² | 与固件 PID 的可跟随范围对齐，起步不打滑 |
| 控制频率 | 10 Hz | 0.25 m/s 时每拍 2.5 cm；VM 上比 20 Hz 稳，20 ms 速度流由 smoother 保证 |
| 车体足迹 | `[-0.135,-0.11] … [0.135,0.11]` | 实测底盘 0.245 × 0.195，四周留约 1 cm |
| 膨胀半径 | 0.30 m | 内切半径约 0.11 m；窄处仍可通过 |
| 全局规划 | NavFn + A*，`allow_unknown: false` | 地图内无未知区；不允许穿越未知区 |
| 目标容差 | 位置 0.05 m，航向 0.15 rad | 定位本身有 4~5 cm 误差，位置容差取 5 cm 才能把总误差压在 10 cm 内 |
| AMCL | 120 束，`sigma_hit=0.10`，每 5 cm / 0.05 rad 更新 | 6 Hz 扫描下逐帧更新；默认参数在稀疏地图上滞后约 12 cm |
| 初始位姿 | `set_initial_pose: true`，`(0,0,0)` | 地图原点就是建图起点；不给初始位姿时 AMCL 不发 `map→odom`，Nav2 激活会卡住 |
| 恢复行为 | spin / backup / drive_on_heading / wait | 默认行为树使用；`backup` 允许 -0.15 m/s |
| lifecycle bond | 4 s（Humble bringup 固定） | `nav2_bringup` 不把 params_file 传给 lifecycle_manager，YAML 里的 `bond_timeout` 实测不生效；高负载时用 `use_composition:=True` 规避 |

## 4. 离线验证条件

**方法**：`tools/bench/nav_sim.py` 不接硬件，直接从 `room2.pgm` 射线求交生成 `/scan`，并按 `/cmd_vel` 积分出 `/odom` 和 TF。目标是验证 launch、参数、插件加载、TF 链和动作接口，不验证轮胎打滑、电机跟随和真实定位。

| 项 | 条件 |
| --- | --- |
| 地图 | `room2`，136 × 75 @0.05 m，origin `[-2.78, -2.53, 0]` |
| 起始位姿 | `(0, 0, 0)`，与建图起点一致 |
| 扫描 | 720 bin @6 Hz，量程 0.12~8 m，距离噪声 σ=0.01 m |
| 里程计 | 50 Hz，差速积分，无打滑；与真值同源 |
| 目标 | `(1.5, -0.5, 0)` |
| 动态障碍 | 无 |
| 运行方式 | `navigation.launch.py rviz:=false`，`use_composition=False`，controller 10 Hz |

复现命令：

```bash
tools/bench/nav_smoke.sh --timeout 120
# 或手动三终端
ros2 launch robot_bringup navigation.launch.py rviz:=false
python3 tools/bench/nav_sim.py --map mobile_manipulator_ws/src/robot_bringup/maps/room2.yaml
python3 tools/bench/nav_goal.py --goal 1.5 -0.5 0
```

## 5. 结果

**结论先说**：Nav2 生命周期正常拉起，AMCL 收敛，`NavigateToPose` 返回 `SUCCEEDED`，三次最终配置下的运行均无人工干预。**但这是离线仿真结果，不是实机到达精度。**

| 轮次 | AMCL 估计终点 `(x,y,yaw)` | 估计误差 | 仿真真值终点 `(x,y,yaw)` | 真值误差 | 耗时 | 结果 |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | `(1.505, -0.470, -0.162)` | 3.0 cm | `(1.53, -0.42, -0.13)` | **8.5 cm** | 14.1 s | SUCCEEDED |
| 2 | `(1.496, -0.458, -0.044)` | 4.2 cm | `(1.52, -0.41, -0.01)` | **9.2 cm** | 17.9 s | SUCCEEDED |
| 3 | `(1.496, -0.467, -0.203)` | 3.3 cm | `(1.52, -0.43, -0.19)` | **7.3 cm** | 13.4 s | SUCCEEDED |

- 误差定义：目标 `(1.5, -0.5, 0)` 与终点位姿的平面距离；航向单列说明。
- AMCL 估计与仿真真值之间仍有约 4~6 cm 偏差，这是稀疏地图 + 合成扫描下的定位误差，也是真值误差的主要来源。
- 三轮均未触发恢复行为；`controller_server` 输出 `Reached the goal!`，`bt_navigator` 输出 `Goal succeeded`。
- 航向：真值终点航向 -0.13 / -0.01 / -0.19 rad；前两轮在 0.15 rad 容差内，第三轮超出约 2.3°。`stateful` 目标检查器会在首次进入容差时锁定成功，之后的速度平滑/停车过程可能再转过一点；实机验收要把航向和位置分开记录。

### 5.1 调参前后的对比

在收紧 AMCL 观测模型之前（`sigma_hit=0.2`、120 束之前为 90 束、每 10 cm 更新），同样目标下 AMCL 估计误差 5~6 cm，但真值超调到 15~17 cm。原因是 AMCL 跟踪滞后，控制器用滞后的地图位姿判断"已到点"。收紧到 `sigma_hit=0.10`、每 5 cm 更新、120 束后，真值误差降到 7.3~9.2 cm。**这说明实机上必须用外部真值（地面标线/卷尺）验收，不能只看 AMCL 自己报的位姿。**

## 6. 调试中发现并修掉的问题

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| `controller_server` 启动即崩溃，`InvalidParameterTypeException` | `local_costmap` 的 `width` / `height` 是整型参数，YAML 写了 `3.0` | 改为 `3` |
| launch 抛 `name 'false' is not defined` | `nav2_bringup` 用 `PythonExpression('not ' + use_composition)` 求值 | 默认值与文档改为 Python 字面量 `False`/`True` |
| 终点前原地转向时 `Failed to make progress`，首次目标被 abort 后重发 | 进度检查只看位移，`0.25 m / 10 s` 把正常转向判成卡住 | `0.10 m / 20 s` |
| RViz 里无法设置目标点，`/bt_navigator` 一直 `inactive` | 启动时没有初始位姿，AMCL 不发 `map→odom`，`planner_server` 的全局代价地图卡在等 TF，lifecycle manager 没走完激活 | AMCL 默认 `set_initial_pose: true`（建图起点 `(0,0,0)`）；不同起点用 2D Pose Estimate 覆盖 |
| 运行约 10 s 后 `smoother_server IS DOWN`，整套节点被 lifecycle manager 关闭 | VM 负载高，默认 4 s bond 心跳超时误判 | 尝试 YAML 设 `bond_timeout: 10`，但 Humble 的 `nav2_bringup` 不给 lifecycle_manager 传 params_file，实测仍是 4 s；后续用 `use_composition:=True` 或降低负载规避 |
| `nav_goal.py` 一直报 `no /amcl_pose yet` | `/amcl_pose` 是 `RELIABLE + TRANSIENT_LOCAL`，工具用 `VOLATILE` 订阅，错过锁存消息 | 订阅 QoS 改为 `TRANSIENT_LOCAL` |
| AMCL 在稀疏地图上滞后约 12 cm | `sigma_hit=0.2` 对 0.05 m 栅格过宽，10 cm 才更新一次 | `sigma_hit=0.10`，5 cm / 0.05 rad 更新，120 束 |
| 实机设目标后约 50 s 停车，`0x0202` | 电池电量低，STM32 欠压/上报中断；换 USB 线后仍复现 | 充电后复测不再出现，问题关闭；`0x0401` 低压 WARN 同源 |

## 7. 未覆盖与实机验收

以下项目**没有**在本次测试中覆盖，不能据此判定 M5 完成：

- 真实底盘的速度跟随、加速度限制、轮胎打滑与里程计漂移；
- X2 实机 `/scan` 的噪声、遮挡与拖尾；
- `set_enable` 门控、200 ms 断流停车与导航的交互；
- 动态障碍、窄门、贴墙通过；
- 定位丢失后的恢复行为（spin / backup）是否安全；
- PC ↔ RK3566 的 DDS 网络延迟与丢包；
- 30 min 以上连续运行、内存与 CPU 占用。

### 7.1 实机问题：低电量导致 0x0202（已修复）

2026-11-05 实机尝试：在板端 `board.launch.py` + PC `navigation.launch.py` 能正常激活、`/odom` 与 `/scan` 有数据的前提下，设置目标后约 50 s 出现 `0x0202 通信超时`，底盘进入 ESTOP。板端记录：

- `serial link lost (533~2532 ms since last frame)`，随后 `no frames from MCU`；
- 22:04:40 内核记录 `usb 3-1: USB disconnect`，约 1 s 后重枚举；
- 更换 STM32 的 USB 线后物理掉线消失，但运动时 MCU 上报仍会停 0.5~2.5 s，`0x0202` 复现；
- 电池约 7.0~7.2 V，期间也出现过 `0x0401` 低压 WARN。

2026-11-06 确认根因：**电池电量低**。低电压下 STM32 欠压/复位，上报流中断 0.5~2.5 s，MCU 锁存 `0x0202`；同一原因触发 `0x0401` 低压 WARN。更换 USB 线后仍复现，说明线材不是这次 `0x0202` 的根因（换线解决的是 22:04 的物理掉线）。

充电后复测：`0x0202` 与 MCU 上报中断不再出现，问题关闭。导航栈本身正常，故障期间也已输出到 `/cmd_vel`，不是 Nav2 或膨胀层。本次尝试不作为 M5 到达精度验收数据，实机精度仍按下面的步骤补测。

实机验收建议：

1. 板端 `ros2 launch robot_bringup board.launch.py`，PC 端确认 `/odom`、`/scan`、TF 正常；
2. `ros2 launch robot_bringup navigation.launch.py`；AMCL 默认按 `(0,0,0)` 初始化，若开机点不同，先在 RViz 用 **2D Pose Estimate** 覆盖，并确认 `ros2 lifecycle get /bt_navigator` 为 `active`；
3. `python3 tools/bench/nav_goal.py --enable --goal X Y YAW`，先跑空载、低速；
4. 在地面用胶带标出目标点，至少 10 次，记录**真值**到达误差、航向误差、耗时和是否触发恢复；
5. 验收线：到达精度 < 10 cm；若 AMCL 估计与真值差超过 5 cm，先查地图质量、雷达安装角和 `angle_offset_deg`，不要靠调大目标容差掩盖。

## 8. 结论

**M5 离线闭环通过**：launch、AMCL、代价地图、NavFn、DWB、恢复行为与 `NavigateToPose` 在 `room2` 上完整跑通；三次最终配置运行的真值误差 7.3~9.2 cm，处于指标边缘，其中一次航向超出 0.15 rad 约 2.3°。实机 `0x0202` 已确认由低电量导致，充电后复测关闭（第 7.1 节）；**M5 的实机到达精度数据仍待按第 7 节补测，因此不标记为完成**。
