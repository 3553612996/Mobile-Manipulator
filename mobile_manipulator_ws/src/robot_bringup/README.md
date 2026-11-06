# robot_bringup

启动编排、参数、地图与 URDF。**只有本包负责“按什么顺序启动哪些节点”**，其他包不互相拉起。

## 目录

```text
robot_bringup/
├── launch/   launch 文件
├── config/   节点参数、标定结果
├── maps/     2D 地图（SLAM Toolbox / Nav2）
└── urdf/     机器人模型与 TF
```

`urdf/`、`config/`、`maps/` 与 launch 已在 M4～M5 填入：建图、板端部署与 Nav2 导航均在本包编排。

## 坐标系与 TF 树

```text
map ──(SLAM / AMCL)── odom ──(base_bridge)── base_link
                                              ├──(robot_state_publisher)── deck2_link ── arm_base_link
                                              ├── laser_link
                                              └── wheel_fl / wheel_fr / wheel_rl / wheel_rr
```

竖直方向（离地高度）：底盘板面 50 mm → 第二层板上表面 102 mm → 机械臂旋转轴 167 mm；雷达扫描中心 92.4 mm。三个臂节（`arm_link1~3`）等量完长度再挂到 `arm_base_link` 下面。

| 变换 | 发布者 | 来源 |
| --- | --- | --- |
| `odom → base_link` | `robot_base` 的 `base_bridge` | 固件 `REPORT_ODOM`，yaw 解卷绕 |
| `base_link → deck2_link / arm_base_link / laser_link / wheel_*` | `robot_state_publisher` | `urdf/robot.urdf.xacro` |
| `map → odom` | SLAM Toolbox / AMCL | 建图与定位 |

本包**不发布** `map → odom`——那是建图/定位节点的事，两边都发会打架，现象是整块地图缓慢自转。

`base_link` 取底盘几何中心、`z = 0` 在地面；`x` 向前、`y` 向左。轮子用 `fixed` 关节：固件只上报转速，没有轮子转角，用 `continuous` 会因缺少 `joint_states` 让 `robot_state_publisher` 刷警告。

### URDF 待实测的尺寸

`urdf/robot.urdf.xacro` 顶部的属性都是占位值，量完直接改，不要动下面的 link。

| 属性 | 当前值 | 状态 |
| --- | --- | --- |
| `base_length` / `base_width` | 0.245 / 0.195 | ✅ 已量（含轮外缘） |
| `base_plate_top_z` | 0.050 | ✅ 2026-11-02：2 mm 板，板面离地 50 mm |
| `wheelbase` / `wheel_separation` | 0.170 / 0.165 | ✅ 2026-11-02 实测；**轮距必须等于固件 `SET_PARAM 0x01`** |
| `deck2_top_z` | 0.102 | ✅ 由层间 50 mm + 2 mm 板厚换算 |
| `deck2_length` / `deck2_width` / `deck2_thickness` | 0.245 / 0.195 / 0.002 | 长宽沿用底盘尺寸（⬜ 待实测）；板厚 ✅ |
| `arm_base_height` | 0.065 | ✅ 底座高出第二层板 65 mm |
| `arm_base_x` / `arm_base_y` | 0.0575 / 0.000 | ✅ 旋转底座中心距车头 65 mm 换算 |
| `laser_x` / `laser_y` / `laser_z` | 0.0625 / 0.000 / 0.0924 | ✅ 2026-11-02 实测；扫描平面不能与任何一层板相交 |

偏航角只写在 `config/lidar.yaml` 的 `angle_offset_deg`，URDF 的 `laser_yaw` 保持 0；两处都设会重复旋转。

## 规划

| 文件 | 内容 | 加入节点 |
| --- | --- | --- |
| `launch/base.launch.py` | 底盘桥接、串口参数、`robot_state_publisher` | M4 |
| `launch/lidar.launch.py` | X2 雷达驱动（`/scan`、`/lidar/points`） | M4 |
| `launch/mapping.launch.py` | 底盘 + 雷达 + SLAM Toolbox + RViz，本机串口建图 | M4 |
| `launch/mapping_external.launch.py` | 订阅主机 `/lidar/raw`、`/stm32/raw` 建图 | M4 测试 |
| `launch/board.launch.py` | RK3566 板端：底盘 + 雷达 + TF，不建图 | M4 |
| `launch/remote_mapping.launch.py` | PC 端：接收板卡 `/scan`/`/odom` + TF，跑 SLAM | M4 |
| `launch/slam.launch.py` | 激光建图与定位（后续拆分/定位模式） | M4 |
| `launch/navigation.launch.py` | PC 端 Nav2：AMCL + NavFn + DWB + 恢复行为 + RViz | M5 |
| `launch/vision.launch.py` | 相机与视觉推理 | M7 |
| `launch/task.launch.py` | 任务状态机 | M8 |
| `launch/robot.launch.py` | 整机启动编排 | M8 |

启动顺序按 `docs/architecture/overview.md`：串口桥接 → 定位/建图 → 导航 → 视觉 → 任务。

## 轮式 + 雷达建图（M4）

```bash
# 底盘和雷达分别占一个串口；PC 上按实际设备改
ros2 launch robot_bringup mapping.launch.py \
    base_port:=/dev/robot_stm32 \
    lidar_port:=/dev/ttyUSB0

# 另一个终端遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 走完一圈后保存地图
ros2 run nav2_map_server map_saver_cli -f maps/room1
```

建图链路：

```text
/scan (laser_link) ─┐
/odom + TF odom→base_link (base_bridge) ─┴─► slam_toolbox
                                              ├─► /map
                                              └─► TF map→odom
```

这一版**不接 IMU/EKF**，`odom→base_link` 只由 `base_bridge` 发布。以后加 EKF 时，把 `base.yaml` 的 `publish_tf` 设为 `false`，由 EKF 接替。参数在 `config/slam_toolbox.yaml`，RViz 布局在 `config/mapping.rviz`。

> 底盘和雷达如果都是 CH340，VID/PID 相同，`/dev/ttyUSB*` 可能互换；用 `config/udev/` 按物理口/序列号固定，或每次启动确认端口。

## Nav2 导航（M5）

运行位置：**PC**。板端 `board.launch.py` 只发 `/odom`、`/scan` 和 TF；PC 端不要重复启动 `robot_state_publisher` / `base_bridge` / `x2_lidar`，否则 TF 会出现两个发布者。

```bash
# 板端
ros2 launch robot_bringup board.launch.py

# PC 端（默认 maps/room2.yaml）
ros2 launch robot_bringup navigation.launch.py
# 换地图 / 不开 RViz / 使用组合容器
ros2 launch robot_bringup navigation.launch.py \
    map:=$PWD/mobile_manipulator_ws/src/robot_bringup/maps/room1.yaml rviz:=false
ros2 launch robot_bringup navigation.launch.py use_composition:=True

# 使能底盘（必须；IDLE 下 base_bridge 把 /cmd_vel 换成零速）
ros2 service call /set_enable std_srvs/srv/SetBool "{data: true}"

# 发目标（也可以用 RViz 的 Nav2 Goal）
python3 tools/bench/nav_goal.py --enable --goal 1.5 -0.5 0
```

启动后：

- `map_server` 加载地图，AMCL 发布 `map → odom`。AMCL 默认按建图起点 `(0, 0, 0)` 自动初始化；如果这次开机位置不在建图起点，用 RViz 的 **2D Pose Estimate** 或 `nav_goal.py --initial-pose X Y YAW` 覆盖。
- 没给初始位姿时 AMCL 不发布 `map → odom`，全局代价地图会卡在等 TF，Nav2 生命周期无法完成激活，RViz 里就点不了目标点。排查：`ros2 lifecycle get /bt_navigator`，应为 `active`。
- `NavigateToPose` 由 `bt_navigator` 提供；控制器输出经 `velocity_smoother` 变成 `/cmd_vel`，交给板端 `base_bridge`。
- 参数在 `config/nav2_params.yaml`：0.25 m/s、1.0 rad/s、差速模型、足迹 0.27 × 0.22 m、位置容差 0.05 m。

```text
/scan + /odom + TF ──► AMCL / costmaps ──► NavFn ──► DWB
                                              └──/cmd_vel_nav──► velocity_smoother ──/cmd_vel──► base_bridge
```

**离线验证**（不接硬件）：

```bash
tools/bench/nav_smoke.sh
```

`nav_sim.py` 按地图射线生成 `/scan`、用 `/cmd_vel` 积分 `/odom`，完整跑 AMCL + Nav2 + `NavigateToPose`。结果与实机验收清单见 [M5 测试报告](../../../docs/nodes/m5/test-report.md)。

注意：

- 导航前必须 `/set_enable true`；`IDLE` 下底座不动，但 AMCL 仍能定位。
- `/scan` 是 `BEST_EFFORT`；用 RViz 或 Best Effort 订阅才能看到。
- 2+8G 板卡不跑 Nav2；等 4+32G 版本再考虑迁移。

## 主机原始数据建图（当前 PC 测试）

主机上的 `/raw_serial_bridge` 节点把两个串口原始字节通过桥接网络转发到虚拟机：

| 主机 topic | 类型 | 内容 |
| --- | --- | --- |
| `/lidar/raw` | `std_msgs/msg/UInt8MultiArray` | X2 原始包流 `AA 55 ...` |
| `/stm32/raw` | `std_msgs/msg/UInt8MultiArray` | STM32 协议原始帧 `AA 55 ...` |

虚拟机里：

```bash
ros2 launch robot_bringup mapping_external.launch.py
# 默认 lidar_raw_topic:=/lidar/raw chassis_raw_topic:=/stm32/raw rviz:=true

# 或手动指定
ros2 launch robot_bringup mapping_external.launch.py \
  lidar_raw_topic:=/lidar/raw \
  chassis_raw_topic:=/stm32/raw \
  rviz:=true
```

`robot_raw_bridge` 的两个节点把原始字节还原成 `/scan`、`/odom`、`TF odom→base_link`、`/robot_status`，然后交给 `slam_toolbox`。这个包是测试用，最终上车仍用 `base_bridge` + `robot_lidar` 直接读串口。

## 约定

1. 节点参数放 `config/*.yaml`，不硬编码在 launch 里；
2. 地图、URDF 与可公开的标定结果入版本库，设备序列号和日期写在文件头；
3. 串口设备号、WiFi 密码等环境相关配置不提交。
