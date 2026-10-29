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

`urdf/` 与 `config/` 已开始填入，`maps/` 与 `launch/` 的其余条目随 M4～M5 加入。

## 坐标系与 TF 树

```text
map ──(SLAM / AMCL)── odom ──(base_bridge)── base_link
                                              ├──(robot_state_publisher)── deck2_link ── arm_base_link
                                              ├── laser_link
                                              └── wheel_fl / wheel_fr / wheel_rl / wheel_rr
```

竖直方向（离地高度）：底盘顶面 140 mm → 第二层板上表面 204 mm → 机械臂旋转轴 274 mm。三个臂节（`arm_link1~3`）等量完长度再挂到 `arm_base_link` 下面。

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
| `ground_clearance` | 0.040 | ✅ 底盘底面离地 |
| `wheelbase` / `wheel_separation` | 0.185 / 0.240 | ✅ 轴距已量；**轮距必须等于固件 `SET_PARAM 0x01`** |
| `deck2_top_z` | 0.204 | ✅ 由"层间 64 mm"换算 |
| `deck2_length` / `deck2_width` / `deck2_thickness` | 0.245 / 0.195 / 0.005 | ⬜ 沿用底盘尺寸，待实测 |
| `arm_base_height` | 0.070 | ✅ 底座高出第二层板 70 mm |
| `arm_base_x` / `arm_base_y` | 0.000 / 0.000 | ⬜ 待实测：底座在板上的位置 |
| `laser_x` / `laser_y` / `laser_z` | 0.080 / 0.000 / 0.150 | ⬜ 待实测；扫描平面不能与任何一层板相交，支撑柱也会挡出窄盲区 |

偏航角只写在 `config/lidar.yaml` 的 `angle_offset_deg`，URDF 的 `laser_yaw` 保持 0；两处都设会重复旋转。

## 规划

| 文件 | 内容 | 加入节点 |
| --- | --- | --- |
| `launch/base.launch.py` | 底盘桥接、串口参数、`robot_state_publisher` | M4 |
| `launch/lidar.launch.py` | X2 雷达驱动（`/scan`、`/lidar/points`） | M4 |
| `launch/slam.launch.py` | 激光建图与定位 | M4 |
| `launch/nav2.launch.py` | Nav2 导航 | M5 |
| `launch/vision.launch.py` | 相机与视觉推理 | M7 |
| `launch/task.launch.py` | 任务状态机 | M8 |
| `launch/robot.launch.py` | 整机启动编排 | M8 |

启动顺序按 `docs/architecture/overview.md`：串口桥接 → 定位/建图 → 导航 → 视觉 → 任务。

## 约定

1. 节点参数放 `config/*.yaml`，不硬编码在 launch 里；
2. 地图、URDF 与可公开的标定结果入版本库，设备序列号和日期写在文件头；
3. 串口设备号、WiFi 密码等环境相关配置不提交。
