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

当前四个目录都是占位，M4 起逐步填入。

## 规划

| 文件 | 内容 | 加入节点 |
| --- | --- | --- |
| `launch/base.launch.py` | 底盘桥接 + 串口参数 | M3 |
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
