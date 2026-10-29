# ROS 2 工作空间

RK3566 侧的 ROS 2 Humble 工作空间。M1 只建立**可编译的包骨架**：每个包都有 `package.xml`、`CMakeLists.txt`、源码占位目录和职责说明，节点与 launch 在后续里程碑逐个加入。

## 环境

| 项目 | 要求 |
| --- | --- |
| 目标板 | RK3566，Ubuntu 20.04（aarch64），ROS 2 Humble |
| 开发机 | Ubuntu 20.04 / 22.04 + ROS 2 Humble（x86，用于 L1/L2 测试） |
| 构建系统 | `colcon` + `ament_cmake` |

## 目录

```text
ros2_ws/
├── README.md
└── src/
    ├── robot_interfaces/   自定义 msg / srv / action
    ├── robot_bringup/      launch、参数、地图、URDF
    ├── robot_base/         串口桥接、里程计
    ├── robot_arm/          关节控制、逆解、抓取
    ├── robot_vision/       图像输入、推理、目标定位
    ├── robot_speech/       Vosk 识别、意图解析
    └── robot_task/         任务状态机
```

## 构建

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

构建产物 `build/`、`install/`、`log/` 已在仓库根目录 `.gitignore` 中忽略，不入库。

## 包职责

| 包 | 职责 | 关键接口 |
| --- | --- | --- |
| `robot_interfaces` | 所有自定义消息、服务、动作 | `RobotStatus`、`SystemEvent`、`ClearFault`、`GetVersion`、`Grasp` |
| `robot_bringup` | 启动编排、参数、地图、URDF | launch 文件 |
| `robot_base` | ROS 2 话题 ↔ 串口帧，唯一的底盘硬件边界 | `/cmd_vel`、`/odom`、`/robot_status`、`/system_event` |
| `robot_arm` | 关节目标下发、逆解、抓取序列 | `/joint_states`、`/grasp` |
| `robot_vision` | 图像 → 检测框 → 三维坐标 | `/camera/color/image_raw`、`/detections` |
| `robot_speech` | 音频 → 文本 → 意图 | 语音事件 / 意图 |
| `robot_task` | 任务状态机，编排导航、视觉、机械臂、语音 | 动作与服务客户端 |

接口与 QoS 的完整定义见[软件结构](../docs/architecture/software-design.md#3-接口定义)。

## robot_interfaces 的状态

`RobotStatus`、`SystemEvent`、`ClearFault`、`GetVersion`、`Grasp` 目前是 **v0 草案**，字段按 `docs/architecture/protocol.md` 的现有上报/命令设计，尚未冻结。冻结前允许改字段，但每次改动必须同步：

1. `docs/architecture/software-design.md` 的接口表；
2. `robot_base` / `robot_arm` 两端的转换代码；
3. 本目录与包内 README。

冻结节点定在 M8 抓取闭环之前。

## M1 范围

- [x] 七个包的 `package.xml` 与 `CMakeLists.txt`
- [x] `include/<包名>/` 与 `src/` 占位目录
- [x] 自定义接口定义
- [x] 工作空间与各包 README
- [ ] 板端 `colcon build` 实测
- [ ] 功能节点与 launch 文件（后续里程碑）

验收记录见 [M1 测试报告](../docs/nodes/m1/test-report.md)。

## 新增包与节点

1. 包名用小写下划线，节点名用同一风格，文件名与节点名一致（见[编码规范](../docs/conventions/coding-style.md)）。
2. 新包必须声明 `package.xml` 依赖，CMake 里只 `find_package` 实际用到的包。
3. 参数放 `robot_bringup/config/*.yaml`，地图放 `robot_bringup/maps/`，URDF 放 `robot_bringup/urdf/`，均入版本库。
4. 不提交串口设备号、WiFi 密码、标定私密数据；可提交的标定结果必须带设备序列号与日期。
