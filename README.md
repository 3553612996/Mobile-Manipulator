# Mobile-Manipulator

六轴移动抓取机器人 | RK3566 (ROS2) + STM32 双控架构 | 激光 SLAM 建图导航 · YOLO+RKNN 端侧视觉 · 手眼标定抓取 · Vosk 离线语音

## 系统概览

移动底盘 + 六轴机械臂的具身抓取系统，两块控制器按实时性分级分工：

| 控制器 | 职责 | 特点 |
| --- | --- | --- |
| RK3566（Linux + ROS 2） | SLAM 建图定位、Nav2 路径规划、YOLO/RKNN 视觉推理、Vosk 语音识别、任务状态机 | 计算密集、软实时 |
| STM32F103RCT6 | 底盘电机 PID 闭环、编码器测速、六轴关节插补、传感器采集、安全保护 | 任务简单、硬实时 |

两者通过 UART 自定义帧协议通信：帧头 + 长度 + 命令字 + 序号 + 负载 + CRC16。

## 目录结构

```
.
├── docs/           项目文档：architecture/ 架构与协议、conventions/ 规范、nodes/ 节点产出
├── hardware/       接线图、物料清单
├── firmware/       STM32 固件：CubeMX 工程 + 分层的 app 代码
├── ros2_ws/src/    ROS 2 功能包
├── models/         模型说明（权重不入库，见 models/README.md）
├── tools/          PC 侧工具：bench/ 台架测试、host_gui/ 上位机、docgen/ 文档生成
├── config/         udev 规则等系统配置
├── scripts/        构建与部署脚本
└── tests/          单元测试（按被测模块分目录，见 tests/README.md）
```

## 文档

| 文档 | 内容 |
| --- | --- |
| [项目介绍](docs/project-intro.md) | 背景、范围、技术选型理由、指标、里程碑 |
| [项目框架](docs/architecture/overview.md) | 双控分工、硬件拓扑、软件分层、数据流、部署 |
| [软件结构](docs/architecture/software-design.md) | 包划分、接口、状态机、并发、错误处理 |
| [串口协议](docs/architecture/protocol.md) | 帧格式、命令字、CRC、超时、故障码、测试向量 |
| [提交规范](docs/conventions/commit-convention.md) | 提交信息、分支模型、版本与标签 |
| [编码规范](docs/conventions/coding-style.md) | 命名、错误处理、ISR 约束 |
| [自测规范](docs/conventions/testing.md) | 测试分层、故障注入、发版检查 |
| [AI 使用说明](docs/conventions/ai-usage.md) | 使用边界与验证要求 |
| [M1 测试报告](docs/nodes/m1/test-report.md) | ROS 2 工作空间骨架、静态检查、板端构建待补项 |
| [M2 测试报告](docs/nodes/m2/test-report.md) | 协议闭环的台架结果、缺陷记录、已知限制 |
| [M2 架构分层检查](docs/nodes/m2/layering-audit.md) | 分层解耦检查方法、证据与差异说明 |

## 开发进度

- [x] 板端环境与 ROS 2 工作空间（骨架完成，板端构建待补）——见 [M1 测试报告](docs/nodes/m1/test-report.md)
- [x] 串口通信打通（协议解析 + CRC 校验 + 超时重发）——见 [M2 测试报告](docs/nodes/m2/test-report.md)
- [ ] 底盘闭环：PWM / 编码器 / PID / 里程计
- [ ] 激光建图与定位
- [ ] Nav2 导航
- [ ] 相机内参与手眼标定
- [ ] YOLO 训练与 RKNN 端侧部署
- [ ] 六轴机械臂控制与抓取联动
- [ ] 语音交互与意图解析

## 构建与运行

ROS 2 工作空间（RK3566 / Ubuntu + ROS 2 Humble）：

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

当前 M1 只有包骨架，节点与 launch 在 M3～M9 补充，详见 [ROS 2 工作空间](ros2_ws/README.md)。固件构建见 `firmware/` 与 `scripts/`。

## 说明

仓库不含数据集、录包、编译产物与模型权重。模型可通过 `tools/` 下的脚本配合 `models/README.md` 中记录的训练参数重新生成。
