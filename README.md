# Mobile-Manipulator

六轴移动抓取机器人 | RK3566 (ROS2) + STM32 双控架构 | 激光 SLAM 建图导航 · YOLO+RKNN 端侧视觉 · 手眼标定抓取 · Vosk 离线语音

## 系统概览

移动底盘 + 六轴机械臂的具身抓取系统，两块控制器按实时性分级分工：

| 控制器 | 职责 | 特点 |
| --- | --- | --- |
| RK3566（Linux + ROS 2） | SLAM 建图定位、Nav2 路径规划、YOLO/RKNN 视觉推理、Vosk 语音识别、任务状态机 | 计算密集、软实时 |
| STM32F411 | 底盘电机 PID 闭环、编码器测速、六轴关节插补、传感器采集、安全保护 | 任务简单、硬实时 |

两者通过 UART 自定义帧协议通信：帧头 + 长度 + 命令字 + 序号 + 负载 + CRC16。

## 目录结构

```
.
├── docs/           项目文档（见下方「文档」）
├── hardware/       接线图、物料清单
├── firmware/       STM32 固件：bootloader / app / shared
├── ros2_ws/src/    ROS 2 功能包
├── models/         模型说明（权重不入库，见 models/README.md）
├── tools/          模型转换、标定、数据采集脚本
├── config/         udev 规则等系统配置
├── scripts/        构建与部署脚本
└── tests/          单元测试
```

## 文档

| 文档 | 内容 |
| --- | --- |
| [项目介绍](docs/project-intro.md) | 背景、范围、技术选型理由、指标、里程碑 |
| [项目框架](docs/architecture.md) | 双控分工、硬件拓扑、软件分层、数据流、部署 |
| [软件结构](docs/software-design.md) | 包划分、接口、状态机、并发、错误处理 |
| [串口协议](docs/protocol.md) | 帧格式、命令字、CRC、超时、故障码、测试向量 |
| [提交规范](docs/commit-convention.md) | 提交信息、分支模型、版本与标签 |
| [编码规范](docs/coding-style.md) | 命名、错误处理、ISR 约束 |
| [自测规范](docs/testing.md) | 测试分层、故障注入、发版检查 |
| [AI 使用说明](docs/ai-usage.md) | 使用边界与验证要求 |

## 开发进度

- [ ] 板端环境与 ROS 2 工作空间
- [ ] 串口通信打通（协议解析 + CRC 校验 + 超时重发）
- [ ] 底盘闭环：PWM / 编码器 / PID / 里程计
- [ ] 激光建图与定位
- [ ] Nav2 导航
- [ ] 相机内参与手眼标定
- [ ] YOLO 训练与 RKNN 端侧部署
- [ ] 六轴机械臂控制与抓取联动
- [ ] 语音交互与意图解析

## 构建与运行

待补。参考 `scripts/` 下的脚本。

## 说明

仓库不含数据集、录包、编译产物与模型权重。模型可通过 `tools/` 下的脚本配合 `models/README.md` 中记录的训练参数重新生成。
