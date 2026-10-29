# CLAUDE.md

六轴移动抓取机器人：RK3566（Linux + ROS 2 Humble）跑导航/视觉/语音，STM32F103RCT6 跑底盘与舵机实时控制，两者用 UART 自定义帧协议通信。

## 目录

| 路径 | 内容 |
| --- | --- |
| `firmware/app/` | STM32 固件，按 `01_APP → 02_Service → 03_Platform → 04_Impl → 05_Middlewares/06_BSP` 分层，依赖只向下 |
| `mobile_manipulator_ws/src/` | ROS 2 包（`robot_base` 串口桥、`robot_arm`、`robot_bringup`、`robot_task` 等） |
| `tools/host_gui/` | PC 上位机（`protocol.py` 是协议参考实现） |
| `tools/bench/` | 真机台架脚本（`proto_loop.py` 自动测试、`wheel_tune.py`、`storage_bench.py`） |
| `tests/unit/` | L1 单元测试，每个文件头注释里有完整 `gcc` 命令 |
| `docs/architecture/` | 协议、软件结构、Flash 布局、存储设计 |
| `docs/nodes/` | 各里程碑的测试报告与分层检查 |

**不入库**：`hardware/1.参考手册/`、`2.芯片手册/`、`3.原理图和引脚定义/`、`4.图纸资料/`（厂商手册），`firmware/app/MDK-ARM/app/`（Keil 编译产物）。不要提交它们。

## 实现 ROS 侧串口桥接前先读

1. `docs/architecture/protocol.md` —— 唯一真源。帧格式、数值编码、命令与上报表、CRC16-CCITT-FALSE（poly 0x1021，初值 0xFFFF）、超时语义、故障码、**第 10 节测试向量**、第 11 节检查清单。
2. `tools/host_gui/protocol.py` —— 已在真机验证的 Python 编解码，照它移植。
3. `tools/host_gui/test_protocol.py` —— 38 个用例，可直接对拍。
4. `tools/bench/proto_loop.py` —— 真实串口循环：重同步、等 ACK、SEQ 去重、上报分流。
5. `docs/architecture/software-design.md` 第 3 节 —— ROS 话题与 QoS 定义。

## 串口事实

| 项 | 值 |
| --- | --- |
| 参数 | 115200 8N1，无硬件流控 |
| Linux 设备 | `/dev/robot_stm32`（udev 规则见 `config/udev/`，未安装时是 `/dev/ttyUSB0`） |
| Windows 台架 | COM7（CH340） |
| 上报实际频率 | 约 40 Hz，**不是**文档表格里的 50 Hz（20 ms 周期 + 10 ms 通信节拍） |
| 时间戳 | 按**到达时间**打；不要假设固定周期 |

## 三个坑

1. **存储类命令的 ACK 表示"受理"，不是"写完"**：`PARAM_SAVE` 回 OK 后，结果看 `0xC8 REPORT_STORAGE` 的 bit0（无脏数据）/ bit3（保存失败）。
2. **没有 `GET_PARAM`**：设备当前参数读不回来，只能写。上位机页面上显示的是编译期默认值。
3. **`0x41`（死区补偿总开关）未实现**，返回 `0x01`；死区用 `0x54~0x5B` 每轮单独设。

## 构建与测试

```bash
# 固件（Keil uVision，ARMCC V5.06）
UV4.exe -b firmware/app/MDK-ARM/app.uvprojx -j0 -o build_log.txt

# L1 单元测试：每个文件头注释的 gcc 命令，产物在 build/
gcc ... tests/unit/service/test_service_chassis.c ... -o build/test_service_chassis -lm && ./build/test_service_chassis

# 上位机协议测试
python tools/host_gui/test_protocol.py

# ROS 2
cd mobile_manipulator_ws && colcon build --symlink-install
```

## 约定

- 分支：从 `develop` 拉功能分支，合回 `develop`，**不要提交 `main`**（见 `docs/conventions/commit-convention.md`）。
- 提交信息用中文，格式 `<type>(<scope>): <简述>`，首行 ≤ 50 字符，写清现象或量化结果。
- 提交信息与 PR **不加任何署名/水印标签**（不要 `Co-Authored-By`、不要 "Generated with"）。
- 提交/推送只在用户明确要求时做。
- 设计决策先问再做：协议字段、冻结接口、硬件归属这类"改起来牵扯面大"的决定，不要替用户猜。
- 文档与测试结论用中文写；结论必须带**测试条件**（架空/落地、电压、增益）。
