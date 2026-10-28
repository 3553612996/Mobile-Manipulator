# 上位机控制台 host_gui

移动抓取机器人的 PC 串口控制台，按 [`docs/architecture/protocol.md`](../../docs/architecture/protocol.md) 协议 v1 实现。
PySide6 绘制界面，pyserial 在独立线程里收发，解码后的帧通过 Qt 信号交给界面线程。

## 安装与运行

要求 Python 3.10+。

```bash
pip install -r tools/host_gui/requirements.txt
python tools/host_gui/main.py            # 选串口 → 连接
python tools/host_gui/main.py --sim      # 接内置模拟 MCU，无需硬件
python tools/host_gui/main.py --demo     # 模拟器 + 自动使能 + 轻微运动
python tools/host_gui/main.py --screenshot shot.png --size 1440x900
```

串口列表每 3 秒刷新一次；波特率 115200 / 230400 / 460800 / 921600。

## 功能

**底盘**
- 摇杆 + 线速度 / 角速度滑杆（±500 mm/s、±1500 mrad/s），50 Hz 周期下发 `SET_VELOCITY`
- 摇杆松开自动回中；`F2` 立即下发 v=0、ω=0
- 使能开关等待 MCU 的 ACK 才点亮；失能后滑杆归零并锁定

**急停**
- `F12` 或红色按钮：先补一帧零速，再发 `ESTOP`，本地立即锁存
- 顶部横幅显示「等待 MCU 确认」→「MCU 已确认」或「未确认」；MCU 主动上报 `EVENT 0x0501` 也会触发同一流程
- 解除必须点「解除急停 / 清故障」，之后仍需重新使能，不能一键恢复运动

**机械臂**
- 6 个关节滑杆（±90°，单位 centi-degree），旁边显示 MCU 回报的实际角度
- 夹持力 0–100 + 张开 / 闭合按钮（`SET_GRIPPER`，带 ACK）
- 「下发关节角度」一次性发送 `SET_ARM_JOINTS`

**遥测**
- `REPORT_STATUS`：状态、故障码、电池电压、温度、心跳计数、调度抖动
- `REPORT_ODOM`：X / Y / 航向 + 左右轮速曲线（最近 10 秒）
- 界面自算：接收帧率、CRC 错误数
- 超过 500 ms 没有收到任何帧即判定掉线：徽标变红、运动控件全部禁用
- 通信日志记录 TX、ACK、事件和错误；周期上报默认折叠，最多每秒 10 行

## 协议覆盖

| 方向 | 内容 |
|------|------|
| 下行 | `SET_VELOCITY` 50 Hz、`SET_ARM_JOINTS`、`SET_GRIPPER`（ACK）、`SET_ENABLE`（ACK）、`ESTOP`（ACK）、`CLEAR_FAULT`（ACK） |
| 上行 | `REPORT_ODOM` 50 Hz、`REPORT_ARM` 20 Hz、`REPORT_STATUS` 10 Hz、`REPORT_EVENT`、`HEARTBEAT` 10 Hz、所有 ACK |
| 未接入界面 | `SET_PARAM`（解码已实现，参数表未定稿，暂不暴露） |

时序：ACK 超时 200 ms、最多重试 3 次；解析器在 50 ms 无字节后复位（协议 §9）。
`LEN` 下限取 2 而不是 3 —— ESTOP 就是 7 字节空负载帧。

## 内置模拟器

`simulator.py` 提供虚拟从机：它吃进 GUI 真实发出的帧，按协议答 ACK 并周期上报，
因此没有硬件也能把界面和编解码跑通。物理模型很轻：差速里程计积分、限速的六关节、夹爪开合；
速度指令 200 ms 断流后自动归零，ESTOP 置故障 `0x0501` 并上报事件。
模拟器发出的帧也会先用本地 `FrameParser` 解析一遍再交给界面，顺便做了编解码自检。

固件已经实现同一套协议（`02_Service/service_proto` + `05_Middlewares/protocol`），
真机联调优先用 [`tools/bench/proto_loop.py`](../bench/proto_loop.py)：它不看界面，直接给出 PASS/FAIL。

## 测试

```bash
python tools/host_gui/test_protocol.py
```

17 个用例，覆盖 CRC-16/CCITT-FALSE、协议 §10 的三条测试向量、CRC 错误后的重同步、
非法 `LEN`（含 `0xAA` 字节引起的假帧头）、随机分片、半帧复位、ACK 字段与上报结构。

## 已知缺口

- `REPORT_STATUS` 的状态字节协议还没有定义枚举。界面暂按
  `0 初始化 / 1 空闲 / 2 手动 / 3 自动 / 4 急停 / 5 故障` 显示，固件需要沿用或回写协议。
- `SET_PARAM` 的参数 ID 表里左/右轮 PID 的划分待定（单轮还是共享），定稿后再做界面。
- 模拟器对任何指令都应答，因此 ACK 超时 / 重试路径无法在 `--sim` 下复现，需要真实硬件或人为断链验证。
