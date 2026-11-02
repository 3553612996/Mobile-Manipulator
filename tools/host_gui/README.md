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

**PID 调参**
- 顶栏「PID 调参」进入独立页面，离开或急停时自动停车并返回主界面
- A/B/C/D 四个轮可单选或多选（至少保留一个）；同一组滑条写所有选中轮
- Kp / Ki 0–10.0，Kd 0–5.0，步进 0.001；滑条停止 250 ms 后按 `SET_PARAM 0x20~0x2B` 下发，逐帧等 ACK
- 只选中一个轮时，滑条载入该轮当前增益；多选时滑条保持原值，写的值同时下发给所有选中轮
- 页面实时显示四轮实测轮速（`REPORT_WHEEL_SPEED`），可勾选测试运动：使能开关 + ±150 mm/s 滑条，角速度为 0
- 页面下方「速度跟随」曲线按轮分成 A/B/C/D 四个小面板：实线是实测轮速，虚线是下发的目标速度，窗口 6 s，共用一根 y 轴；悬停看该时刻四轮数值。四个轮用四个面板而不是四条同图曲线，是因为四色在深色底上无法通过全配对色盲校验，轮次身份由面板标题承担
- 增益只写 MCU 的 RAM，掉电恢复固件初值；重新进入页面不会自动重发

**标定**
- 顶栏「标定」进入独立页面；进入时先停车。标定写入要求底盘失能，使能时相关控件与保存按钮自动禁用
- 标定区：轮距、轮径、四轮编码器比例 `counts_per_mm`、四轮正向/反向起转占空比、四轮方向（`SET_PARAM 0x01/0x02/0x50–0x5F`）
- 参数区：控制指令超时、最大线速度、最大角速度（`0x10–0x12`，运行中可写）
- 「写入已修改项」逐帧发 `SET_PARAM` 并等 ACK，任何一帧失败就中止并显示原因，不做“改了没反应”的静默继续
- 保存是独立动作：`PARAM_SAVE scope=1` 存标定区、`scope=0` 存参数区。页面直接显示 `REPORT_STORAGE` 的 bit0（无脏数据）/ bit3（保存失败）以及两个 seq，ACK 只代表受理
- 「恢复出厂参数」= `PARAM_RESET scope=0` + `0xA5`，只重置参数区，标定区保留；恢复只改 RAM，仍需再保存

**存储器**
- 顶栏「存储器」进入独立页面，进入即发一次 `STORAGE_QUERY`
- 存储状态：8 个标志位按协议 bit 顺序显示，另有参数/标定 seq、累计启动、日志占用、擦除次数、上次保存秒数
- 日志导出：来源（运行日志/故障事件）+ 起始 seq，按 20 Hz 接收，结束帧关闭导出；按 seq 去重（导出期间环还在增长）；记录可保存为 CSV
- 崩溃现场：slot 0–15 或「最新有效」，按 offset 拼接、按总长判定完成；分片不连续直接中止并提示重读；十六进制 + ASCII 显示，可保存 `.bin`；勾选「读后标记已读」会在完成后刷新 `REPORT_STORAGE` bit6
- 清空：`LOG_CLEAR` 带二次确认（崩溃现场是唯一副本），可选运行日志 / 故障事件 / 崩溃现场 / 全部
- 日志导出 1.5 s、崩溃读取 1.0 s 没有新帧即判超时，不假装收完

**遥测**
- `REPORT_STATUS`：状态、故障码、电池电压、温度、心跳计数、调度抖动
- `REPORT_ODOM`：X / Y / 航向 + 左右轮速曲线（最近 10 秒）
- 界面自算：接收帧率、CRC 错误数
- 超过 500 ms 没有收到任何帧即判定掉线：徽标变红、运动控件全部禁用
- 通信日志记录 TX、ACK、事件和错误；周期上报默认折叠，最多每秒 10 行

## 协议覆盖

| 方向 | 内容 |
|------|------|
| 下行 | `SET_VELOCITY` 50 Hz、`SET_ARM_JOINTS`、`SET_GRIPPER`（ACK）、`SET_ENABLE`（ACK）、`SET_PARAM` PID 增益与标定/限幅 id（ACK）、`ESTOP`（ACK）、`CLEAR_FAULT`（ACK）、`PARAM_SAVE` / `PARAM_RESET` / `STORAGE_QUERY` / `LOG_EXPORT` / `CRASH_READ` / `LOG_CLEAR`（ACK） |
| 上行 | `REPORT_ODOM` 50 Hz、`REPORT_WHEEL_SPEED` 50 Hz、`REPORT_WHEEL_DUTY` 50 Hz、`REPORT_ARM` 20 Hz、`REPORT_STATUS` 10 Hz、`REPORT_STORAGE`、`REPORT_LOG`、`REPORT_CRASH`、`REPORT_EVENT`、`HEARTBEAT` 10 Hz、所有 ACK |
| 界面未用 | 台架测试占空比 `0x30~0x33` 只在 `tools/bench/wheel_tune.py` 里用；`0x41` 死区开关固件尚未实现 |

**固件实现状态**：`0x08~0x0D` 存储命令与 `REPORT_STORAGE`/`REPORT_LOG`/`REPORT_CRASH` 已实现（见 [storage-design.md](../../docs/architecture/storage-design.md) 第 12 节），存储器页可直接对真机使用。`SET_PARAM` 的标定/限幅 id（`0x01`/`0x02`/`0x10~0x12`/`0x50~0x5F`）也已实现：标定值写入 RAM 后由控制任务在下一个 10 ms 节拍推入底盘服务，热更新立即生效；标定类 id 在底盘使能时返回 `0x02`，页面已经据此禁用控件。`0x41` 死区总开关仍未实现（替换为按轮设置的 `0x54~0x5B`），本页不使用该 id。

时序：ACK 超时 200 ms、最多重试 3 次；解析器在 50 ms 无字节后复位（协议 §9）。
`LEN` 下限取 2 而不是 3 —— ESTOP 就是 7 字节空负载帧。

## 内置模拟器

`simulator.py` 提供虚拟从机：它吃进 GUI 真实发出的帧，按协议答 ACK 并周期上报，
因此没有硬件也能把界面和编解码跑通。物理模型很轻：差速里程计积分、限速的六关节、夹爪开合；
速度指令 200 ms 断流后自动归零，ESTOP 置故障 `0x0501` 并上报事件。
模拟器发出的帧也会先用本地 `FrameParser` 解析一遍再交给界面，顺便做了编解码自检。

模拟器也实现了存储子系统：`PARAM_SAVE`/`PARAM_RESET` 更新脏标志与 seq 并发 `REPORT_STORAGE`；
`LOG_EXPORT` 从种子日志环按 50 ms 一条推送、末尾发结束帧；`CRASH_READ` 推送一条 1 KB 的假现场
（32 B 一片）；`LOG_CLEAR` 清空对应区。标定 id 与限幅 id 按协议范围校验、使能时拒绝。
存储区放在 RAM 里，重启模拟器即恢复种子数据。

固件已经实现同一套协议（`02_Service/service_proto` + `05_Middlewares/protocol`），
真机联调优先用 [`tools/bench/proto_loop.py`](../bench/proto_loop.py)：它不看界面，直接给出 PASS/FAIL。

## 测试

```bash
python tools/host_gui/test_protocol.py
```

38 个用例，覆盖 CRC-16/CCITT-FALSE、协议 §10 的三条测试向量、CRC 错误后的重同步、
非法 `LEN`（含 `0xAA` 字节引起的假帧头）、随机分片、半帧复位、ACK 字段、上报结构、
轮速与占空比上报、PID/标定/限幅 ID 分组、`REPORT_STORAGE` 标志位顺序、崩溃分片拼接
（完整、offset 跳变、slot 切换、非法长度、reset）。

## 已知缺口

- 协议 v1 没有 `GET_PARAM`：标定页显示的是编译期默认值，不是设备当前值；断电重连后页面回到默认值，写入前请先确认页面上不是旧值。
- 模拟器对任何指令都应答，因此 ACK 超时 / 重试路径无法在 `--sim` 下复现，需要真实硬件或人为断链验证。
- 模拟器的轮速是差速近似，不模拟编码器量化与 PID 动态；调参效果只能在真机上看。
- 调参页面限速 150 mm/s、ω=0 是**界面约束**，固件不限制绕过页面的速度指令。
