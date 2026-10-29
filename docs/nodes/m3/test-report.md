# M3 节点测试报告：四轮速度环（PC 阶段）

> **结论：软件实现完成，L1 全部通过；M3 尚未验收。** 四轮 PID、编码器测速、超时/急停复位、开环回退、运行期逐轮调参和上位机调参页面已落地并有回归用例，但轮胎未到货，Keil 编译、台架闭环、速度误差和里程计标定都未执行。现阶段不能按“闭环达标”签署 M3。

| 项目 | 内容 |
| --- | --- |
| 测试日期 | 2026-10-29 |
| 节点 | M3 底盘闭环：PWM / 编码器 / PID / 里程计 |
| 版本 | 未发版 |
| 代码 | `feat/pid-tuning-app-refactor`（基于 `develop`） |
| 测试执行 | PC 单元测试（L1）；无台架、无真车 |

---

## 1. 本次范围

| 项 | 状态 | 说明 |
| --- | --- | --- |
| 通用 PID | ✅ | `03_Platform/platform_middleware/platform_pid`，位置式，微分作用于测量值，条件积分抗饱和，输出限幅 |
| 四轮速度环 | ✅ | `service_chassis_apply(now_ms, dt_ms)` 每轮独立计算，编码器累计计数差分得到实测速度 |
| 增益配置 | ✅ | `service_chassis_cfg_t` 提供每轮 `pid_kp/ki/kd`；应用层按左/右两组填初值 |
| 超时安全 | ✅ | 200 ms 无有效速度帧：目标清零、`platform_motor_stop_all()`、PID 冻结零输出；恢复后重采样编码器 |
| 使能/急停 | ✅ | 控制任务在 `apply()` 中检测状态跳变并复位 PID，避免通信任务并发写浮点状态 |
| 零目标防蠕动 | ✅ | 目标为 0 且实测低于 5 mm/s 时清积分、输出 0，防止刹车积分造成反转蠕动 |
| 开环回退 | ✅ | `counts_per_mm_x1000 = 0` 时退回原占空比映射 |
| 逐轮运行期调参 | ✅ | `SET_PARAM` `0x20~0x2B` → `appPid` 队列 → 控制任务 `service_chassis_set_pid_gain()`；只写 RAM |
| 轮速上报 | ✅ | `REPORT_WHEEL_SPEED`（`0xC6`）4 × `int16` FL/FR/RL/RR，20 ms |
| 上位机调参页 | ✅ | ABCD 多选、Kp/Ki/Kd 滑条、限速测试运动；仅软件冒烟，未接真机 |
| APP 模块化 | ✅ | 状态机/故障/命令/任务/上报/电池/user_init 拆分，常量任务表与队列表 |
| 里程计标定 | ⬜ | 仍使用计算值 24828 counts/mm，未做直行/原地旋转实测 |
| Keil/ARM 编译 | ⬜ | 未执行 |
| 台架闭环 | ⬜ | 轮胎未到货，底盘无法落地测试 |

---

## 2. L1 单元测试

编译选项 `-std=c99 -Wall -Wextra`，七个 C 套件全部退出码 0；上位机编解码测试 20 个用例通过。

| 套件 | 覆盖 | 结果 |
| --- | --- | --- |
| `tests/unit/platform/test_platform_middleware.c` | 环形缓冲、CRC、滤波、PID 参数校验、P/D、积分与抗饱和 | 全部通过 |
| `tests/unit/service/test_service_chassis.c` | 麦轮解算、速度反馈、积分复位、零目标、超时重启、开环回退、急停、里程计、逐轮增益 | PASS |
| `tests/unit/protocol/test_protocol_codec.c` | 协议表、组帧、解帧、CRC、轮速上报 | PASS |
| `tests/unit/service/test_service_proto.c` | 分发、ACK、SEQ 去重、上报组帧 | PASS |
| `tests/unit/service/test_service_key.c` | 短按/长按策略 | PASS |
| `tests/unit/app/test_app_state.c` | 状态迁移、清故障后不自动使能、WARN 保持 MANUAL | PASS |
| `tests/unit/app/test_app_fault.c` | 作用域表、WARN/ERROR/FATAL 降级、仲裁、条件未消失拒绝清除 | PASS |
| `tools/host_gui/test_protocol.py` | 编解码、轮速上报、PID 参数 ID | 20 tests OK |

实际输出：

```text
test_platform_middleware: 全部通过
test_service_chassis:     PASS: service_chassis tests
test_protocol_codec:      PASS: protocol codec tests
test_service_proto:       PASS: service_proto tests
test_service_key:         PASS: service_key tests
test_app_state:           PASS: app_state tests
test_app_fault:           PASS: app_fault tests
```

`01_APP` 全部 `.c` 另用 `-Wall -Wextra -Werror -fsyntax-only` 检查通过。

### 2.1 关键回归用例

| 用例 | 断言 |
| --- | --- |
| PID 参数非法 | 空指针、负增益、输出上下限相等时拒绝初始化 |
| 微分作用于测量值 | 设定值阶跃不产生微分冲击 |
| 条件积分 | 输出饱和且误差同向时不再累加积分 |
| 速度反馈 | 编码器给出 100 mm/s 后，P 输出从 400 降到约 199 |
| 使能复位 | 失能再使能后积分从零开始，不继承旧值 |
| 零目标 | 目标 0、实测 0 时清积分，占空比为 0 |
| 超时恢复 | 超时后新帧从新的编码器基线启动，不产生速度尖峰 |
| 开环回退 | `counts_per_mm_x1000 = 0` 时仍按目标映射占空比 |
| 里程计 | 24828 counts 对应 1000 mm，无新计数时位置不变 |
| 逐轮增益写入 | 改 FL 的 Kp 后只有 FL 的占空比变化，其余三轮不变 |
| 增益写入校验 | 轮号 4、增益序号 3、负值均被拒绝，原增益保持 |
| 故障降级 | WARN 只上报且保持 MANUAL；底盘 ERROR 只失能底盘；机械臂 ERROR 清零底盘目标；FATAL 全停并进 ESTOP |
| 故障清除 | 条件未消失（低压）返回 STATE；清除后回 IDLE，不自动使能 |

---

## 3. 参数与设计选择

| 项 | 当前值 | 依据 | 风险 |
| --- | --- | --- | --- |
| 控制周期 | 10 ms | 1 ms 时一个编码器计数约对应 40 mm/s，量化太粗 | 架构文档已同步为 10 ms 并记录理由 |
| 速度量化 | 约 4 mm/s / 10 ms | 24828 counts/mm | 低速段误差占比大，后续需要滤波或更长窗口 |
| 左轮增益 | Kp=1.0, Ki=0.6, Kd=0 | 保守启动值，非标定结果 | 负载、轮胎未定，可能偏慢或超调 |
| 右轮增益 | Kp=1.0, Ki=0.6, Kd=0 | 同上 | 左右电机差异未补偿 |
| 微分 | Kd=0 | 10 ms 速度噪声大 | 暂不引入微分噪声 |
| 里程计比例 | 24828 counts/mm | 13 线 × 4 倍频 × 90 / (π × 60 mm) | 轮径、打滑、胎压未实测 |
| 零速阈值 | 5 mm/s | 低于 10 ms 内一个计数对应的约 4 mm/s | 阈值需随控制周期/编码器变化复核 |
| 调参范围 | Kp/Ki ≤ 10.0，Kd ≤ 5.0 | 防止一次误拖把速度环打成开关 | 若极低速确需更高增益，先确认量化噪声能接受 |
| 增益存储 | RAM | 台架临时值，出厂默认尚未确定 | 掉电恢复编译期初值，重启后需重新下发 |
| 调参限速 | 150 mm/s，ω = 0 | 上位机页面约束 | 固件不限制，绕过页面直接发速度仍可高速 |

---

## 4. PID 调参与 APP 重构

### 4.1 调参链路

```text
上位机滑条 → SET_PARAM(0x20~0x2B) → app_handlers 校验 → appPid 队列
           → appCtrl 出队 → service_chassis_set_pid_gain() → platform_pid_init()
```

- 一轮一条 `SET_PARAM`，选中多轮时上位机对该增益发多帧；同一组滑条写所有选中轮。
- 回调不直接改 PID：PID 状态只有控制任务一个写者，避免通信任务与 10 ms 控制环并发写浮点。
- 队列长度 8，满时回 `FAILED`；上位机对每帧等 ACK，失败会在日志里标红。
- 写入时清该轮实测速度并重采样编码器，避免旧基线被当成速度尖峰。

### 4.2 APP 结构

`app_robot.c` 只保留 `init/start` 转发，其余拆成 `app_user_init / app_tasks / app_handlers / app_state / app_fault / app_report / app_battery`。任务与队列放在常量表里循环创建，加任务改表即可。

### 4.3 故障策略变化

本轮把 WARN 从"进 FAULT"改为"只上报"：低电压和 CRC 告警不再停车，底盘过流才失能底盘，通信超时和急停才全停。上位机把 `0x0202` 与 `0x0501` 都当作急停处理；`tools/bench/proto_loop.py` 的期望已同步（低压时仍应进 MANUAL）。**该行为未在台架验证。**

---

---

## 5. 未执行项与原因

| 测试 | 原因 |
| --- | --- |
| ~~Keil MDK 全量编译~~ | **已完成**：ARMCC V5.06，0 error / 0 warning |
| ~~电机正反转/编码器方向~~ | **架空已完成**：±150‰ 下四轮都为"正占空比 → 正轮速"，无一路反号；落地复测待补 |
| ~~速度闭环误差 < 5%~~ | **架空已完成**：Kp=5.0 / Ki=6.0，200 mm/s 下四轮误差 2.3~2.6 %，数据见 [`docs/bench/pid-tuning.md`](../../bench/pid-tuning.md)；落地后复测待补 |
| 里程计直行 2 m / 原地 360° | 需实际轮胎与场地 |
| 断流 200 ms 停车、急停 100 ms | 需真机台架复测 |
| 运行中调参 | 需台架：确认改增益后轮速跟随变化、其他轮不受影响、停车后重新加速无尖峰 |
| 掉电后增益 | 需真机：确认重启恢复编译期初值，上位机需重新下发 |
| 堵转/过流保护 | 硬件保护未接 |
| L2 回放、L4 实车 | ROS 节点与场地尚未就绪 |

---

## 6. 结论与下一步

1. 四轮速度环的软件路径已经完整：目标 → 逆解 → 限幅 → 编码器测速 → PID → 占空比；
2. 超时、使能、急停、零目标四条安全路径都有 L1 回归，未发现跨任务写 PID 状态的问题；
3. PID 增益和里程计比例都**不是标定结果**，不能作为精度结论引用；
4. 轮胎到位后按以下顺序补齐：
   - Keil 编译，确认 ARMCC 下无告警；
   - 架空底盘，确认四轮正反转与编码器方向；
   - 用上位机调参页逐轮整定：先 Kp、再 Ki，目标 0.3 m/s 误差 < 5%；整定完成后把增益回填 `app_robot_config.h`；
   - 复测调参期间其他轮不受影响、限速 150 mm/s 生效、急停会退出调参页；
   - 直行 2 m 与原地 360° 标定 `counts_per_mm_x1000`，回填 `app_robot_config.h`；
   - 复测断流、急停、零目标停车；
   - 将实测数据补进本报告，届时才可判定 M3 通过。

**M3 判定：未完成（软件实现与 L1 通过，台架与标定待补）。**
