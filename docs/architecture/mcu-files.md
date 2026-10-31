# MCU 侧文件说明

按"从应用层到驱动层"的顺序，逐个说明 `firmware/app/` 里每个文件**做什么、对外给什么接口**。

- 想看双控分工与整体架构，读 [overview.md](overview.md)；
- 想看包/模块划分与并发约定，读 [software-design.md](software-design.md)；
- 想看帧格式、命令字与故障码，读 [protocol.md](protocol.md)；
- 想看 flash 分区与掉电安全，读 [flash-layout.md](flash-layout.md)；
- ROS 2 侧对应的文件说明见 [ros2-files.md](ros2-files.md)。

本文只覆盖 `firmware/app/` 下**自己写的 68 个 `.c`**（约 1.9 万行），不含 CubeMX 生成的 `Core/`、`Drivers/`、`Middlewares/`，也不含 `firmware/商家源代码/`（只作移植依据，不入库）。

---

## 0. 分层总览

```text
01_APP          应用层   状态机 / 故障 / 参数 / 上报节拍 / 任务与队列
02_Service      服务层   底盘 / 机械臂 / 按键 / 协议 / 存储
03_Platform     平台层   OS 契约 / MCU 外设契约 / 板级设备 ops 表 / 算法中间件
04_Impl         适配层   STM32 HAL、FreeRTOS、BSP 驱动与上面契约之间的桥
05_Middlewares  中间件   protocol：纯 C 的帧编解码与命令表，无 OS、无硬件
06_BSP          驱动层   电机 PWM、总线舵机（YH ASCII）、W25Q64、按键、LED、蜂鸣器、电池 ADC
```

依赖只向下。每层的 `.h` 是契约、`.c` 是实现；上层只包含下层的头文件，不直接碰 HAL。

两个调用链要分清楚：

| 链路 | 路径 |
| --- | --- |
| 片内外设（GPIO/PWM/UART/…） | `02_Service` → `03_Platform/platform_mcu/*.c`（薄转发）→ `04_Impl/impl_wrapper`（带 trace）→ `04_Impl/impl_mcu`（真实 HAL 调用） |
| 板级设备（电机/舵机/flash/按键…） | `02_Service` → `platform_xxx_*()`（`03_Platform/platform_bsp` 的契约，注册与分派实现在 `04_Impl/impl_bsp/*.c`）→ `impl_bsp_xxx_ops` → `06_BSP` 驱动 |

### 0.1 启动顺序

1. CubeMX 的 `MX_*_Init()` 配好时钟与外设；
2. `board_init()`（`04_Impl/impl_board`）：填设备 `cfg`、逐个 `platform_*_register()`，并把电机桥、ADC 先初始化到安全状态；舵机总线只注册，等串口就绪后再初始化；
3. `app_user_init_init()`：两路串口（USART3 + DMA 环、UART5 半双工）、各 `service_*_init()`、加载参数/标定、按标定映射舵机、注册命令 handler、创建队列（**不建任务**）；
4. `osKernelStart()`，起始任务调用 `app_user_init_start()`：按任务表创建四个任务，把 UART RX 事件挂到 comm 任务。

### 0.2 任务与队列

| 任务 | 优先级 | 栈 | 周期 | 职责 |
| --- | --- | --- | --- | --- |
| `appComm` | HIGH | 1024 B | 10 ms 或 RX 事件唤醒 | 唯一读串口、唯一写串口：收帧、分发 handler、发 ACK、发上报、存储导出、上报机械臂故障 |
| `appCtrl` | ABOVE_NORMAL | 1024 B | 10 ms | 按键采样、PID 队列、`service_chassis_apply()`、里程计积分、机械臂保护判定、蜂鸣器 |
| `appStorage` | LOW | 1024 B | 10 ms 空闲 tick | 唯一写外部 flash：保存 blob、写日志环、导出、崩溃区 |
| `appServo` | LOW | 1024 B | 5 ms | 唯一驱动 UART5：舵机总线状态机、运动帧、PRAD/PRTV 轮询、配置请求 |

| 队列 | 长度 | 方向 |
| --- | --- | --- |
| PID / 测试占空比 | 8 | comm → control |
| 存储请求（描述符） | 8 | 任意任务 → storage |
| 导出记录 | 4 | storage → comm |

---

## 1. 应用层：`01_APP/app_robot/`

### 1.1 `app_robot.h` / `app_robot.c`

给 `main.c` 的两个入口。

| 函数 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `app_robot_init` | `void` | `platform_err_t` | 调度器启动前调用。初始化串口 DMA 传输与各服务，创建队列，注册 handler；不创建任务 |
| `app_robot_start` | `void` | `platform_err_t` | 在第一个 FreeRTOS 任务里调用。建任务、挂 UART RX 事件，此后整机运行 |

### 1.2 `app_user_init.h` / `app_user_init.c`

"接线"文件：任务表、队列表、各服务的 `cfg` 都在这里组装，避免 `app_robot_start()` 里堆一长串赋值。

| 函数 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `app_user_init_init` | `void` | `platform_err_t` | 串口、服务、参数加载、handler、队列；不建任务 |
| `app_user_init_start` | `void` | `platform_err_t` | 按 `s_task_table` 建四个任务，注册 UART RX 事件 |
| `app_user_init_pid_queue` | `void` | `platform_os_queue_t` | comm→control 队列句柄，init 之前返回 NULL |

任务表就是四行结构体（名字、入口、栈、优先级），加任务等于加一行。参数加载在这里完成：从 flash 读 param/calib 两个 blob，`app_params_parse()` 成功才覆盖默认值。机械臂的初始化顺序在这里固定为"用标定映射 → `platform_servo_init()` → `service_arm_init()` → `service_arm_enable(TRUE)`"，上电即保持姿态，不会因为等上位机第一条命令而掉下来。

### 1.3 `app_tasks.h` / `app_tasks.c`

四个任务体，以及跨任务的消息定义。**它是唯一允许 comm 任务和 control 任务互相"递话"的地方。**

消息种类（`app_ctrl_msg_kind_t`）：`PID_GAIN`（改单轮增益）、`TEST_DUTY`（开环标定占空比）、`PID_RELOAD`（PARAM_RESET 后整体重载）、`LIMITS_RELOAD`（只重载超时/限速，不动 PID 状态）。

| 函数 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `app_tasks_comm_entry` | `void *` | — | comm 任务体：等通知 → `service_proto_process()` → `app_storage_pump()` → 按键急停 → 机械臂故障待处理 → `app_fault_poll()` → `app_report_tick()` |
| `app_tasks_control_entry` | `void *` | — | control 任务体：按键、PID 队列、`service_chassis_apply()`、里程计积分、`service_arm_update()`、`app_tasks_watch_arm_fault()`、蜂鸣器，10 ms 一拍 |
| `app_tasks_servo_entry` | `void *` | — | 舵机总线任务体：每 5 ms 调一次 `service_arm_poll()`（唯一碰 UART5 的地方） |
| `app_tasks_post_pid_param` | wheel, gain, value | `platform_bool_t` | 入队一次增益修改；队列满返回 FALSE，调用方转成 ACK 失败 |
| `app_tasks_post_test_duty` | wheel, permille | `platform_bool_t` | 入队一次开环占空比 |
| `app_tasks_reload_params` | — | `platform_bool_t` | 请 control 任务把全部参数推给底盘服务 |
| `app_tasks_reload_limits` | — | `platform_bool_t` | 只推超时与限速（可在线改，不能重置 PID） |
| `app_tasks_request_estop` | `beep_until_ms` | `void` | control 任务（长按）请求急停；串口只由 comm 任务碰 |
| `app_tasks_take_pending_estop` | — | `platform_bool_t` | comm 任务取走急停请求 |
| `app_tasks_take_pending_arm_fault` | code out | `platform_bool_t` | comm 任务取走 control 锁存的机械臂故障码，用 `app_fault_raise()` 发 EVENT |
| `app_tasks_clear_arm_fault_pending` | — | `void` | `CLEAR_FAULT` 时允许同一条件再次上报 |

机械臂故障的跨任务约定：control 任务检测到条件后**先掉扭矩**（不等通信），把协议码放进一个单深度槽；comm 任务是唯一能发 EVENT 的，取走后按严重级别仲裁。这样保护动作与上报解耦，10 ms 的保护不会被 10 Hz 的上报节奏拖住。

`app_tasks_drain_pid_queue()`（文件内静态）在每拍 `apply()` 之前把队列清空，保证增益写入与 PID 计算不并发。

### 1.4 `app_state.h` / `app_state.c`

六态状态机：`INIT / IDLE / MANUAL / AUTO / ESTOP / FAULT`，数值与协议一致。

| 函数 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `app_state_init` | — | `void` | 从 INIT 开始 |
| `app_state_ready` | — | `void` | INIT → IDLE，表示可以接命令了 |
| `app_state_get` / `app_state_protocol` | — | 状态 / `uint8_t` | 内部读取 / 上报用的数值 |
| `app_state_motion_allowed` | — | `platform_bool_t` | 只有 MANUAL 接受速度与关节命令 |
| `app_state_enable_allowed` | — | `platform_bool_t` | IDLE 或 MANUAL 才允许使能驱动 |
| `app_state_on_enabled` / `_on_disabled` | — | `void` | 进入/退出 MANUAL |
| `app_state_on_estop` / `_on_fault` / `_on_cleared` | — | `void` | 进 ESTOP、进 FAULT、故障清除后回落 |

### 1.5 `app_fault.h` / `app_fault.c`

故障锁存表：每个故障码有严重级别（WARN/ERROR/FATAL）和降级范围（底盘/机械臂/全部）。

| 函数 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `app_fault_init` | — | `void` | 清空锁存 |
| `app_fault_code` | — | `uint16_t` | 当前锁存的位图，直接进 REPORT_STATUS |
| `app_fault_scope_get` | code | `app_fault_scope_t` | 该码的降级范围（与是否已锁存无关） |
| `app_fault_raise` | code, now_ms, force | `platform_bool_t` | 按级别仲裁并锁存，执行降级；`force` 给急停用；接受时发 EVENT |
| `app_fault_clear` | code, now_ms | `protocol_result_t` | 清一个码 / 0xFFFF 全清 / 清急停 |
| `app_fault_poll` | now_ms | `void` | 轮询型条件（200 ms 指令超时、连续 CRC 告警）的边沿检测，comm 任务每拍调用 |

未知故障码按 ERROR 处理——不能因为"表里没学过"就输给已锁存的 WARN。

### 1.6 `app_handlers.h` / `app_handlers.c`

把协议命令接到服务，并处理板载按键。

| 函数 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `app_handlers_register` | — | `platform_err_t` | 每个下行命令注册一个 handler（调度器启动前调用），含 `0x0E SERVO_CONFIG` |
| `app_handlers_on_key` | id, event, user_data | `void` | `service_key` 回调：短按在"前进 150 mm/s / 停"之间切换（仅 MANUAL、未急停时），长按立即急停并请求蜂鸣 |
| `app_handlers_process_pending` | now_ms | `void` | comm 任务里把急停请求和机械臂故障请求变成锁存故障并发 EVENT |

`app_handle_servo_config()` 是唯一会**阻塞等待**的 handler：请求交给总线驱动后每 1 ms 让出 CPU，最多等 `APP_SERVO_CONFIG_TIMEOUT_MS`（1000 ms）；拿到结果后把新 ID / 新零点写回 `app_params` 并置脏，所以改完 ID 不保存，下次上电会按旧地址找不到舵机。机械臂使能中返回 `0x05`，越界或 ID 冲突返回 `0x01`。

### 1.7 `app_report.h` / `app_report.c`

周期上报的节拍器，只做"到点就发"，数据从各服务读。

| 周期（`app_robot_config.h`） | 上报 |
| --- | --- |
| 20 ms（50 Hz） | `ODOM` |
| 20 ms（50 Hz） | `WHEEL_SPEED` + `WHEEL_DUTY`，永远成对发送（台架工具按到达顺序配对） |
| 50 ms（20 Hz） | `ARM`（0xC2，六关节角度） |
| 100 ms（10 Hz） | `STATUS`、`HEARTBEAT` |
| 500 ms（2 Hz） | `ARM_STATUS`（0xCB，电压、温度、逐关节故障位） |
| 事件驱动 | `EVENT`、`STORAGE`、`LOG`、`CRASH` |

| 函数 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `app_report_init` | — | `void` | 清各计时锚点 |
| `app_report_tick` | now_ms | `void` | comm 任务每拍调用一次，发到点的那几条 |

### 1.8 `app_battery.h` / `app_battery.c`

| 函数 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `app_battery_mv` | — | `uint16_t` | 电池电压 mV；还没有有效采样时返回 0 |
| `app_battery_low` | — | `platform_bool_t` | 有有效采样且低于 `APP_BATTERY_LOW_MV`（6800 mV） |

### 1.9 `app_params.h` / `app_params.c`

参数与标定的内存模型：默认值来自 `app_robot_config.h`，flash 里的 blob 覆盖默认值。

| 分类 | 主要接口 |
| --- | --- |
| 读 | `app_params_pid_gain_x1000`、`_wheelbase_mm`、`_half_length_mm`、`_counts_per_mm_x1000`、`_wheel_counts_per_mm_x1000`、`_wheel_dir`、`_deadzone_fwd/rev_permille`、`_cmd_timeout_ms`、`_max_linear_mm_s`、`_max_angular_mrad_s`、`_log_enabled` |
| 机械臂读 | `app_params_servo_id`、`_servo_offset_centi_deg`、`_arm_overtemp_c`、`_arm_undervolt_mv`、`_arm_overvolt_mv`、`_arm_stall_error_centi_deg`、`_arm_stall_time_ms`、`_arm_stall_margin_ms` |
| 写 | `app_params_set_pid_gain`、`_set_log_level`、`_set_track_mm`、`_set_wheel_diameter_mm`、`_set_wheel_counts`、`_set_wheel_dir`、`_set_wheel_deadzone`、`_set_cmd_timeout_ms`、`_set_max_linear`、`_set_max_angular` |
| 机械臂写 | `app_params_set_servo_id`、`_set_servo_offset`、`_set_arm_overtemp`、`_set_arm_undervolt`、`_set_arm_overvolt`、`_set_arm_stall_error`、`_set_arm_stall_time` |
| 应用 | `app_params_apply_to_chassis`（全部）、`app_params_apply_limits_to_chassis`（只超时/限速） |
| 序列化 | `app_params_serialize` / `app_params_parse`（小端、带版本，当前 v3：参数区 72 B、标定区 128 B，兼容读 v1/v2） |
| 脏标记 | `app_params_dirty` / `_mark_dirty` / `_mark_saved` |

v3 在标定区末尾加了 6 个舵机地址和 6 个标定零点（每项 int32），在参数区加了 6 个机械臂保护阈值。地址必须全局唯一，`app_params_parse()` 会做两两比对，冲突时整份标定不生效——用默认映射而不是带着一个会驱动两颗舵机的地址表运行。

约定：两个 `apply_*` **只能由 control 任务调用**（它拥有 `service_chassis_apply()`）；写入函数只校验并改 RAM，落盘由 `app_storage_save_blob()` 完成。

### 1.10 `app_storage.h` / `app_storage.c`

存储任务的对外接口。请求是**描述符**（约 24 B），不是数据本体；小记录先复制进两个静态 staging 槽，调用方的栈变量可以立刻失效。

| 函数 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `app_storage_init` | 请求队列, 导出队列, service_ok | `platform_err_t` | 绑定两个队列并记住服务是否挂载成功 |
| `app_storage_entry` / `app_storage_step` | `void *` / timeout | — / `void` | 任务体；`step()` 是"一次迭代"，供 L1 测试直接驱动 |
| `app_storage_save_blob` | which, data, len, tag, done | `platform_bool_t` | 排队保存参数/标定 blob；缓冲在回调前必须有效 |
| `app_storage_append_log` | level, tag, text, len | `platform_bool_t` | 排队写一条运行日志；被过滤/队列满/槽占满时丢弃并计数 |
| `app_storage_append_fault` | code | `platform_bool_t` | 写一条故障事件（模块号取码高位，级别查表） |
| `app_storage_clear` | source, done | `platform_bool_t` | LOG_CLEAR，擦除在存储任务里做 |
| `app_storage_export` | source, start, from_seq, done | `platform_bool_t` | 开始/停止日志导出，同时只允许一路 |
| `app_storage_crash_read` | slot, mark_read, done | `platform_bool_t` | 读崩溃区，0xFF 表示最新 |
| `app_storage_request_status` | — | `void` | 下一次 pump 发一条 REPORT_STORAGE |
| `app_storage_pump` | now_ms, link_alive | `void` | comm 任务每拍调用：取导出记录并序列化；链路静默 500 ms 时停导出 |
| `app_storage_available` / `_exporting` / `_saves_pending` / `_get_stats` | — / out | bool / void | 状态查询，PARAM_SAVE 用它回答 BUSY |

写日志的失败策略是**丢**：上报路径不能因为 flash 忙而变慢。

### 1.11 `app_robot_config.h`

不是代码，是全部编译期常量，改整机行为先看这里：

| 分组 | 内容 |
| --- | --- |
| 传输 | 上位机 `UART_BUS_0` 115200；舵机 `UART_BUS_1`（UART5）115200 半双工 |
| 几何 | 半轴距 150 mm、半轮距 120 mm、`COUNTS_PER_MM_X1000 = 24828`、每圈 4680 计数、轮径 60 mm |
| 死区 | 四轮正/反向可靠启动占空比（2026-10-29 台架实测：A 75/67、B 67/75、C 82/80、D 80/100） |
| 限幅 | vx/vy 500 mm/s、wz 1500 mrad/s、轮速 500 mm/s |
| 速度环 | Kp 5.0、Ki 6.0、Kd 0（编码器未滤波前不做微分） |
| 时序 | 控制 10 ms、舵机总线 5 ms、里程计/轮速 20 ms、关节 50 ms、状态/心跳 100 ms、`0xCB` 500 ms、指令超时 200 ms |
| 机械臂 | 关节限位 ±90°、默认运动 800 ms、过温 65 °C、欠压 5500 mV、过压 8400 mV、堵转 3° 持续 600 ms、稳定余量 1000 ms、配置超时 1000 ms |
| 其它 | 电量门限 6800 mV、长按 1500 ms、存储导出 20 Hz、各任务栈 |

flash 里存过的 blob 优先级高于这里的默认值；两者必须同步修改。

---

## 2. 服务层：`02_Service/`

### 2.1 `service_chassis/`

底盘服务：把"整车速度"变成四轮占空比，再把编码器积分成里程计。

| 函数 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `service_chassis_init` | `cfg` | `platform_err_t` | 绑定四个电机、几何、限幅、四轮 PID 增益、超时 |
| `service_chassis_set_velocity` | vx, vy, wz, now_ms | `platform_err_t` | 整车坐标目标；超限返回 PARAM 且保留旧目标 |
| `service_chassis_forward` / `_backward` / `_strafe` / `_rotate` | speed, now_ms | `platform_err_t` | 给按键和上层用的便捷封装 |
| `service_chassis_stop` | — | `void` | 目标清零，保持使能 |
| `service_chassis_enable` | enable, now_ms | `platform_err_t` | 使能/失能；急停锁存期间拒绝使能 |
| `service_chassis_emergency_stop` / `_clear_estop` | — | `void` / `platform_err_t` | 锁存急停（失能、拒绝使能）/ 解除 |
| `service_chassis_apply` | now_ms, dt_ms | `platform_bool_t` | 控制拍：看门狗、四轮速度环、写占空比；返回超时**刚刚**生效的那一拍 |
| `service_chassis_update_odometry` | dt_ms | `void` | 控制拍积分里程计（每拍都调，上报按 20 ms 取用） |
| `service_chassis_get_odometry` | `out` | `void` | x/y/航向（±180° 回绕）/左右轮速 |
| `service_chassis_get_wheel_speeds` / `_get_wheel_duty` | 四元数组 | `void` | 上报用实测轮速 / 实际占空比 |
| `service_chassis_set_pid_gain` / `_get_pid_gain` | wheel, gain, value | `platform_err_t` | 在线改单轮增益（会重置该轮 PID，仅 control 任务） |
| `service_chassis_set_geometry` | half_length, half_width | `platform_err_t` | 在线改几何（标定用，要求底盘失能） |
| `service_chassis_set_wheel_scale` | wheel, counts_per_mm_x1000, dir | `platform_err_t` | 单轮比例与方向；dir 只接受 ±1 |
| `service_chassis_set_start_duty` | wheel, fwd, rev | `platform_err_t` | 单轮正/反向可靠启动占空比 |
| `service_chassis_set_limits` | timeout, vx, vy, wz | `platform_err_t` | 在线改超时与限速，不碰 PID 状态 |
| `service_chassis_set_test_duty` | wheel, permille, now_ms | `platform_err_t` | 台架开环（≤400‰，300 ms 不刷新即溜车停） |
| `service_chassis_is_test_mode` / `_cmd_timed_out` / `_is_enabled` / `_is_estopped` | — | bool | 状态查询 |
| `service_chassis_counts_per_mm_x1000` | counts_per_rev, diameter_mm | `int32_t` | 由轮径推比例（这个公式只放在这一处） |
| `service_chassis_inverse_kinematics` | vx, vy, wz, k_mm, out[4] | `void` | 纯函数麦轮逆解，便于离线验证 |

超时期间的约定：四轮输出保持 0、积分持续清零；收到新帧后从新的编码器基线重启。`model` 参数里 `counts_per_mm_x1000 == 0` 表示没有编码器比例，退化为开环占空比。

### 2.2 `service_arm/`

六自由度总线舵机服务：位置命令、保护判定、配置转发。总线本身由驱动层管理，这里只维护业务状态。

| 函数 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `service_arm_init` | `cfg`（每关节 min/max/home、默认运动时间、保护阈值） | `platform_err_t` | 校验阈值与限位，初始为失能 |
| `service_arm_enable` / `_is_enabled` | enable | `platform_err_t` / bool | 使能输出；有故障锁存时拒绝使能 |
| `service_arm_set_joints` | 六个 0.01° 目标, move_time_ms | `platform_err_t` | 整帧先校验再下发，越限返回 PARAM，未使能返回 FAIL，旧目标保留 |
| `service_arm_poll` | now_ms | `platform_err_t` | 只做 `platform_servo_poll()`，由 `appServo` 任务调用（总线唯一主人） |
| `service_arm_update` | now_ms | `platform_err_t` | 每 10 ms 评估过温/欠压/过压/堵转，锁存到每关节位图；命中即掉扭矩 |
| `service_arm_get_status` | out | `platform_err_t` | 使能、故障锁存、每关节故障位（`0xCB` 用） |
| `service_arm_has_fault` / `_clear_faults` | — | bool / `platform_err_t` | 查询 / 清应用侧锁存（`CLEAR_FAULT` 调用，驱动侧离线位由下一次轮询结果决定） |
| `service_arm_get_targets` / `_get_states` | out | `platform_err_t` | 最后一次接受的目标 / 总线回读的角度、温度、电压、故障位 |
| `service_arm_get/set_protection` | protection | `platform_err_t` | 读/改阈值，`SET_PARAM 0x60~0x64` 用 |
| `service_arm_config` | `req{joint,op,value}` | `platform_err_t` | 转发配置；使能中返回 BUSY |
| `service_arm_config_ready` / `_take` / `_cancel` | out_value | bool / `platform_err_t` / void | 轮询配置结果、取走结果、放弃超时请求 |

堵转判定的前置条件有两道：**必须收到过至少一条 `SET_ARM_JOINTS`**（`has_target`，否则上电时舵机只是保持当前姿态，与 home 无关），**并且已过"运动时间 + 稳定余量"**（余量要覆盖约 0.8~1 s 的角度扫描周期，否则会拿运动中途的旧读数判堵转）。

### 2.3 `service_key/`

按键去抖与短按/长按识别。

| 函数 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `service_key_init` | cfg, callback, user_data | `platform_err_t` | 绑定按键 BSP 与回调；`long_press_ms = 0` 用默认 1500 ms |
| `service_key_update` | elapsed_ms | `platform_err_t` | 采样并最多产生一个事件；传入真实间隔，不用标称周期 |
| `service_key_held_ms` | — | `uint32_t` | 当前按下时长 |

### 2.4 `service_proto/`

串口服务：收、分发、回 ACK、发上报。**唯一碰 UART 的一层。**

| 函数 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `service_proto_init` | `cfg{bus}` | `platform_err_t` | 初始化解析器与统计 |
| `service_proto_register` | cmd, handler, user_data | `platform_err_t` | 注册一个下行命令的 handler；重复注册报错 |
| `service_proto_process` | now_ms | `platform_err_t` | 吸干环形缓冲、逐帧分发、发 ACK；含 50 ms 半帧复位与 1000 ms SEQ 去重 |
| `service_proto_send_odom` | x, y, heading, vl, vr | `platform_err_t` | 0xC1 |
| `service_proto_send_arm` | 六关节, 夹爪位置, 状态 | `platform_err_t` | 0xC2 |
| `service_proto_send_wheel_speed` / `_wheel_duty` | 四元数组 | `platform_err_t` | 0xC6 / 0xC7 |
| `service_proto_send_status` | state, fault, mv, temp, jitter_us | `platform_err_t` | 0xC3 |
| `service_proto_send_event` | code, timestamp_ms | `platform_err_t` | 0xC4 |
| `service_proto_send_heartbeat` | state, counter | `platform_err_t` | 0xC5 |
| `service_proto_send_storage` | flags, level, seq… | `platform_err_t` | 0xC8 |
| `service_proto_send_log` / `_send_crash` | 记录字段 / 分片 | `platform_err_t` | 0xC9 / 0xCA，长度按协议上限截断 |
| `service_proto_rx_age_ms` | now_ms | `uint32_t` | 距最后一帧有效下行的时间 |
| `service_proto_has_received` / `_crc_alarm` | — | bool | 是否收到过帧；连续 10 帧 CRC 错后报警 |
| `service_proto_get_stats` | out | `void` | 帧数、CRC 错、长度错、重同步、未知命令、ACK 数、发送失败 |

handler 签名固定为 `protocol_result_t (*)(const protocol_frame_t *, void *)`，返回值就是要回给上位机的结果码。

### 2.5 `service_storage/`

外部 flash 的唯一管理者：分区、blob、两个日志环、崩溃区、磨损统计。

| 函数 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `service_storage_init` | now_ms | `platform_err_t` | 挂载 flash、恢复磨损计数、累加启动次数、探测 blob 有效性 |
| `service_storage_save_blob` | which, data, len, now_ms | `platform_err_t` | 擦-写-回读校验；新副本校验通过前旧副本仍有效 |
| `service_storage_load_blob` | which, buf, cap, out_len, out_seq | `platform_err_t` | 读出一个 blob |
| `service_storage_blob_valid` / `_blob_seq` | which | bool / `uint32_t` | 有效性、记录序号 |
| `service_storage_append` | ring, level, tag, text, len, now_ms | `platform_err_t` | 往故障环/日志环追加一条 |
| `service_storage_iter_begin` / `_iter_next` | ring, iter, from_seq / entry | `platform_err_t` | 从旧到新遍历；"没有更多"与 I/O 错统一报告 |
| `service_storage_ring_clear` / `_ring_used` / `_ring_erase_total` | ring | `platform_err_t` / `uint32_t` | 逻辑清空（失效扇区头）、已用字节、累计擦除次数 |
| `service_storage_crash_write` | data, len, now_ms | `platform_err_t` | 写最旧槽：扇区尾 4 B 的提交 CRC 最后写 |
| `service_storage_crash_info` / `_read` / `_mark_read` / `_crash_clear` | slot, … | `platform_err_t` | 崩溃区读取与清除 |
| `service_storage_crash_slot_valid` / `_latest` / `_crash_unread` | — | bool | 只用缓存、不碰总线，供协议任务快速回答 |
| `service_storage_get_status` | out | `void` | REPORT_STORAGE 的字段 |
| `service_storage_flush_wear` | now_ms | `platform_err_t` | 立即写磨损记录（保存后、正常关机） |
| `service_storage_deinit` | — | `platform_err_t` | 卸载 |

分区表（W25Q64 8 MB）全部以宏写在头文件里：blob 双副本 A/B、镜像 A/B、崩溃区 16 槽、故障环 32 扇区、日志环 224 扇区、磨损记录双副本、参数/标定在 0x050000/0x060000。

---

## 3. 平台层：`03_Platform/`

### 3.1 `platform_common/`

| 文件 | 内容 |
| --- | --- |
| `platform_type.h` | 定宽整数、`float_t`、`platform_bool_t` |
| `platform_error.h` | `platform_err_t` 枚举（OK/TIMEOUT/PARAM/BUSY/…）与 `PLATFORM_IS_OK/IS_ERR` |
| `platform_def.h` | 上面两个的汇总，另有 `platform_delay_ms/us` |

### 3.2 `platform_os/`

FreeRTOS 抽象。上层只认这里，换 OS 不改业务代码。

| 文件 | 接口 |
| --- | --- |
| `platform_os_task.h` | `task_create`（填 `cfg`）、`delete`、`delay_ms`、`delay_until`、`notify(_from_isr)`、`wait_notify`、`get_current`、`yield` |
| `platform_os_queue.h` | `queue_create/send/receive/peek/count/free/reset/destroy`，带 `_from_isr` 变体 |
| `platform_os_time.h` | `tick_get`、`ms_to_ticks`、`ticks_to_ms`、`is_in_isr`、`kernel_init/start` |
| `platform_os_critical.h` | `critical_enter` 返回状态、`critical_exit(state)` |
| `platform_os_mutex/semaphore/event/timer.h` | 契约已定义，**目前没有实现体**（见第 8 节） |
| `platform_os_common.h` | `WAIT_FOREVER`、`NO_WAIT`、优先级枚举 |

`platform_os_*` 的实现都在 `04_Impl/impl_os/impl_os.c`，没有本层的 `.c`。

### 3.3 `platform_mcu/`

芯片外设契约，外加一层极薄的转发 `.c`（只调 `impl_*_wrap()`，不带逻辑）。

| 文件 | 接口 |
| --- | --- |
| `platform_mcu_gpio.h` | `init/deinit/write/read/toggle/irq_attach/irq_detach` |
| `platform_mcu_uart.h` | `init/write/read/write_async/read_async/get_rx_count/rx_read/rx_available/flush_rx/register_event_cb` |
| `platform_mcu_pwm.h` | `init/start/stop/set_duty(‰)/set_frequency/get_duty` |
| `platform_mcu_encoder.h` | `init/read_delta(int16_t*)/get_count/reset` |
| `platform_mcu_adc.h` | `init/read_raw/read_mv`，`reference_mv = 0` 用板级默认 |
| `platform_mcu_spi.h` | 同步 `transfer/write/read`、异步 `write_async` + 完成回调、`cs_set`、`lock/unlock` |
| `platform_mcu_i2c.h` | 普通读写、寄存器读写（1/2 字节地址）、`is_ready` 探测、`lock/unlock` |
| `platform_mcu_timer.h` | 单次定时器：`init/start/stop/set_next_interval_us`，回调里可以续下一次 |
| `platform_mcu_wdg.h` | `init(timeout_ms)`、`feed`、`get_reset_reason` |

### 3.4 `platform_bsp/`

板级设备契约，全部是 ops 表：先 `platform_xxx_register(&ops, context)`，再 `platform_xxx_init()`。上层拿到的是**句柄与整数**，看不到 GPIO 号。

| 文件 | 接口 |
| --- | --- |
| `platform_bsp_motor.h` | 4 个电机 id：`register/init/deinit/set_enable/set_duty(‰)/get_encoder/get_state/stop_all` |
| `platform_bsp_servo.h` | 六关节：`register/init/deinit/set_enable/set_angles(角度, 时间)/get_states`，另有 `set_joint_map`（地址+零点，init 前调用）与 `config/config_ready/config_take/config_cancel`（改 ID、偏置、零点、力矩） |
| `platform_bsp_key.h` | 4 个按键 id：`register/init/deinit/poll(elapsed_ms)/get_state(按下、原始电平、按住时长)` |
| `platform_bsp_led.h` / `_buzzer.h` | `register/init/deinit/set(on)` |
| `platform_bsp_battery.h` | `register/init/deinit/read(state{mv, raw, valid})` |
| `platform_bsp_flash.h` | `register/init/deinit/read/write/erase/get_info/is_busy` |
| `platform_bsp_temphumi.h` | `register/init/deinit/read(data, timeout)`——**没有对应驱动** |

注册与分派函数（`platform_motor_register` 等）的实现不在本层，在 `04_Impl/impl_bsp/`。

### 3.5 `platform_middleware/`

与硬件无关的算法，纯 C、可 PC 单测。

| 文件 | 接口 |
| --- | --- |
| `platform_crc.h` | `crc16_ccitt(_update)`、`crc32(_update)`；常量含 CCITT-FALSE 的 0x1021/0xFFFF 与校验值 0x29B1 |
| `platform_ringbuf.h` | 单生产者单消费者：`init/write/read/write_byte/read_byte/peek/drop/count/free/is_empty/is_full/reset`；短返回不是错 |
| `platform_filter.h` | 滑动平均（int64 求和防溢出、首样本直接初始化）、一阶低通 `y += α(x-y)` |
| `platform_pid.h` | `init/reset/update(setpoint, measurement, dt_s)/get_output`；输出限幅、dt ≤ 0 返回 0 |
| `platform_logstore.h` | 扇区化追加日志：`init/append/clear/iter_begin/iter_next/used_bytes/written_bytes/wrap_count/erase_total` |
| `platform_wearlevel.h` | 选扇区：`pick(hint)`、`pick_filtered(…, filter, …)`、`note_erase`、`set/get_count`、`min/max/average` |

`logstore` 的 `clear()` 只把每个扇区头改 2 字节（逻辑清空），不做 224 次擦除——掉电中途也是自洽的。

---

## 4. 适配层：`04_Impl/`

### 4.1 `impl_board/board_init.c`

板级接线表，整个项目**唯一**知道"哪个电机接哪个 PWM 通道、舵机挂在哪条串口"的地方。

| 内容 | 说明 |
| --- | --- |
| 静态实例 | `s_motor[4]`、`s_servo`、`s_key`、`s_led`、`s_buzzer`、`s_battery`、`s_flash` |
| `s_motor_wiring` | A: PWM0/1，B: PWM2/3，C: PWM5/6，D: PWM4/7 |
| `s_servo.cfg` | UART5（`PLATFORM_UART_BUS_1`）、6 关节、脉宽 500~2500 µs、帧间隔 100 ms、应答超时 30 ms、连续 3 次判离线；出厂映射为"关节 n ↔ 舵机 n、零点 0" |
| `board_init()` | 逐项填 cfg → `platform_*_register()` → 初始化必须先活的设备（电机桥滑行、ADC 校准）；舵机只注册不初始化，等 `app_user_init` 把持久化映射装进去 |

`board_types.h` 只有定宽 typedef（STM32F103 上 `long` 是 32 位，`int64` 必须写 `long long`）。

### 4.2 `impl_mcu/`

HAL 适配器：每个外设一个文件，**唯一出现 `stm32f1xx_hal.h` 的地方**。

| 文件 | 内容 |
| --- | --- |
| `impl_mcu_gpio.c` | 逻辑端口 0..4 → GPIOA..GPIOE；EXTI 16 线分发给 `platform_gpio_irq_attach()` 注册的回调；不碰 CubeMX 配好的复用脚 |
| `impl_mcu_uart.c` | 阻塞收发、DMA 异步、环形接收、事件回调 |
| `impl_mcu_pwm.c` | 占空比 ‰ → CCR；启停通道 |
| `impl_mcu_encoder.c` | 16 位硬件计数 → 差值；软件侧 32 位累加在 BSP 电机驱动里 |
| `impl_mcu_adc.c` | 校准、单次转换、raw 与 mV 两种读法 |
| `impl_mcu_spi.c` / `impl_mcu_i2c.c` | 同步传输、异步传输、片选、总线锁 |
| `impl_mcu_timer.c` | 单次定时器与回调续期 |
| `impl_mcu_wdg.c` | IWDG 启动、喂狗、复位原因 |
| `impl_mcu_time.c` | 微秒/毫秒延时与 `platform_delay_*` |

### 4.3 `impl_wrapper/`

`impl_xxx_wrap()`：与 `impl_mcu_xxx()` 一一对应的薄包装，只多了 trace 打点（`IMPL_TRACE_ENTER/EXIT`、`IMPL_TRACE_ISR_*`），方便串口/调试器看调用序列。`impl_mcu_wrapper.h` 汇总全部声明。

### 4.4 `impl_bsp/`

ops 表 + 注册表。`impl_bsp_motor.c` 里既有 `impl_bsp_motor_ops`（五个函数指针指向 `bsp_motor_dual_pwm_*`），也有 `platform_motor_register/init/set_duty/...` 的分派实现（槽位表记录 ops、context、registered、initialized）。

| 文件 | 对应驱动 |
| --- | --- |
| `impl_bsp_motor.c` | `bsp_motor_dual_pwm` |
| `impl_bsp_servo.c` | `bsp_servo_bus`（`bsp_servo_softpwm` 为未注册的备用驱动） |
| `impl_bsp_flash.c` | `bsp_flash_w25q64` |
| `impl_bsp_key.c` | `bsp_key_gpio` |
| `impl_bsp_led.c` / `impl_bsp_buzzer.c` | `bsp_led_gpio` / `bsp_buzzer_gpio` |
| `impl_bsp_battery.c` | `bsp_battery_adc` |

### 4.5 `impl_os/impl_os.c`

FreeRTOS 实现（657 行）。已落地：任务（含静态池与 trampoline）、时间、临界区、队列。未落地：互斥量、信号量、事件组、软件定时器——头文件有契约，链接时会缺符号。

---

## 5. 协议中间件：`05_Middlewares/protocol/`

纯 C，不依赖 OS，也不含硬件调用；ROS 2 侧的 `robot_base/protocol.cpp` 与它保持同一份线格式。

### 5.1 `protocol_def.h`

常量与命令表：帧头 `AA 55`、`LEN 2..66`、ACK 标志 `0x80`、50 ms 半帧复位、命令字 0x01–0x0E、上报字 0xC1–0xCB、结果码、每个命令/上报的**固定负载长度**、参数 id（0x01–0x64，含 PID A/B/C/D、台架测试占空比、机械臂保护阈值）、故障码与严重级别、轮序 FL/FR/RL/RR、关节故障位与 SERVO_CONFIG 操作码。

| 函数 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `protocol_cmd_lookup` | cmd | `const protocol_cmd_info_t *` | 查命令表；未知/ACK/上报返回 NULL |
| `protocol_cmd_payload_ok` | cmd, len | `platform_bool_t` | 负载长度是否**恰好**等于表里承诺的值 |
| `protocol_fault_severity` | code | `protocol_severity_t` | 故障级别；未知码按 ERROR |

### 5.2 `protocol_codec.h`

| 函数 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `protocol_parser_init` / `_reset` | parser | `void` | 复位解析器；`reset` 保留统计 |
| `protocol_parser_feed` | data, len, now_ms | `uint32_t` | 追加字节；缓冲满时返回实际接收数，间隔超 50 ms 先丢半帧 |
| `protocol_parser_tick` | now_ms | `void` | 定期调用，处理"长时间没有新字节"的半帧超时 |
| `protocol_parser_next` | out | `platform_bool_t` | 取出下一个完整且 CRC 通过的帧 |
| `protocol_encode` | cmd, seq, payload, len, out, size | `uint32_t` | 组帧，返回总长；装不下返回 0 |
| `protocol_get/put_i16/u16/i32` | src / dst, value | — | 小端字段读写 |

结构体：`protocol_frame_t{cmd, seq, payload_len, payload[64]}`、`protocol_parser_stats_t{frames_ok, crc_errors, len_errors, resyncs}`。

---

## 6. 驱动层：`06_BSP/`

只做"把芯片外设变成物理量"，不含业务判断。每个驱动一个实例结构体，`cfg` 由 `board_init()` 填。

### 6.1 `bsp_motor_dual_pwm.c`

一个电机 = 两路 PWM（正/反转输入）+ 一路正交编码器。

| 函数 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `bsp_motor_dual_pwm_init` | motor（cfg 已填） | `platform_err_t` | 配 PWM 与编码器，两个桥臂拉低（滑行），驱动保持失能 |
| `bsp_motor_dual_pwm_set_enable` | motor, enable | `platform_err_t` | 使能时启动输出并重放占空比；失能时两臂拉低且停输出 |
| `bsp_motor_dual_pwm_set_duty` | motor, permille | `platform_err_t` | 有符号占空比，两路 PWM 交替斩波 |
| `bsp_motor_dual_pwm_update_encoder` | motor | `platform_err_t` | 读一次硬件差值并累加到 32 位位置；**每个控制拍只调一次** |
| `bsp_motor_dual_pwm_get_encoder` | motor | `int32_t` | 累计计数 |
| `bsp_motor_dual_pwm_deinit` | motor | `platform_err_t` | 停输出、释放通道 |

`cfg` 里 `invert_direction`（电机装反：占空比与计数同时反）与 `invert_encoder`（编码器 A/B 接反：只反计数）是两件独立的事，不能合成一个标志。PWM 18 kHz、编码器硬件滤波 10。

### 6.2 `bsp_servo_bus.c`

六轴总线舵机。UART5 半双工单线，YH ASCII v2.0，115200 8N1。驱动是一个**由任务推动的状态机**，不是"调用即发送"：一次只允许一个未完成事务，帧间至少 100 ms。

| 函数 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `bsp_servo_bus_init` | bus（cfg 已填） | `platform_err_t` | 清缓存与扫描状态；串口必须已初始化 |
| `bsp_servo_bus_set_map` | bus, 六个地址, 六个零点 | `void` | 装持久化映射，必须在 `init` 前（或任务启动前）调用 |
| `bsp_servo_bus_set_enable` | bus, enable | `platform_err_t` | 投递 `#255PDST!` 或逐关节 `PULR`，由下一次 poll 发出 |
| `bsp_servo_bus_request_move` | bus, 六关节, move_time_ms | `platform_err_t` | 序列号锁定的单写者请求；不阻塞 |
| `bsp_servo_bus_poll` | bus, now_ms | `void` | 推进一格：运动优先，但每两次运动之间至少一次查询 |
| `bsp_servo_bus_get_states` | bus, 角度/温度/电压/故障位/在线 | `void` | 只读缓存，不碰串口 |
| `bsp_servo_bus_request_config` | bus, joint, op, value | `platform_err_t` | 投递 ID/偏置/零点/力矩操作，同时只允许一条 |
| `bsp_servo_bus_config_ready` / `_take` / `_cancel` | out_value | bool / `platform_err_t` / void | 查询、取走、放弃超时请求 |

协议帧：移动 `#IDPpwmTtime!`、读角 `#IDPRAD!`、读温度电压 `#IDPRTV!`、改 ID `#IDPIDnew!`、偏置 `#IDPSCK±xxx!`、力矩 `#IDPULK!/PULR!`、停止 `#255PDST!`。

扫描调度：六颗一轮 `PRAD`（每颗间隔 100 ms，约 0.8~1 s 一圈）→ 一颗 `PRTV` 轮流补温度/电压（整圈约 5 s）。`queries_since_move` 保证连续目标下查询不被饿死；`misses[]` 连续 3 次无应答置离线位。零点是本板保存的偏移，对外角度 = 原始角 − 零点。

`bsp_servo_softpwm.c` 是上一版六路 GPIO 软件 PWM 驱动（20 ms 帧、2.5 ms 槽、单次定时器链），代码保留但未注册；换回模拟舵机时在 `board_init.c` 改注册它即可。

### 6.3 `bsp_flash_w25q64.c`

| 函数 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `bsp_flash_w25q64_init` | flash | `platform_err_t` | 开 SPI、等上电时间、读 JEDEC ID（EF 40 17），填 `info` |
| `bsp_flash_w25q64_read` | addr, data, len | `platform_err_t` | 任意长度读 |
| `bsp_flash_w25q64_write` | addr, data, len | `platform_err_t` | 单页内（≤256 B）写，目标必须已擦 |
| `bsp_flash_w25q64_erase` | addr, len | `platform_err_t` | 4 KiB 对齐擦除 |
| `bsp_flash_w25q64_get_info` / `_is_busy` | out | `platform_err_t` | 容量/扇区/页大小、忙状态 |

默认 4.5 MHz。擦除时上层（存储任务）会阻塞约 1 s，这正是"只有存储任务碰 flash"的原因。

### 6.4 `bsp_key_gpio.c`

| 函数 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `bsp_key_gpio_init` | key | `platform_err_t` | 配输入与上下拉 |
| `bsp_key_gpio_poll` | key, elapsed_ms | `platform_err_t` | 采样并推进去抖（默认 20 ms）；elapsed 为 0 保持原状 |
| `bsp_key_gpio_is_pressed` / `_pressed_ms` | key | bool / `uint32_t` | 去抖后的电平 / 按住时长 |

### 6.5 `bsp_led_gpio.c` / `bsp_buzzer_gpio.c`

两个都是"一个引脚 + 极性"：`init/deinit/set(on)`，`active_low` / `active_high` 由 cfg 决定，上层只传 on/off。

### 6.6 `bsp_battery_adc.c`

| 函数 | 参数 | 返回 | 作用 |
| --- | --- | --- | --- |
| `bsp_battery_adc_init` | battery | `platform_err_t` | 配 ADC 通道 |
| `bsp_battery_adc_read` | battery | `platform_err_t` | 采一次样并更新缓存电压（阻塞，不能进 ISR） |
| `bsp_battery_adc_get_mv` | battery | `uint32_t` | 缓存值 = ADC mV × 分压比 |

### 6.7 驱动层内部约定

- 所有函数返回 `platform_err_t`，不打印、不阻塞超过毫秒级（flash 擦除除外）；
- 时间由调用方通过 `elapsed_ms` 传入，驱动内部不取时钟；
- ISR 里的代码只做 GPIO/计数器级动作，不做协议、不做日志。

---

## 7. 三条主要数据流

### 7.1 下行：上位机 → 电机

```text
UART RX 中断/DMA → platform_ringbuf
  → comm 任务: service_proto_process()       // 解帧、1000 ms SEQ 去重
  → app_handlers 里的 handler                // 校验状态与范围
  → service_chassis_set_velocity()           // 存目标、刷新超时计时
  → 回 ACK (cmd|0x80, [seq, result])
control 任务（10 ms）:
  → service_chassis_apply()                  // 看门狗 + 四轮速度环
  → bsp_motor_dual_pwm_set_duty()
```

### 7.2 上行：编码器 → 上位机

```text
control 任务: bsp_motor_dual_pwm_update_encoder() → PID → apply()
              service_chassis_update_odometry()   // 每拍积分
comm 任务（app_report_tick）:
  → service_chassis_get_odometry() → service_proto_send_odom()   // 50 Hz
  → get_wheel_speeds/duty         → send_wheel_speed/duty        // 调参时
```

### 7.3 存储：参数与日志

```text
任意任务 → app_storage_append_log() / save_blob() / export()   // 只入队
storage 任务 → service_storage_append/save_blob/crash_write     // 唯一写者
            → 导出队列
comm 任务（app_storage_pump）→ REPORT_LOG / REPORT_CRASH        // 20 Hz 上限
```

---

## 8. 空目录与未实现

| 位置 | 状态 |
| --- | --- |
| `firmware/bootloader/` | 只有 `.gitkeep`，升级流程还没写 |
| `firmware/shared/Inc`、`shared/Src` | 空；"两端共用"的协议代码实际在 `app/05_Middlewares/protocol` |
| `app/00_Config`、`app/00_Docs`、`app/07_Toolchain`、`app/08_Vender`、`app/99_Utils` | 空目录，预留给 CubeMX 配置导出、工具链与第三方包 |
| `platform_os_mutex/semaphore/event/timer` | 头文件有契约，`impl_os.c` 未实现 |
| `platform_bsp_temphumi` | 有契约与注册接口，没有 BSP 驱动，`board_init()` 也没注册 |
| `service_arm` 的堵转判定 | 用总线舵机的角度回读做"目标 vs 实际"，没有电流采样；舵机本身没有过流回读时，堵转只能靠位置偏差推断 |

以上这些在 `software-design.md` 的模块表里同样标为未实现，改动时两处一起更新。
