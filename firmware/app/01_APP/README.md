# 01_APP 设计说明

应用层：把服务编排成一台机器人。这里只做**决策与分发**，不含运动学、协议字节或寄存器操作。

## 落地文件

| 文件 | 职责 |
| --- | --- |
| `app_robot/app_robot.c` | 组合根：`app_robot_init()` / `app_robot_start()` 转发给 `app_user_init` |
| `app_robot/app_user_init.c` | 串口、服务、状态机、故障、命令表的初始化；常量任务表/队列表；循环创建任务 |
| `app_robot/app_tasks.c` | `appComm` 与 `appCtrl` 两个任务的主循环，以及 PID 参数队列 |
| `app_robot/app_handlers.c` | 协议命令处理表、按键手势、`SET_PARAM` 解析入队 |
| `app_robot/app_state.c` | 主状态机，纯状态迁移，不做 I/O |
| `app_robot/app_fault.c` | 故障锁存与仲裁、WARN/ERROR/FATAL 降级动作 |
| `app_robot/app_report.c` | 周期上报调度、LED 状态 |
| `app_robot/app_battery.c` | 电池电压读取与低压判定 |
| `app_robot/app_robot_config.h` | 全部需要实测/标定的数字（几何、限速、周期、阈值、增益上限） |

每个 `.h` 只暴露其他模块需要的函数；模块间不共享内部状态变量。

## 启动顺序

```text
main()
 ├─ HAL / 外设初始化（CubeMX）
 ├─ board_init()            注册并初始化板级设备
 ├─ app_robot_init()        串口 DMA + 各服务 + 状态机 + 故障 + 命令表 + 队列
 ├─ osKernelInitialize()
 └─ osKernelStart()
      └─ StartDefaultTask → app_robot_start()  循环创建任务，挂 UART 事件
```

`app_robot_init()` 必须在调度器之前：UART DMA 先跑起来，第一帧数据不会丢；队列也在这里建（平台队列接口不依赖调度器）。任务必须在调度器之后创建，因为要调用 FreeRTOS API。

## 任务表

任务不是散落在代码里逐个创建，而是放在常量表里循环建。加任务只改表：

```c
static const app_task_def_t s_task_table[] = {
    {"appComm", app_tasks_comm_entry,    APP_COMM_TASK_STACK_BYTES,
     PLATFORM_OS_PRIORITY_HIGH,         &s_comm_task},
    {"appCtrl", app_tasks_control_entry, APP_CONTROL_TASK_STACK_BYTES,
     PLATFORM_OS_PRIORITY_ABOVE_NORMAL, &s_control_task},
};
```

| 任务 | 优先级 | 周期 | 做什么 |
| --- | --- | --- | --- |
| `appComm` | HIGH | 10 ms 或 UART 事件唤醒 | `service_proto_process()`、命令回调、故障轮询、全部上报帧 |
| `appCtrl` | ABOVE_NORMAL | 10 ms | 按键、PID 参数出队、`service_chassis_apply()`（超时停车 + 四轮速度 PID）、里程计、蜂鸣器 |

**只有 `appComm` 发送串口数据**。控制任务要上报时（例如按键长按急停）置一个 pending 标志，由通信任务变成事件帧，避免两个任务同时调用阻塞发送。

## 队列表

```c
static const app_queue_def_t s_queue_table[] = {
    {"appPid", sizeof(app_pid_param_msg_t), APP_PID_QUEUE_LEN, &s_pid_queue},
};
```

| 队列 | 元素 | 生产者 → 消费者 | 用途 |
| --- | --- | --- | --- |
| `appPid` | `{kind, wheel, gain, duty, value}` | `appComm` → `appCtrl` | PID 增益、台架测试占空比、参数/标定重载的写入串行化到控制任务 |

回调里不直接改 PID：协议回调跑在通信任务，PID 状态归控制任务所有。入队失败返回 ACK `FAILED`，而不是假装成功。队列元素带 `kind` 字段区分四类消息：增益、测试占空比、`PID_RELOAD`（`app_params_apply_to_chassis()`，重载增益/几何/四轮比例/方向/起转占空比/限速）和 `LIMITS_RELOAD`（`app_params_apply_limits_to_chassis()`，只重载超时与限速，不重置积分）。`duty` 只在测试占空比消息里有意义。

## 命令处理

| 命令 | app 的动作 | 返回 |
| --- | --- | --- |
| `SET_VELOCITY` | `service_chassis_set_velocity(vx, 0, wz)`（无横移） | 无 ACK |
| `SET_ARM_JOINTS` | `service_arm_set_joints()`；ESTOP/FAULT 中拒绝 | OK / RANGE / STATE / FAILED |
| `SET_GRIPPER` | 无夹爪硬件，如实返回失败 | FAILED |
| `SET_ENABLE` | 状态允许且未急停时先使能机械臂再使能底盘；底盘失败则回退机械臂 | OK / STATE |
| `SET_PARAM` | 校验 ID 与范围：PID 增益、测试占空比、标定/限速分别入队；标定类 ID 要求底盘失能 | OK / RANGE / STATE / FAILED / BUSY |
| `ESTOP` | `service_chassis_emergency_stop()` + `service_arm_enable(FALSE)` + 强制锁存 `0x0501` | OK |
| `CLEAR_FAULT` | 交给 `app_fault_clear()`：解除锁存、清故障码；**不自动使能** | OK / STATE |

处理函数只调用模块接口、返回 `protocol_result_t`，不碰帧字节——解帧、ACK、去重都在 `service_proto`。

### PID 参数

| id | 含义 |
| --- | --- |
| `0x20~0x2B` | A/B/C/D 四轮各自的 Kp/Ki/Kd，每轮连续 3 个 |
| `0x03~0x05` | 兼容别名，同时写 A 左前 + C 左后 |
| `0x06~0x08` | 兼容别名，同时写 B 右前 + D 右后 |

值 = 增益 × 1000。Kp/Ki 上限 10000，Kd 上限 5000；负数、NaN、越界均返回 `RANGE`。队列满返回 `FAILED`。增益只写 RAM，掉电恢复 `APP_CHASSIS_LEFT/RIGHT_PID_*` 初值。

### 台架测试占空比

| id | 含义 |
| --- | --- |
| `0x30~0x33` | A/B/C/D 轮的**开环**测试占空比，值 = ‰，范围 `-400~+400` |

用途是量电机死区：速度环会把死区补偿掉，量到的就不是电机本身的特性。处理函数在这里多一道检查——底盘没使能或已急停直接回 `STATE`，因为真正的写入发生在控制任务，等到那时再拒绝主机已经收到 OK 了。

每个轮子单独计时，300 ms 不刷新就按 0 处理；任何速度指令、停车、失能、急停都会退出测试模式。参数上限和语义见 `docs/architecture/protocol.md` §5.4，实测流程见 `docs/bench/pid-tuning.md`。

### 标定与限速参数

| id | 含义 | 生效方式 |
| --- | --- | --- |
| `0x01` | 轮距，mm | 写入后控制任务重载 `service_chassis_set_geometry()` |
| `0x02` | 轮径，mm | 由 `4680/(π·d)` 重算四轮 `counts_per_mm` 后重载 |
| `0x50~0x53` | 四轮编码器比例 ×1000 | `service_chassis_set_wheel_scale()`，会重设该轮 PID 与里程计基线 |
| `0x54~0x57` / `0x58~0x5B` | 四轮正/反向可靠起转占空比 ‰ | `service_chassis_set_start_duty()`，只在轮子静止且目标非零时抬升输出 |
| `0x5C~0x5F` | 四轮方向 `+1/-1` | 反转该轮的编码器符号（测速与里程计同时生效） |
| `0x10~0x12` | 指令超时 / 最大线速度 / 最大角速度 | 单独的限速重载，不重置 PID，可在使能状态下写 |

标定类 id（`0x01`、`0x02`、`0x50~0x5F`）在底盘使能或急停锁存时返回 `STATE`：行进中改尺子会让里程计跳变。值只写 RAM 并置脏，`PARAM_SAVE` 才落盘。写轮径会覆盖 `0x50~0x53` 的单轮微调，顺序应是**先轮径后微调**。

上电顺序：`app_user_init_storage()` 读两个 blob → `app_params_init()` 的默认值被覆盖 → `service_chassis_init(cfg)` → `app_params_apply_to_chassis()` 推入四轮比例/方向/起转占空比与限速。此时还没有任务在跑，是控制任务之外唯一允许调用 `service_chassis` setter 的地方。

> 注意 `0x41`（死区总开关）目前返回 `RANGE`：起转补偿已经按轮独立配置，用 `0x54~0x5B` 写 0 即关闭。

## 状态机

```text
INIT ──ready──► IDLE ◄──SET_ENABLE(0)──► MANUAL
                 ▲          ──SET_ENABLE(1)──►

任意状态 ──ERROR──► FAULT ──CLEAR_FAULT──► IDLE
任意状态 ──FATAL──► ESTOP ──CLEAR_FAULT──► IDLE
```

- 运动指令只在 MANUAL 被接受；使能只在 IDLE/MANUAL 被接受。
- `CLEAR_FAULT` 从 FAULT/ESTOP 回到 IDLE，**必须重新 `SET_ENABLE`** 才能动。
- 清 WARN 时保持当前状态（在 MANUAL 就还是 MANUAL）。
- AUTO 在协议中预留，当前没有内部自主运动任务，固件不会主动进入。

状态机不知道电机和串口的存在，只判断"允不允许"；具体动作由 `app_fault.c` 和 `app_handlers.c` 做。

## 故障策略

| 级别 | 触发例 | 降级动作 |
| --- | --- | --- |
| WARN | CRC 超限 `0x0201`、电池低压 `0x0401`、过温 `0x0402` | 只锁存并上报事件，不停任何东西 |
| ERROR | 过流 `0x0101`、编码器 `0x0102`、关节限位/堵转/轨迹 `0x0301~0x0303` | 停故障作用域：底盘失能 / 机械臂失能 / 两者；进 FAULT |
| FATAL | 通信超时 `0x0202`、急停 `0x0501` | 四路桥臂 coast、机械臂失能；进 ESTOP |

作用域表在 `app_fault_scope_get()`：底盘故障不动机械臂，机械臂故障只把底盘目标清零（保留桥臂使能，让速度环刹停），FATAL 全停。未知故障码按全停处理，宁可保守。

`REPORT_STATUS` 只有一个故障码字段，多个条件同时成立时**按协议第 8 节的级别仲裁**：`protocol_fault_severity()` 给出 WARN / ERROR / FATAL，新故障级别高于已锁存的才顶替（同级保留先到的，急停是显式强制覆盖）。这样电池低压（WARN）不会把随后的通信超时（FATAL）挡掉。

`CLEAR_FAULT` 清的是软件锁存，同时会复位底盘的超时标志——清除之后不会"清了又立刻报"。要再次触发 `0x0202`，必须先有一个新的速度目标，再让它超时。

电池低压是唯一可测量的"条件"：当要清的正是 `0x0401`（直接指定，或 `CLEAR_ALL` 时当前锁存的就是它）而电压仍低于阈值，请求返回 `STATE`。若锁存的是急停/超时、只是电池恰好偏低，`CLEAR_ALL` 照常成功，低压会在下一次 STATUS 按新条件重新上报。

故障检测入口有两个：协议命令（`ESTOP`）和 `app_fault_poll()`（每个通信周期检查 200 ms 超时上升沿、CRC 告警一次）。

## 按键手势

| 手势 | 动作 |
| --- | --- |
| 短按 | MANUAL 时在"前进 150 mm/s"与"停车"之间切换 |
| 长按 1.5 s | 立即急停（不等松手）、蜂鸣 300 ms、通信任务上报 `0x0501` |

## 上报节奏

| 帧 | 周期 | 数据来源 |
| --- | --- | --- |
| `REPORT_ODOM` | 20 ms | `service_chassis_get_odometry()` |
| `REPORT_WHEEL_SPEED` | 20 ms | `service_chassis_get_wheel_speeds()`（四轮实测 mm/s） |
| `REPORT_WHEEL_DUTY` | 20 ms | `service_chassis_get_wheel_duty()`（四轮实际占空比 ‰），与上一行同拍发送 |
| `REPORT_ARM` | 50 ms | `service_arm_get_states()`（不支持时退回目标角） |
| `REPORT_STATUS` | 100 ms | 状态字 + 故障码 + 电池 + 链路抖动 |
| `REPORT_HEARTBEAT` | 100 ms | 状态字 + 运行计数 |
| `REPORT_EVENT` | 事件触发 | 故障产生/清除 |

调度抖动用"实际间隔与名义周期之差"的最大值统计，随 STATUS 上报，用来发现任务被长时间阻塞。LED 镜像底盘使能状态。

## 标定项

`app_robot_config.h` 里带 `TODO(measure)` 的数字必须上车实测。它们只是**出厂默认值**：主机可以用 `SET_PARAM` + `PARAM_SAVE` 把实测结果写进标定区，上电时保存值优先。

- `APP_CHASSIS_HALF_LENGTH_MM` / `APP_CHASSIS_HALF_WIDTH_MM`：影响旋转解算与里程计航向；
- `APP_CHASSIS_COUNTS_PER_MM_X1000`：按 13 线 × 4 倍频 × 90 减速比 / (π × 60 mm) = 24828 计算，轮径变了要重算（运行时写 `0x02` 即可，固件用同一个 4680 counts/rev 常数）；
- `APP_CHASSIS_MAX_WHEEL_MM_S`：轮速目标上限；`counts_per_mm_x1000 = 0` 时也是开环回退的满量程；
- `APP_CHASSIS_LEFT/RIGHT_PID_*`：速度环启动增益，**不是标定结果**。轮胎未装、负载未知，先用 `tools/bench/wheel_tune.py` 在台架上整定，数据记到 `docs/bench/pid-tuning.md`，再把选定值回填这里。

## 依赖方向

```text
01_APP ──► 02_Service ──► 03_Platform ──► 04_Impl ──► 06_BSP
   └────► 03_Platform/platform_os（任务、队列、时间）
```

app 不包含 `stm32f1xx_hal.h`、`FreeRTOS.h`、`usart.h`；需要串口就用 `platform_mcu_uart.h`，需要任务就用 `platform_os_task.h`。
