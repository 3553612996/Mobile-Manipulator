# 01_APP 设计说明

应用层：把服务编排成一台机器人。这里只做**决策与分发**，不含运动学、协议字节或寄存器操作。

## 落地文件

| 文件 | 职责 |
| --- | --- |
| `app_robot/app_robot.h` | `app_robot_init()` / `app_robot_start()` |
| `app_robot/app_robot.c` | 任务、命令处理函数、按键回调、故障策略、上报调度 |
| `app_robot/app_robot_config.h` | 全部需要实测/标定的数字（几何、限速、周期、阈值） |

## 启动顺序

```text
main()
 ├─ HAL / 外设初始化（CubeMX）
 ├─ board_init()            注册并初始化板级设备
 ├─ app_robot_init()        串口 DMA + 各服务 + 注册命令处理函数（不建任务）
 ├─ osKernelInitialize()
 └─ osKernelStart()
      └─ StartDefaultTask → app_robot_start()  建通信/控制任务，挂 UART 事件
```

`app_robot_init()` 必须在调度器之前：UART DMA 先跑起来，第一帧数据不会丢；任务必须在调度器之后创建，因为要调用 FreeRTOS API。

## 任务划分

| 任务 | 优先级 | 周期 | 做什么 |
| --- | --- | --- | --- |
| `appComm` | HIGH | 10 ms 或 UART 事件唤醒 | `service_proto_process()`：解析、回调、ACK；故障判定；上报 ODOM/ARM/STATUS/HEARTBEAT |
| `appCtrl` | ABOVE_NORMAL | 10 ms | `service_key_update()`、`service_chassis_apply()`（超时停车）、里程计积分、蜂鸣器 |

**只有 `appComm` 发送串口数据**。控制任务要上报时（例如按键长按急停）置一个 `s_pending_estop` 标志，由通信任务变成事件帧，避免两个任务同时调用阻塞发送。

## 命令处理

| 命令 | app 的动作 | 返回 |
| --- | --- | --- |
| `SET_VELOCITY` | `service_chassis_set_velocity(vx, 0, wz)`（v1 无横移） | 无 ACK |
| `SET_ARM_JOINTS` | `service_arm_set_joints()`；急停中拒绝 | OK / RANGE / STATE / FAILED |
| `SET_GRIPPER` | 无夹爪硬件，如实返回失败 | FAILED |
| `SET_ENABLE` | `service_chassis_enable()`；急停中拒绝使能 | OK / RANGE / STATE |
| `SET_PARAM` | 尚未实现参数存储 | FAILED |
| `ESTOP` | `service_chassis_emergency_stop()` + 锁存 `0x0501` + 事件帧 | OK |
| `CLEAR_FAULT` | 解除急停锁存、清故障码；**不自动使能** | OK / STATE |

处理函数只调用服务、返回 `protocol_result_t`，不碰帧字节——解帧、ACK、去重都在 `service_proto`。

## 按键手势

| 手势 | 动作 |
| --- | --- |
| 短按 | 已使能时在"前进 150 mm/s"与"停车"之间切换 |
| 长按 1.5 s | 立即急停（不等松手）、蜂鸣 300 ms、通信任务上报 `0x0501` |

## 故障策略

| 来源 | 故障码 | 说明 |
| --- | --- | --- |
| 按键长按 / `ESTOP` 命令 | `0x0501` | 强制覆盖当前故障码，最高优先级 |
| 底盘 200 ms 超时（上升沿） | `0x0202` | 同级不覆盖，可压过低级别故障 |
| CRC 连续错误 ≥ 10 帧（每次启动报一次） | `0x0201` | 只上报，不停止 |
| 电池低于 6.8 V（2S，3.4 V/节） | `0x0401` | 只上报，不停止 |

`REPORT_STATUS` 只有一个故障码字段，多个条件同时成立时只能显示一个，**按协议第 8 节的级别仲裁**：`protocol_fault_severity()` 给出 WARN / ERROR / FATAL，新故障级别高于已锁存的才顶替（同级保留先到的，急停是显式强制覆盖）。这样电池低压（WARN）不会把随后的通信超时（FATAL）挡掉——后者恰恰是更要紧的那一个。代价是被顶替的低级别故障不再显示，但条件仍在时下一次 `CLEAR_FAULT` 之后会重新上报。

`CLEAR_FAULT` 清的是软件锁存，同时会复位底盘的超时标志——清除之后不会"清了又立刻报"。要再次触发 `0x0202`，必须先有一个新的速度目标，再让它超时。

电池低压是唯一可测量的"条件"：当要清的正是 `0x0401`（直接指定，或 `CLEAR_ALL` 时当前锁存的就是它）而电压仍低于阈值，请求返回 `STATE`，按协议第 8 节"条件没消失就不许清除"处理。若锁存的是急停/超时、只是电池恰好偏低，`CLEAR_ALL` 照常成功，低压会在下一次 STATUS 按新条件重新上报——这是如实反映现状，不是"清了又立刻报"。急停和超时本身就是锁存，清除即释放。

## 上报节奏

| 帧 | 周期 | 数据来源 |
| --- | --- | --- |
| `REPORT_ODOM` | 20 ms | `service_chassis_get_odometry()` |
| `REPORT_ARM` | 50 ms | `service_arm_get_states()`（不支持时退回目标角） |
| `REPORT_STATUS` | 100 ms | 状态字 + 故障码 + 电池 + 链路抖动 |
| `REPORT_HEARTBEAT` | 100 ms | 状态字 + 运行计数 |
| `REPORT_EVENT` | 事件触发 | 故障产生/清除 |

调度抖动用"实际间隔与名义周期之差"的最大值统计，随 STATUS 上报，用来发现任务被长时间阻塞。

## 标定项

`app_robot_config.h` 里带 `TODO(measure)` 的数字必须上车实测：

- `APP_CHASSIS_HALF_LENGTH_MM` / `APP_CHASSIS_HALF_WIDTH_MM`：影响旋转解算与里程计航向；
- `APP_CHASSIS_COUNTS_PER_MM_X1000`：按 13 线 × 4 倍频 × 90 减速比 / (π × 60 mm) = 24828 计算，轮径变了要重算；
- `APP_CHASSIS_MAX_WHEEL_MM_S`：开环占空比的满量程，换成闭环后由 PID 取代。

## 依赖方向

```text
01_APP ──► 02_Service ──► 03_Platform ──► 04_Impl ──► 06_BSP
   └────► 03_Platform/platform_os（任务、时间）
```

app 不包含 `stm32f1xx_hal.h`、`FreeRTOS.h`、`usart.h`；需要串口就用 `platform_mcu_uart.h`，需要任务就用 `platform_os_task.h`。
