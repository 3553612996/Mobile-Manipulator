# 02_Service 设计说明

功能服务层：把"机器人会做什么"抽象成可复用的服务，供 `01_APP` 调用。服务不读时钟、不建任务、不直接碰寄存器，只做状态与算法。

## 落地文件

| 服务 | 职责 | 关键接口 |
| --- | --- | --- |
| `service_key` | 按键去抖之上的短按/长按策略 | `service_key_init/update`，回调 `(id, event)` |
| `service_chassis` | 麦轮逆运动学、200 ms 指令超时、使能/急停、开环占空比、里程计 | `forward/backward/strafe/rotate/set_velocity/stop/emergency_stop` |
| `service_arm` | 六轴限位校验、目标跟踪、移动到舵机总线 | `set_joints/get_targets/get_states` |
| `service_proto` | 串口协议实现：取字节、解帧、分发、ACK、SEQ 去重、上报组帧 | `process/register/send_*` |

## 为什么这些属于 service 而不是 app

app 只表达**意图**："前进"、"注册 SET_ENABLE 的处理函数"、"每 20 ms 上报里程计"。从意图到硬件之间有三类可复用逻辑：

1. **运动学与安全策略**（麦轮解算、超时停车、急停锁存）——换一个应用、换一个上位机都还要用；
2. **协议实现**（粘包处理、CRC、ACK、去重）——属于通信契约，不属于业务；
3. **设备策略**（短按/长按、限位）——属于交互约定。

把它们放进 service，app 就能写成"事件 → 服务调用"的薄层，也就能在 PC 上直接测试：`service_chassis_inverse_kinematics()` 是纯函数，`service_key_update()` 的时间是参数。

## 依赖方向

```text
01_APP
  |
  v
02_Service/service_*  ──►  05_Middlewares/protocol（仅 service_proto）
  |
  v
03_Platform/platform_bsp、platform_mcu、platform_os
  |
  v
04_Impl ──► 06_BSP
```

服务之间**不互相调用**。需要组合时由 app 编排（例如按键长按 → `service_chassis_emergency_stop()` + 上报事件）。

## 时间注入

所有服务都不读系统时钟，时间由调用方传入：

```c
service_chassis_set_velocity(vx, vy, wz, now_ms);   /* now_ms 由 app 提供 */
service_chassis_apply(now_ms);                      /* 超时判定 */
service_chassis_update_odometry(elapsed_ms);
service_key_update(elapsed_ms);
```

这样超时、去抖、里程积分都能在 PC 上用确定的时间序列测试，不依赖调度器。

## 线程模型

- 服务本身**不是线程安全的**，每个服务的状态归一个任务所有。
- 当前分配：`service_chassis` 的 `apply/odometry` 与 `service_key` 在控制任务；`service_proto`、`service_arm` 和底盘目标设置（`set_velocity/enable/emergency_stop`）在通信任务。
- 跨任务共享的只有底盘的目标速度、使能位和急停位，都是单字读写，Cortex-M3 上原子。
- `service_proto` 的 TX 只有通信任务一个调用者，因此不需要发送互斥锁。

## 安全语义

| 情况 | 行为 |
| --- | --- |
| 200 ms 没有新的有效速度帧 | 目标清零、`platform_motor_stop_all()`、`cmd_timed_out` 置位 |
| `enable` / `emergency_stop` | 同时开关四路 H 桥（`platform_motor_set_enable`）。只写占空比不够——桥臂使能前 BSP 只记录数值、不输出 |
| `emergency_stop` | 锁存急停、失能驱动、目标清零、四路桥臂 coast，直到 `clear_estop` |
| `clear_estop` | 只解除锁存，**不自动使能**，必须重新 `SET_ENABLE` |
| 关节越界 | 整帧拒绝，六个关节一个都不动 |
| CRC 连续错误 ≥ 10 帧 | `service_proto_crc_alarm()` 置位，由 app 上报 |

`platform_motor_get_encoder()` 返回的是**累计**计数（平台层文档就是这么定义的），BSP 内部用增量累加。里程计要的是两次采样之间的增量，所以服务自己保存 `prev_counts` 做差——把它当增量用，速度会随着累计值一直涨到 `int16` 饱和（真机上一脚 200 mm/s 会读成 32767）。

## PC 单测

```bash
gcc -std=c99 -Wall -Wextra \
    -I firmware/app/03_Platform/platform_common \
    -I firmware/app/04_Impl \
    -I firmware/app/03_Platform/platform_bsp \
    -I firmware/app/02_Service/service_chassis \
    tests/unit/service/test_service_chassis.c \
    firmware/app/02_Service/service_chassis/service_chassis.c \
    -o build/test_service_chassis -lm

gcc -std=c99 -Wall -Wextra \
    -I firmware/app/03_Platform/platform_common \
    -I firmware/app/04_Impl \
    -I firmware/app/03_Platform/platform_bsp \
    -I firmware/app/02_Service/service_key \
    tests/unit/service/test_service_key.c \
    firmware/app/02_Service/service_key/service_key.c \
    -o build/test_service_key
```

协议服务的编译命令见 `tests/unit/service/test_service_proto.c` 文件头。
