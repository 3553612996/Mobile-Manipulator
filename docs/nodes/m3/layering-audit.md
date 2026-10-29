# M3 架构分层检查

> **结论：未发现跨层直接调用。** 五类机械判定条件全部 0 命中；本次新增的存储/参数模块（`app_params`、`app_storage`）和底盘运行期标定接口都在层内。上一轮检查后新出现的两处"越界但不违规"的依赖已在发布前收敛，记录在第 3 节。

| 项目 | 内容 |
| --- | --- |
| 检查日期 | 2026-10-29 |
| 代码 | `develop` @ `8c694f4` |
| 依据 | [软件结构](../../architecture/software-design.md) 第 4 节、[存储设计](../../architecture/storage-design.md) |
| 范围 | `firmware/app/{01_APP,02_Service,03_Platform,04_Impl,05_Middlewares,06_BSP}` |

---

## 1. 规则

沿用 [M2 检查](../m2/layering-audit.md) 拆出的五类条件：

| 编号 | 条件 | 判定 |
| --- | --- | --- |
| A | 上层（`01_APP` / `02_Service` / `05_Middlewares`）直接使用 HAL / RTOS / 寄存器 | 出现即违规 |
| B | 上层直接调用 `impl_*`（跳过 Platform 契约） | 出现即违规 |
| C | 下层（`03_Platform` / `04_Impl` / `06_BSP`）反向调用 `app_*` / `service_*` | 出现即违规 |
| D | CubeMX 生成的头文件出现在 `04_Impl/impl_mcu` 之外 | 出现即违规 |
| E | 相对路径 `#include "../..."`（绕过 `-I` 契约） | 出现即违规 |

M3 追加两条与并发模型相关的约定，静态扫描查不出，逐条人工核对：

| 编号 | 条件 | 判定 |
| --- | --- | --- |
| F | 服务层读时钟、建任务、碰寄存器 | 出现即违规（时间由调用方以 `now_ms` 传入） |
| G | Flash 写入者不止一个，或 UART 写入者不止一个 | 出现即违规 |

---

## 2. 检查方法与证据

以下命令在 `firmware/app/` 下执行，结果可复现。

### 2.1 A~E：五类静态条件

```bash
grep -rnE "HAL_[A-Za-z]|FreeRTOS\.h|__HAL_|->(CCR|ODR|CR1|IDR)|cmsis_os" 01_APP 02_Service 05_Middlewares   # A: 0 处
grep -rnE "\bimpl_[a-z_]+\(" 01_APP 02_Service 05_Middlewares                                           # B: 0 处
grep -rnE "\b(app_[a-z_]+\(|service_[a-z_]+\()" 03_Platform 04_Impl 06_BSP                               # C: 0 处
grep -rln "stm32f1xx_hal.h" 01_APP 02_Service 03_Platform 04_Impl 05_Middlewares 06_BSP                  # D: 仅 impl_mcu
grep -rn '#include "\.\./' 01_APP 02_Service 03_Platform 04_Impl 05_Middlewares 06_BSP                  # E: 0 处
```

D 的命中全部落在 `04_Impl/impl_mcu/`（HAL 适配层），与 M2 结论一致；`06_BSP` 仍然不包含任何 HAL 头文件。

### 2.2 F：服务层是否越权

```bash
grep -rn "HAL_GetTick\|platform_os_tick_get\|platform_os_ticks_to_ms\|xTaskCreate\|GPIO" 02_Service
```

结果：**0 处**。`service_storage_*` / `service_chassis_*` 的所有时间参数都是调用方传进来的 `now_ms`，任务由 `01_APP` 的常量任务表创建。

### 2.3 G：单写者约定

| 资源 | 唯一写者 | 证据 |
| --- | --- | --- |
| W25Q64（SPI 总线） | `appStorage` 任务 | app 层无 `platform_bsp_flash_*` / `platform_wearlevel_*` 调用；`service_storage_save_blob()` 只在 [app_storage.c](../../../firmware/app/01_APP/app_robot/app_storage.c) 的任务体里被调用 |
| UART（协议帧） | `appComm` 任务 | `service_proto_send_storage()` 位于 `app_storage_pump()`，后者由通信任务循环调用 |
| 底盘 PID 状态 | `appCtrl` 任务 | 协议处理只写 `app_params`，下推走 `APP_CTRL_MSG_PID_RELOAD` / `APP_CTRL_MSG_LIMITS_RELOAD` 队列 |

### 2.4 实际包含关系（M3 新增部分）

```text
01_APP/app_params   -> 02_Service/service_chassis（运行期标定 + 轮径换算）
01_APP/app_storage  -> 02_Service/service_{storage,proto}
01_APP/app_storage  -> 03_Platform/platform_os_{task,time,critical}
02_Service/service_storage -> 03_Platform/platform_bsp_flash、platform_logstore、platform_crc
02_Service/service_chassis -> 03_Platform/platform_pid、platform_bsp_motor
```

方向全部向下。

---

## 3. 发布前收敛的两处跨层依赖

两处都"能跑但越过服务抽象"，在 M3 收尾时改掉：

### 3.1 app 直接推进 platform 的日志迭代器

`app_storage` 原先用 `platform_logstore_iter_next()` 推进 `service_storage_iter_begin()` 返回的迭代器，等于把 Platform 类型和平台 API 泄漏进 app。

处理：`service_storage` 增加 `service_storage_iter_next()` 与 `service_storage_iter_t` / `service_storage_entry_t` 别名，`app_storage.c` 不再包含 `platform_logstore.h`。改动不改变行为，多一层转发。

### 3.2 轮径反算编码器比例留在 app

`app_params` 自己实现了 `counts_per_mm = 4680 / (π·d)`，另带一份 π。这是轮子几何，属于底盘服务。

处理：新增 `service_chassis_counts_per_mm_x1000(counts_per_rev, diameter_mm)`；每转计数是板级常量，由 app 传入，服务里不出现 4680。`app_params.c` 删除了本地公式与 `APP_PARAMS_PI`。

---

## 4. 检查看不到的部分

| 情况 | 核对方式 | 结论 |
| --- | --- | --- |
| app 在启动阶段（调度器未运行）调用服务 | 查 `app_user_init_chassis()` 与 `app_params_apply_to_chassis()` | 有意的启动期例外：此时无并发，代码内已写明理由；运行期只从控制任务调用 |
| 回调注册（底层持有上层函数指针） | 查 `service_proto_register` / 存储完成回调 | app 注册自己的处理函数，属于受控回调 |
| ISR 中调用上层逻辑 | 查 UART / TIM6 / 软 PWM 中断 | ISR 只搬运和通知，不调用 app/service |
| 跨层 `extern` | 全仓 grep | 无 |

---

## 5. 后续节点的复用

M4（自主导航桥接）新增 ROS 侧模块时，重跑第 2 节命令并把新模块纳入 F/G 两条；若出现新的例外，在本文件追加说明。
