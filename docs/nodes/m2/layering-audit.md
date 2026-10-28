# M2 架构分层检查

> **结论：未发现跨层直接调用。** `01_APP` / `02_Service` / `05_Middlewares` 中没有 HAL、FreeRTOS、寄存器或 `impl_*` 的痕迹；`03_Platform` / `04_Impl` / `06_BSP` 没有反向调用 `app_*` / `service_*`。发现两处"文档与实际依赖不一致"和一处有意的双跳转发，均已说明，不构成违规。

| 项目 | 内容 |
| --- | --- |
| 检查日期 | 2026-10-29（发布前复核） |
| 代码 | `develop` @ M2 发布提交（目录重构后） |
| 依据 | [项目框架 · 固件目录与职责](../../architecture/overview.md)、[软件结构](../../architecture/software-design.md)、各层 `README.md` |
| 范围 | `firmware/app/{01_APP,02_Service,03_Platform,04_Impl,05_Middlewares,06_BSP}` |

---

## 1. 规则

`docs/architecture/overview.md` 对固件依赖的约束：

> 依赖只向下：`01_APP → 02_Service → 03_Platform → 04_Impl → 06_BSP`。app 里不允许出现 `HAL_*`、`FreeRTOS.h`、寄存器名；service 里不允许包含板级头文件。

检查把这条规则拆成五类可机械判定的条件：

| 编号 | 条件 | 判定 |
| --- | --- | --- |
| A | 上层代码直接使用 HAL / RTOS / 寄存器 | 出现即违规 |
| B | 上层代码直接调用 `impl_*`（跳过 Platform 契约） | 出现即违规 |
| C | 下层代码反向调用 `app_*` / `service_*` | 出现即违规 |
| D | CubeMX 生成的头文件出现在 `04_Impl/impl_mcu` 之外 | 出现即违规 |
| E | 相对路径 `#include "../..."`（绕过 `-I` 契约、暗示跨目录耦合） | 出现即违规 |

"上层"指 `01_APP`、`02_Service`、`05_Middlewares`；`03_Platform` 定义契约，允许依赖 `platform_common` 和被它调用的 `04_Impl` 实现。

---

## 2. 检查方法与证据

以下命令在 `firmware/app/` 下执行，结果可复现。

### 2.1 A：上层是否碰硬件

```bash
grep -rnE "HAL_[A-Za-z]|FreeRTOS\.h|__HAL_|->(CCR|ODR|CR1|IDR)|cmsis_os" \
    01_APP 02_Service 05_Middlewares
```

结果：**唯一命中是 `01_APP/README.md` 里的一句说明文字**（"app 不包含 `stm32f1xx_hal.h`…"），代码 0 处。

### 2.2 B：上层是否跳过 Platform 直接调实现

```bash
grep -rnE "\bimpl_[a-z_]+\(" 01_APP 02_Service 05_Middlewares
```

结果：**0 处**。

### 2.3 C：下层是否回调上层

```bash
grep -rnE "\b(app_[a-z_]+\(|service_[a-z_]+\()" 03_Platform 04_Impl 06_BSP
```

结果：**0 处**。

### 2.4 D：CubeMX 头文件的位置

```bash
grep -rln "stm32f1xx_hal.h\|main.h\|usart.h" \
    01_APP 02_Service 03_Platform 04_Impl 05_Middlewares 06_BSP
```

结果：代码中只出现在 `04_Impl/impl_mcu`（HAL 适配层）；另有三处命中是各层 `README.md` 的说明文字。**`06_BSP` 不包含任何 HAL 头文件**。

### 2.5 E：相对路径包含

```bash
grep -rn '#include "\.\./' 01_APP 02_Service 03_Platform 04_Impl 05_Middlewares 06_BSP
```

结果：**0 处**。全部包含都靠 `-I` 目录契约，头文件名全局唯一。

### 2.6 实际包含关系

用脚本扫描 `#include "..."`，按头文件归属统计跨目录边（数字为引用次数）：

```text
01_APP        -> 02_Service/service_{arm,chassis,key,proto}      4
01_APP        -> 03_Platform/platform_bsp（battery/buzzer/led）   3
01_APP        -> 03_Platform/platform_mcu（uart）                1
01_APP        -> 03_Platform/platform_os（task/time）             2
01_APP        -> 03_Platform/platform_common（error）             1

02_Service    -> 03_Platform/platform_bsp（key/motor/servo）      3
02_Service    -> 03_Platform/platform_mcu（uart）                 1
02_Service    -> 05_Middlewares/protocol（仅 service_proto）      1

05_Middlewares-> 03_Platform/platform_common / middleware          2

03_Platform   -> 04_Impl/impl_wrapper（platform_mcu 转发）         9
03_Platform   -> 04_Impl/impl_board（board_types）                 1

04_Impl       -> 03_Platform/platform_*（实现平台契约）            21
04_Impl       -> 06_BSP（板级驱动）                                12

06_BSP        -> 03_Platform/platform_mcu                         8
```

五条边方向全部符合预期，没有"下层回头依赖上层"的边。

---

## 3. 三处需要说明的地方

### 3.1 `06_BSP` 依赖 `platform_mcu`，文档原来写成 HAL

原架构文档的目录表写 `06_BSP` 依赖 `HAL`，实际代码里 BSP 头文件包含的是 `platform_mcu_{pwm,encoder,adc,gpio,timer}.h`，**没有任何 BSP 文件包含 HAL**。真实链路是：

```text
impl_bsp_motor → bsp_motor_dual_pwm → platform_pwm_set_duty
              → impl_pwm_set_duty_wrap → impl_mcu_pwm → HAL
```

也就是说，物理调用在 `04_Impl → 06_BSP` 之后又折回 `03_Platform/platform_mcu`。`platform_mcu` 不是"上层接口"，而是**给 BSP 用的外设薄封装**，逻辑位置在 BSP 之下。架构文档已补上这段说明，并把目录表的 06 依赖改为 `platform_mcu`。

这样设计的收益：BSP 不持有 HAL 句柄，换 MCU 时只改 `04_Impl/impl_mcu`；代价是依赖图不再是一条直线，读代码时要记住这个折返。

### 3.2 `01_APP` 直接使用板级设备

`app_robot.c` 直接调用 `platform_bsp_battery` / `platform_bsp_led` / `platform_bsp_buzzer`，以及 `platform_os_*`、`platform_mcu_uart`（串口总线号）。文档定义的 app 依赖是"Service、Platform"，因此**不违规**。

若要严格执行"app 只调用 service"，这三类设备应各包一层薄 service。当前规模下没有这个必要：它们没有策略可复用，包一层只会多出三对转发文件和一次函数调用。**记录在案，等出现第二个调用方再抽。**

### 3.3 `platform_mcu` ↔ `impl_*_wrap` 的双跳转发

`platform_mcu_*.c` 的函数体基本是 `return impl_*_wrap(...);`，例如：

```c
/* platform_mcu_adc.c */
return impl_adc_init_wrap(cfg);
```

两层声明、两次调用带来一点开销，但换来的是：`platform_mcu_*` 是稳定契约（BSP 和 service 只认它），`impl_*_wrap` 是编译期可替换的实现入口（换 MCU/换驱动只改 wrapper）。这是有意的隔离层，**不是冗余**，不建议合并。

---

## 4. 检查看不到的部分

静态扫描只能证明"没有写出来的直接调用"，以下三类需要人工确认，本次逐一核对过：

| 情况 | 核对方式 | 结论 |
| --- | --- | --- |
| 回调注册（底层持有上层函数指针） | 查 `service_proto_register` / `platform_uart_register_event_cb` 的调用点 | app 注册的是自己的处理函数与"仅唤醒任务"的回调，属于受控回调，不是反向依赖 |
| ISR 中调用上层逻辑 | 查 `USART3_IRQHandler` / `DMA1_Channel3_IRQHandler` | ISR 只做搬运和通知，不调用 app/service |
| 同名宏或全局符号越层访问 | 查 `extern` 声明 | 无跨层 `extern` |

---

## 5. 后续节点的复用

M3 及之后每个节点发布前，重跑第 2 节的五条命令即可；新增的模块（PID、里程计标定、ROS 桥接）按同样五类条件纳入范围。若出现新的例外（例如感知层需要直接读 DMA），必须在本文件追加说明，而不是让规则默默失效。
