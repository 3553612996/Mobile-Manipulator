# 存储器驱动与服务层设计（W25Q64）

> **状态：驱动、存储服务层、磨损均衡与应用接入（`appStorage` 任务、0x08~0x0D 命令、日志/崩溃导出、参数落盘）已实现，并通过 12 个 L1 测试套件与 Keil 编译；HardFault 现场写入尚未接入（见第 13 节）。**
> 布局与协议格式的定义仍是 [flash-layout.md](flash-layout.md)，本文说明代码怎么实现它、哪些地方做了具体化。两者冲突时以本文的"差异"一节为准，并回头修正 flash-layout.md。

相关文档：[Flash 布局](flash-layout.md) · [串口协议](protocol.md) · [软件结构](software-design.md) · [测试索引](../../tests/README.md)

| 项目 | 内容 |
| --- | --- |
| 日期 | 2026-10-29 |
| 处理器 / 编译 | STM32F103RCT6 @72 MHz，Keil ARMCC V5.06 update 7 |
| 介质 | W25Q64JV，8 MiB，SPI2（PB12 CS / PB13 SCK / PB14 MISO / PB15 MOSI） |
| 参与编译 | `app.axf` 0 error 0 warning，Code 54 020 B / RO 1 324 B / RW 704 B / ZI 27 408 B |
| 测试 | 12 个 PC 套件全部退出码 0（见第 9 节） |

---

## 1. 本次交付范围

| 层 | 交付物 | 文件 |
| --- | --- | --- |
| CubeMX 生成层 | SPI2 初始化、PB13/14/15 复用、PB12 软件片选 | `Core/Src/spi.c`、`Core/Inc/spi.h`、`Core/Src/gpio.c`、`app.ioc` |
| Platform 接口 | 契约不变；新增实现 | `03_Platform/platform_mcu/platform_mcu_spi.c` |
| Impl 适配 | SPI 转发、HAL 阻塞收发、总线互斥 | `04_Impl/impl_wrapper/impl_spi_wrapper.c`、`04_Impl/impl_mcu/impl_mcu_spi.c` |
| Impl 适配 | 时间基元 `platform_delay_ms/us` | `04_Impl/impl_mcu/impl_mcu_time.c` |
| BSP 驱动 | W25Q64 命令集、BUSY/WEL 轮询、页/扇区约束 | `06_BSP/bsp_flash_w25q64.c/.h` |
| Impl 分发 | `platform_flash_*` → BSP ops | `04_Impl/impl_bsp/impl_bsp_flash.c/.h` |
| Middleware | 磨损均衡策略 | `03_Platform/platform_middleware/platform_wearlevel.c/.h` |
| Middleware | 通用追加式记录环 | `03_Platform/platform_middleware/platform_logstore.c/.h` |
| Service | 分区、A/B 记录、故障/日志环、崩溃槽、启动计数、寿命统计 | `02_Service/service_storage/service_storage.c/.h` |
| Service | `REPORT_STORAGE` / `REPORT_LOG` / `REPORT_CRASH` 组帧 | `02_Service/service_proto/service_proto.c/.h` |
| App | Flash 单写者任务、请求/导出队列、分级暂存池 | `01_APP/app_robot/app_storage.c/.h` |
| App | 参数与标定的 RAM 副本、默认值、字节布局、脏标记 | `01_APP/app_robot/app_params.c/.h` |
| App | 0x08~0x0D 处理函数、故障入环、控制任务增益重载 | `app_handlers.c`、`app_fault.c`、`app_tasks.c`、`app_user_init.c` |
| 板级装配 | 注册并初始化 Flash，失败不阻断上电 | `04_Impl/impl_board/board_init.c` |
| 测试 | 驱动模型、中间件、服务层、应用接入 | `tests/unit/bsp/`、`tests/unit/platform/`、`tests/unit/service/`、`tests/unit/app/` |

**没有移植第三方代码。** 磨损均衡在这里只是一个"候选扇区里挑擦除次数最小者"的贪心策略，公共实现（如 LittleFS 的 `lfs_alloc`、SPIFFS/FatFs 的磨损层）都比本项目所需的复杂：它们要支持文件系统语义、可挂载任意目录结构，而本项目的环是固定区域、单写者、顺序追加。重复实现约 100 行比引入一个文件系统更小、更可测，因此算法按公开的贪心理路重写，未复制代码。

---

## 2. 分层与调用链

```
service_storage            (02_Service)
   ├── platform_logstore   (03_Platform/platform_middleware)  ── 环的扇区头、记录头、挂载扫描
   │      └── platform_wearlevel                              ── 选下一个要擦的扇区
   ├── platform_crc        (03_Platform/platform_middleware)  ── CRC-16 / CRC-32
   └── platform_flash_*    (03_Platform/platform_bsp)         ── 读/写/擦/几何/忙
          └── impl_bsp_flash (04_Impl/impl_bsp)               ── ops 表与分发
                 └── bsp_flash_w25q64 (06_BSP)                ── 命令、片选、轮询
                        └── platform_spi_* (03_Platform/platform_mcu)
                               └── impl_spi_wrapper (04_Impl/impl_wrapper)
                                      └── impl_mcu_spi (04_Impl/impl_mcu)
                                             └── HAL SPI2 ──► W25Q64
```

调用方向严格保持 `01_APP → 02_Service → 03_Platform → 04_Impl → 06_BSP` 的既有约定：BSP 只依赖 Platform 契约（`platform_spi_*`、`platform_gpio_*`、`platform_delay_*`），不认识 HAL；`impl_bsp_flash` 是唯一认识 `bsp_flash_w25q64_t` 的地方。

---

## 3. SPI 实现（04_Impl）

### 3.1 接线与初始化

| 信号 | 引脚 | 配置 |
| --- | --- | --- |
| SCK | PB13 | 复用推挽，50 MHz |
| MISO | PB14 | 浮空输入 |
| MOSI | PB15 | 复用推挽，50 MHz |
| CS | PB12 | 推挽输出，上电默认高（`FLASH_CS`） |

`spi.c` 按 CubeMX 风格生成，`MX_SPI2_Init()` 在主循环前调用；`app.ioc` 同步加上了 SPI2 与四个引脚，重新生成不会把配置冲掉。

| 参数 | 取值 | 说明 |
| --- | --- | --- |
| 模式 | 0（CPOL=0, CPHA=1Edge） | W25Q64 支持模式 0/3 |
| 数据宽度 | 8 bit，MSB first | |
| NSS | 软件 | 片选由 BSP 显式控制事务边界 |
| 分频 | /8 → 4.5 Mbit/s | PCLK1 = 36 MHz；低于 W25Q64 的 104 MHz 上限，留给长排线 |

逻辑总线只映射了 `PLATFORM_SPI_BUS_0 → SPI2`；其余 ID 返回 `NOT_SUPPORTED`，避免"没接线却以为能用"。

### 3.2 传输与错误

- 只用 HAL 阻塞接口（`HAL_SPI_Transmit/Receive/TransmitReceive`）。异步回调接口保留为 `NOT_SUPPORTED`：当前没有任何调用者，先不引入 DMA 缓冲与完成回调的复杂度（DMA 是 flash-layout.md 的未来项，不影响功能）。
- `len == 0` 直接成功；`len > 0xFFFF` 返回 `PARAM`（HAL 的 Size 是 16 位）。
- HAL 的 `HAL_TIMEOUT`/`HAL_BUSY` 映射为 `PLATFORM_ERR_TIMEOUT`/`PLATFORM_ERR_BUSY`。
- 发送和接收分开实现；接收时 HAL 自动输出 `0xFF`，正好是 NOR 期望的空字节。

### 3.3 总线互斥

`impl_mcu_spi.c` 持有一个静态分配的 FreeRTOS 互斥量（`xSemaphoreCreateMutexStatic`），第一次 `platform_spi_init()` 时创建：

- 调度器启动前 `xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED`，加解锁直接返回成功——否则 `board_init()` 在 `vTaskStartScheduler()` 之前读 JEDEC 会永远等不到。
- 调度器启动后是非递归互斥量。**调用者不得在已持有该锁时再次调用驱动**；头文件已注明这一点。
- 锁的粒度是"一次完整事务"（CS 拉低到释放），BSP 的每个公共函数各取一次。

---

## 4. W25Q64 BSP 驱动

### 4.1 约束

| 约束 | 实现 |
| --- | --- |
| 容量 | 8 MiB（`0x800000`），所有地址/长度先做越界检查 |
| 写粒度 | 256 B 页；**跨页写直接返回 `PARAM`，不拆分**——拆分会把调用者的 bug 藏起来 |
| 擦除粒度 | 4 KiB 扇区；地址和长度都必须是 4096 的整数倍，否则 `PARAM` |
| 擦除后语义 | 全 `0xFF`；写只能把 1 变 0 |

### 4.2 命令与时序

| 命令 | 字节 | 超时 |
| --- | --- | --- |
| Write Enable | `0x06` | 轮询 WEL 最长 10 ms |
| Read Status 1 | `0x05` | — |
| Read Data | `0x03` | — |
| Page Program | `0x02` | tPP 3 ms → 50 ms |
| Sector Erase | `0x20` | tSE 400 ms → 1000 ms |
| JEDEC ID | `0x9F` | — |

- 每次写/擦前先 `0x06` 并**确认 WEL 置位**；WEL 不置位说明总线已死，直接超时返回，不盲写。
- 写/擦后轮询 SR1 的 BUSY 位；未接 R/B 引脚，也不用挂起/恢复命令。
- 初始化先等 tPUW（5 ms），再等 BUSY 清零（热复位可能落在擦除中途），最后读 JEDEC 并比对 `EF 40 17`。ID 不符返回 `FAIL` 并释放 SPI。

### 4.3 上电失败策略

`board_init()` 先 `platform_flash_register()`（失败即致命，属于装配错误），再 `platform_flash_init()` 但**忽略其返回值**：

- 芯片没焊、SPI 坏、ID 不符时，设备保持"未初始化"状态；
- 底盘、机械臂、串口照常工作；
- 存储服务初始化时 `platform_flash_get_info()` 返回 `NOT_INITIALIZED`，由上层转成故障码 `0x0601`/`0x0604`。

理由：一块存不了参数的板子仍然应该能开、能走、能调试；直接卡在启动里反而失去现场。

---

## 5. 存储服务层（02_Service）

### 5.1 接口

| 接口 | 作用 |
| --- | --- |
| `service_storage_init(now_ms)` | 挂载两个环 → 读寿命记录 → 探测参数/标定副本 → 启动计数 +1 → 扫描崩溃槽 |
| `service_storage_deinit()` | 只释放服务状态，不动 Flash |
| `service_storage_save_blob(which, data, len, now_ms)` | 写参数/标定 A/B 副本，擦→写→回读校验 |
| `service_storage_load_blob(...)` | 两份都校验，取 `seq` 大者；都坏返回 `found=FALSE` |
| `service_storage_blob_valid/seq` | 状态查询 |
| `service_storage_append(ring, level, tag, text, len, now_ms)` | 故障环 / 日志环追加 |
| `service_storage_iter_begin` | 从最旧记录开始遍历 |
| `service_storage_ring_clear(ring, now_ms)` | 逻辑清空（见 6.4） |
| `service_storage_crash_write(data, len, now_ms)` | 写崩溃现场，提交 CRC 最后写 |
| `service_storage_crash_info/read/mark_read` | 崩溃槽分片读取与已读标记 |
| `service_storage_get_status(&status)` | 供 `REPORT_STORAGE` 的 24 B 视图 |
| `service_storage_flush_wear(now_ms)` | 立即落盘寿命记录 |

服务层**不理解参数结构体**：应用序列化成字节后交进来，服务只保证"读回来的字节与写进去的完全相同，或者什么都没有"。

### 5.2 初始化顺序（也是上电恢复顺序）

1. `platform_flash_get_info()`，容量必须覆盖到出厂信息区（`0x7F0000 + 4 KiB`）；
2. 挂载故障环、日志环：扫描每扇区头，找最新扇区，从第一条坏记录处"封口"；
3. 读寿命记录（A/B），用它恢复日志环的逐扇区擦除计数；
4. 探测参数、标定副本（只校验头与 CRC，不加载内容）；
5. 启动计数：找到第一个 `0xFFFFFFFF` 槽，写入 `0x00000001`；
6. 扫描 16 个崩溃槽，置"有未读崩溃"标志；
7. 记录当前环擦除总数，作为下次落盘寿命记录的基准。

任何一步 I/O 失败都返回错误，由上层决定是否降级；`init` 不会"修好再说"。

---

## 6. 磨损均衡（03_Platform/platform_middleware）

### 6.1 问题

W25Q64 每扇区标称 10 万次擦除。日志环 896 KB（224 个扇区）顺序轮转，正常使用时每个扇区被擦的次数本来就相等；真正会打破均衡的是：

- `LOG_CLEAR` 把环清空后，下一次写入若仍从固定扇区开始，头部扇区会比尾部多擦一轮；
- 未来若把某段子区域固定分配给某类数据，冷数据会长期占着被擦过的扇区。

因此策略只需保证"**在允许擦除的扇区里，永远挑擦除次数最少的**"，同时不能破坏顺序环的局部性。

### 6.2 算法

```
pick_filtered(region, hint, filter):
    lowest = min{ count[i] : filter(i) }
    for k in 0..n-1:
        i = (hint + k) mod n
        if filter(i) and count[i] == lowest:
            return i
```

- 时间复杂度 O(n)，环最大 256 个扇区，单次最多 256 次比较，不占控制环时间；
- 空间是每个扇区一个 `uint16` 计数器（224 × 2 = 448 B，静态分配）；
- `filter` 由数据所有者提供，因为环里**不是所有扇区都能擦**：还有活记录的扇区不能动。

### 6.3 在日志环里的候选集

`platform_logstore` 打开新扇区时：

- 环**未写满**：可以选任何从未使用（扇区头无效，`sector_seq == 0`）的扇区 → 候选集是全部空闲扇区，取擦除次数最少者。这就是清空后的重新均衡。
- 环**已写满**：只有 `sector_seq` 最小的那个扇区可以回收，候选集退化为一个元素 → 策略不产生任何额外行为，环保持严格顺序。

健康状态下（环满且每个扇区次数相同）`hint = last_sector + 1` 恰好就是最冷扇区，返回它，与普通顺序环完全一致。**不均衡只会在真正发生不均衡时被纠正**，这是把"磨损均衡"做成独立中间件却几乎零开销的原因。

### 6.4 计数器从哪来、到哪去

| 时机 | 动作 |
| --- | --- |
| 每次擦扇区 | `platform_wearlevel_note_erase()`，饱和到 0xFFFF 不回绕 |
| 上电 | 从寿命记录恢复**日志环逐扇区计数**（448 B 表） |
| 落盘 | 每累计 16 次环擦除、每次参数保存后、每次崩溃写入后 |

故障环的逐扇区计数不持久化：故障是稀疏事件，逐扇区表没有信息量，只累加总次数（`erase_count[4]`）。日志环才是磨损热点，所以只有它需要 224 项的明细。

### 6.5 与"静态磨损均衡"的距离

静态磨损均衡会把长期不动的数据搬家，以利用冷块的寿命。本项目的持久区全部是**周期性重写的环或 A/B 副本**，不存在"永远不动的热数据"，因此不需要搬移；参数区 A/B 交替写本身已经把擦除次数对半分。将来若加入只读大对象（字库、图片），应放到独立分区并单独评估，而不是给日志环加搬移逻辑。

---

## 7. Flash 数据格式（实现细节）

### 7.1 通用记录头（32 B）

与 flash-layout.md 第 4 节一致，字段偏移固定：

| 偏移 | 类型 | 字段 |
| --- | --- | --- |
| 0 | u32 | magic |
| 4 | u16 | layout_ver = 1 |
| 6 | u16 | flags（bit0 有效，bit1 崩溃未读） |
| 8 | u32 | seq |
| 12 | u32 | payload_len |
| 16 | u32 | crc32 |
| 20 | u32 | ts（秒） |
| 24 | u16 | writer = 1（App） |
| 26 | u16 | 0xFFFF |
| 28 | u32 | 0xFFFFFFFF |

**CRC-32 覆盖 `0..15` + `20..31` + payload**：跳过 `16..19`（CRC 自己的字段），这样可以先算 CRC 再填进去。flash-layout.md 只写了"覆盖 magic..payload"，这是实现时的具体化，不改变任何偏移。

### 7.2 参数 / 标定 / 寿命记录（A/B）

```
0x050000  PARAM A (4 KiB 有效)      0x058000  PARAM B
0x060000  CALIB A                   0x068000  CALIB B
0x180000  WEAR  A (4 KiB)           0x181000  WEAR  B
```

写入流程（`s_ab_store`）：

1. 目标 = 当前**非活动**副本；
2. 擦除目标扇区（活动副本不动）；
3. 写 32 B 头 + 负载（按 256 B 页自动拆分）；
4. **回读并重新校验整条记录**；
5. 校验通过才更新 RAM 里的活动副本指针。

第 4 步失败时，旧副本仍是活动的，掉电语义与"这次保存没发生"相同。`seq = 上次 + 1`，0 回绕到 1，加载时取有效且 `seq` 大者。

### 7.3 寿命记录（488 B 负载）

| 偏移 | 内容 |
| --- | --- |
| 0 | 启动计数代数 `boot_epoch` |
| 4..27 | `erase_count[6]`：参数、标定、镜像、故障环、日志环、崩溃 |
| 28 | 日志累计写入字节 |
| 32 | 日志环回卷次数 |
| 36 | 上次写入时间 |
| 40..487 | 日志环 224 个扇区的 `uint16` 擦除计数 |

寿命记录自己也是 A/B 副本，写入时机：参数/标定保存后、崩溃现场写入后、累计 16 次环擦除后。这样它不会变成新的磨损热点，掉电最多丢 16 次以内的计数增量。

### 7.4 故障环 / 日志环

| 项 | 故障环 | 日志环 |
| --- | --- | --- |
| 起始 | `0x080000` | `0x0A0000` |
| 扇区数 | 32 | 224 |
| 容量 | 32 × 4080 ≈ 127 KB | 224 × 4080 ≈ 892 KB |
| 单条负载上限 | 240 B | 240 B |

扇区头 16 B：`u16 magic 0x5EC7`、`u16 state`（0 = 有效）、`u32 sector_seq`、`u16 first_record_offset`、`u16 crc16`、`u32 0xFFFFFFFF`。

记录头 16 B：`u16 magic 0x5BF5`、`u8 level`、`u8 tag`、`u32 ts`、`u16 len`、`u16 crc16`、`u32 seq`，后接 4 B 对齐的负载。**记录不跨扇区**。

挂载：从最新扇区（`sector_seq` 最大）往后读，遇到第一条 magic/CRC 不成立的记录即"封口"——把 `active_offset` 置为扇区末尾，后面的空间不再使用。这样半条记录不会把更晚的记录藏在自己后面。

清空（`LOG_CLEAR`）：**把每个扇区头的 magic 写 0**（2 字节），不擦除。理由：

- 224 次扇区擦除在低速 SPI 上要十几秒，而写 2 字节 × 224 不到一秒；
- 擦除被推迟到扇区真正被复用时，由磨损均衡选最冷的扇区；
- 掉电在中途只会留下"部分清空"的环，仍然自洽：已清空的扇区视为空闲，未清空的照常读取。

### 7.5 崩溃现场

16 槽 × 4 KiB，槽 `n` 位于 `0x070000 + n × 0x1000`。选择槽的顺序：先找无效槽，否则找 `seq` 最小的有效槽。

写入顺序：

```
擦除槽
→ 写 32 B 记录头（flags = VALID | UNREAD，头里带 CRC32）
→ 写负载
→ 最后 4 字节写 CRC32（提交标记）
```

校验同时要求：`tail == header.crc32` **且** 按同一算法重算整条记录通过。任一不满足则该槽作废。这样"写了一半崩溃"的槽既不会被当成有效现场，也不会阻止下一次写入。

**崩溃记录的 CRC 不覆盖 flags 字段**（偏移 6..7）：标记已读只需把 UNREAD 位清 0，而 NOR 只能 1→0，若 flags 在 CRC 内就没法原地改。flags 之外的字段（magic、版本、seq、长度、时间、负载）全部被覆盖。

### 7.6 启动计数

`0x004000` 起 1024 个 `u32` 槽。启动时顺序找到第一个 `0xFFFFFFFF` 写入 `0x00000001`；写满才擦整个扇区，并把代数 `boot_epoch` 加 1（存在寿命记录里）。

`boot_total = epoch × 1024 + slot + 1`。

**已知边界**：若上电过程中掉电恰好发生在"启动计数已写、寿命记录未更新"之间，`boot_epoch` 会退回到旧值，下次启动会重用一个已经用过的槽。最坏后果是启动总数少算，不会破坏任何数据；在 bootloader 接管代数管理前，这个误差可接受，写入寿命记录前也会重新读槽位。

---

## 8. 掉电安全矩阵

| 时刻 | 后果 | 恢复 |
| --- | --- | --- |
| 参数擦除非活动副本中 | 活动副本完好，本次保存丢失 | 下次保存重试 |
| 参数写负载中 | 新记录 CRC 失败 | 用旧副本（`seq` 较小但有效） |
| 参数回读校验中 | 同上 | 同上 |
| 环追加记录中 | 该条 CRC 失败，挂载时封口 | 上一条为最新 |
| 环打开新扇区（擦除）中 | 该扇区头无效，视为空闲 | 下次重选（磨损均衡会选最冷者） |
| `LOG_CLEAR` 中途 | 部分扇区已逻辑清空 | 剩余记录仍可读，再次清空即可 |
| 崩溃现场写负载中 | 无提交 CRC | 该槽作废 |
| 崩溃现场写提交 CRC 中 | CRC 不完整，校验失败 | 该槽作废 |
| 寿命记录写中 | 旧记录仍有效 | 丢 ≤16 次擦除增量 |
| 启动计数写入中 | 槽内可能是 0xFFFFFFFE 之类的半写值 | 该槽被当作已用，计数少算 1 |

---

## 9. 测试与编译结果

### 9.1 PC 单元测试（L1）

全部测试按文件头注释里的 `gcc` 命令编译（MinGW gcc，`-std=c99 -Wall -Wextra`，部分套件加 `-Werror`），退出码全为 0：

| 套件 | 覆盖 | 结果 |
| --- | --- | --- |
| `test_protocol_codec` | 帧编解码、CRC、分片 | PASS |
| `test_platform_middleware` | ringbuf / crc / filter / pid | 全部通过 |
| `test_platform_storage` | 磨损均衡 + 记录环，16 KiB RAM NOR 模型 | 通过 |
| `test_bsp_flash_w25q64` | 驱动：JEDEC、读、写、擦、页/扇区约束、WEL 失败 | 通过 |
| `test_service_proto` | 协议服务 | PASS |
| `test_service_chassis` | 底盘服务（含 `-lm`） | PASS |
| `test_service_key` | 按键服务 | PASS |
| `test_service_storage` | 存储服务：A/B、环、崩溃、启动计数、寿命统计 | 通过 |
| `test_app_state` | 应用状态机 | PASS |
| `test_app_fault` | 故障仲裁（含故障入环） | PASS |
| `test_app_params` | 参数 RAM 副本、默认值、序列化、坏 blob 拒绝 | PASS |
| `test_app_storage` | 暂存/借用、回调、丢弃计数、导出节拍与结束帧 | PASS |

驱动测试用一个 8 MiB 的内存 W25Q64 模型替代 `platform_spi_*`：它按 CS 边沿解析命令/地址，擦除置 `0xFF`，页编程按位与，状态寄存器的 BUSY 需要几次轮询才清零，并提供"JEDEC 错误"和"WEL 不锁存"两个故障注入开关。这样驱动的时间等待、错误映射和页边界逻辑都被真正执行，而不是被桩函数绕开。

服务层测试在同一模型上额外覆盖了**重启语义**：写崩溃现场 → 反初始化 → 重新初始化 → 记录仍有效；清掉提交 CRC → 再重启 → 记录作废。这一条是修复"崩溃记录头 CRC 与提交 CRC 不一致"缺陷的回归用例。

### 9.2 Keil / ARMCC

```
Program Size: Code=54020 RO-data=1324 RW-data=704 ZI-data=27408
"app\app.axf" - 0 Error(s), 0 Warning(s).
```

应用接入后，`service_storage_*` 与两条新命令路径都被引用，链接器不再丢弃它们。相比只有驱动时的 43 528 B，代码增加约 10.5 KB（服务层 6 KB 左右 + 应用任务、参数模块与 6 个处理函数 4.5 KB 左右）。ZI 增加 3 384 B，主要是暂存池 480 B、两个静态队列 464 B 与 `appStorage` 任务栈 1 024 B，另有控制任务队列消息改为 24 B 带来的少量增长。所有新文件在 ARMCC V5.06（默认 C90 语义）下编译通过，无警告。

RAM 余量：48 KB 中 ZI 27.4 KB + RW 0.7 KB ≈ 28.1 KB，剩余约 20 KB 给 FreeRTOS 静态堆与中断栈，仍有余量。

---

## 10. 接线后的台架验证步骤

| 步骤 | 操作 | 通过标准 |
| --- | --- | --- |
| 1 | 上电，读 JEDEC（调试器，或发 `STORAGE_QUERY` 看是否回 `0x00` 而非 `0x06`） | `EF 40 17` |
| 2 | 连续复位 5 次，读启动计数 | 每次 +1，复位后不丢 |
| 3 | 保存参数 → 断电 → 上电加载 | 字节完全一致，`seq` 增加 |
| 4 | 保存过程中拔电 ×20 | 每次上电都能读到一份有效参数或回退默认值 |
| 5 | 写满故障环（可用测试命令循环） | 最旧记录被覆盖，无记录损坏 |
| 6 | `LOG_CLEAR` 后写新日志 | 旧日志不可读，新记录从 seq 继续 |
| 7 | 人为 HardFault | 重启后可读出完整现场，`mark_read` 后不再报未读 |
| 8 | 连续擦写 200 次后读 `REPORT_STORAGE` | 擦除计数误差 ≤16 |

---

## 11. 与 flash-layout.md 的具体化差异

| # | flash-layout.md | 实现 | 原因 |
| --- | --- | --- | --- |
| 1 | 记录头 CRC32 覆盖 `magic..payload` | 跳过 CRC 字段自身（`16..19`） | 否则无法"先算后填"；其余字节全部覆盖 |
| 2 | 崩溃现场 CRC 单独放最后一页 | 头里存一份，最后 4 字节再写一份提交副本 | 尾部是提交标记；头里那份用于常规校验与分片读取 |
| 3 | 崩溃记录未说明已读标记 | flags 的 UNREAD 位，且 flags 不参与 CRC | NOR 只能 1→0，标记已读必须能原地改 |
| 4 | 环清空语义未定 | 逐扇区把 magic 写 0，延迟擦除 | 2 字节 × 224 远快于 224 次扇区擦除 |
| 5 | 日志记录头 `len ≤ 240` | 实现上限 240 B，且整条记录不得跨扇区 | 与 4 KiB 扇区、16 B 头一致 |
| 6 | 日志环逐扇区擦除计数在 RAM 维护、定期落盘 | 实现为寿命记录的 448 B 表 | 有明确落盘载体，掉电增量有界 |
| 7 | 故障环同样有逐扇区计数 | 只存总擦除次数 | 故障稀疏，明细无信息量 |
| 8 | 启动计数槽写 `0x00000001` | 相同 | 实现已按文档修正 |

---

## 12. 应用层接入：`appStorage` 任务

### 12.1 为什么必须有一个任务

一次扇区擦除要 150 ms~1 s，期间 SPI 总线被互斥锁占住；读操作同样要排队。如果通信任务直接调用 `service_storage_*`，200 ms 指令超时必然触发；控制任务被卡住则电机保持上一次占空比。所以 Flash 只能有一个所有者：`appStorage`，优先级 `PLATFORM_OS_PRIORITY_LOW`，低于通信与控制任务——存储永远不紧急。

### 12.2 队列只传描述符

一个静态队列槽位总共只有 512 B（`IMPL_OS_QUEUE_STORAGE_BYTES`），4 KiB 的参数区放不进消息里。做法是：

| 负载大小 | 机制 | 规则 |
| --- | --- | --- |
| ≤ 240 B（日志、故障码、参数 60 B、标定 80 B） | 入队时复制进 2 个暂存槽之一 | 槽位由临界区保护；两个槽都占用时入队失败，调用方拿到 FALSE |
| > 240 B | 借用调用方缓冲区 | 同一时间只允许一笔借用；第二笔被拒，调用方因此总能知道"我的缓冲区什么时候可以释放" |

请求结构体 24 B（x86 上是 40 B，由 `sizeof` 决定），字段是 `kind/arg0..arg2/length/staging/payload/done/user`。回调在存储任务里、Flash 操作返回之后**恰好调用一次**；入队被拒时一次都不调用。回调里不允许阻塞，也不允许再调用本模块。

### 12.3 谁来发帧

存储任务只产生中立的导出记录（`app_storage_export_t`）放进第二个队列；通信任务在 `service_proto_process()` 之后调用 `app_storage_pump()`，是唯一把 `REPORT_LOG` / `REPORT_CRASH` / `REPORT_STORAGE` 写进 UART 的地方。这与"单发送者"约定一致，也保证了 ACK 先于它所预告的报告帧。

### 12.4 队列满与丢弃

| 事件 | 行为 | 计数 |
| --- | --- | --- |
| 日志行被等级过滤 | 直接丢弃 | `stats.drops` |
| 暂存槽全忙 / 请求队列满 | 丢弃 | `stats.drops` |
| `PARAM_SAVE` 被拒 | 返回 ACK `0x05`，任务照常运行 | — |
| 导出队列满 | 本周期不产生下一条，下个 50 ms 再试 | — |
| 状态帧请求 | `status_pending` 是布尔量，多次请求只发一帧 | — |

日志可以丢，参数保存不能：前者是消耗品，后者是用户按了按钮的动作，必须让主机知道成功与否。

### 12.5 导出的节拍与结束

- 每条 `REPORT_LOG` 的 `flags` 恒带 `MORE`（bit0），导出结束时单独发一帧 `flags=0x02、text_len=0` 的结束帧。主机看到 `MORE` 一直不消失，就知道自己丢了结束帧。
- 崩溃导出按 57 B 分片，**没有结束帧**：主机按 `offset + total` 判定完成，超时即重来。
- 存储任务每 10 ms 醒一次，但每 50 ms 才产生一条记录（`APP_STORAGE_EXPORT_PERIOD_MS`），对应 protocol.md 的 20 Hz 上限；节拍取 10 ms 是因为它整除 50 ms，20 ms 会让实际速率漂到 16.7 Hz。
- 主机静默 500 ms（`service_proto_rx_age_ms`）后，`app_storage_pump()` 主动停止导出；停止会同时取消运行中的日志导出与崩溃导出——对着一台已经离线的上位机继续发帧没有意义。

### 12.6 0x08~0x0D 的检查次序

| 命令 | 检查 | ACK 结果 |
| --- | --- | --- |
| `PARAM_SAVE` | Flash 可用 → 底盘状态（标定要求失能）→ 上一笔保存是否结束 | `0x06` / `0x02` / `0x05` / `0x00` |
| `PARAM_RESET` | confirm → 底盘状态 → 保存是否在途 | `0x01` / `0x02` / `0x05` / `0x00` |
| `STORAGE_QUERY` | Flash 可用 | `0x06` / `0x00` + 一帧 `0xC8` |
| `LOG_EXPORT` | 参数范围 → 是否已有导出 | `0x01` / `0x05` / `0x00` |
| `CRASH_READ` | slot 有效性（查挂载扫描缓存，不读 Flash） | `0x01` / `0x04`（无记录）/ `0x05` / `0x00` |
| `LOG_CLEAR` | confirm → source → Flash 可用 | `0x01` / `0x06` / `0x00` |

所有 ACK 只表示"受理"；真正成功与否由随后的 `REPORT_STORAGE`（bit3 = 最近一次保存失败）和 `REPORT_LOG` 结束帧表达。

### 12.7 参数、标定与脏标记

`app_params` 持有 RAM 副本与字节布局（版本号 2，参数 60 B，标定 80 B，全部显式小端；加载时仍接受 v1 的 56 B/28 B，见 flash-layout.md 5.3.1）。`SET_PARAM` / `PARAM_RESET` 只改 RAM 并置脏；`PARAM_SAVE` 序列化后入队，写成功的回调里 `app_params_mark_saved(scope)` 清掉对应脏位，两个 scope 都干净时才把 `REPORT_STORAGE` bit0 置 1。

`PARAM_RESET` 与标定类 `SET_PARAM` 都不直接碰 `service_chassis`：它们通过控制任务队列发一条 `APP_CTRL_MSG_PID_RELOAD`，由控制任务调用 `app_params_apply_to_chassis()` 重载增益、几何、四轮比例/方向/起转占空比和限速——`service_chassis_set_pid_gain()` 会重置积分器，不能和 `apply()` 并发。限速类 `SET_PARAM`（`0x10~0x12`）允许在使能状态下写，因此走单独的 `APP_CTRL_MSG_LIMITS_RELOAD`，只调用 `app_params_apply_limits_to_chassis()`，不重置 PID 状态。标定区现在也是热更新的：写入后下一个控制节拍即生效，不需重启。

### 12.8 资源

| 项目 | 数量 | 说明 |
| --- | --- | --- |
| 任务 | 1（`appStorage`） | 栈 1 024 B，优先级 LOW |
| 静态队列 | 2 | 请求 24 B×8，导出 ~68 B×4，均在 512 B 限制内 |
| 暂存池 | 2×240 B | 由临界区保护 |
| 状态 RAM | 约 200 B | 模块状态、导出迭代器、统计 |

---

## 13. 后续工作（按依赖顺序）

| # | 事项 | 依赖 | 说明 |
| --- | --- | --- | --- |
| 1 | HardFault 现场写入 | 12 | SPI 驱动持非递归互斥锁，不能进中断上下文；需要先做"故障时只记录到 RAM，复位后由 `appStorage` 落盘"的路径，或一个不取锁的紧急写入口 |
| 2 | bootloader 与 OTA | — | 镜像槽、引导控制块、回滚策略在 flash-layout.md 已定义，尚未实现 |
| 3 | `SET_PARAM` 0x41 死区总开关 | — | 每轮起转占空比（`0x54~0x5B`）已实现，写 0 即关闭；`0x41` 目前仍返回 `0x01`，要么删掉这个 id，要么把它定义为"一键清零/恢复全部起转值" |
| 4 | SPI DMA | — | 阻塞传输已满足 4.5 Mbit/s 与擦除超时，DMA 只是留 CPU 给控制环的优化 |
| 5 | 关机前 `flush_wear` | 12 | 掉电检测电路未定；当前按"每 16 次擦除落盘"兜底，误差有界 |
| 6 | 日志正文的格式化入口 | 12 | 目前入环的是调用方给的定长文本；需要一个 `printf` 风格的封装，且不能在临界区里格式化 |
| 7 | 温度/寿命告警 | 6 | 擦除计数已在 `REPORT_STORAGE` 里，主机侧可以据此提前换机 |

---

**一句话总结**：分层保持 `Service → Middleware → Platform → Impl → BSP`，服务层用 A/B 副本与 CRC 保证"要么读到完整的一份，要么什么都没有"，环用扇区头 + 记录头 + 挂载封口保证半写记录不掩盖后续数据；磨损均衡是一个 100 行的贪心选择器，只在候选扇区里挑擦除次数最少者，健康环上退化为顺序环、零额外开销；崩溃现场用"最后写提交 CRC"标记完整性，并用不参与 CRC 的 flags 位支持原地标记已读。
