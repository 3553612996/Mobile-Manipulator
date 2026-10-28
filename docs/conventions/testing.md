# 自测规范

> 没有硬件也能测的东西，一定要在没有硬件的时候测掉。**把问题挡在台架之前，是成本最低的做法。**

---

## 1. 测试分层

| 层级 | 测什么 | 在哪里跑 | 频率 | 依赖硬件 |
| --- | --- | --- | --- | --- |
| L1 单元测试 | 纯逻辑：协议解析、CRC、坐标变换、状态机 | PC（x86） | 每次提交 | 否 |
| L2 回放测试 | 节点行为：给定录包输入，检查输出 | PC | 每次改节点 | 否 |
| L3 台架测试 | 电机/关节动作、超时、急停 | 底盘架空、机械臂悬空 | 每次改固件 | 是 |
| L4 实车测试 | 完整任务链 | 真实环境 | 每次发版 | 是 |

**L1 能覆盖的部分必须 100% 覆盖**。协议解析出错是最常见的联调事故，而它在 PC 上十分钟就能测完。

测试代码索引在 [`tests/README.md`](../../tests/README.md)；每个节点的测试报告与分层检查归档在 [`docs/nodes/<节点>/`](../nodes/)。

---

## 2. L1：单元测试

### 2.1 什么代码可以被单测

能被单测的前提是**不依赖硬件**。当前已经可以在 PC 上跑的模块：

| 测试 | 覆盖对象 | 依赖 |
| --- | --- | --- |
| `tests/unit/protocol/test_protocol_codec.c` | `05_Middlewares/protocol`：协议表、组帧、解帧、CRC | 无（只需 gcc） |
| `tests/unit/service/test_service_proto.c` | `02_Service/service_proto`：分发、ACK、SEQ 去重、上报组帧 | UART 用内存管道替身 |
| `tests/unit/service/test_service_chassis.c` | `02_Service/service_chassis`：麦轮解算、超时、急停、里程计 | 电机用桩函数替身 |
| `tests/unit/service/test_service_key.c` | `02_Service/service_key`：短按/长按策略 | 按键用脚本状态替身 |
| `tests/unit/platform/test_platform_middleware.c` | 环形缓冲、CRC、滤波 | 无 |

**测不了的**：`impl_mcu_*.c`、`impl_bsp_*.c`、`bsp_*.c`——它们直接操作寄存器和 HAL。

所以设计上的要求是：**能抽出来的逻辑一定抽出来**。服务层把时间、设备访问都做成参数或桩，纯逻辑（运动学、超时判定、按键策略）就能脱离硬件测试；`impl_*` 只负责把结果写进寄存器。

### 2.2 怎么写

```c
/* tests/unit/protocol/test_protocol_codec.c */
#include <stdio.h>
#include <string.h>
#include "protocol.h"

static int g_failed = 0;

#define CHECK(cond, msg)                          \
    do {                                          \
        if (!(cond)) {                            \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); \
            g_failed++;                           \
        }                                         \
    } while (0)

static void test_crc_selfcheck(void)
{
    CHECK(crc16_ccitt((const uint8_t *)"123456789", 9) == 0x29B1,
          "CRC 自检值应为 0x29B1");
}

static void test_set_velocity(void)
{
    const uint8_t frame[] = {0xAA,0x55,0x06,0x01,0x01,0x2C,0x01,0xF4,0x01,0x72,0x3F};
    proto_frame_t f;
    int got = 0;
    for (size_t i = 0; i < sizeof(frame); i++)
        got |= proto_feed_byte(frame[i], &f);

    CHECK(got == 1,      "应解析出 1 帧");
    CHECK(f.cmd == 0x01, "CMD 应为 SET_VELOCITY");
    CHECK(f.seq == 0x01, "SEQ 应为 1");
    CHECK(f.len == 4,    "负载长度应为 4");
}

int main(void)
{
    test_crc_selfcheck();
    test_set_velocity();
    printf(g_failed ? "\n%d 项失败\n" : "\n全部通过\n", g_failed);
    return g_failed ? 1 : 0;
}
```

### 2.3 必须有测试的用例

协议部分**至少**覆盖：

| 用例 | 期望 |
| --- | --- |
| CRC 对 `"123456789"` | `0x29B1` |
| 标准 SET_VELOCITY 帧 | 解析成功，字段正确 |
| 空负载 ESTOP 帧（最短帧） | 解析成功 |
| CRC 错一位 | 丢弃，状态复位 |
| `LEN = 1`（小于 2） | 丢弃，不越界 |
| `LEN = 255`（大于 66） | 丢弃，不越界 |
| 半帧后中断 | 超时复位，后续正常帧能解出 |
| 连续喂 10 万随机字节 | 不崩溃、不越界 |
| 逐字节喂 vs 一次性喂 | 结果一致 |

### 2.4 运行

完整编译命令写在每个测试文件的头部注释里。汇总：

```bash
# 协议表 + 编解码
gcc -std=c99 -Wall -Wextra \
    -I firmware/app/03_Platform/platform_common \
    -I firmware/app/04_Impl \
    -I firmware/app/03_Platform/platform_middleware \
    -I firmware/app/05_Middlewares/protocol \
    tests/unit/protocol/test_protocol_codec.c \
    firmware/app/05_Middlewares/protocol/protocol_codec.c \
    firmware/app/05_Middlewares/protocol/protocol_def.c \
    firmware/app/03_Platform/platform_middleware/platform_crc.c \
    -o build/test_protocol_codec

# 服务层（命令见文件头）
#   tests/unit/service/test_service_proto.c
#   tests/unit/service/test_service_chassis.c   （需要 -lm）
#   tests/unit/service/test_service_key.c
```

**通过标准**：退出码 0，输出 `PASS: ...`。任何一个 FAIL 都要在联调前修掉。

---

## 3. L2：回放测试

用录好的 bag 重放，验证节点的输入输出是否正常。

```bash
# 录包（只录需要的话题，否则文件巨大）
ros2 bag record -o ~/bags/2026-10-27-nomap \
    /scan /odom /cmd_vel /tf /camera/color/image_raw/compressed

# 回放
ros2 bag play ~/bags/2026-10-27-nomap --rate 0.5
```

**检查项**：

- `ros2 topic hz /cmd_vel` 频率稳定在 50 Hz ±20%
- 节点无崩溃、无日志 ERROR
- 输出的数值在物理合理范围内（速度不超过上限、角度不超过限位）

**录包的时机**：每次实车测试都录。出问题时能复现，比去现场重跑一百次都强。

---

## 4. L3：台架测试

底盘架空（轮子悬空）、机械臂悬空（不接触任何东西）。

### 4.1 检查清单

| 项目 | 通过标准 |
| --- | --- |
| 电机正反转 | 指令为正时轮子正转，方向符合定义 |
| 编码器方向 | 正转计数增加，反转减少 |
| 速度闭环 | 目标 0.3 m/s，实测误差 < 5% |
| 里程计 | 直行 2 m 误差 < 2%；原地转 360° 误差 < 2% |
| 关节限位 | 超过限位时拒绝执行并上报，不撞机械限位 |
| 关节重复精度 | 同一目标角往返 10 次，位置标准差 < 0.5° |
| 通信超时 | 拔掉串口线，200 ms 内电机输出归零 |
| 急停 | 按下后 100 ms 内所有输出关闭 |
| 过流保护 | 手动堵转，电流超阈值后停止并上报 |

### 4.2 怎么测超时

不要在代码里改超时时间（改了就忘了改回来）。用物理方式：**直接拔串口线**。这是最接近真实故障的测法。

### 4.3 协议闭环

[`tools/bench/proto_loop.py`](../../tools/bench/proto_loop.py) 按协议 v1 逐条验证"下发 → ACK → 上报"三段是否一致：

```bash
python tools/bench/proto_loop.py --list                        # 找串口
python tools/bench/proto_loop.py --port COM3 --monitor         # 只看上行帧和频率
python tools/bench/proto_loop.py --port COM3 --auto            # 静态项：使能/急停/清故障/拒绝非法帧
python tools/bench/proto_loop.py --port COM3 --auto --move     # 加上速度、断流停车（底盘必须架空）
python tools/bench/proto_loop.py --sim --auto --move           # 用内置模拟器自检脚本本身
```

每一项都先看 ACK，再看紧随其后的 `STATUS`/`ODOM`/`EVENT`：只回了 ACK 不算通过，状态没变说明处理函数和应答对不上。通过标准是**全部 PASS、0 帧 CRC 错误**。

`0x0202`（断流停车）只有真机能测：模拟器只把速度归零，不锁存故障，这一项在 `--sim` 下显示为 SKIP。

**电池低于 6.8 V 时**（`0x0401` 锁存，状态字节停在 `FAULT`），`MANUAL` 和"清除后回到 IDLE"两项无论如何都到不了。脚本按实测电压把它们记为 SKIP 并打印 mV，而不是 FAIL——否则红灯会指向固件，实际是台架条件不对。要让这两项跑绿，先把电池充到 6.8 V 以上（或换稳压电源）。

---

## 5. L4：实车测试

固定场景跑固定任务序列：**导航到 A → 抓 B → 放到 C**。

### 5.1 记录什么

| 指标 | 说明 |
| --- | --- |
| 成功率 | N 次中成功几次（至少 20 次才有统计意义） |
| 失败类型分布 | 各类原因各占多少（用于定位主要瓶颈） |
| 单次耗时 | 从下发指令到完成 |
| 导航定位误差 | 到达点与目标点的距离 |

### 5.2 测试条件必须写明

同一组参数在不同地面、不同光照下表现可能完全不同。每次测试记录：

- 地面材质（瓷砖 / 地毯 / 环氧）
- 光照条件（自然光 / 灯 / 逆光）
- 电池电压（低压时性能下降明显）
- 目标物体（类别、颜色、尺寸、是否反光）

**没有测试条件的成功率没有意义**——"成功率 90%" 在特定条件下成立，换个场景可能只有 50%。

---

## 6. 故障注入

主动制造故障，验证降级逻辑是否按设计工作。**这一步能发现大部分"只在真出问题时才暴露"的 bug。**

| 注入方式 | 期望行为 |
| --- | --- |
| 拔串口线 | 200 ms 内停车，上报通信超时 |
| 遮挡雷达 | 定位发散时停止导航，不盲目前进 |
| 堵转电机 | 电流保护触发，该轮停止，上报 |
| 断网（WiFi） | 本地功能不受影响（这正是选离线语音的原因） |
| `kill -9` 视觉节点 | systemd 自动拉起；期间禁止抓取任务，导航不受影响 |
| 降 CPU 频率 | `cpufreq-set -f 408000`，检查是否出现丢帧、超时 |
| 电池电压降低 | 触发低压告警，禁止新任务 |
| 升级中途断电 | 重启后保留旧固件，不变成砖 |

---

## 7. 缺陷记录

发现问题先记下来，再修。记录要包含**复现步骤**，否则过两天自己都想不起来怎么触发的。

```
标题：导航中途偶发停车，无日志报错

现象：约 1/5 次导航在行进 30~60 秒后停止，cmd_vel 仍在发布，底盘无响应
复现：A 点导航至 C 点，中途经过电机启动区域
初步判断：STM32 收到失能指令或通信中断
待查：串口 CRC 错误率、MCU 侧故障码
```

**"偶发"不是描述**。"约 1/5 次，发生在电机启动区域"才是。

节点发布时，本节点的缺陷连同复现步骤、根因、修复和验证记录写进该节点的 `test-report.md`（见 [`docs/nodes/`](../nodes/)），不要只留在提交信息里。

---

## 8. 发版前检查清单

- [ ] L1 全部通过
- [ ] L2 回放无 ERROR
- [ ] L3 台架清单全部通过
- [ ] L4 实车连续 20 次任务，记录成功率
- [ ] 故障注入 8 项全部符合预期
- [ ] 断电重启后能自动恢复运行
- [ ] 版本号正确写入二进制
- [ ] 标定文件与当前硬件匹配（相机序列号、标定日期）
- [ ] 文档与代码一致（协议、目录结构）
- [ ] 已知问题已记录，没有隐瞒

**最后一条最重要**：发版时把已知问题写清楚，比假装没有更能建立信任。
