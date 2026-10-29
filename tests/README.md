# 测试索引

> `tests/` 里只放**能编译、能跑**的测试代码。测试结论、缺陷记录写进 [`docs/nodes/`](../docs/nodes/)。

## 目录

```text
tests/
└── unit/                    L1 单元测试：PC 上跑，不依赖硬件
    ├── protocol/            05_Middlewares/protocol
    ├── platform/            03_Platform/platform_middleware
    ├── service/             02_Service/service_*
    ├── bsp/                 06_BSP 驱动
    └── app/                 01_APP/app_robot
```

## 测试清单与节点归属

| 测试 | 层级 | 被测模块 | 引入节点 | 依赖替身 |
| --- | --- | --- | --- | --- |
| [`unit/protocol/test_protocol_codec.c`](unit/protocol/test_protocol_codec.c) | L1 | `05_Middlewares/protocol` | M2 | 无 |
| [`unit/platform/test_platform_middleware.c`](unit/platform/test_platform_middleware.c) | L1 | `platform_ringbuf` / `platform_crc` / `platform_filter` / `platform_pid` | M2（M3 加 PID） | 无 |
| [`unit/service/test_service_proto.c`](unit/service/test_service_proto.c) | L1 | `02_Service/service_proto` | M2 | UART 内存管道 |
| [`unit/service/test_service_chassis.c`](unit/service/test_service_chassis.c) | L1 | `02_Service/service_chassis` | M2（M3 继续用） | 电机桩函数 |
| [`unit/service/test_service_key.c`](unit/service/test_service_key.c) | L1 | `02_Service/service_key` | M2 | 按键状态脚本 |
| [`unit/platform/test_platform_storage.c`](unit/platform/test_platform_storage.c) | L1 | `platform_wearlevel` / `platform_logstore` | 存储子系统 | RAM NOR 仿真（擦 0xFF、只清位、跨页拒绝） |
| [`unit/service/test_service_storage.c`](unit/service/test_service_storage.c) | L1 | `02_Service/service_storage` | 存储子系统 | 8 MiB RAM NOR 仿真 |
| [`unit/bsp/test_bsp_flash_w25q64.c`](unit/bsp/test_bsp_flash_w25q64.c) | L1 | `06_BSP/bsp_flash_w25q64` | 存储子系统 | 内存 W25Q64 模型（命令/CS/BUSY/WEL/JEDEC） |
| [`unit/app/test_app_state.c`](unit/app/test_app_state.c) | L1 | `01_APP/app_robot/app_state` | M3 | 无 |
| [`unit/app/test_app_fault.c`](unit/app/test_app_fault.c) | L1 | `01_APP/app_robot/app_fault` | M3 | 底盘/机械臂/协议服务桩 |

上位机的编解码测试在 [`tools/host_gui/test_protocol.py`](../tools/host_gui/test_protocol.py)，与被测代码同目录（依赖 `host_gui.protocol` 的 import 关系），不复制到 `tests/`。

L3 台架测试不在 `tests/`：它需要在真机上跑，工具是 [`tools/bench/proto_loop.py`](../tools/bench/proto_loop.py)。

## 为什么按模块分，而不是按节点分

同一个测试会在后续节点继续跑：`test_service_chassis.c` 在 M3 加 PID 后仍要验证超时和急停，`test_protocol_codec.c` 在协议升 v2 时仍是最先要过的一关。按节点建目录，文件到期就得搬家，编译命令和文档链接跟着失效；按被测模块建目录，文件位置与固件分层一一对应，节点归属看上面的表就够了。

## 运行

每份测试的开头注释里都有完整的 `gcc` 命令（包含全部 `-I` 和源文件），复制即可。汇总：

| 测试 | 产物 |
| --- | --- |
| `test_protocol_codec.c` | `build/test_protocol_codec` |
| `test_platform_middleware.c` | `build/test_platform_middleware` |
| `test_service_proto.c` | `build/test_service_proto` |
| `test_service_chassis.c` | `build/test_service_chassis`（需要 `-lm`） |
| `test_service_key.c` | `build/test_service_key` |
| `test_platform_storage.c` | `build/test_platform_storage` |
| `test_service_storage.c` | `build/test_service_storage` |
| `test_bsp_flash_w25q64.c` | `build/test_bsp_flash_w25q64` |
| `test_app_state.c` | `build/test_app_state` |
| `test_app_fault.c` | `build/test_app_fault` |
| `host_gui/test_protocol.py` | `python tools/host_gui/test_protocol.py` |

**通过标准**：进程退出码 0。测试用 `CHECK()` 宏累计失败数，有任何一项 FAIL 都会返回非 0，CI 与手工执行都以退出码为准。

测试规范（分层、用例要求、缺陷记录格式）见 [`docs/conventions/testing.md`](../docs/conventions/testing.md)。
