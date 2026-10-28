# 05_Middlewares 设计说明

与具体产品无关的中间件。和 `03_Platform/platform_middleware` 的区别：那里是**通用算法**（环形缓冲、CRC、滤波），这里是**本项目对外契约的实现**（串口协议）。

## 落地文件

| 文件 | 职责 |
| --- | --- |
| `protocol/protocol_def.h/.c` | 协议表：帧常量、命令字、负载长度、ACK 规则、结果码、故障码、状态枚举 |
| `protocol/protocol_codec.h/.c` | 组帧、重组与解帧、字段小端读写、解析统计 |

依赖只有 `platform_common`（`platform_def.h` / `platform_error.h`）和 `platform_middleware`（CRC）。**不包含 HAL、FreeRTOS、UART 头**，可以在 PC 上直接编译。

## 为什么协议表放中间件

协议表是**双方共享的契约**：MCU、上位机、PC 单测、文档都引同一组数字。放在中间件里：

- 只有一处定义命令字和负载长度，`service_proto` 与 app 都不许硬编码字节；
- 可以在没有硬件、没有 RTOS 的情况下跑单元测试（`tests/test_protocol_codec.c`）；
- 升级协议时改动集中在一个目录，review 时看得见。

## 协议表

```c
const protocol_cmd_info_t *info = protocol_cmd_lookup(cmd);
/* info->payload_len 是精确长度，不是上限 */
protocol_cmd_payload_ok(cmd, frame->payload_len);
```

`protocol_def.c` 里的表同时给出 `requires_ack`：`SET_VELOCITY` / `SET_ARM_JOINTS` 不要 ACK，其余命令要。`service_proto` 依据这一列决定是否应答、是否做 SEQ 去重。

## 组帧

```c
uint8_t frame[PROTOCOL_FRAME_MAX];
uint32_t len = protocol_encode(cmd, seq, payload, payload_len,
                               frame, sizeof(frame));
```

CRC 覆盖范围是 `LEN` 到负载最后一字节（含），小端存放。`len == 0` 表示参数非法（未知命令、负载超长、缓冲区不够），调用方不应发送。

## 解帧

```c
protocol_parser_t parser;
protocol_parser_init(&parser);

uint32_t used = protocol_parser_feed(&parser, bytes, n, now_ms);
while (protocol_parser_next(&parser, &frame)) {
    /* 处理完整帧 */
}
```

- `feed()` 返回**接收了多少字节**，不一定是全部：缓冲区满时需要先 `next()` 取走帧再继续喂；
- 解析器对任意切分安全，负载中出现 `AA 55` 不会错位；
- 一帧非法只丢 1 字节重新同步，并在 `stats` 中计数；
- 两次 `feed()` 间隔超过 50 ms 时丢弃半帧。

## 测试

```bash
gcc -std=c99 -Wall -Wextra \
    -I firmware/app/03_Platform/platform_common \
    -I firmware/app/04_Impl \
    -I firmware/app/03_Platform/platform_middleware \
    -I firmware/app/05_Middlewares/protocol \
    tests/test_protocol_codec.c \
    firmware/app/05_Middlewares/protocol/protocol_codec.c \
    firmware/app/05_Middlewares/protocol/protocol_def.c \
    firmware/app/03_Platform/platform_middleware/platform_crc.c \
    -o build/test_protocol_codec
```

测试覆盖文档第 10 节的固定字节、往返编码、任意分块、噪声重同步、CRC 错误、负载内伪帧头、非法 LEN、超时复位。**改协议先跑这组测试**。

## 契约冻结

头文件是服务层依赖的稳定接口。新增命令要同时更新：协议文档、`protocol_def.h` 的枚举与长度宏、`protocol_def.c` 的表、PC 测试向量。
