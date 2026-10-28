/* L1 unit test for 05_Middlewares/protocol. Runs on the PC, no hardware.
 *
 * Build (MinGW or any host gcc):
 *   gcc -std=c99 -Wall -Wextra \
 *       -I firmware/app/03_Platform/platform_common \
 *       -I firmware/app/04_Impl \
 *       -I firmware/app/03_Platform/platform_middleware \
 *       -I firmware/app/05_Middlewares/protocol \
 *       tests/unit/protocol/test_protocol_codec.c \
 *       firmware/app/05_Middlewares/protocol/protocol_codec.c \
 *       firmware/app/05_Middlewares/protocol/protocol_def.c \
 *       firmware/app/03_Platform/platform_middleware/platform_crc.c \
 *       -o build/test_protocol_codec
 */

#include <stdio.h>
#include <string.h>

#include "protocol_codec.h"

static int g_failed = 0;

#define CHECK(cond, msg)                                                  \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);        \
            g_failed++;                                                   \
        }                                                                 \
    } while (0)

/* ---- Vectors from docs/architecture/protocol.md section 10 ---------- */

static const uint8_t k_set_velocity[] =
{
    0xAA, 0x55, 0x06, 0x01, 0x01, 0x2C, 0x01, 0xF4, 0x01, 0x72, 0x3F
};

static const uint8_t k_set_enable[] =
{
    0xAA, 0x55, 0x03, 0x04, 0x02, 0x01, 0x9F, 0xB5
};

static const uint8_t k_estop[] =
{
    0xAA, 0x55, 0x02, 0x06, 0x03, 0x39, 0x38
};

/* Feed a block and collect every frame it yields. */
static uint32_t feed_and_collect(protocol_parser_t *parser,
                                 const uint8_t *data,
                                 uint32_t len,
                                 uint32_t now_ms,
                                 protocol_frame_t *frames,
                                 uint32_t max_frames)
{
    uint32_t offset = 0u;
    uint32_t count = 0u;

    while (offset < len)
    {
        uint32_t taken = protocol_parser_feed(parser, &data[offset],
                                              len - offset, now_ms);
        if (taken == 0u)
        {
            break;
        }
        offset += taken;

        while ((count < max_frames) &&
               protocol_parser_next(parser, &frames[count]))
        {
            count++;
        }
    }

    while ((count < max_frames) &&
           protocol_parser_next(parser, &frames[count]))
    {
        count++;
    }

    return count;
}

static void test_encode_vectors(void)
{
    uint8_t frame[PROTOCOL_FRAME_MAX];
    uint8_t payload[PROTOCOL_PAYLOAD_MAX];
    uint32_t len;

    payload[0] = 0x2C;
    payload[1] = 0x01;
    payload[2] = 0xF4;
    payload[3] = 0x01;
    len = protocol_encode(PROTOCOL_CMD_SET_VELOCITY, 1u, payload, 4u,
                          frame, sizeof(frame));
    CHECK(len == sizeof(k_set_velocity), "SET_VELOCITY 帧长应为 11");
    CHECK(memcmp(frame, k_set_velocity, sizeof(k_set_velocity)) == 0,
          "SET_VELOCITY 编码应与协议 §10.2 一致");

    payload[0] = 0x01;
    len = protocol_encode(PROTOCOL_CMD_SET_ENABLE, 2u, payload, 1u,
                          frame, sizeof(frame));
    CHECK(len == sizeof(k_set_enable), "SET_ENABLE 帧长应为 8");
    CHECK(memcmp(frame, k_set_enable, sizeof(k_set_enable)) == 0,
          "SET_ENABLE 编码应与协议 §10.3 一致");

    len = protocol_encode(PROTOCOL_CMD_ESTOP, 3u, NULL, 0u,
                          frame, sizeof(frame));
    CHECK(len == sizeof(k_estop), "ESTOP 帧长应为 7");
    CHECK(memcmp(frame, k_estop, sizeof(k_estop)) == 0,
          "ESTOP 编码应与协议 §10.4 一致");
}

static void test_encode_rejects_bad_arguments(void)
{
    uint8_t frame[PROTOCOL_FRAME_MAX];
    uint8_t payload[8] = { 0 };

    CHECK(protocol_encode(PROTOCOL_CMD_ESTOP, 1u, NULL, 0u,
                          frame, 6u) == 0u,
          "输出缓冲不足时应失败");
    CHECK(protocol_encode(PROTOCOL_CMD_ESTOP, 1u, NULL, 0u,
                          NULL, 10u) == 0u,
          "out == NULL 时应失败");
    CHECK(protocol_encode(PROTOCOL_CMD_SET_PARAM, 1u, NULL, 5u,
                          frame, sizeof(frame)) == 0u,
          "payload == NULL 且长度非零时应失败");
    CHECK(protocol_encode(PROTOCOL_CMD_SET_PARAM, 1u, payload,
                          (uint8_t)65u, frame, sizeof(frame)) == 0u,
          "payload 超过 64 字节时应失败");
}

static void test_decode_single_frames(void)
{
    protocol_parser_t parser;
    protocol_frame_t frame;
    const uint8_t *vectors[] = { k_set_velocity, k_set_enable, k_estop };
    const uint32_t sizes[] =
    {
        (uint32_t)sizeof(k_set_velocity),
        (uint32_t)sizeof(k_set_enable),
        (uint32_t)sizeof(k_estop)
    };
    uint32_t i;

    for (i = 0u; i < 3u; i++)
    {
        uint32_t j;

        protocol_parser_init(&parser);
        for (j = 0u; j < sizes[i]; j++)
        {
            CHECK(protocol_parser_feed(&parser, &vectors[i][j], 1u,
                                       1000u) == 1u,
                  "逐字节喂入应接受");
            if (j + 1u < sizes[i])
            {
                CHECK(protocol_parser_next(&parser, &frame) == PLATFORM_FALSE,
                      "半帧不应产出");
            }
        }

        CHECK(protocol_parser_next(&parser, &frame) == PLATFORM_TRUE,
              "完整帧应产出");
        CHECK(parser.len == 0u, "产出一帧后缓冲区应为空");
    }

    CHECK(frame.cmd == PROTOCOL_CMD_ESTOP, "最后一帧命令字应为 ESTOP");
    CHECK(frame.seq == 3u, "ESTOP 的 SEQ 应为 3");
    CHECK(frame.payload_len == 0u, "ESTOP 无负载");
}

static void test_decode_two_frames_in_one_feed(void)
{
    protocol_parser_t parser;
    protocol_frame_t frames[4];
    uint8_t stream[sizeof(k_set_enable) + sizeof(k_estop)];
    uint32_t count;

    (void)memcpy(stream, k_set_enable, sizeof(k_set_enable));
    (void)memcpy(&stream[sizeof(k_set_enable)], k_estop, sizeof(k_estop));

    protocol_parser_init(&parser);
    count = feed_and_collect(&parser, stream, sizeof(stream), 10u,
                             frames, 4u);

    CHECK(count == 2u, "一次喂入两帧应解出两帧");
    CHECK(frames[0].cmd == PROTOCOL_CMD_SET_ENABLE, "第一帧应为 SET_ENABLE");
    CHECK(frames[1].cmd == PROTOCOL_CMD_ESTOP, "第二帧应为 ESTOP");
    CHECK(parser.stats.frames_ok == 2u, "成功帧计数应为 2");
}

static void test_decode_chunked_and_noisy(void)
{
    protocol_parser_t parser;
    protocol_frame_t frames[4];
    uint8_t stream[32];
    uint32_t pos = 0u;
    uint32_t count;

    /* Leading noise (including a lone 0xAA) must be skipped. */
    stream[pos++] = 0x00;
    stream[pos++] = 0xAA;
    stream[pos++] = 0x13;
    stream[pos++] = 0xAA;
    stream[pos++] = 0x55;
    (void)memcpy(&stream[pos], k_set_velocity, sizeof(k_set_velocity));
    pos += (uint32_t)sizeof(k_set_velocity);
    stream[pos++] = 0xFF;
    (void)memcpy(&stream[pos], k_estop, sizeof(k_estop));
    pos += (uint32_t)sizeof(k_estop);

    protocol_parser_init(&parser);
    count = feed_and_collect(&parser, stream, pos, 42u, frames, 4u);

    CHECK(count == 2u, "带噪声的流应仍解出两帧");
    CHECK(frames[0].cmd == PROTOCOL_CMD_SET_VELOCITY, "噪声后应为 SET_VELOCITY");
    CHECK(frames[1].cmd == PROTOCOL_CMD_ESTOP, "第二帧应为 ESTOP");
    CHECK(parser.stats.resyncs > 0u, "应记录重同步字节");
}

static void test_decode_crc_error_resyncs(void)
{
    protocol_parser_t parser;
    protocol_frame_t frames[4];
    uint8_t stream[32];
    uint32_t pos = 0u;
    uint32_t count;

    (void)memcpy(&stream[pos], k_set_velocity, sizeof(k_set_velocity));
    stream[pos + 9u] ^= 0x01u;                 /* Corrupt CRC low byte. */
    pos += (uint32_t)sizeof(k_set_velocity);

    (void)memcpy(&stream[pos], k_estop, sizeof(k_estop));
    pos += (uint32_t)sizeof(k_estop);

    protocol_parser_init(&parser);
    count = feed_and_collect(&parser, stream, pos, 50u, frames, 4u);

    CHECK(count == 1u, "坏 CRC 帧应被丢弃，后面的好帧应保留");
    CHECK(frames[0].cmd == PROTOCOL_CMD_ESTOP, "保留的应是 ESTOP");
    CHECK(parser.stats.crc_errors == 1u, "CRC 错误计数应为 1");
}

static void test_decode_false_header_inside_payload(void)
{
    protocol_parser_t parser;
    protocol_frame_t frames[4];
    uint8_t stream[24];
    uint32_t pos = 0u;
    uint32_t count;
    uint8_t payload[3];

    /* Payload itself contains AA 55 02; it must not be taken as a header. */
    payload[0] = 0xAA;
    payload[1] = 0x55;
    payload[2] = 0x02;

    pos = protocol_encode(PROTOCOL_CMD_SET_PARAM, 7u, payload, 3u,
                          stream, sizeof(stream));
    CHECK(pos > 0u, "构造测试帧应成功");

    (void)memcpy(&stream[pos], k_estop, sizeof(k_estop));
    pos += (uint32_t)sizeof(k_estop);

    protocol_parser_init(&parser);
    count = feed_and_collect(&parser, stream, pos, 60u, frames, 4u);

    CHECK(count == 2u, "负载里的假帧头不应吞掉真帧");
    CHECK(frames[0].cmd == PROTOCOL_CMD_SET_PARAM, "第一帧应为 SET_PARAM");
    CHECK(frames[1].cmd == PROTOCOL_CMD_ESTOP, "第二帧应为 ESTOP");
}

static void test_decode_bad_len(void)
{
    protocol_parser_t parser;
    protocol_frame_t frames[2];
    uint8_t stream[16];
    uint32_t pos = 0u;
    uint32_t count;

    /* LEN = 1 is below the minimum; LEN = 67 is above the maximum. */
    stream[pos++] = 0xAA;
    stream[pos++] = 0x55;
    stream[pos++] = 0x01;
    stream[pos++] = 0x00;
    stream[pos++] = 0x00;
    stream[pos++] = 0x00;

    (void)memcpy(&stream[pos], k_estop, sizeof(k_estop));
    pos += (uint32_t)sizeof(k_estop);

    protocol_parser_init(&parser);
    count = feed_and_collect(&parser, stream, pos, 70u, frames, 2u);

    CHECK(count == 1u, "非法 LEN 后应能恢复并解出 ESTOP");
    CHECK(parser.stats.len_errors > 0u, "应记录 LEN 错误");
}

static void test_decode_gap_reset(void)
{
    protocol_parser_t parser;
    protocol_frame_t frame;
    uint32_t i;

    protocol_parser_init(&parser);

    /* Half of a frame at t = 0. */
    for (i = 0u; i < 5u; i++)
    {
        (void)protocol_parser_feed(&parser, &k_set_velocity[i], 1u, 1000u);
    }
    CHECK(parser.len == 5u, "半帧应留在缓冲区");

    /* First byte after a 50 ms gap resets, then the full frame arrives. */
    (void)protocol_parser_feed(&parser, k_estop, sizeof(k_estop), 1100u);
    CHECK(protocol_parser_next(&parser, &frame) == PLATFORM_TRUE,
          "超时复位后应解出完整帧");
    CHECK(frame.cmd == PROTOCOL_CMD_ESTOP, "复位后解出的应是 ESTOP");

    /* tick() must do the same without new data. */
    (void)protocol_parser_feed(&parser, k_estop, 3u, 2000u);
    CHECK(parser.len == 3u, "半帧应留在缓冲区");
    protocol_parser_tick(&parser, 2100u);
    CHECK(parser.len == 0u, "50 ms 无新字节应清空半帧");
}

static void test_decode_max_payload(void)
{
    protocol_parser_t parser;
    protocol_frame_t frame;
    uint8_t payload[PROTOCOL_PAYLOAD_MAX];
    uint8_t stream[PROTOCOL_FRAME_MAX];
    uint32_t i;
    uint32_t len;

    for (i = 0u; i < PROTOCOL_PAYLOAD_MAX; i++)
    {
        payload[i] = (uint8_t)i;
    }

    len = protocol_encode(PROTOCOL_CMD_SET_PARAM, 9u, payload,
                          PROTOCOL_PAYLOAD_MAX, stream, sizeof(stream));
    CHECK(len == PROTOCOL_FRAME_MAX, "64 字节负载的帧长应为 71");

    protocol_parser_init(&parser);
    CHECK(feed_and_collect(&parser, stream, len, 3000u, &frame, 1u) == 1u,
          "最大负载帧应解出");
    CHECK(frame.payload_len == PROTOCOL_PAYLOAD_MAX, "负载长度应为 64");
    CHECK(memcmp(frame.payload, payload, PROTOCOL_PAYLOAD_MAX) == 0,
          "负载内容应完整");
}

static void test_field_helpers(void)
{
    uint8_t buf[4];
    int16_t i16 = (int16_t)-12345;
    int32_t i32 = (int32_t)-123456789;

    protocol_put_i16(buf, i16);
    CHECK(protocol_get_i16(buf) == i16, "int16 小端往返应一致");
    CHECK(buf[0] == 0xC7 && buf[1] == 0xCF, "int16 应按小端存放");

    protocol_put_i32(buf, i32);
    CHECK(protocol_get_i32(buf) == i32, "int32 小端往返应一致");
    CHECK(buf[0] == 0xEB && buf[1] == 0x32 && buf[2] == 0xA4 && buf[3] == 0xF8,
          "int32 应按小端存放");

    protocol_put_u16(buf, 0xBEEFu);
    CHECK(protocol_get_u16(buf) == 0xBEEFu, "uint16 往返应一致");
}

static void test_command_table(void)
{
    const protocol_cmd_info_t *info;

    info = protocol_cmd_lookup(PROTOCOL_CMD_SET_VELOCITY);
    CHECK(info != NULL, "SET_VELOCITY 应在命令表中");
    CHECK(info->payload_len == 4u, "SET_VELOCITY 负载应为 4 字节");
    CHECK(info->requires_ack == PLATFORM_FALSE, "SET_VELOCITY 不需要 ACK");

    info = protocol_cmd_lookup(PROTOCOL_CMD_ESTOP);
    CHECK(info != NULL && info->requires_ack == PLATFORM_TRUE,
          "ESTOP 需要 ACK");

    CHECK(protocol_cmd_lookup(PROTOCOL_CMD_ESTOP | PROTOCOL_ACK_FLAG) == NULL,
          "ACK 字不应被当作下行命令");
    CHECK(protocol_cmd_lookup(PROTOCOL_REPORT_ODOM) == NULL,
          "上报字不应被当作下行命令");

    CHECK(protocol_cmd_payload_ok(PROTOCOL_CMD_SET_VELOCITY, 4u) ==
          PLATFORM_TRUE, "SET_VELOCITY 长度 4 应合法");
    CHECK(protocol_cmd_payload_ok(PROTOCOL_CMD_SET_VELOCITY, 5u) ==
          PLATFORM_FALSE, "SET_VELOCITY 长度 5 应非法");
    CHECK(protocol_cmd_payload_ok(PROTOCOL_CMD_ESTOP, 0u) ==
          PLATFORM_TRUE, "ESTOP 长度 0 应合法");
}

static void test_fault_severity(void)
{
    /* A low battery must not outrank a comm timeout (only one code is sent). */
    CHECK(protocol_fault_severity(PROTOCOL_FAULT_BATTERY_LOW) <
          protocol_fault_severity(PROTOCOL_FAULT_COMM_TIMEOUT),
          "电池低压 WARN 应低于通信超时 FATAL");
    CHECK(protocol_fault_severity(PROTOCOL_FAULT_OVERCURRENT) <
          protocol_fault_severity(PROTOCOL_FAULT_ESTOP),
          "过流 ERROR 应低于急停 FATAL");
    CHECK(protocol_fault_severity(PROTOCOL_FAULT_ESTOP) ==
          protocol_fault_severity(PROTOCOL_FAULT_COMM_TIMEOUT),
          "急停与通信超时应同为 FATAL");
    CHECK(protocol_fault_severity((uint16_t)0x9999u) ==
          PROTOCOL_SEVERITY_ERROR,
          "未知故障码应按 ERROR 处理，不能输给 WARN");
}

int main(void)
{
    test_encode_vectors();
    test_encode_rejects_bad_arguments();
    test_decode_single_frames();
    test_decode_two_frames_in_one_feed();
    test_decode_chunked_and_noisy();
    test_decode_crc_error_resyncs();
    test_decode_false_header_inside_payload();
    test_decode_bad_len();
    test_decode_gap_reset();
    test_decode_max_payload();
    test_field_helpers();
    test_command_table();
    test_fault_severity();

    if (g_failed == 0)
    {
        printf("PASS: protocol codec tests\n");
        return 0;
    }

    printf("%d check(s) failed\n", g_failed);
    return 1;
}
