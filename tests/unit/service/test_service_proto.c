/* L1 unit test for 02_Service/service_proto. Runs on the PC, no hardware.
 *
 * The service owns the wire behavior above the codec (dispatch, ACK, SEQ
 * de-duplication, report serialization). The UART is replaced by an in-memory
 * pipe so the test can push frames and inspect exactly what was transmitted.
 *
 * Build (MinGW or any host gcc):
 *   gcc -std=c99 -Wall -Wextra \
 *       -I firmware/app/03_Platform/platform_common \
 *       -I firmware/app/04_Impl \
 *       -I firmware/app/03_Platform/platform_middleware \
 *       -I firmware/app/03_Platform/platform_mcu \
 *       -I firmware/app/05_Middlewares/protocol \
 *       -I firmware/app/02_Service/service_proto \
 *       tests/unit/service/test_service_proto.c \
 *       firmware/app/02_Service/service_proto/service_proto.c \
 *       firmware/app/05_Middlewares/protocol/protocol_codec.c \
 *       firmware/app/05_Middlewares/protocol/protocol_def.c \
 *       firmware/app/03_Platform/platform_middleware/platform_crc.c \
 *       -o build/test_service_proto
 */

#include <stdio.h>
#include <string.h>

#include "service_proto.h"

static int g_failed = 0;

#define CHECK(cond, msg)                                                  \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);        \
            g_failed++;                                                   \
        }                                                                 \
    } while (0)

/* ------------------------------- UART pipe ----------------------------- */

static uint8_t g_rx[512];
static uint32_t g_rx_len;
static uint32_t g_rx_pos;

static uint8_t g_tx[512];
static uint32_t g_tx_len;

platform_err_t platform_uart_flush_rx(platform_uart_bus_t bus)
{
    (void)bus;
    g_rx_len = 0u;
    g_rx_pos = 0u;
    return PLATFORM_ERR_OK;
}

platform_err_t platform_uart_rx_read(platform_uart_bus_t bus,
                                     uint8_t *data,
                                     uint32_t max_len,
                                     uint32_t *out_len)
{
    uint32_t avail = g_rx_len - g_rx_pos;
    uint32_t take = (avail < max_len) ? avail : max_len;

    (void)bus;

    memcpy(data, &g_rx[g_rx_pos], take);
    g_rx_pos += take;
    *out_len = take;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_uart_write(platform_uart_bus_t bus,
                                   const uint8_t *data,
                                   uint32_t len,
                                   uint32_t timeout_ms)
{
    (void)bus;
    (void)timeout_ms;

    if ((g_tx_len + len) > sizeof(g_tx))
    {
        return PLATFORM_ERR_NO_MEMORY;
    }

    memcpy(&g_tx[g_tx_len], data, len);
    g_tx_len += len;

    return PLATFORM_ERR_OK;
}

static void push(const uint8_t *data, uint32_t len)
{
    memcpy(&g_rx[g_rx_len], data, len);
    g_rx_len += len;
}

static void reset_io(void)
{
    g_rx_len = 0u;
    g_rx_pos = 0u;
    g_tx_len = 0u;
}

/* Parse the first captured TX frame. Returns FALSE if there is none. */
static platform_bool_t first_tx(protocol_frame_t *frame)
{
    protocol_parser_t parser;

    if (g_tx_len == 0u)
    {
        return PLATFORM_FALSE;
    }

    protocol_parser_init(&parser);

    if (protocol_parser_feed(&parser, g_tx, g_tx_len, 0u) == 0u)
    {
        return PLATFORM_FALSE;
    }

    return protocol_parser_next(&parser, frame);
}

/* --------------------------- handler recording ------------------------- */

static int      g_enable_calls;
static uint8_t  g_enable_value;
static uint8_t  g_enable_seq;
static protocol_result_t g_enable_result;

static protocol_result_t on_set_enable(const protocol_frame_t *frame,
                                       void *user_data)
{
    (void)user_data;

    g_enable_calls++;
    g_enable_value = frame->payload[0];
    g_enable_seq = frame->seq;

    return g_enable_result;
}

static void reset_handlers(void)
{
    g_enable_calls = 0;
    g_enable_value = 0xFFu;
    g_enable_seq = 0xFFu;
    g_enable_result = PROTOCOL_RESULT_OK;
}

static void init_service(void)
{
    service_proto_cfg_t cfg;

    cfg.bus = PLATFORM_UART_BUS_0;

    reset_io();
    reset_handlers();

    CHECK(service_proto_init(&cfg) == PLATFORM_ERR_OK, "init");
    CHECK(service_proto_register(PROTOCOL_CMD_SET_ENABLE, on_set_enable,
                                 NULL) == PLATFORM_ERR_OK, "register");
}

/* Encode one downstream frame into g_rx. */
static void push_cmd(uint8_t cmd, uint8_t seq,
                     const uint8_t *payload, uint8_t payload_len)
{
    uint8_t frame[PROTOCOL_FRAME_MAX];
    uint32_t len;

    len = protocol_encode(cmd, seq, payload, payload_len, frame,
                          (uint32_t)sizeof(frame));

    if (len == 0u)
    {
        printf("FAIL: protocol_encode(%02X) -> 0 (%s:%d)\n", cmd, __FILE__,
               __LINE__);
        g_failed++;
        return;
    }

    push(frame, len);
}

/* -------------------------------- tests -------------------------------- */

static void test_dispatch_and_ack(void)
{
    uint8_t payload = 1u;
    protocol_frame_t frame;

    init_service();

    push_cmd(PROTOCOL_CMD_SET_ENABLE, 7u, &payload, 1u);
    CHECK(service_proto_process(100u) == PLATFORM_ERR_OK, "process");
    CHECK(g_enable_calls == 1, "handler called once");
    CHECK(g_enable_value == 1u, "payload passed through");
    CHECK(g_enable_seq == 7u, "seq passed through");

    CHECK(first_tx(&frame) == PLATFORM_TRUE, "ack sent");
    CHECK(frame.cmd == (PROTOCOL_CMD_SET_ENABLE | PROTOCOL_ACK_FLAG),
          "ack command byte");
    CHECK(frame.payload_len == 2u, "ack payload length");
    CHECK(frame.payload[0] == 7u, "acked seq");
    CHECK(frame.payload[1] == PROTOCOL_RESULT_OK, "ack result");
    CHECK(service_proto_has_received() == PLATFORM_TRUE, "rx latched");
}

static void test_dedupe(void)
{
    uint8_t payload = 1u;
    protocol_frame_t frame;
    protocol_parser_t parser;

    init_service();

    push_cmd(PROTOCOL_CMD_SET_ENABLE, 9u, &payload, 1u);
    (void)service_proto_process(100u);

    /* Same SEQ within the window: a retransmission, not a new command. */
    push_cmd(PROTOCOL_CMD_SET_ENABLE, 9u, &payload, 1u);
    (void)service_proto_process(200u);
    CHECK(g_enable_calls == 1, "retransmission not executed twice");

    protocol_parser_init(&parser);
    (void)protocol_parser_feed(&parser, g_tx, g_tx_len, 0u);
    CHECK(protocol_parser_next(&parser, &frame) == PLATFORM_TRUE, "ack 1");
    CHECK(protocol_parser_next(&parser, &frame) == PLATFORM_TRUE, "ack 2");
    CHECK(frame.payload[0] == 9u, "second ack carries the same seq");
}

static void test_result_mapping(void)
{
    uint8_t payload = 1u;
    protocol_frame_t frame;

    init_service();

    g_enable_result = PROTOCOL_RESULT_RANGE;
    push_cmd(PROTOCOL_CMD_SET_ENABLE, 3u, &payload, 1u);
    (void)service_proto_process(100u);

    CHECK(first_tx(&frame) == PLATFORM_TRUE, "ack sent");
    CHECK(frame.payload[1] == PROTOCOL_RESULT_RANGE, "handler result in ack");
}

static void test_unknown_command(void)
{
    uint8_t payload[2] = { 0u, 0u };
    protocol_frame_t frame;

    init_service();

    /* 0x3A is inside the downstream range but not in the table. */
    push_cmd(0x3Au, 1u, payload, 2u);
    (void)service_proto_process(100u);

    CHECK(first_tx(&frame) == PLATFORM_TRUE, "ack sent");
    CHECK(frame.cmd == (0x3Au | PROTOCOL_ACK_FLAG), "unknown ack cmd");
    CHECK(frame.payload[1] == PROTOCOL_RESULT_UNKNOWN_CMD, "unknown result");
}

static void test_bad_payload_length(void)
{
    uint8_t payload[2] = { 1u, 0u };
    protocol_frame_t frame;

    init_service();

    /* SET_ENABLE takes exactly one byte. */
    push_cmd(PROTOCOL_CMD_SET_ENABLE, 4u, payload, 2u);
    (void)service_proto_process(100u);

    CHECK(g_enable_calls == 0, "handler not called");
    CHECK(first_tx(&frame) == PLATFORM_TRUE, "ack sent");
    CHECK(frame.payload[1] == PROTOCOL_RESULT_RANGE, "range result");
}

static void test_no_ack_for_velocity(void)
{
    uint8_t payload[4] = { 0u, 0u, 0u, 0u };

    init_service();

    /* No handler registered, but SET_VELOCITY does not require an ACK. */
    push_cmd(PROTOCOL_CMD_SET_VELOCITY, 5u, payload, 4u);
    (void)service_proto_process(100u);

    CHECK(g_tx_len == 0u, "silent for unhandled velocity");
}

static void test_crc_error_is_dropped(void)
{
    uint8_t frame[PROTOCOL_FRAME_MAX];
    uint32_t len;

    init_service();

    len = protocol_encode(PROTOCOL_CMD_SET_ENABLE, 1u, (const uint8_t *)"\x01",
                          1u, frame, (uint32_t)sizeof(frame));
    frame[len - 1u] ^= 0xFFu;
    push(frame, len);

    (void)service_proto_process(100u);

    CHECK(g_enable_calls == 0, "bad crc not dispatched");
    CHECK(g_tx_len == 0u, "no ack for a corrupt frame");
}

static void test_reports(void)
{
    int16_t angles[PROTOCOL_ARM_JOINT_COUNT] = { 0, 0, 0, 0, 0, 0 };
    protocol_frame_t frame;
    protocol_parser_t parser;

    init_service();
    reset_io();

    CHECK(service_proto_send_odom(100, -200, 9000, 10, -10) ==
          PLATFORM_ERR_OK, "odom");
    CHECK(service_proto_send_heartbeat(1u, 42u) == PLATFORM_ERR_OK, "hb");

    protocol_parser_init(&parser);
    (void)protocol_parser_feed(&parser, g_tx, g_tx_len, 0u);

    CHECK(protocol_parser_next(&parser, &frame) == PLATFORM_TRUE, "odom frame");
    CHECK(frame.cmd == PROTOCOL_REPORT_ODOM, "odom cmd");
    CHECK(frame.payload_len == PROTOCOL_LEN_REPORT_ODOM, "odom length");
    CHECK(protocol_get_i32(&frame.payload[0]) == 100, "odom x");
    CHECK(protocol_get_i32(&frame.payload[4]) == -200, "odom y");
    CHECK(protocol_get_i16(&frame.payload[8]) == 9000, "odom heading");

    CHECK(protocol_parser_next(&parser, &frame) == PLATFORM_TRUE, "hb frame");
    CHECK(frame.cmd == PROTOCOL_REPORT_HEARTBEAT, "hb cmd");
    CHECK(frame.payload[0] == 1u, "hb state");
    CHECK(protocol_get_u16(&frame.payload[1]) == 42u, "hb counter");

    reset_io();
    CHECK(service_proto_send_arm(angles, 0, 0u) == PLATFORM_ERR_OK, "arm");
    protocol_parser_init(&parser);
    (void)protocol_parser_feed(&parser, g_tx, g_tx_len, 0u);
    CHECK(protocol_parser_next(&parser, &frame) == PLATFORM_TRUE, "arm frame");
    CHECK(frame.payload_len == PROTOCOL_LEN_REPORT_ARM, "arm length");
}

int main(void)
{
    test_dispatch_and_ack();
    test_dedupe();
    test_result_mapping();
    test_unknown_command();
    test_bad_payload_length();
    test_no_ack_for_velocity();
    test_crc_error_is_dropped();
    test_reports();

    if (g_failed != 0)
    {
        printf("FAIL: service_proto tests (%d failure(s))\n", g_failed);
        return 1;
    }

    printf("PASS: service_proto tests\n");
    return 0;
}
