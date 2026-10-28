/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file service_proto.c
 *
 * @par dependencies
 * - service_proto.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement protocol dispatch, ACKs and report serialization.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "service_proto.h"

//******************************** Declaring ********************************//

typedef struct
{
    uint8_t                 cmd;
    service_proto_handler_t handler;
    void *                  user_data;

    /* SEQ de-duplication state, only used when the command requires an ACK. */
    platform_bool_t         has_last;
    uint8_t                 last_seq;
    uint8_t                 last_result;
    uint32_t                last_ms;
} service_proto_entry_t;

//******************************** Variables ********************************//

static struct
{
    platform_bool_t         initialized;
    platform_uart_bus_t     bus;
    protocol_parser_t       parser;
    service_proto_entry_t   handlers[SERVICE_PROTO_MAX_HANDLERS];

    uint8_t                 tx_seq;
    uint32_t                last_rx_ms;
    platform_bool_t         has_rx;

    uint32_t                unknown_cmds;
    uint32_t                acks_sent;
    uint32_t                tx_failures;

    uint32_t                last_crc_errors;
    uint32_t                consecutive_crc;
    platform_bool_t         crc_alarm;
} s_proto;

//******************************** Functions ********************************//

static service_proto_entry_t *service_proto_find(uint8_t cmd)
{
    uint8_t i;

    for (i = 0u; i < SERVICE_PROTO_MAX_HANDLERS; i++)
    {
        if ((s_proto.handlers[i].cmd == cmd) &&
            (s_proto.handlers[i].handler != NULL))
        {
            return &s_proto.handlers[i];
        }
    }

    return NULL;
}

static platform_err_t service_proto_send_frame(uint8_t cmd,
                                               const uint8_t *payload,
                                               uint8_t payload_len)
{
    uint8_t        frame[PROTOCOL_FRAME_MAX];
    uint32_t       len;
    platform_err_t err;

    len = protocol_encode(cmd, s_proto.tx_seq, payload, payload_len,
                          frame, (uint32_t)sizeof(frame));
    if (len == 0u)
    {
        return PLATFORM_ERR_PARAM;
    }

    err = platform_uart_write(s_proto.bus, frame, len,
                              SERVICE_PROTO_TX_TIMEOUT_MS);
    if (PLATFORM_IS_ERR(err))
    {
        s_proto.tx_failures++;
        return err;
    }

    s_proto.tx_seq++;
    return PLATFORM_ERR_OK;
}

static void service_proto_send_ack(uint8_t cmd,
                                   uint8_t acked_seq,
                                   protocol_result_t result)
{
    uint8_t payload[2];

    payload[0] = acked_seq;
    payload[1] = (uint8_t)result;

    if (PLATFORM_IS_OK(service_proto_send_frame(
                            (uint8_t)(cmd | PROTOCOL_ACK_FLAG),
                            payload, 2u)))
    {
        s_proto.acks_sent++;
    }
}

static void service_proto_dispatch(const protocol_frame_t *frame,
                                   uint32_t now_ms)
{
    const protocol_cmd_info_t *info;
    service_proto_entry_t     *entry;
    protocol_result_t          result;

    /*
     * Only downstream commands are handled. An ACK or report echoed by the
     * host (0x80..0xFF) is not an error worth answering, or two devices
     * could ping-pong ACKs forever.
     */
    if ((frame->cmd == 0u) || (frame->cmd > PROTOCOL_CMD_MAX))
    {
        return;
    }

    s_proto.has_rx = PLATFORM_TRUE;
    s_proto.last_rx_ms = now_ms;

    info = protocol_cmd_lookup(frame->cmd);

    if (info == NULL)
    {
        s_proto.unknown_cmds++;
        service_proto_send_ack(frame->cmd, frame->seq,
                               PROTOCOL_RESULT_UNKNOWN_CMD);
        return;
    }

    if (protocol_cmd_payload_ok(frame->cmd, frame->payload_len) ==
        PLATFORM_FALSE)
    {
        if (info->requires_ack == PLATFORM_TRUE)
        {
            service_proto_send_ack(frame->cmd, frame->seq,
                                   PROTOCOL_RESULT_RANGE);
        }
        return;
    }

    entry = service_proto_find(frame->cmd);

    if (entry == NULL)
    {
        /* Known command, but this application does not implement it. */
        if (info->requires_ack == PLATFORM_TRUE)
        {
            service_proto_send_ack(frame->cmd, frame->seq,
                                   PROTOCOL_RESULT_FAILED);
        }
        return;
    }

    /*
     * A retried state-changing command must not execute twice (SET_PARAM
     * would erase/write Flash again). Answer with the cached result instead.
     * The window also handles a SEQ wrap landing on the same value only after
     * 256 frames, which cannot happen inside one second at any legal rate.
     */
    if ((info->requires_ack == PLATFORM_TRUE) &&
        (entry->has_last == PLATFORM_TRUE) &&
        (entry->last_seq == frame->seq) &&
        ((now_ms - entry->last_ms) <= SERVICE_PROTO_DEDUPE_MS))
    {
        service_proto_send_ack(frame->cmd, frame->seq,
                               (protocol_result_t)entry->last_result);
        return;
    }

    result = entry->handler(frame, entry->user_data);

    if (info->requires_ack == PLATFORM_TRUE)
    {
        service_proto_send_ack(frame->cmd, frame->seq, result);

        entry->has_last = PLATFORM_TRUE;
        entry->last_seq = frame->seq;
        entry->last_result = (uint8_t)result;
        entry->last_ms = now_ms;
    }
}

platform_err_t service_proto_init(const service_proto_cfg_t *cfg)
{
    uint32_t i;

    if (cfg == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    protocol_parser_init(&s_proto.parser);

    for (i = 0u; i < SERVICE_PROTO_MAX_HANDLERS; i++)
    {
        s_proto.handlers[i].cmd = 0u;
        s_proto.handlers[i].handler = NULL;
        s_proto.handlers[i].user_data = NULL;
        s_proto.handlers[i].has_last = PLATFORM_FALSE;
        s_proto.handlers[i].last_seq = 0u;
        s_proto.handlers[i].last_result = 0u;
        s_proto.handlers[i].last_ms = 0u;
    }

    s_proto.bus = cfg->bus;
    s_proto.tx_seq = 0u;
    s_proto.last_rx_ms = 0u;
    s_proto.has_rx = PLATFORM_FALSE;
    s_proto.unknown_cmds = 0u;
    s_proto.acks_sent = 0u;
    s_proto.tx_failures = 0u;
    s_proto.last_crc_errors = 0u;
    s_proto.consecutive_crc = 0u;
    s_proto.crc_alarm = PLATFORM_FALSE;
    s_proto.initialized = PLATFORM_TRUE;

    (void)platform_uart_flush_rx(s_proto.bus);

    return PLATFORM_ERR_OK;
}

platform_err_t service_proto_register(uint8_t cmd,
                                      service_proto_handler_t handler,
                                      void *user_data)
{
    uint32_t i;

    if (s_proto.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if ((handler == NULL) || (protocol_cmd_lookup(cmd) == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (service_proto_find(cmd) != NULL)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    for (i = 0u; i < SERVICE_PROTO_MAX_HANDLERS; i++)
    {
        if (s_proto.handlers[i].handler == NULL)
        {
            s_proto.handlers[i].cmd = cmd;
            s_proto.handlers[i].handler = handler;
            s_proto.handlers[i].user_data = user_data;
            s_proto.handlers[i].has_last = PLATFORM_FALSE;
            return PLATFORM_ERR_OK;
        }
    }

    return PLATFORM_ERR_NO_RESOURCE;
}

platform_err_t service_proto_process(uint32_t now_ms)
{
    uint8_t         chunk[SERVICE_PROTO_RX_CHUNK];
    protocol_frame_t frame;
    uint32_t        got;
    uint32_t        offset;
    uint32_t        fed;
    uint32_t        dispatched = 0u;
    uint32_t        crc_now;
    platform_bool_t drained;
    platform_err_t  err;

    if (s_proto.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    for (;;)
    {
        err = platform_uart_rx_read(s_proto.bus, chunk,
                                    (uint32_t)sizeof(chunk), &got);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }

        if (got == 0u)
        {
            break;
        }

        offset = 0u;

        for (;;)
        {
            fed = protocol_parser_feed(&s_proto.parser, &chunk[offset],
                                       got - offset, now_ms);
            offset += fed;

            drained = PLATFORM_FALSE;

            while (protocol_parser_next(&s_proto.parser, &frame) ==
                   PLATFORM_TRUE)
            {
                service_proto_dispatch(&frame, now_ms);
                dispatched++;
                drained = PLATFORM_TRUE;
            }

            if (offset >= got)
            {
                break;
            }

            /* Buffer full and nothing extractable: avoid an endless loop. */
            if ((fed == 0u) && (drained == PLATFORM_FALSE))
            {
                break;
            }
        }
    }

    protocol_parser_tick(&s_proto.parser, now_ms);

    /*
     * "Consecutive" is evaluated per process call: any valid frame in the
     * call clears the streak, otherwise the new CRC errors extend it.
     */
    crc_now = s_proto.parser.stats.crc_errors;

    if (dispatched > 0u)
    {
        s_proto.consecutive_crc = 0u;
        s_proto.crc_alarm = PLATFORM_FALSE;
    }
    else
    {
        s_proto.consecutive_crc += (crc_now - s_proto.last_crc_errors);

        if (s_proto.consecutive_crc >= SERVICE_PROTO_CRC_ALARM_FRAMES)
        {
            s_proto.crc_alarm = PLATFORM_TRUE;
        }
    }

    s_proto.last_crc_errors = crc_now;

    return PLATFORM_ERR_OK;
}

platform_err_t service_proto_send_odom(int32_t x_mm,
                                       int32_t y_mm,
                                       int16_t heading_centi_deg,
                                       int16_t vl_mm_s,
                                       int16_t vr_mm_s)
{
    uint8_t payload[PROTOCOL_LEN_REPORT_ODOM];

    protocol_put_i32(&payload[0], x_mm);
    protocol_put_i32(&payload[4], y_mm);
    protocol_put_i16(&payload[8], heading_centi_deg);
    protocol_put_i16(&payload[10], vl_mm_s);
    protocol_put_i16(&payload[12], vr_mm_s);

    return service_proto_send_frame(PROTOCOL_REPORT_ODOM, payload,
                                    PROTOCOL_LEN_REPORT_ODOM);
}

platform_err_t service_proto_send_arm(
                            const int16_t joint_centi_deg
                                [PROTOCOL_ARM_JOINT_COUNT],
                            int16_t gripper_position,
                            uint8_t gripper_state)
{
    uint8_t payload[PROTOCOL_LEN_REPORT_ARM];
    uint8_t i;

    if (joint_centi_deg == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    for (i = 0u; i < PROTOCOL_ARM_JOINT_COUNT; i++)
    {
        protocol_put_i16(&payload[(uint32_t)i * 2u], joint_centi_deg[i]);
    }

    protocol_put_i16(&payload[12], gripper_position);
    payload[14] = gripper_state;

    return service_proto_send_frame(PROTOCOL_REPORT_ARM, payload,
                                    PROTOCOL_LEN_REPORT_ARM);
}

platform_err_t service_proto_send_status(uint8_t state,
                                         uint16_t fault_code,
                                         uint16_t battery_mv,
                                         int8_t temperature_c,
                                         uint16_t max_jitter_us)
{
    uint8_t payload[PROTOCOL_LEN_REPORT_STATUS];

    payload[0] = state;
    protocol_put_u16(&payload[1], fault_code);
    protocol_put_u16(&payload[3], battery_mv);
    payload[5] = (uint8_t)temperature_c;
    protocol_put_u16(&payload[6], max_jitter_us);

    return service_proto_send_frame(PROTOCOL_REPORT_STATUS, payload,
                                    PROTOCOL_LEN_REPORT_STATUS);
}

platform_err_t service_proto_send_event(uint16_t event_code,
                                        uint32_t timestamp_ms)
{
    uint8_t payload[PROTOCOL_LEN_REPORT_EVENT];

    protocol_put_u16(&payload[0], event_code);
    protocol_put_i32(&payload[2], (int32_t)timestamp_ms);

    return service_proto_send_frame(PROTOCOL_REPORT_EVENT, payload,
                                    PROTOCOL_LEN_REPORT_EVENT);
}

platform_err_t service_proto_send_heartbeat(uint8_t state,
                                            uint16_t run_counter)
{
    uint8_t payload[PROTOCOL_LEN_REPORT_HEARTBEAT];

    payload[0] = state;
    protocol_put_u16(&payload[1], run_counter);

    return service_proto_send_frame(PROTOCOL_REPORT_HEARTBEAT, payload,
                                    PROTOCOL_LEN_REPORT_HEARTBEAT);
}

uint32_t service_proto_rx_age_ms(uint32_t now_ms)
{
    if (s_proto.has_rx == PLATFORM_FALSE)
    {
        return 0xFFFFFFFFu;
    }

    return now_ms - s_proto.last_rx_ms;
}

platform_bool_t service_proto_has_received(void)
{
    return s_proto.has_rx;
}

platform_bool_t service_proto_crc_alarm(void)
{
    return s_proto.crc_alarm;
}

void service_proto_get_stats(service_proto_stats_t *out)
{
    if (out == NULL)
    {
        return;
    }

    out->frames_ok = s_proto.parser.stats.frames_ok;
    out->crc_errors = s_proto.parser.stats.crc_errors;
    out->len_errors = s_proto.parser.stats.len_errors;
    out->resyncs = s_proto.parser.stats.resyncs;
    out->unknown_cmds = s_proto.unknown_cmds;
    out->acks_sent = s_proto.acks_sent;
    out->tx_failures = s_proto.tx_failures;
}
