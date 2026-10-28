/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file protocol_codec.c
 *
 * @par dependencies
 * - protocol_codec.h
 * - platform_crc.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement frame encoding and the reassembly-buffer decoder.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "protocol_codec.h"
#include "platform_crc.h"

#include <string.h>

//******************************** Functions ********************************//

static void protocol_discard_front(protocol_parser_t *parser, uint32_t count)
{
    if (count >= parser->len)
    {
        parser->len = 0u;
        return;
    }

    (void)memmove(parser->buffer,
                  &parser->buffer[count],
                  (size_t)(parser->len - count));
    parser->len -= count;
}

void protocol_parser_init(protocol_parser_t *parser)
{
    if (parser == NULL)
    {
        return;
    }

    parser->len = 0u;
    parser->last_rx_ms = 0u;
    parser->has_timestamp = PLATFORM_FALSE;
    parser->stats.frames_ok = 0u;
    parser->stats.crc_errors = 0u;
    parser->stats.len_errors = 0u;
    parser->stats.resyncs = 0u;
}

void protocol_parser_reset(protocol_parser_t *parser)
{
    if (parser == NULL)
    {
        return;
    }

    parser->len = 0u;
    parser->has_timestamp = PLATFORM_FALSE;
}

void protocol_parser_tick(protocol_parser_t *parser, uint32_t now_ms)
{
    if ((parser == NULL) || (parser->len == 0u) ||
        (parser->has_timestamp == PLATFORM_FALSE))
    {
        return;
    }

    if ((now_ms - parser->last_rx_ms) > PROTOCOL_RX_GAP_RESET_MS)
    {
        protocol_parser_reset(parser);
    }
}

uint32_t protocol_parser_feed(protocol_parser_t *parser,
                              const uint8_t *data,
                              uint32_t len,
                              uint32_t now_ms)
{
    uint32_t space;
    uint32_t take;

    if ((parser == NULL) || (data == NULL))
    {
        return 0u;
    }

    protocol_parser_tick(parser, now_ms);

    parser->last_rx_ms = now_ms;
    parser->has_timestamp = PLATFORM_TRUE;

    space = (uint32_t)PROTOCOL_FRAME_MAX - parser->len;
    take = (len < space) ? len : space;

    if (take > 0u)
    {
        (void)memcpy(&parser->buffer[parser->len], data, (size_t)take);
        parser->len += take;
    }

    return take;
}

platform_bool_t protocol_parser_next(protocol_parser_t *parser,
                                     protocol_frame_t *out)
{
    uint8_t  len_field;
    uint32_t total;
    uint32_t crc_calc;
    uint32_t crc_frame;
    uint32_t i;

    if ((parser == NULL) || (out == NULL))
    {
        return PLATFORM_FALSE;
    }

    for (;;)
    {
        if (parser->len < 1u)
        {
            return PLATFORM_FALSE;
        }

        if (parser->buffer[0] != PROTOCOL_MAGIC_0)
        {
            protocol_discard_front(parser, 1u);
            parser->stats.resyncs++;
            continue;
        }

        if (parser->len < 2u)
        {
            return PLATFORM_FALSE;
        }

        if (parser->buffer[1] != PROTOCOL_MAGIC_1)
        {
            protocol_discard_front(parser, 1u);
            parser->stats.resyncs++;
            continue;
        }

        if (parser->len < 3u)
        {
            return PLATFORM_FALSE;
        }

        len_field = parser->buffer[2];

        /* Validate LEN before trusting it as a buffer index. */
        if ((len_field < PROTOCOL_LEN_MIN) ||
            (len_field > PROTOCOL_LEN_MAX))
        {
            parser->stats.len_errors++;
            protocol_discard_front(parser, 1u);
            parser->stats.resyncs++;
            continue;
        }

        total = (uint32_t)len_field + (uint32_t)PROTOCOL_FRAME_OVERHEAD;
        if (parser->len < total)
        {
            return PLATFORM_FALSE;
        }

        /* CRC covers LEN..payload, i.e. len_field + 1 bytes from offset 2. */
        crc_calc = (uint32_t)platform_crc16_ccitt(&parser->buffer[2],
                                                  (uint32_t)len_field + 1u);
        crc_frame = (uint32_t)parser->buffer[3u + len_field] |
                    ((uint32_t)parser->buffer[4u + len_field] << 8);

        if (crc_calc != crc_frame)
        {
            parser->stats.crc_errors++;
            protocol_discard_front(parser, 1u);
            parser->stats.resyncs++;
            continue;
        }

        out->cmd = parser->buffer[3];
        out->seq = parser->buffer[4];
        out->payload_len = (uint8_t)(len_field - 2u);

        for (i = 0u; i < (uint32_t)out->payload_len; i++)
        {
            out->payload[i] = parser->buffer[5u + i];
        }

        protocol_discard_front(parser, total);
        parser->stats.frames_ok++;

        return PLATFORM_TRUE;
    }
}

uint32_t protocol_encode(uint8_t cmd,
                         uint8_t seq,
                         const uint8_t *payload,
                         uint8_t payload_len,
                         uint8_t *out,
                         uint32_t out_size)
{
    uint32_t len_field;
    uint32_t total;
    uint32_t crc;

    if (out == NULL)
    {
        return 0u;
    }

    if (payload_len > PROTOCOL_PAYLOAD_MAX)
    {
        return 0u;
    }

    if ((payload == NULL) && (payload_len > 0u))
    {
        return 0u;
    }

    len_field = (uint32_t)payload_len + 2u;
    total = len_field + (uint32_t)PROTOCOL_FRAME_OVERHEAD;

    if (out_size < total)
    {
        return 0u;
    }

    out[0] = PROTOCOL_MAGIC_0;
    out[1] = PROTOCOL_MAGIC_1;
    out[2] = (uint8_t)len_field;
    out[3] = cmd;
    out[4] = seq;

    if (payload_len > 0u)
    {
        (void)memcpy(&out[5], payload, (size_t)payload_len);
    }

    /* CRC covers LEN..payload, as specified in docs/protocol.md section 6. */
    crc = (uint32_t)platform_crc16_ccitt(&out[2], len_field + 1u);
    out[3u + len_field] = (uint8_t)(crc & 0xFFu);
    out[4u + len_field] = (uint8_t)((crc >> 8) & 0xFFu);

    return total;
}

int16_t protocol_get_i16(const uint8_t *src)
{
    return (int16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8));
}

uint16_t protocol_get_u16(const uint8_t *src)
{
    return (uint16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8));
}

int32_t protocol_get_i32(const uint8_t *src)
{
    return (int32_t)((uint32_t)src[0] |
                     ((uint32_t)src[1] << 8) |
                     ((uint32_t)src[2] << 16) |
                     ((uint32_t)src[3] << 24));
}

void protocol_put_i16(uint8_t *dst, int16_t value)
{
    uint16_t raw = (uint16_t)value;

    dst[0] = (uint8_t)(raw & 0xFFu);
    dst[1] = (uint8_t)((raw >> 8) & 0xFFu);
}

void protocol_put_u16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)((value >> 8) & 0xFFu);
}

void protocol_put_i32(uint8_t *dst, int32_t value)
{
    uint32_t raw = (uint32_t)value;

    dst[0] = (uint8_t)(raw & 0xFFu);
    dst[1] = (uint8_t)((raw >> 8) & 0xFFu);
    dst[2] = (uint8_t)((raw >> 16) & 0xFFu);
    dst[3] = (uint8_t)((raw >> 24) & 0xFFu);
}
