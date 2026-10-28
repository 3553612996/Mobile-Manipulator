/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file protocol_codec.h
 *
 * @par dependencies
 * - protocol_def.h
 * - platform_crc.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Encode and stream-decode protocol v1 frames.
 *
 * The decoder is a reassembly buffer rather than a byte state machine: bytes
 * are appended as they arrive, and protocol_parser_next() extracts one frame
 * at a time. A failed length or CRC drops a single byte and rescans, so a
 * false 0xAA 0x55 inside a payload cannot swallow the next real frame. The
 * buffer is bounded by PROTOCOL_FRAME_MAX, so the cost is constant and the
 * parser needs no heap.
 *
 * Time is injected (now_ms) so the parser is testable on a PC and the caller
 * decides which clock to use.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PROTOCOL_CODEC_H__
#define __PROTOCOL_CODEC_H__

//******************************** Includes *********************************//

#include "protocol_def.h"

//******************************** Declaring ********************************//

typedef struct
{
    uint8_t     cmd;
    uint8_t     seq;
    uint8_t     payload_len;
    uint8_t     payload[PROTOCOL_PAYLOAD_MAX];
} protocol_frame_t;

typedef struct
{
    uint32_t    frames_ok;
    uint32_t    crc_errors;
    uint32_t    len_errors;
    uint32_t    resyncs;        /* Bytes discarded while hunting for a header. */
} protocol_parser_stats_t;

typedef struct
{
    uint8_t                 buffer[PROTOCOL_FRAME_MAX];
    uint32_t                len;            /* Bytes currently buffered. */
    uint32_t                last_rx_ms;
    platform_bool_t         has_timestamp;
    protocol_parser_stats_t stats;
} protocol_parser_t;

void protocol_parser_init(protocol_parser_t *parser);

/* Drop partial data but keep the statistics (used after a link reset). */
void protocol_parser_reset(protocol_parser_t *parser);

/*
 * Append bytes. Returns how many bytes were accepted, which may be less than
 * len when the reassembly buffer is full. The caller must drain frames with
 * protocol_parser_next() and feed the remainder; accepting 0 while len > 0
 * means the buffer is full of an incomplete frame that cannot grow.
 *
 * A gap of more than PROTOCOL_RX_GAP_RESET_MS between feeds resets the
 * partial frame first.
 */
uint32_t protocol_parser_feed(protocol_parser_t *parser,
                              const uint8_t *data,
                              uint32_t len,
                              uint32_t now_ms);

/* Reset the partial frame if it has been idle too long. Call periodically. */
void protocol_parser_tick(protocol_parser_t *parser, uint32_t now_ms);

/*
 * Extract the next complete, CRC-checked frame. Returns PLATFORM_TRUE when
 * out was filled; PLATFORM_FALSE when more bytes are needed.
 */
platform_bool_t protocol_parser_next(protocol_parser_t *parser,
                                     protocol_frame_t *out);

/*
 * Build one frame into out. Returns the total frame length, or 0 when the
 * arguments do not fit. payload may be NULL when payload_len == 0.
 */
uint32_t protocol_encode(uint8_t cmd,
                         uint8_t seq,
                         const uint8_t *payload,
                         uint8_t payload_len,
                         uint8_t *out,
                         uint32_t out_size);

/* Little-endian field helpers (docs/protocol.md 3). */
int16_t  protocol_get_i16(const uint8_t *src);
uint16_t protocol_get_u16(const uint8_t *src);
int32_t  protocol_get_i32(const uint8_t *src);
void     protocol_put_i16(uint8_t *dst, int16_t value);
void     protocol_put_u16(uint8_t *dst, uint16_t value);
void     protocol_put_i32(uint8_t *dst, int32_t value);

#endif /* __PROTOCOL_CODEC_H__ */
