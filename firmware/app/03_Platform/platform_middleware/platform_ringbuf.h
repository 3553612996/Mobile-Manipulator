/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_ringbuf.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide a byte-oriented single-producer/single-consumer ring buffer.
 *
 * The buffer is lock-free for exactly one producer and one consumer (the
 * common ISR-produces/task-consumes UART case). Multiple producers or multiple
 * consumers must add their own lock.
 *
 * Capacity must be a power of two. The implementation uses a mask instead of
 * a modulo, which keeps ISR cost constant and avoids a division.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_RINGBUF_H__
#define __PLATFORM_RINGBUF_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"

//******************************** Declaring ********************************//

typedef struct
{
    uint8_t *               buffer;         /* Caller-owned storage. */
    uint32_t                size;           /* Power of two. */
    uint32_t                mask;           /* size - 1. */
    volatile uint32_t       write_index;    /* Producer-owned. */
    volatile uint32_t       read_index;     /* Consumer-owned. */
} platform_ringbuf_t;

platform_err_t platform_ringbuf_init(platform_ringbuf_t *ringbuf,
                                     uint8_t *storage,
                                     uint32_t size);

/*
 * All functions below return the number of bytes actually handled. A short
 * return is not an error: it means the buffer was empty/full. This is
 * deliberate, so the caller can decide whether to drop data or retry.
 */
uint32_t platform_ringbuf_write(platform_ringbuf_t *ringbuf,
                                const uint8_t *data,
                                uint32_t len);
uint32_t platform_ringbuf_write_byte(platform_ringbuf_t *ringbuf,
                                     uint8_t byte);
uint32_t platform_ringbuf_read(platform_ringbuf_t *ringbuf,
                               uint8_t *data,
                               uint32_t len);
uint32_t platform_ringbuf_read_byte(platform_ringbuf_t *ringbuf,
                                    uint8_t *byte);
uint32_t platform_ringbuf_peek(const platform_ringbuf_t *ringbuf,
                               uint8_t *data,
                               uint32_t len);
uint32_t platform_ringbuf_drop(platform_ringbuf_t *ringbuf, uint32_t len);

uint32_t platform_ringbuf_count(const platform_ringbuf_t *ringbuf);
uint32_t platform_ringbuf_free(const platform_ringbuf_t *ringbuf);
uint32_t platform_ringbuf_capacity(const platform_ringbuf_t *ringbuf);

platform_bool_t platform_ringbuf_is_empty(const platform_ringbuf_t *ringbuf);
platform_bool_t platform_ringbuf_is_full(const platform_ringbuf_t *ringbuf);

/* Only call when no producer or consumer is active. */
void platform_ringbuf_reset(platform_ringbuf_t *ringbuf);

#endif /* __PLATFORM_RINGBUF_H__ */
