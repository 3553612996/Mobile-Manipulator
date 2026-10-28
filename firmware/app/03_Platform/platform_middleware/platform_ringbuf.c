/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_ringbuf.c
 *
 * @par dependencies
 * - platform_ringbuf.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement the byte-oriented SPSC ring buffer.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "platform_ringbuf.h"

//******************************** Defines **********************************//

/*
 * The data copy and the index update must not be reordered by the compiler.
 * Cortex-M is strongly ordered at the hardware level, so a compiler barrier is
 * sufficient for the single-core SPSC case.
 */
#if defined(__CC_ARM)
    #define PLATFORM_RINGBUF_BARRIER()  __schedule_barrier()
#elif defined(__GNUC__) || defined(__clang__)
    #define PLATFORM_RINGBUF_BARRIER()  __asm volatile ("" ::: "memory")
#elif defined(__ICCARM__)
    #define PLATFORM_RINGBUF_BARRIER()  __iar_EmptyInstruction()
#else
    #define PLATFORM_RINGBUF_BARRIER()  do { } while (0)
#endif

//******************************** Functions ********************************//

platform_err_t platform_ringbuf_init(platform_ringbuf_t *ringbuf,
                                     uint8_t *storage,
                                     uint32_t size)
{
    if ((ringbuf == NULL) || (storage == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    /* Two bytes is the smallest useful buffer; power of two enables masking. */
    if ((size < 2u) || ((size & (size - 1u)) != 0u))
    {
        return PLATFORM_ERR_PARAM;
    }

    ringbuf->buffer = storage;
    ringbuf->size = size;
    ringbuf->mask = size - 1u;
    ringbuf->write_index = 0u;
    ringbuf->read_index = 0u;

    return PLATFORM_ERR_OK;
}

uint32_t platform_ringbuf_write(platform_ringbuf_t *ringbuf,
                                const uint8_t *data,
                                uint32_t len)
{
    uint32_t free_len;
    uint32_t i;

    if ((ringbuf == NULL) || (data == NULL) || (ringbuf->buffer == NULL))
    {
        return 0u;
    }

    free_len = ringbuf->size - (ringbuf->write_index - ringbuf->read_index);
    if (len > free_len)
    {
        len = free_len;
    }

    for (i = 0u; i < len; i++)
    {
        ringbuf->buffer[(ringbuf->write_index + i) & ringbuf->mask] = data[i];
    }

    PLATFORM_RINGBUF_BARRIER();
    ringbuf->write_index += len;

    return len;
}

uint32_t platform_ringbuf_write_byte(platform_ringbuf_t *ringbuf,
                                     uint8_t byte)
{
    if ((ringbuf == NULL) || (ringbuf->buffer == NULL))
    {
        return 0u;
    }

    if ((ringbuf->write_index - ringbuf->read_index) >= ringbuf->size)
    {
        return 0u;
    }

    ringbuf->buffer[ringbuf->write_index & ringbuf->mask] = byte;
    PLATFORM_RINGBUF_BARRIER();
    ringbuf->write_index++;

    return 1u;
}

uint32_t platform_ringbuf_read(platform_ringbuf_t *ringbuf,
                               uint8_t *data,
                               uint32_t len)
{
    uint32_t available;
    uint32_t i;

    if ((ringbuf == NULL) || (data == NULL) || (ringbuf->buffer == NULL))
    {
        return 0u;
    }

    available = ringbuf->write_index - ringbuf->read_index;
    if (len > available)
    {
        len = available;
    }

    for (i = 0u; i < len; i++)
    {
        data[i] = ringbuf->buffer[(ringbuf->read_index + i) & ringbuf->mask];
    }

    PLATFORM_RINGBUF_BARRIER();
    ringbuf->read_index += len;

    return len;
}

uint32_t platform_ringbuf_read_byte(platform_ringbuf_t *ringbuf,
                                    uint8_t *byte)
{
    if ((ringbuf == NULL) || (byte == NULL) || (ringbuf->buffer == NULL))
    {
        return 0u;
    }

    if (ringbuf->write_index == ringbuf->read_index)
    {
        return 0u;
    }

    *byte = ringbuf->buffer[ringbuf->read_index & ringbuf->mask];
    PLATFORM_RINGBUF_BARRIER();
    ringbuf->read_index++;

    return 1u;
}

uint32_t platform_ringbuf_peek(const platform_ringbuf_t *ringbuf,
                               uint8_t *data,
                               uint32_t len)
{
    uint32_t available;
    uint32_t i;

    if ((ringbuf == NULL) || (data == NULL) || (ringbuf->buffer == NULL))
    {
        return 0u;
    }

    available = ringbuf->write_index - ringbuf->read_index;
    if (len > available)
    {
        len = available;
    }

    for (i = 0u; i < len; i++)
    {
        data[i] = ringbuf->buffer[(ringbuf->read_index + i) & ringbuf->mask];
    }

    return len;
}

uint32_t platform_ringbuf_drop(platform_ringbuf_t *ringbuf, uint32_t len)
{
    uint32_t available;

    if ((ringbuf == NULL) || (ringbuf->buffer == NULL))
    {
        return 0u;
    }

    available = ringbuf->write_index - ringbuf->read_index;
    if (len > available)
    {
        len = available;
    }

    PLATFORM_RINGBUF_BARRIER();
    ringbuf->read_index += len;

    return len;
}

uint32_t platform_ringbuf_count(const platform_ringbuf_t *ringbuf)
{
    if ((ringbuf == NULL) || (ringbuf->buffer == NULL))
    {
        return 0u;
    }

    return ringbuf->write_index - ringbuf->read_index;
}

uint32_t platform_ringbuf_free(const platform_ringbuf_t *ringbuf)
{
    if ((ringbuf == NULL) || (ringbuf->buffer == NULL))
    {
        return 0u;
    }

    return ringbuf->size - (ringbuf->write_index - ringbuf->read_index);
}

uint32_t platform_ringbuf_capacity(const platform_ringbuf_t *ringbuf)
{
    if ((ringbuf == NULL) || (ringbuf->buffer == NULL))
    {
        return 0u;
    }

    return ringbuf->size;
}

platform_bool_t platform_ringbuf_is_empty(const platform_ringbuf_t *ringbuf)
{
    return (platform_ringbuf_count(ringbuf) == 0u) ?
           PLATFORM_TRUE : PLATFORM_FALSE;
}

platform_bool_t platform_ringbuf_is_full(const platform_ringbuf_t *ringbuf)
{
    if ((ringbuf == NULL) || (ringbuf->buffer == NULL))
    {
        return PLATFORM_FALSE;
    }

    return (platform_ringbuf_count(ringbuf) == ringbuf->size) ?
           PLATFORM_TRUE : PLATFORM_FALSE;
}

void platform_ringbuf_reset(platform_ringbuf_t *ringbuf)
{
    if (ringbuf == NULL)
    {
        return;
    }

    ringbuf->write_index = 0u;
    ringbuf->read_index = 0u;
}
