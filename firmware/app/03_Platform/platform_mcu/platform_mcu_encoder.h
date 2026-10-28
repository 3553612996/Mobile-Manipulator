/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_mcu_encoder.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform quadrature encoder interface.
 *
 * An encoder channel is a logical resource, not a timer number. 04_Impl maps it
 * to (TIMx, CH1 pin, CH2 pin) and owns the GPIO/alternate-function setup.
 *
 * The hardware counter is 16 bit. read_delta() returns the counts accumulated
 * since the previous call and clears the hardware counter, so the caller can
 * build a 32-bit cumulative position without losing an overflow that happens
 * between two reads. The read-clear pair is not atomic with respect to the
 * encoder edges; a delta is therefore accurate to one count. Consumers that
 * need absolute position must accumulate in software (see 06_BSP motor driver).
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_MCU_ENCODER_H__
#define __PLATFORM_MCU_ENCODER_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"

//******************************** Defines **********************************//

#define PLATFORM_ENCODER_CHANNEL_0      ((platform_encoder_channel_t)0u)
#define PLATFORM_ENCODER_CHANNEL_1      ((platform_encoder_channel_t)1u)
#define PLATFORM_ENCODER_CHANNEL_2      ((platform_encoder_channel_t)2u)
#define PLATFORM_ENCODER_CHANNEL_3      ((platform_encoder_channel_t)3u)
#define PLATFORM_ENCODER_CHANNEL_INVALID ((platform_encoder_channel_t)0xFFu)

/* Input capture filter, 0 (no filtering) .. 15 (longest). */
#define PLATFORM_ENCODER_FILTER_MAX     15u

//******************************** Declaring ********************************//

typedef uint8_t platform_encoder_channel_t;

typedef struct
{
    platform_encoder_channel_t  channel;
    uint8_t                     filter;         /* 0..15. */
    platform_bool_t             invert;         /* Swap the counting direction. */
} platform_encoder_cfg_t;

platform_err_t platform_encoder_init(const platform_encoder_cfg_t *cfg);
platform_err_t platform_encoder_deinit(platform_encoder_channel_t channel);

/*
 * Read the counts accumulated since the previous call and clear the counter.
 * The returned value is signed: positive/negative follows the configured
 * direction. Call it from a single task; it is not reentrant.
 */
platform_err_t platform_encoder_read_delta(platform_encoder_channel_t channel,
                                           int16_t *delta);

/*
 * Read the raw 16-bit hardware counter without clearing it. Debug/diagnostics
 * only: the value wraps and is meaningless on its own.
 */
platform_err_t platform_encoder_get_count(platform_encoder_channel_t channel,
                                          int16_t *count);

platform_err_t platform_encoder_reset(platform_encoder_channel_t channel);

#endif /* __PLATFORM_MCU_ENCODER_H__ */
