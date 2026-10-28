/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_filter.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide basic integer/floating-point filters.
 *
 * Both filters have caller-owned state and allocate nothing. They are pure
 * computation, but their state is not thread-safe: one task must own a filter
 * instance, or the caller must add a lock.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_FILTER_H__
#define __PLATFORM_FILTER_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"

//******************************** Declaring ********************************//

/*
 * Moving average. storage must hold window_size int32_t elements and must
 * outlive the filter. The sum is kept as int64_t so a long window of large
 * samples cannot overflow.
 */
typedef struct
{
    int32_t *               samples;
    uint32_t                window_size;
    uint32_t                index;
    uint32_t                count;
    int64_t                 sum;
} platform_filter_moving_average_t;

/*
 * First-order low-pass: y += alpha * (x - y).
 * alpha is in (0, 1]; smaller alpha means heavier smoothing but more lag.
 * The first sample initializes the output directly, avoiding a startup ramp
 * from zero.
 */
typedef struct
{
    float_t                 alpha;
    float_t                 value;
    platform_bool_t         initialized;
} platform_filter_lowpass_t;

platform_err_t platform_filter_moving_average_init(
                                        platform_filter_moving_average_t *filter,
                                        int32_t *storage,
                                        uint32_t window_size);
int32_t platform_filter_moving_average_update(
                                        platform_filter_moving_average_t *filter,
                                        int32_t sample);
int32_t platform_filter_moving_average_get(
                                        const platform_filter_moving_average_t *filter);
void platform_filter_moving_average_reset(
                                        platform_filter_moving_average_t *filter);

platform_err_t platform_filter_lowpass_init(platform_filter_lowpass_t *filter,
                                            float_t alpha);
float_t platform_filter_lowpass_update(platform_filter_lowpass_t *filter,
                                       float_t sample);
void platform_filter_lowpass_reset(platform_filter_lowpass_t *filter);

#endif /* __PLATFORM_FILTER_H__ */
