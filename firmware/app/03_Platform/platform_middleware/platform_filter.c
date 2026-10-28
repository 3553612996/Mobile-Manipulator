/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_filter.c
 *
 * @par dependencies
 * - platform_filter.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement the moving-average and first-order low-pass filters.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "platform_filter.h"

//******************************** Functions ********************************//

platform_err_t platform_filter_moving_average_init(
                                        platform_filter_moving_average_t *filter,
                                        int32_t *storage,
                                        uint32_t window_size)
{
    uint32_t i;

    if ((filter == NULL) || (storage == NULL) || (window_size == 0u))
    {
        return PLATFORM_ERR_PARAM;
    }

    filter->samples = storage;
    filter->window_size = window_size;
    filter->index = 0u;
    filter->count = 0u;
    filter->sum = 0;

    for (i = 0u; i < window_size; i++)
    {
        storage[i] = 0;
    }

    return PLATFORM_ERR_OK;
}

int32_t platform_filter_moving_average_update(
                                        platform_filter_moving_average_t *filter,
                                        int32_t sample)
{
    if ((filter == NULL) || (filter->samples == NULL) ||
        (filter->window_size == 0u))
    {
        return 0;
    }

    if (filter->count < filter->window_size)
    {
        filter->sum += (int64_t)sample;
        filter->count++;
    }
    else
    {
        filter->sum -= (int64_t)filter->samples[filter->index];
        filter->sum += (int64_t)sample;
    }

    filter->samples[filter->index] = sample;
    filter->index = (filter->index + 1u) % filter->window_size;

    return (int32_t)(filter->sum / (int64_t)filter->count);
}

int32_t platform_filter_moving_average_get(
                                        const platform_filter_moving_average_t *filter)
{
    if ((filter == NULL) || (filter->samples == NULL) ||
        (filter->window_size == 0u) || (filter->count == 0u))
    {
        return 0;
    }

    return (int32_t)(filter->sum / (int64_t)filter->count);
}

void platform_filter_moving_average_reset(
                                        platform_filter_moving_average_t *filter)
{
    uint32_t i;

    if ((filter == NULL) || (filter->samples == NULL))
    {
        return;
    }

    for (i = 0u; i < filter->window_size; i++)
    {
        filter->samples[i] = 0;
    }

    filter->index = 0u;
    filter->count = 0u;
    filter->sum = 0;
}

platform_err_t platform_filter_lowpass_init(platform_filter_lowpass_t *filter,
                                            float_t alpha)
{
    if (filter == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if ((alpha <= 0.0f) || (alpha > 1.0f))
    {
        return PLATFORM_ERR_PARAM;
    }

    filter->alpha = alpha;
    filter->value = 0.0f;
    filter->initialized = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

float_t platform_filter_lowpass_update(platform_filter_lowpass_t *filter,
                                       float_t sample)
{
    if (filter == NULL)
    {
        return 0.0f;
    }

    if (filter->initialized == PLATFORM_FALSE)
    {
        filter->value = sample;
        filter->initialized = PLATFORM_TRUE;
    }
    else
    {
        filter->value += filter->alpha * (sample - filter->value);
    }

    return filter->value;
}

void platform_filter_lowpass_reset(platform_filter_lowpass_t *filter)
{
    if (filter == NULL)
    {
        return;
    }

    filter->value = 0.0f;
    filter->initialized = PLATFORM_FALSE;
}
