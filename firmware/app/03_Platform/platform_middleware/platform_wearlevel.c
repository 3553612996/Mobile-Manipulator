/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_wearlevel.c
 *
 * @par dependencies
 * - platform_wearlevel.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement least-erase-count sector selection.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "platform_wearlevel.h"

//******************************** Functions ********************************//

/*
 * Validity check kept in one place: every public entry point calls it so a
 * half-initialised struct cannot index the caller's counter array.
 */
static platform_bool_t s_wearlevel_ready(const platform_wearlevel_t *wearlevel)
{
    if (wearlevel == NULL)
    {
        return PLATFORM_FALSE;
    }
    if (wearlevel->erase_counts == NULL)
    {
        return PLATFORM_FALSE;
    }
    if ((wearlevel->sector_count == 0u) ||
        (wearlevel->sector_count > PLATFORM_WEARLEVEL_MAX_SECTORS))
    {
        return PLATFORM_FALSE;
    }

    return PLATFORM_TRUE;
}

platform_err_t platform_wearlevel_init(platform_wearlevel_t *wearlevel,
                                       uint32_t base_address,
                                       uint32_t sector_size,
                                       uint32_t sector_count,
                                       uint16_t *erase_counts)
{
    uint32_t i;

    if ((wearlevel == NULL) || (erase_counts == NULL) || (sector_size == 0u))
    {
        return PLATFORM_ERR_PARAM;
    }
    if ((sector_count == 0u) ||
        (sector_count > PLATFORM_WEARLEVEL_MAX_SECTORS))
    {
        return PLATFORM_ERR_PARAM;
    }

    wearlevel->base_address = base_address;
    wearlevel->sector_size  = sector_size;
    wearlevel->sector_count = sector_count;
    wearlevel->erase_counts = erase_counts;

    for (i = 0u; i < sector_count; i++)
    {
        erase_counts[i] = 0u;
    }

    return PLATFORM_ERR_OK;
}

uint32_t platform_wearlevel_pick_filtered(
                            const platform_wearlevel_t *wearlevel,
                            uint32_t hint,
                            platform_wearlevel_filter_t filter,
                            void *context,
                            platform_bool_t *found)
{
    uint32_t start;
    uint32_t i;
    uint32_t index;
    uint16_t lowest;
    platform_bool_t any;

    if (found != NULL)
    {
        *found = PLATFORM_FALSE;
    }

    if (s_wearlevel_ready(wearlevel) == PLATFORM_FALSE)
    {
        return 0u;
    }

    lowest = PLATFORM_WEARLEVEL_COUNT_MAX;
    any = PLATFORM_FALSE;
    for (i = 0u; i < wearlevel->sector_count; i++)
    {
        if ((filter != NULL) && (filter(i, context) == PLATFORM_FALSE))
        {
            continue;
        }
        any = PLATFORM_TRUE;
        if (wearlevel->erase_counts[i] < lowest)
        {
            lowest = wearlevel->erase_counts[i];
        }
    }

    if (any == PLATFORM_FALSE)
    {
        return 0u;
    }

    /*
     * Walk forward from the hint and stop at the first coldest sector. In the
     * common case (all sectors equal) the first candidate is the hint itself,
     * so the ring stays sequential and mount order is unaffected.
     */
    start = hint % wearlevel->sector_count;
    for (i = 0u; i < wearlevel->sector_count; i++)
    {
        index = start + i;
        if (index >= wearlevel->sector_count)
        {
            index -= wearlevel->sector_count;
        }
        if ((filter != NULL) && (filter(index, context) == PLATFORM_FALSE))
        {
            continue;
        }
        if (wearlevel->erase_counts[index] == lowest)
        {
            if (found != NULL)
            {
                *found = PLATFORM_TRUE;
            }
            return index;
        }
    }

    return 0u;
}

uint32_t platform_wearlevel_pick(const platform_wearlevel_t *wearlevel,
                                 uint32_t hint)
{
    return platform_wearlevel_pick_filtered(wearlevel, hint, NULL, NULL, NULL);
}

void platform_wearlevel_note_erase(platform_wearlevel_t *wearlevel,
                                   uint32_t index)
{
    if (s_wearlevel_ready(wearlevel) == PLATFORM_FALSE)
    {
        return;
    }
    if (index >= wearlevel->sector_count)
    {
        return;
    }
    if (wearlevel->erase_counts[index] < PLATFORM_WEARLEVEL_COUNT_MAX)
    {
        wearlevel->erase_counts[index]++;
    }
}

void platform_wearlevel_set_count(platform_wearlevel_t *wearlevel,
                                  uint32_t index,
                                  uint32_t count)
{
    if (s_wearlevel_ready(wearlevel) == PLATFORM_FALSE)
    {
        return;
    }
    if (index >= wearlevel->sector_count)
    {
        return;
    }

    if (count > PLATFORM_WEARLEVEL_COUNT_MAX)
    {
        count = PLATFORM_WEARLEVEL_COUNT_MAX;
    }
    wearlevel->erase_counts[index] = (uint16_t)count;
}

uint32_t platform_wearlevel_get_count(
                                const platform_wearlevel_t *wearlevel,
                                uint32_t index)
{
    if (s_wearlevel_ready(wearlevel) == PLATFORM_FALSE)
    {
        return 0u;
    }
    if (index >= wearlevel->sector_count)
    {
        return 0u;
    }

    return (uint32_t)wearlevel->erase_counts[index];
}

uint32_t platform_wearlevel_min(const platform_wearlevel_t *wearlevel)
{
    uint32_t i;
    uint16_t lowest;

    if (s_wearlevel_ready(wearlevel) == PLATFORM_FALSE)
    {
        return 0u;
    }

    lowest = PLATFORM_WEARLEVEL_COUNT_MAX;
    for (i = 0u; i < wearlevel->sector_count; i++)
    {
        if (wearlevel->erase_counts[i] < lowest)
        {
            lowest = wearlevel->erase_counts[i];
        }
    }

    return (uint32_t)lowest;
}

uint32_t platform_wearlevel_max(const platform_wearlevel_t *wearlevel)
{
    uint32_t i;
    uint16_t highest;

    if (s_wearlevel_ready(wearlevel) == PLATFORM_FALSE)
    {
        return 0u;
    }

    highest = 0u;
    for (i = 0u; i < wearlevel->sector_count; i++)
    {
        if (wearlevel->erase_counts[i] > highest)
        {
            highest = wearlevel->erase_counts[i];
        }
    }

    return (uint32_t)highest;
}

uint32_t platform_wearlevel_average(const platform_wearlevel_t *wearlevel)
{
    uint32_t i;
    uint32_t sum;

    if (s_wearlevel_ready(wearlevel) == PLATFORM_FALSE)
    {
        return 0u;
    }

    sum = 0u;
    for (i = 0u; i < wearlevel->sector_count; i++)
    {
        sum += (uint32_t)wearlevel->erase_counts[i];
    }

    return sum / wearlevel->sector_count;
}

uint32_t platform_wearlevel_sector_address(
                                const platform_wearlevel_t *wearlevel,
                                uint32_t index)
{
    if (s_wearlevel_ready(wearlevel) == PLATFORM_FALSE)
    {
        return 0u;
    }
    if (index >= wearlevel->sector_count)
    {
        return 0u;
    }

    return wearlevel->base_address + (index * wearlevel->sector_size);
}
