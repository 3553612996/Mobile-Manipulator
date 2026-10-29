/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_wearlevel.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the sector allocation policy that keeps erase counts even.
 *
 * Why this is a middleware and not part of a driver
 * -------------------------------------------------
 * "Which sector do I erase next" is a policy, not a register write. Keeping it
 * free of HAL lets the PC unit test drive millions of erases against a RAM
 * fake and show that the worst sector stays within one erase of the best.
 *
 * Algorithm
 * ---------
 * Greedy least-erased allocation with ring ordering:
 *   1. find the smallest erase count in the region;
 *   2. starting from the caller's hint, return the first sector whose count
 *      equals that minimum.
 * When every sector has been erased the same number of times (the normal case
 * for a ring) step 2 degenerates to "hint", i.e. a plain sequential ring with
 * no extra cost. If a sector is erased more often because a caller cleared a
 * sub-range, the next allocation starts from the coldest sector instead, so
 * wear converges back to even. This is dynamic wear leveling; static
 * relocation (moving cold data out of a worn block) is unnecessary here
 * because the whole region is a log that is rewritten cyclically anyway.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_WEARLEVEL_H__
#define __PLATFORM_WEARLEVEL_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"

//******************************** Defines **********************************//

/* One log store may span at most this many sectors (224 used by the log). */
#define PLATFORM_WEARLEVEL_MAX_SECTORS      ((uint32_t)256u)

/* Saturating counter: a sector that reached this is reported, never wrapped. */
#define PLATFORM_WEARLEVEL_COUNT_MAX        ((uint16_t)0xFFFFu)

//******************************** Declaring ********************************//

typedef struct
{
    uint32_t    base_address;       /* Byte address of sector 0. */
    uint32_t    sector_size;        /* 4096 on the W25Q64. */
    uint32_t    sector_count;       /* 1..PLATFORM_WEARLEVEL_MAX_SECTORS. */
    uint16_t *  erase_counts;       /* sector_count entries, caller-owned. */
} platform_wearlevel_t;

/*
 * Bind the policy to a region. erase_counts is zeroed: the caller restores the
 * persisted counters afterwards with platform_wearlevel_set_count().
 */
platform_err_t platform_wearlevel_init(platform_wearlevel_t *wearlevel,
                                       uint32_t base_address,
                                       uint32_t sector_size,
                                       uint32_t sector_count,
                                       uint16_t *erase_counts);

/*
 * Pick the sector to use next. hint is the caller's sequence position; the
 * return value is in [0, sector_count). O(1) memory, O(n) time, n <= 256.
 */
uint32_t platform_wearlevel_pick(const platform_wearlevel_t *wearlevel,
                                 uint32_t hint);

/*
 * Pick the coldest sector that the caller allows. A ring cannot always pick
 * globally coldest: sectors that still hold live records must not be erased,
 * so the owner of the data supplies the predicate. When the predicate accepts
 * nothing, *found is set to PLATFORM_FALSE and 0 is returned.
 */
typedef platform_bool_t (*platform_wearlevel_filter_t)(uint32_t index,
                                                       void *context);

uint32_t platform_wearlevel_pick_filtered(
                            const platform_wearlevel_t *wearlevel,
                            uint32_t hint,
                            platform_wearlevel_filter_t filter,
                            void *context,
                            platform_bool_t *found);

/* Record one erase. Saturates; out-of-range indexes are ignored. */
void platform_wearlevel_note_erase(platform_wearlevel_t *wearlevel,
                                   uint32_t index);

/* Restore a persisted counter (clamped to COUNT_MAX). */
void platform_wearlevel_set_count(platform_wearlevel_t *wearlevel,
                                  uint32_t index,
                                  uint32_t count);

uint32_t platform_wearlevel_get_count(
                                const platform_wearlevel_t *wearlevel,
                                uint32_t index);

uint32_t platform_wearlevel_min(const platform_wearlevel_t *wearlevel);
uint32_t platform_wearlevel_max(const platform_wearlevel_t *wearlevel);
uint32_t platform_wearlevel_average(const platform_wearlevel_t *wearlevel);

/* Byte address of one sector, or 0 when the index is out of range. */
uint32_t platform_wearlevel_sector_address(
                                const platform_wearlevel_t *wearlevel,
                                uint32_t index);

#endif /* __PLATFORM_WEARLEVEL_H__ */
