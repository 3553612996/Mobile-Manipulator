/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_os_event.h
 *
 * @par dependencies
 * - platform_os_common.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform event-group interface.
 *
 * Event groups are useful when one task waits for several independent
 * conditions (for example "sensor ready" OR "fault"). Prefer a queue when the
 * data itself matters; use bits only for state.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_OS_EVENT_H__
#define __PLATFORM_OS_EVENT_H__

//******************************** Includes *********************************//

#include "platform_os_common.h"

//******************************** Declaring ********************************//

typedef struct platform_os_event *platform_os_event_t;

typedef uint32_t platform_os_event_bits_t;

typedef enum
{
    PLATFORM_OS_EVENT_WAIT_ALL = 0,
    PLATFORM_OS_EVENT_WAIT_ANY
} platform_os_event_wait_mode_t;

platform_err_t platform_os_event_create(const char *name,
                                        platform_os_event_t *out_event);

platform_err_t platform_os_event_set(platform_os_event_t event,
                                     platform_os_event_bits_t bits);
platform_err_t platform_os_event_set_from_isr(platform_os_event_t event,
                                              platform_os_event_bits_t bits);
platform_err_t platform_os_event_clear(platform_os_event_t event,
                                       platform_os_event_bits_t bits);

/*
 * Wait for bits according to mode. When clear_on_exit is TRUE the matching
 * bits are cleared before returning. out_bits receives the bit value observed
 * at the moment of return, including the timeout case.
 */
platform_err_t platform_os_event_wait(platform_os_event_t event,
                                      platform_os_event_bits_t bits,
                                      platform_os_event_wait_mode_t mode,
                                      platform_bool_t clear_on_exit,
                                      uint32_t timeout_ms,
                                      platform_os_event_bits_t *out_bits);

platform_err_t platform_os_event_destroy(platform_os_event_t event);

#endif /* __PLATFORM_OS_EVENT_H__ */
