/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_os_timer.h
 *
 * @par dependencies
 * - platform_os_common.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform software-timer interface.
 *
 * Timer callbacks run in the kernel timer task, not in an ISR. They may call
 * non-blocking OS APIs, but must stay short and must not block.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_OS_TIMER_H__
#define __PLATFORM_OS_TIMER_H__

//******************************** Includes *********************************//

#include "platform_os_common.h"

//******************************** Declaring ********************************//

typedef struct platform_os_timer *platform_os_timer_t;

typedef void (*platform_os_timer_cb_t)(void *argument);

typedef struct
{
    const char *                name;
    uint32_t                    period_ms;
    platform_bool_t             auto_reload;    /* TRUE: periodic. */
    platform_os_timer_cb_t      callback;
    void *                      argument;
} platform_os_timer_cfg_t;

platform_err_t platform_os_timer_create(const platform_os_timer_cfg_t *cfg,
                                        platform_os_timer_t *out_timer);

platform_err_t platform_os_timer_start(platform_os_timer_t timer);
platform_err_t platform_os_timer_stop(platform_os_timer_t timer);
platform_err_t platform_os_timer_reset(platform_os_timer_t timer);
platform_err_t platform_os_timer_set_period(platform_os_timer_t timer,
                                            uint32_t period_ms);
platform_err_t platform_os_timer_destroy(platform_os_timer_t timer);

#endif /* __PLATFORM_OS_TIMER_H__ */
