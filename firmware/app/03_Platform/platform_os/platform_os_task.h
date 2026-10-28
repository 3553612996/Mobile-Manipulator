/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_os_task.h
 *
 * @par dependencies
 * - platform_os_common.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform task and task-notification interface.
 *
 * Stack sizes are always expressed in bytes. 04_Impl converts them to the
 * kernel's word-based unit, which is exactly the kind of adaptation this
 * layer exists for.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_OS_TASK_H__
#define __PLATFORM_OS_TASK_H__

//******************************** Includes *********************************//

#include "platform_os_common.h"

//******************************** Declaring ********************************//

typedef struct platform_os_task *platform_os_task_t;

typedef void (*platform_os_task_entry_t)(void *argument);

typedef struct
{
    const char *                name;
    platform_os_task_entry_t    entry;
    void *                      argument;
    uint32_t                    stack_size_bytes;
    platform_os_priority_t      priority;
} platform_os_task_cfg_t;

/*
 * Create a task. The task object must be allocated by 04_Impl from its static
 * pool (the project forbids heap allocation at runtime). Returns
 * PLATFORM_ERR_NO_RESOURCE when the static pool is exhausted.
 */
platform_err_t platform_os_task_create(const platform_os_task_cfg_t *cfg,
                                       platform_os_task_t *out_task);

/* task == NULL deletes the calling task. */
platform_err_t platform_os_task_delete(platform_os_task_t task);

platform_err_t platform_os_task_delay_ms(uint32_t delay_ms);

/*
 * Periodic delay. last_wake_tick must be initialized with
 * platform_os_tick_get() and is updated by this call. period_ms is converted
 * to ticks by 04_Impl. This avoids the cumulative drift of delay_ms().
 */
platform_err_t platform_os_task_delay_until(uint32_t *last_wake_tick,
                                           uint32_t period_ms);

/* Wake a task. task == NULL means the calling task. */
platform_err_t platform_os_task_notify(platform_os_task_t task);

/*
 * ISR-safe variant. 04_Impl is responsible for calling portYIELD_FROM_ISR()
 * when a context switch is required; the caller does not need to know.
 */
platform_err_t platform_os_task_notify_from_isr(platform_os_task_t task);

/*
 * Wait for a notification and return the accumulated count. Returns
 * PLATFORM_ERR_TIMEOUT if nothing arrived within timeout_ms.
 */
platform_err_t platform_os_task_wait_notify(uint32_t timeout_ms,
                                            uint32_t *notify_count);

platform_os_task_t platform_os_task_get_current(void);
void platform_os_task_yield(void);

#endif /* __PLATFORM_OS_TASK_H__ */
