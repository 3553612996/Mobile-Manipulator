/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_os_semaphore.h
 *
 * @par dependencies
 * - platform_os_common.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform binary/counting semaphore interface.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_OS_SEMAPHORE_H__
#define __PLATFORM_OS_SEMAPHORE_H__

//******************************** Includes *********************************//

#include "platform_os_common.h"

//******************************** Declaring ********************************//

typedef struct platform_os_sem *platform_os_sem_t;

platform_err_t platform_os_sem_binary_create(const char *name,
                                             platform_os_sem_t *out_sem);
platform_err_t platform_os_sem_counting_create(const char *name,
                                               uint32_t max_count,
                                               uint32_t initial_count,
                                               platform_os_sem_t *out_sem);

/* PLATFORM_ERR_TIMEOUT when the semaphore was not available in time. */
platform_err_t platform_os_sem_wait(platform_os_sem_t sem,
                                    uint32_t timeout_ms);

/* Returns PLATFORM_ERR_BUSY when a counting semaphore is already at max. */
platform_err_t platform_os_sem_post(platform_os_sem_t sem);
platform_err_t platform_os_sem_post_from_isr(platform_os_sem_t sem);

platform_err_t platform_os_sem_destroy(platform_os_sem_t sem);

#endif /* __PLATFORM_OS_SEMAPHORE_H__ */
