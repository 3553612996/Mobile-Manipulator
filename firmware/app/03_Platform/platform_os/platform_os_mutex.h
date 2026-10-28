/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_os_mutex.h
 *
 * @par dependencies
 * - platform_os_common.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform mutex interface.
 *
 * Mutexes may only be used from task context. Never take a mutex from an ISR.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_OS_MUTEX_H__
#define __PLATFORM_OS_MUTEX_H__

//******************************** Includes *********************************//

#include "platform_os_common.h"

//******************************** Declaring ********************************//

typedef struct platform_os_mutex *platform_os_mutex_t;

platform_err_t platform_os_mutex_create(const char *name,
                                        platform_os_mutex_t *out_mutex);
platform_err_t platform_os_mutex_create_recursive(
                                        const char *name,
                                        platform_os_mutex_t *out_mutex);

/*
 * Returns PLATFORM_ERR_TIMEOUT if the mutex was not acquired in time.
 * A recursive mutex may be taken more than once by the same task and must be
 * released the same number of times.
 */
platform_err_t platform_os_mutex_lock(platform_os_mutex_t mutex,
                                      uint32_t timeout_ms);
platform_err_t platform_os_mutex_unlock(platform_os_mutex_t mutex);
platform_err_t platform_os_mutex_destroy(platform_os_mutex_t mutex);

#endif /* __PLATFORM_OS_MUTEX_H__ */
