/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_os.h
 *
 * @par dependencies
 * - platform_os_task.h
 * - platform_os_time.h
 * - platform_os_mutex.h
 * - platform_os_semaphore.h
 * - platform_os_queue.h
 * - platform_os_event.h
 * - platform_os_timer.h
 * - platform_os_critical.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Aggregate header for the platform OS interfaces.
 *
 * Upper layers include this file only. They must not include FreeRTOS.h or
 * cmsis_os.h.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_OS_H__
#define __PLATFORM_OS_H__

//******************************** Includes *********************************//

#include "platform_os_common.h"
#include "platform_os_task.h"
#include "platform_os_time.h"
#include "platform_os_mutex.h"
#include "platform_os_semaphore.h"
#include "platform_os_queue.h"
#include "platform_os_event.h"
#include "platform_os_timer.h"
#include "platform_os_critical.h"

#endif /* __PLATFORM_OS_H__ */
