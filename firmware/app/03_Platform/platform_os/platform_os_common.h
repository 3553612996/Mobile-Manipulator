/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_os_common.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Common definitions shared by all platform OS interfaces.
 *
 * The OS layer hides FreeRTOS/CMSIS-RTOS from upper layers. Handles are opaque
 * pointers; their layout and lifetime belong to 04_Impl.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_OS_COMMON_H__
#define __PLATFORM_OS_COMMON_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"

//******************************** Defines **********************************//

#define PLATFORM_OS_WAIT_FOREVER    (0xFFFFFFFFu)
#define PLATFORM_OS_NO_WAIT         (0u)

//******************************** Declaring ********************************//

/*
 * Relative priority, not a raw FreeRTOS number. 04_Impl owns the mapping
 * (for example REALTIME -> osPriorityRealtime). Keeping a small common set
 * prevents upper layers from hard-coding configMAX_PRIORITIES-dependent
 * numbers that break when the kernel configuration changes.
 */
typedef enum
{
    PLATFORM_OS_PRIORITY_IDLE = 0,
    PLATFORM_OS_PRIORITY_LOW,
    PLATFORM_OS_PRIORITY_NORMAL,
    PLATFORM_OS_PRIORITY_ABOVE_NORMAL,
    PLATFORM_OS_PRIORITY_HIGH,
    PLATFORM_OS_PRIORITY_REALTIME,
    PLATFORM_OS_PRIORITY_COUNT
} platform_os_priority_t;

#endif /* __PLATFORM_OS_COMMON_H__ */
