/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_os_critical.h
 *
 * @par dependencies
 * - platform_os_common.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform critical-section interface.
 *
 * Keep critical sections short: they disable interrupts (directly or through
 * the kernel) and therefore add latency to every ISR on the MCU.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_OS_CRITICAL_H__
#define __PLATFORM_OS_CRITICAL_H__

//******************************** Includes *********************************//

#include "platform_os_common.h"

//******************************** Declaring ********************************//

/*
 * Enter/exit a critical section. The returned state must be passed back to
 * platform_os_critical_exit(); it allows nesting and preserves the previous
 * interrupt mask.
 */
typedef uint32_t platform_os_critical_state_t;

platform_os_critical_state_t platform_os_critical_enter(void);
void platform_os_critical_exit(platform_os_critical_state_t state);

#endif /* __PLATFORM_OS_CRITICAL_H__ */
