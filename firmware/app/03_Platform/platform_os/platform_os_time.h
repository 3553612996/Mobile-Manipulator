/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_os_time.h
 *
 * @par dependencies
 * - platform_os_common.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide tick, time conversion and kernel startup interfaces.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_OS_TIME_H__
#define __PLATFORM_OS_TIME_H__

//******************************** Includes *********************************//

#include "platform_os_common.h"

//******************************** Declaring ********************************//

/* Monotonic tick counter. The unit is kernel ticks, not milliseconds. */
uint32_t platform_os_tick_get(void);

uint32_t platform_os_ms_to_ticks(uint32_t milliseconds);
uint32_t platform_os_ticks_to_ms(uint32_t ticks);

/* TRUE when called from an interrupt context. */
platform_bool_t platform_os_is_in_isr(void);

/*
 * Kernel lifecycle. platform_os_kernel_init() must be called after the MCU
 * clocks and GPIO are ready and before any task/queue creation.
 * platform_os_kernel_start() does not return when it succeeds.
 */
platform_err_t platform_os_kernel_init(void);
platform_err_t platform_os_kernel_start(void);

#endif /* __PLATFORM_OS_TIME_H__ */
