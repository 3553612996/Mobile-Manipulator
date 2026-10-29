/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_middleware.h
 *
 * @par dependencies
 * - platform_ringbuf.h
 * - platform_crc.h
 * - platform_filter.h
 * - platform_pid.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Aggregate header for the platform middleware.
 *
 * These modules depend on no MCU, RTOS or driver header and can be compiled on
 * the PC for unit tests.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_MIDDLEWARE_H__
#define __PLATFORM_MIDDLEWARE_H__

//******************************** Includes *********************************//

#include "platform_ringbuf.h"
#include "platform_crc.h"
#include "platform_filter.h"
#include "platform_pid.h"

#endif /* __PLATFORM_MIDDLEWARE_H__ */
