/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file platform_def.h
 *
 * @par dependencies
 * - platform_type.h
 *
 * @author Jack | R&D Dept. | EternalChip
 *
 * @brief Provide the platform common definitions.
 *
 * @version V1.0 2026-05-10
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_DEF_H__
#define __PLATFORM_DEF_H__

//******************************** Includes *********************************//

#include "platform_type.h"

//******************************** Defines **********************************//

#define PLATFORM_OK             0
#define PLATFORM_ERROR          1
#define PLATFORM_TRUE           1
#define PLATFORM_FALSE          0

typedef uint8_t                 platform_bool_t;

#ifndef NULL
#define NULL                    ((void *)0)
#endif

#define PLATFORM_ALIGN_SIZE     4u
#define PLATFORM_ALIGN(n)       (((n) + PLATFORM_ALIGN_SIZE - 1u) & ~(PLATFORM_ALIGN_SIZE - 1u))

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr)         (sizeof(arr) / sizeof((arr)[0]))
#endif

#define PLATFORM_DELAY_MS(ms)   platform_delay_ms(ms)
#define PLATFORM_DELAY_US(us)   platform_delay_us(us)

//******************************** Declaring ********************************//

void platform_delay_ms(uint32_t ms);
void platform_delay_us(uint32_t us);

#endif /* __PLATFORM_DEF_H__ */
