/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_bsp_led.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform status-LED device interface.
 *
 * "On" means visually lit. The active level (high or low) is a property of the
 * concrete driver and is fixed at registration time, so upper layers never have
 * to know whether the board lights the LED by sinking or sourcing current.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_BSP_LED_H__
#define __PLATFORM_BSP_LED_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"

//******************************** Declaring ********************************//

typedef struct
{
    platform_err_t (*init)(void *context);
    platform_err_t (*deinit)(void *context);
    platform_err_t (*set)(void *context, platform_bool_t on);
} platform_led_ops_t;

/* Only one status LED is assumed for this robot. */
platform_err_t platform_led_register(const platform_led_ops_t *ops,
                                     void *context);

platform_err_t platform_led_init(void);
platform_err_t platform_led_deinit(void);
platform_err_t platform_led_set(platform_bool_t on);

#endif /* __PLATFORM_BSP_LED_H__ */
