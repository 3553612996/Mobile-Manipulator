/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_bsp_key.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform user-key device interface.
 *
 * The key is a device, not a pin: active level, pull resistor and debounce
 * policy belong to the concrete driver in 05_BSP. Upper layers only see a
 * debounced pressed flag and the time it has been held.
 *
 * Time injection: poll() takes the milliseconds elapsed since the previous
 * call instead of reading a clock. The platform has no clock dependency, the
 * policy stays testable on a PC, and the caller decides the sampling period.
 * The device contract does not fix that period; the driver configuration does.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_BSP_KEY_H__
#define __PLATFORM_BSP_KEY_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"

//******************************** Defines **********************************//

#define PLATFORM_KEY_ID_0           ((platform_key_id_t)0u)
#define PLATFORM_KEY_ID_MAX         4u

//******************************** Declaring ********************************//

typedef uint8_t platform_key_id_t;

typedef struct
{
    platform_bool_t pressed;        /* Debounced level, true while held. */
    platform_bool_t raw_pressed;    /* Level of the last sample. */
    uint32_t        pressed_ms;     /* Held time in ms, 0 when released. */
} platform_key_state_t;

typedef struct
{
    platform_err_t (*init)(void *context);
    platform_err_t (*deinit)(void *context);

    /*
     * Sample the pin once and advance the debounce/hold counters.
     * elapsed_ms is the time since the previous poll call. Must be called from
     * task context at the period configured in the driver.
     */
    platform_err_t (*poll)(void *context, uint32_t elapsed_ms);

    platform_err_t (*get_state)(void *context, platform_key_state_t *state);
} platform_key_ops_t;

platform_err_t platform_key_register(platform_key_id_t id,
                                     const platform_key_ops_t *ops,
                                     void *context);

platform_err_t platform_key_init(platform_key_id_t id);
platform_err_t platform_key_deinit(platform_key_id_t id);

platform_err_t platform_key_poll(platform_key_id_t id, uint32_t elapsed_ms);
platform_err_t platform_key_get_state(platform_key_id_t id,
                                      platform_key_state_t *state);

#endif /* __PLATFORM_BSP_KEY_H__ */
