/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file service_key.c
 *
 * @par dependencies
 * - service_key.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement the short/long press policy.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "service_key.h"

//******************************** Variables ********************************//

static struct
{
    platform_bool_t         initialized;
    platform_key_id_t       id;
    uint32_t                long_press_ms;
    service_key_event_cb_t  callback;
    void *                  user_data;
    platform_bool_t         was_pressed;
    platform_bool_t         long_fired;
    uint32_t                held_ms;
} s_key;

//******************************** Functions ********************************//

platform_err_t service_key_init(const service_key_cfg_t *cfg,
                                service_key_event_cb_t callback,
                                void *user_data)
{
    if (cfg == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    s_key.id = cfg->id;
    s_key.long_press_ms = (cfg->long_press_ms == 0u)
                              ? SERVICE_KEY_LONG_PRESS_MS_DEFAULT
                              : cfg->long_press_ms;
    s_key.callback = callback;
    s_key.user_data = user_data;
    s_key.was_pressed = PLATFORM_FALSE;
    s_key.long_fired = PLATFORM_FALSE;
    s_key.held_ms = 0u;
    s_key.initialized = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t service_key_update(uint32_t elapsed_ms)
{
    platform_key_state_t state;
    platform_err_t       err;

    if (s_key.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = platform_key_poll(s_key.id, elapsed_ms);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = platform_key_get_state(s_key.id, &state);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    s_key.held_ms = state.pressed_ms;

    if (state.pressed == PLATFORM_TRUE)
    {
        if (s_key.was_pressed == PLATFORM_FALSE)
        {
            /* Rising edge: start a new gesture. */
            s_key.was_pressed = PLATFORM_TRUE;
            s_key.long_fired = PLATFORM_FALSE;
        }
        else if ((s_key.long_fired == PLATFORM_FALSE) &&
                 (s_key.held_ms >= s_key.long_press_ms))
        {
            s_key.long_fired = PLATFORM_TRUE;

            if (s_key.callback != NULL)
            {
                s_key.callback(s_key.id, SERVICE_KEY_EVENT_LONG_PRESS,
                               s_key.user_data);
            }
        }
        else
        {
            /* Still held below the long threshold; nothing to report. */
        }
    }
    else
    {
        if (s_key.was_pressed == PLATFORM_TRUE)
        {
            /* Falling edge: a short press only counts if LONG did not fire. */
            if ((s_key.long_fired == PLATFORM_FALSE) &&
                (s_key.callback != NULL))
            {
                s_key.callback(s_key.id, SERVICE_KEY_EVENT_SHORT_PRESS,
                               s_key.user_data);
            }

            s_key.was_pressed = PLATFORM_FALSE;
            s_key.long_fired = PLATFORM_FALSE;
        }
    }

    return PLATFORM_ERR_OK;
}

uint32_t service_key_held_ms(void)
{
    return s_key.held_ms;
}
