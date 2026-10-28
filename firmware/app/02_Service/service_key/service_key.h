/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file service_key.h
 *
 * @par dependencies
 * - platform_bsp_key.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Turn the debounced key device into short/long press events.
 *
 * The platform key only reports "pressed" and "held for N ms". This service
 * adds the policy the application cares about:
 *
 *   SHORT_PRESS  fired on release, when the key was held < long_press_ms
 *   LONG_PRESS   fired once when the hold crosses long_press_ms, while the
 *                key is still down
 *
 * Firing LONG while still pressed (instead of on release) is what makes the
 * gesture usable for safety actions: the user does not have to let go of the
 * emergency-stop key for it to take effect.
 *
 * The service is passive: the application task calls service_key_update()
 * every poll period. Time is injected, so the policy is testable on a PC.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __SERVICE_KEY_H__
#define __SERVICE_KEY_H__

//******************************** Includes *********************************//

#include "platform_bsp_key.h"

//******************************** Defines **********************************//

#define SERVICE_KEY_LONG_PRESS_MS_DEFAULT   ((uint32_t)1500u)

//******************************** Declaring ********************************//

typedef enum
{
    SERVICE_KEY_EVENT_SHORT_PRESS = 0,
    SERVICE_KEY_EVENT_LONG_PRESS
} service_key_event_t;

typedef void (*service_key_event_cb_t)(platform_key_id_t id,
                                       service_key_event_t event,
                                       void *user_data);

typedef struct
{
    platform_key_id_t   id;
    uint32_t            long_press_ms;  /* 0 selects the default. */
} service_key_cfg_t;

platform_err_t service_key_init(const service_key_cfg_t *cfg,
                                service_key_event_cb_t callback,
                                void *user_data);

/*
 * Sample the key and fire at most one event. elapsed_ms is the real time
 * since the previous call (the app measures it from the kernel tick), not the
 * nominal period: scheduling jitter must not stretch the debounce window.
 */
platform_err_t service_key_update(uint32_t elapsed_ms);

/* Held time of the last sample, useful for a UI or a test. */
uint32_t service_key_held_ms(void);

#endif /* __SERVICE_KEY_H__ */
