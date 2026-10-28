/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file bsp_key_gpio.h
 *
 * @par dependencies
 * - platform_mcu_gpio.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Debounce one push button wired to a plain GPIO.
 *
 * This board has a single user key on PB2. The pin is pulled down on the board
 * and the key shorts it to 3.3 V, so the pressed level is high; active_high in
 * the configuration keeps that a property of the board, not of the driver.
 *
 * Debounce is sample-counted, not time-counted by the driver itself: poll()
 * receives the elapsed milliseconds from the caller, so the same code works
 * whether the status task runs every 1 ms or every 20 ms, and it can be tested
 * on a PC without a clock.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __BSP_KEY_GPIO_H__
#define __BSP_KEY_GPIO_H__

//******************************** Includes *********************************//

#include "platform_mcu_gpio.h"

//******************************** Defines **********************************//

#define BSP_KEY_DEBOUNCE_MS_DEFAULT     ((uint16_t)20u)

//******************************** Declaring ********************************//

typedef struct
{
    platform_gpio_t      pin;
    platform_gpio_pull_t pull;
    platform_bool_t      active_high;
    uint16_t             debounce_ms;
} bsp_key_gpio_cfg_t;

typedef struct
{
    bsp_key_gpio_cfg_t  cfg;
    platform_bool_t     raw_pressed;    /* Level of the last sample. */
    platform_bool_t     pressed;        /* Debounced level. */
    uint32_t            stable_ms;      /* Time the raw level has been stable. */
    uint32_t            pressed_ms;     /* Held time of the debounced press. */
    platform_bool_t     initialized;
} bsp_key_gpio_t;

//******************************** Functions ********************************//

platform_err_t bsp_key_gpio_init(bsp_key_gpio_t *key);
platform_err_t bsp_key_gpio_deinit(bsp_key_gpio_t *key);

/*
 * Sample the pin and advance the debounce state. elapsed_ms is the time since
 * the previous call; a value of 0 keeps the current state unchanged.
 */
platform_err_t bsp_key_gpio_poll(bsp_key_gpio_t *key, uint32_t elapsed_ms);

platform_bool_t bsp_key_gpio_is_pressed(const bsp_key_gpio_t *key);

uint32_t bsp_key_gpio_pressed_ms(const bsp_key_gpio_t *key);

#endif /* __BSP_KEY_GPIO_H__ */
