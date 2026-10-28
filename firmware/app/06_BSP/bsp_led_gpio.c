/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file bsp_led_gpio.c
 *
 * @par dependencies
 * - 06_BSP/bsp_led_gpio.h
 * - platform_mcu_gpio.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief GPIO status-LED driver.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "bsp_led_gpio.h"

//******************************** Functions ********************************//

/* Map the logical "on" onto the electrical level the board needs. */
static platform_gpio_level_t bsp_led_gpio_level(const bsp_led_gpio_t *led,
                                                platform_bool_t on)
{
    platform_bool_t high;

    high = (led->cfg.active_low == PLATFORM_TRUE)
           ? ((on == PLATFORM_TRUE) ? PLATFORM_FALSE : PLATFORM_TRUE)
           : on;

    return (high == PLATFORM_TRUE) ? PLATFORM_GPIO_LEVEL_HIGH
                                   : PLATFORM_GPIO_LEVEL_LOW;
}

platform_err_t bsp_led_gpio_init(bsp_led_gpio_t *led)
{
    platform_gpio_cfg_t gpio_cfg;
    platform_err_t      err;

    if (led == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (led->initialized == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    gpio_cfg.pin           = led->cfg.pin;
    gpio_cfg.mode          = PLATFORM_GPIO_MODE_OUTPUT_PP;
    gpio_cfg.pull          = PLATFORM_GPIO_PULL_NONE;
    gpio_cfg.initial_level = bsp_led_gpio_level(led, PLATFORM_FALSE);

    err = platform_gpio_init(&gpio_cfg);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    led->on          = PLATFORM_FALSE;
    led->initialized = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t bsp_led_gpio_deinit(bsp_led_gpio_t *led)
{
    if (led == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (led->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    (void)platform_gpio_write(led->cfg.pin,
                              bsp_led_gpio_level(led, PLATFORM_FALSE));
    (void)platform_gpio_deinit(led->cfg.pin);

    led->on          = PLATFORM_FALSE;
    led->initialized = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

platform_err_t bsp_led_gpio_set(bsp_led_gpio_t *led, platform_bool_t on)
{
    platform_err_t err;

    if (led == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (led->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = platform_gpio_write(led->cfg.pin, bsp_led_gpio_level(led, on));
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    led->on = (on == PLATFORM_TRUE) ? PLATFORM_TRUE : PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}
