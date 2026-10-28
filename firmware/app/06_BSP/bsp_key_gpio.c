/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file bsp_key_gpio.c
 *
 * @par dependencies
 * - 06_BSP/bsp_key_gpio.h
 * - platform_mcu_gpio.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief GPIO push-button driver with time-based debounce.
 *
 * The debounce rule is deliberately simple: a new level is accepted only after
 * it has been seen continuously for debounce_ms. A bouncing edge therefore
 * restarts the timer instead of leaking a short press.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "bsp_key_gpio.h"

//******************************** Functions ********************************//

static platform_bool_t bsp_key_gpio_read_raw(const bsp_key_gpio_t *key)
{
    platform_gpio_level_t level = PLATFORM_GPIO_LEVEL_LOW;

    if (PLATFORM_IS_ERR(platform_gpio_read(key->cfg.pin, &level)))
    {
        return PLATFORM_FALSE;
    }

    return ((level == PLATFORM_GPIO_LEVEL_HIGH) ==
            (key->cfg.active_high == PLATFORM_TRUE))
           ? PLATFORM_TRUE : PLATFORM_FALSE;
}

platform_err_t bsp_key_gpio_init(bsp_key_gpio_t *key)
{
    platform_gpio_cfg_t gpio_cfg;
    platform_err_t      err;

    if (key == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (key->initialized == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    if (key->cfg.debounce_ms == 0u)
    {
        key->cfg.debounce_ms = BSP_KEY_DEBOUNCE_MS_DEFAULT;
    }

    gpio_cfg.pin           = key->cfg.pin;
    gpio_cfg.mode          = PLATFORM_GPIO_MODE_INPUT;
    gpio_cfg.pull          = key->cfg.pull;
    gpio_cfg.initial_level = PLATFORM_GPIO_LEVEL_LOW;

    err = platform_gpio_init(&gpio_cfg);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    key->raw_pressed = bsp_key_gpio_read_raw(key);
    key->pressed     = key->raw_pressed;
    key->stable_ms   = 0;
    key->pressed_ms  = 0;
    key->initialized = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t bsp_key_gpio_deinit(bsp_key_gpio_t *key)
{
    if (key == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (key->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    (void)platform_gpio_deinit(key->cfg.pin);

    key->pressed     = PLATFORM_FALSE;
    key->raw_pressed = PLATFORM_FALSE;
    key->pressed_ms  = 0;
    key->initialized = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

platform_err_t bsp_key_gpio_poll(bsp_key_gpio_t *key, uint32_t elapsed_ms)
{
    platform_bool_t raw;

    if (key == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (key->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    raw = bsp_key_gpio_read_raw(key);

    if (raw != key->raw_pressed)
    {
        /* The level changed: restart the stable period. */
        key->raw_pressed = raw;
        key->stable_ms   = 0u;
    }
    else if (key->stable_ms < (uint32_t)key->cfg.debounce_ms)
    {
        key->stable_ms += elapsed_ms;
    }

    if ((key->stable_ms >= (uint32_t)key->cfg.debounce_ms) &&
        (key->pressed != key->raw_pressed))
    {
        key->pressed = key->raw_pressed;

        if (key->pressed == PLATFORM_TRUE)
        {
            key->pressed_ms = 0u;
        }
    }

    if (key->pressed == PLATFORM_TRUE)
    {
        key->pressed_ms += elapsed_ms;
    }
    else
    {
        key->pressed_ms = 0u;
    }

    return PLATFORM_ERR_OK;
}

platform_bool_t bsp_key_gpio_is_pressed(const bsp_key_gpio_t *key)
{
    return (key == NULL) ? PLATFORM_FALSE : key->pressed;
}

uint32_t bsp_key_gpio_pressed_ms(const bsp_key_gpio_t *key)
{
    return (key == NULL) ? 0u : key->pressed_ms;
}
