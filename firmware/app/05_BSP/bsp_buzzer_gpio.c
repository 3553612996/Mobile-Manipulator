/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file bsp_buzzer_gpio.c
 *
 * @par dependencies
 * - 05_BSP/bsp_buzzer_gpio.h
 * - platform_mcu_gpio.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief GPIO active-buzzer driver.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "bsp_buzzer_gpio.h"

//******************************** Functions ********************************//

static platform_gpio_level_t bsp_buzzer_gpio_level(
                                    const bsp_buzzer_gpio_t *buzzer,
                                    platform_bool_t on)
{
    platform_bool_t high;

    high = (buzzer->cfg.active_high == PLATFORM_TRUE)
           ? on
           : ((on == PLATFORM_TRUE) ? PLATFORM_FALSE : PLATFORM_TRUE);

    return (high == PLATFORM_TRUE) ? PLATFORM_GPIO_LEVEL_HIGH
                                   : PLATFORM_GPIO_LEVEL_LOW;
}

platform_err_t bsp_buzzer_gpio_init(bsp_buzzer_gpio_t *buzzer)
{
    platform_gpio_cfg_t gpio_cfg;
    platform_err_t      err;

    if (buzzer == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (buzzer->initialized == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    gpio_cfg.pin           = buzzer->cfg.pin;
    gpio_cfg.mode          = PLATFORM_GPIO_MODE_OUTPUT_PP;
    gpio_cfg.pull          = PLATFORM_GPIO_PULL_NONE;
    gpio_cfg.initial_level = bsp_buzzer_gpio_level(buzzer, PLATFORM_FALSE);

    err = platform_gpio_init(&gpio_cfg);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    buzzer->on          = PLATFORM_FALSE;
    buzzer->initialized = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t bsp_buzzer_gpio_deinit(bsp_buzzer_gpio_t *buzzer)
{
    if (buzzer == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (buzzer->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    (void)platform_gpio_write(buzzer->cfg.pin,
                              bsp_buzzer_gpio_level(buzzer, PLATFORM_FALSE));
    (void)platform_gpio_deinit(buzzer->cfg.pin);

    buzzer->on          = PLATFORM_FALSE;
    buzzer->initialized = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

platform_err_t bsp_buzzer_gpio_set(bsp_buzzer_gpio_t *buzzer,
                                   platform_bool_t on)
{
    platform_err_t err;

    if (buzzer == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (buzzer->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = platform_gpio_write(buzzer->cfg.pin,
                              bsp_buzzer_gpio_level(buzzer, on));
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    buzzer->on = (on == PLATFORM_TRUE) ? PLATFORM_TRUE : PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}
