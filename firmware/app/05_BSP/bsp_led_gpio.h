/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file bsp_led_gpio.h
 *
 * @par dependencies
 * - platform_mcu_gpio.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Status LED on a plain GPIO.
 *
 * The board sinks the LED current, so a lit LED means a low pin. active_low in
 * the configuration converts that into the driver's "on" semantics once, so
 * upper layers never deal with polarity.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __BSP_LED_GPIO_H__
#define __BSP_LED_GPIO_H__

//******************************** Includes *********************************//

#include "platform_mcu_gpio.h"

//******************************** Declaring ********************************//

typedef struct
{
    platform_gpio_t pin;
    platform_bool_t active_low;
} bsp_led_gpio_cfg_t;

typedef struct
{
    bsp_led_gpio_cfg_t cfg;
    platform_bool_t    on;
    platform_bool_t    initialized;
} bsp_led_gpio_t;

//******************************** Functions ********************************//

platform_err_t bsp_led_gpio_init(bsp_led_gpio_t *led);
platform_err_t bsp_led_gpio_deinit(bsp_led_gpio_t *led);

platform_err_t bsp_led_gpio_set(bsp_led_gpio_t *led, platform_bool_t on);

#endif /* __BSP_LED_GPIO_H__ */
