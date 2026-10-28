/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file bsp_buzzer_gpio.h
 *
 * @par dependencies
 * - platform_mcu_gpio.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Active buzzer on a plain GPIO.
 *
 * The board drives the buzzer high to sound it, and it is an active type, so
 * the driver only switches the pin. Beep patterns and their timing stay in the
 * application: a blocking delay inside the BSP would stall the calling task.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __BSP_BUZZER_GPIO_H__
#define __BSP_BUZZER_GPIO_H__

//******************************** Includes *********************************//

#include "platform_mcu_gpio.h"

//******************************** Declaring ********************************//

typedef struct
{
    platform_gpio_t pin;
    platform_bool_t active_high;
} bsp_buzzer_gpio_cfg_t;

typedef struct
{
    bsp_buzzer_gpio_cfg_t cfg;
    platform_bool_t       on;
    platform_bool_t       initialized;
} bsp_buzzer_gpio_t;

//******************************** Functions ********************************//

platform_err_t bsp_buzzer_gpio_init(bsp_buzzer_gpio_t *buzzer);
platform_err_t bsp_buzzer_gpio_deinit(bsp_buzzer_gpio_t *buzzer);

platform_err_t bsp_buzzer_gpio_set(bsp_buzzer_gpio_t *buzzer,
                                   platform_bool_t on);

#endif /* __BSP_BUZZER_GPIO_H__ */
