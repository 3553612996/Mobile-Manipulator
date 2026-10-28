/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_bsp_led.h
 *
 * @par dependencies
 * - platform_bsp_led.h
 * - 06_BSP/bsp_led_gpio.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Ops adapter for the platform status-LED contract.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __IMPL_BSP_LED_H__
#define __IMPL_BSP_LED_H__

//******************************** Includes *********************************//

#include "platform_bsp_led.h"

//******************************** Declaring ********************************//

/*
 * Pass this table and a bsp_led_gpio_t * to platform_led_register(). The
 * instance's cfg (pin and polarity) must be filled first.
 */
extern const platform_led_ops_t impl_bsp_led_ops;

#endif /* __IMPL_BSP_LED_H__ */
