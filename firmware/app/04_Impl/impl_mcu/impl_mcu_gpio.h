/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_mcu_gpio.h
 *
 * @par dependencies
 * - platform_mcu_gpio.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Declare the GPIO implementation adapter.
 *
 * These functions contain the parameter translation and the vendor API calls.
 * The Impl wrapper is the only caller. Do not include this header from
 * 03_Platform or any layer above it.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __IMPL_MCU_GPIO_H__
#define __IMPL_MCU_GPIO_H__

//******************************** Includes *********************************//

#include "platform_mcu_gpio.h"

//******************************** Declaring ********************************//

platform_err_t impl_gpio_init(const platform_gpio_cfg_t *cfg);
platform_err_t impl_gpio_deinit(platform_gpio_t pin);

platform_err_t impl_gpio_write(platform_gpio_t pin,
                               platform_gpio_level_t level);
platform_err_t impl_gpio_read(platform_gpio_t pin,
                              platform_gpio_level_t *level);
platform_err_t impl_gpio_toggle(platform_gpio_t pin);

platform_err_t impl_gpio_irq_attach(platform_gpio_t pin,
                                    platform_gpio_irq_edge_t edge,
                                    platform_gpio_irq_cb_t callback,
                                    void *user_data);
platform_err_t impl_gpio_irq_detach(platform_gpio_t pin);

#endif /* __IMPL_MCU_GPIO_H__ */
