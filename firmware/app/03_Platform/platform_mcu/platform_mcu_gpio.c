/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_mcu_gpio.c
 *
 * @par dependencies
 * - platform_mcu_gpio.h
 * - impl_wrapper/impl_mcu_wrapper.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Forward the platform GPIO contract to the Impl wrapper.
 *
 * This file contains no validation, adaptation or tracing. The platform layer
 * owns only the stable names; call-side behavior belongs to the Impl wrapper.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "platform_mcu_gpio.h"
#include "impl_wrapper/impl_mcu_wrapper.h"

//******************************** Functions ********************************//

platform_err_t platform_gpio_init(const platform_gpio_cfg_t *cfg)
{
    return impl_gpio_init_wrap(cfg);
}

platform_err_t platform_gpio_deinit(platform_gpio_t pin)
{
    return impl_gpio_deinit_wrap(pin);
}

platform_err_t platform_gpio_write(platform_gpio_t pin,
                                   platform_gpio_level_t level)
{
    return impl_gpio_write_wrap(pin, level);
}

platform_err_t platform_gpio_read(platform_gpio_t pin,
                                  platform_gpio_level_t *level)
{
    return impl_gpio_read_wrap(pin, level);
}

platform_err_t platform_gpio_toggle(platform_gpio_t pin)
{
    return impl_gpio_toggle_wrap(pin);
}

platform_err_t platform_gpio_irq_attach(platform_gpio_t pin,
                                        platform_gpio_irq_edge_t edge,
                                        platform_gpio_irq_cb_t callback,
                                        void *user_data)
{
    return impl_gpio_irq_attach_wrap(pin, edge, callback, user_data);
}

platform_err_t platform_gpio_irq_detach(platform_gpio_t pin)
{
    return impl_gpio_irq_detach_wrap(pin);
}
