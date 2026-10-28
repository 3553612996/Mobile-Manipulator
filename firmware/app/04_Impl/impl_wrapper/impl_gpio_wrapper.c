/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_gpio_wrapper.c
 *
 * @par dependencies
 * - impl_wrapper/impl_mcu_wrapper.h
 * - impl_wrapper/impl_trace.h
 * - impl_mcu/impl_mcu_gpio.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Add trace hooks around the GPIO implementation adapter.
 *
 * The wrapper owns instrumentation only. It must not validate parameters,
 * translate units or call vendor APIs; those belong to impl_mcu_gpio.c.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_wrapper/impl_mcu_wrapper.h"
#include "impl_wrapper/impl_trace.h"
#include "impl_mcu/impl_mcu_gpio.h"

//******************************** Functions ********************************//

platform_err_t impl_gpio_init_wrap(const platform_gpio_cfg_t *cfg)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_gpio_init");
    err = impl_gpio_init(cfg);
    IMPL_TRACE_EXIT("platform_gpio_init", err);

    return err;
}

platform_err_t impl_gpio_deinit_wrap(platform_gpio_t pin)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_gpio_deinit");
    err = impl_gpio_deinit(pin);
    IMPL_TRACE_EXIT("platform_gpio_deinit", err);

    return err;
}

platform_err_t impl_gpio_write_wrap(platform_gpio_t pin,
                                    platform_gpio_level_t level)
{
    platform_err_t err;

    IMPL_TRACE_ISR_ENTER("platform_gpio_write");
    err = impl_gpio_write(pin, level);
    IMPL_TRACE_ISR_EXIT("platform_gpio_write", err);

    return err;
}

platform_err_t impl_gpio_read_wrap(platform_gpio_t pin,
                                   platform_gpio_level_t *level)
{
    platform_err_t err;

    IMPL_TRACE_ISR_ENTER("platform_gpio_read");
    err = impl_gpio_read(pin, level);
    IMPL_TRACE_ISR_EXIT("platform_gpio_read", err);

    return err;
}

platform_err_t impl_gpio_toggle_wrap(platform_gpio_t pin)
{
    platform_err_t err;

    IMPL_TRACE_ISR_ENTER("platform_gpio_toggle");
    err = impl_gpio_toggle(pin);
    IMPL_TRACE_ISR_EXIT("platform_gpio_toggle", err);

    return err;
}

platform_err_t impl_gpio_irq_attach_wrap(platform_gpio_t pin,
                                         platform_gpio_irq_edge_t edge,
                                         platform_gpio_irq_cb_t callback,
                                         void *user_data)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_gpio_irq_attach");
    err = impl_gpio_irq_attach(pin, edge, callback, user_data);
    IMPL_TRACE_EXIT("platform_gpio_irq_attach", err);

    return err;
}

platform_err_t impl_gpio_irq_detach_wrap(platform_gpio_t pin)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_gpio_irq_detach");
    err = impl_gpio_irq_detach(pin);
    IMPL_TRACE_EXIT("platform_gpio_irq_detach", err);

    return err;
}
