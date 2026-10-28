/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_mcu_timer.c
 *
 * @par dependencies
 * - platform_mcu_timer.h
 * - impl_wrapper/impl_mcu_wrapper.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Forward the platform one-shot timer contract to the Impl wrapper.
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

#include "platform_mcu_timer.h"
#include "impl_wrapper/impl_mcu_wrapper.h"

//******************************** Functions ********************************//

platform_err_t platform_timer_init(const platform_timer_cfg_t *cfg)
{
    return impl_timer_init_wrap(cfg);
}

platform_err_t platform_timer_deinit(platform_timer_id_t timer)
{
    return impl_timer_deinit_wrap(timer);
}

platform_err_t platform_timer_start(platform_timer_id_t timer)
{
    return impl_timer_start_wrap(timer);
}

platform_err_t platform_timer_stop(platform_timer_id_t timer)
{
    return impl_timer_stop_wrap(timer);
}

platform_err_t platform_timer_set_next_interval_us(platform_timer_id_t timer,
                                                   uint32_t interval_us)
{
    return impl_timer_set_next_interval_us_wrap(timer, interval_us);
}
