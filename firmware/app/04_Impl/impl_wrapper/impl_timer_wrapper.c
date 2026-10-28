/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_timer_wrapper.c
 *
 * @par dependencies
 * - impl_wrapper/impl_mcu_wrapper.h
 * - impl_wrapper/impl_trace.h
 * - impl_mcu/impl_mcu_timer.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Add trace hooks around the one-shot timer implementation adapter.
 *
 * The wrapper owns instrumentation only. It must not validate parameters,
 * translate units or call vendor APIs; those belong to impl_mcu_timer.c.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_wrapper/impl_mcu_wrapper.h"
#include "impl_wrapper/impl_trace.h"
#include "impl_mcu/impl_mcu_timer.h"

//******************************** Functions ********************************//

platform_err_t impl_timer_init_wrap(const platform_timer_cfg_t *cfg)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_timer_init");
    err = impl_timer_init(cfg);
    IMPL_TRACE_EXIT("platform_timer_init", err);

    return err;
}

platform_err_t impl_timer_deinit_wrap(platform_timer_id_t timer)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_timer_deinit");
    err = impl_timer_deinit(timer);
    IMPL_TRACE_EXIT("platform_timer_deinit", err);

    return err;
}

platform_err_t impl_timer_start_wrap(platform_timer_id_t timer)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_timer_start");
    err = impl_timer_start(timer);
    IMPL_TRACE_EXIT("platform_timer_start", err);

    return err;
}

platform_err_t impl_timer_stop_wrap(platform_timer_id_t timer)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_timer_stop");
    err = impl_timer_stop(timer);
    IMPL_TRACE_EXIT("platform_timer_stop", err);

    return err;
}

platform_err_t impl_timer_set_next_interval_us_wrap(platform_timer_id_t timer,
                                                    uint32_t interval_us)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_timer_set_next_interval_us");
    err = impl_timer_set_next_interval_us(timer, interval_us);
    IMPL_TRACE_EXIT("platform_timer_set_next_interval_us", err);

    return err;
}
