/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_wdg_wrapper.c
 *
 * @par dependencies
 * - impl_wrapper/impl_mcu_wrapper.h
 * - impl_wrapper/impl_trace.h
 * - impl_mcu/impl_mcu_wdg.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Add trace hooks around the watchdog implementation adapter.
 *
 * The wrapper owns instrumentation only. It must not validate parameters,
 * translate units or call vendor APIs; those belong to impl_mcu_wdg.c.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_wrapper/impl_mcu_wrapper.h"
#include "impl_wrapper/impl_trace.h"
#include "impl_mcu/impl_mcu_wdg.h"

//******************************** Functions ********************************//

platform_err_t impl_wdg_init_wrap(uint32_t timeout_ms)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_wdg_init");
    err = impl_wdg_init(timeout_ms);
    IMPL_TRACE_EXIT("platform_wdg_init", err);

    return err;
}

platform_err_t impl_wdg_feed_wrap(void)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_wdg_feed");
    err = impl_wdg_feed();
    IMPL_TRACE_EXIT("platform_wdg_feed", err);

    return err;
}

platform_err_t impl_wdg_get_reset_reason_wrap(
                                        platform_wdg_reset_reason_t *reason)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_wdg_get_reset_reason");
    err = impl_wdg_get_reset_reason(reason);
    IMPL_TRACE_EXIT("platform_wdg_get_reset_reason", err);

    return err;
}
