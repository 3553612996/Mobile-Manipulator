/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_mcu_wdg.c
 *
 * @par dependencies
 * - platform_mcu_wdg.h
 * - impl_wrapper/impl_mcu_wrapper.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Forward the platform watchdog contract to the Impl wrapper.
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

#include "platform_mcu_wdg.h"
#include "impl_wrapper/impl_mcu_wrapper.h"

//******************************** Functions ********************************//

platform_err_t platform_wdg_init(uint32_t timeout_ms)
{
    return impl_wdg_init_wrap(timeout_ms);
}

platform_err_t platform_wdg_feed(void)
{
    return impl_wdg_feed_wrap();
}

platform_err_t platform_wdg_get_reset_reason(
                                        platform_wdg_reset_reason_t *reason)
{
    return impl_wdg_get_reset_reason_wrap(reason);
}
