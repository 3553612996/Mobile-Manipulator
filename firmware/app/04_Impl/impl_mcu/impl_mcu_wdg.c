/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_mcu_wdg.c
 *
 * @par dependencies
 * - impl_mcu/impl_mcu_wdg.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Adapt the platform watchdog contract to the MCU (skeleton).
 *
 * This is the only layer allowed to include stm32f4xx_hal.h once the
 * adaptation is written. The wrapper and the platform layer must never see
 * vendor headers.
 *
 * TODO: replace each PLATFORM_ERR_NOT_SUPPORTED body with the HAL/LL
 * implementation. Keep the signatures unchanged because the wrapper depends
 * on them.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_mcu/impl_mcu_wdg.h"

//******************************** Functions ********************************//

platform_err_t impl_wdg_init(uint32_t timeout_ms)
{
    (void)timeout_ms;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_wdg_feed(void)
{
    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_wdg_get_reset_reason(
                                        platform_wdg_reset_reason_t *reason)
{
    if (reason != NULL)
    {
        *reason = PLATFORM_WDG_RESET_REASON_UNKNOWN;
    }

    return PLATFORM_ERR_NOT_SUPPORTED;
}
