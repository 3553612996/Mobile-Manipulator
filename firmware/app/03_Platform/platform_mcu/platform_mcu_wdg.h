/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_mcu_wdg.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform watchdog interface.
 *
 * The watchdog belongs to the platform layer because every board has some
 * independent reset source; upper layers only need "feed" and "why did I
 * reset". 04_Impl selects the concrete IWDG/WWDG or external device.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_MCU_WDG_H__
#define __PLATFORM_MCU_WDG_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"

//******************************** Declaring ********************************//

typedef enum
{
    PLATFORM_WDG_RESET_REASON_UNKNOWN = 0,
    PLATFORM_WDG_RESET_REASON_POWER_ON,
    PLATFORM_WDG_RESET_REASON_WATCHDOG,
    PLATFORM_WDG_RESET_REASON_SOFTWARE,
    PLATFORM_WDG_RESET_REASON_PIN
} platform_wdg_reset_reason_t;

/*
 * Start the watchdog. timeout_ms must be larger than the worst-case period of
 * the task that feeds it. A value of 0 disables the watchdog if supported.
 */
platform_err_t platform_wdg_init(uint32_t timeout_ms);
platform_err_t platform_wdg_feed(void);
platform_err_t platform_wdg_get_reset_reason(
                                        platform_wdg_reset_reason_t *reason);

#endif /* __PLATFORM_MCU_WDG_H__ */
