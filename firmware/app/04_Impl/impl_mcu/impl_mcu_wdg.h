/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_mcu_wdg.h
 *
 * @par dependencies
 * - platform_mcu_wdg.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Declare the watchdog implementation adapter.
 *
 * These functions contain the timeout-to-prescaler translation and the vendor
 * API calls. The Impl wrapper is the only caller. Do not include this header
 * from 03_Platform or any layer above it.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __IMPL_MCU_WDG_H__
#define __IMPL_MCU_WDG_H__

//******************************** Includes *********************************//

#include "platform_mcu_wdg.h"

//******************************** Declaring ********************************//

platform_err_t impl_wdg_init(uint32_t timeout_ms);
platform_err_t impl_wdg_feed(void);
platform_err_t impl_wdg_get_reset_reason(
                                        platform_wdg_reset_reason_t *reason);

#endif /* __IMPL_MCU_WDG_H__ */
