/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_mcu_timer.h
 *
 * @par dependencies
 * - platform_mcu_timer.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Declare the one-shot interval-timer implementation adapter.
 *
 * These functions contain the microsecond-to-tick translation and the vendor
 * API calls. The Impl wrapper is the only caller. Do not include this header
 * from 03_Platform or any layer above it.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __IMPL_MCU_TIMER_H__
#define __IMPL_MCU_TIMER_H__

//******************************** Includes *********************************//

#include "platform_mcu_timer.h"
#include "stm32f1xx_hal.h"

//******************************** Declaring ********************************//

/*
 * Forward a HAL timer update to the one-shot adapter. main() owns the global
 * HAL_TIM_PeriodElapsedCallback() because the same function serves the HAL
 * timebase (TIM6) and this adapter (TIM7); it calls this hook for every timer.
 */
void impl_timer_period_elapsed(TIM_HandleTypeDef *htim);

platform_err_t impl_timer_init(const platform_timer_cfg_t *cfg);
platform_err_t impl_timer_deinit(platform_timer_id_t timer);

platform_err_t impl_timer_start(platform_timer_id_t timer);
platform_err_t impl_timer_stop(platform_timer_id_t timer);

platform_err_t impl_timer_set_next_interval_us(platform_timer_id_t timer,
                                               uint32_t interval_us);

#endif /* __IMPL_MCU_TIMER_H__ */
