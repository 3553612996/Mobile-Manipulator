/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_battery.h
 *
 * @par dependencies
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Pack voltage filtering and the debounced low-battery decision.
 *
 * A single ADC sample is not a battery state. Motor inrush pulls the pack down
 * for a few tens to a few hundred milliseconds, so the decision is made on a
 * median of the last APP_BATTERY_FILTER_WINDOW samples, and only after
 * APP_BATTERY_LOW_COUNT consecutive medians stay below APP_BATTERY_LOW_MV.
 *
 * app_battery_tick() owns the sampling; call it from the periodic task before
 * anyone reads app_battery_mv() / app_battery_low().
 *
 * @version V1.1 2026-11-05
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __APP_BATTERY_H__
#define __APP_BATTERY_H__

//******************************** Includes *********************************//

#include "platform_def.h"

//******************************** Declaring ********************************//

/* Reset the filter window and the debounce counter. */
void app_battery_init(void);

/* Sample the monitor at APP_BATTERY_SAMPLE_PERIOD_MS and update the filter. */
void app_battery_tick(uint32_t now_ms);

/*
 * Filtered pack voltage in mV. Returns the median once the window is full,
 * the latest good sample while it fills, and 0 before any valid sample.
 */
uint16_t app_battery_mv(void);

/* TRUE only after APP_BATTERY_LOW_COUNT consecutive filtered lows. */
platform_bool_t app_battery_low(void);

#endif /* __APP_BATTERY_H__ */
