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
 * @brief Pack voltage helpers shared by the fault policy and the reports.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __APP_BATTERY_H__
#define __APP_BATTERY_H__

//******************************** Includes *********************************//

#include "platform_def.h"

//******************************** Declaring ********************************//

/* Pack voltage in mV, or 0 while the monitor has no valid sample yet. */
uint16_t app_battery_mv(void);

/* TRUE only when a valid sample exists and is below APP_BATTERY_LOW_MV. */
platform_bool_t app_battery_low(void);

#endif /* __APP_BATTERY_H__ */
