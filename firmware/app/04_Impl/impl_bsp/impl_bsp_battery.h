/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_bsp_battery.h
 *
 * @par dependencies
 * - platform_bsp_battery.h
 * - 05_BSP/bsp_battery_adc.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Ops adapter for the platform battery-monitor contract.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __IMPL_BSP_BATTERY_H__
#define __IMPL_BSP_BATTERY_H__

//******************************** Includes *********************************//

#include "platform_bsp_battery.h"

//******************************** Declaring ********************************//

/*
 * Pass this table and a bsp_battery_adc_t * to platform_battery_register().
 * The instance's cfg (ADC channel and divider ratio) must be filled first.
 */
extern const platform_battery_ops_t impl_bsp_battery_ops;

#endif /* __IMPL_BSP_BATTERY_H__ */
