/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file bsp_battery_adc.h
 *
 * @par dependencies
 * - platform_mcu_adc.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Battery voltage from a resistor divider measured by the ADC.
 *
 * The board divides the pack voltage by 11 before it reaches ADC1_IN13, so the
 * driver converts the measured millivolts back to the pack voltage with the
 * configured ratio. The ratio is a board property and stays here; upper layers
 * only see millivolts.
 *
 * Averaging belongs to the caller (platform_filter) so that the status task can
 * choose the window without blocking the ADC for it.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __BSP_BATTERY_ADC_H__
#define __BSP_BATTERY_ADC_H__

//******************************** Includes *********************************//

#include "platform_mcu_adc.h"

//******************************** Declaring ********************************//

typedef struct
{
    platform_adc_channel_t channel;
    uint16_t               divider_ratio;   /* Pack voltage = ADC mV * ratio. */
    uint32_t               reference_mv;    /* 0 -> platform default. */
} bsp_battery_adc_cfg_t;

typedef struct
{
    bsp_battery_adc_cfg_t cfg;
    uint32_t              voltage_mv;
    uint16_t              raw;
    platform_bool_t       valid;
    platform_bool_t       initialized;
} bsp_battery_adc_t;

//******************************** Functions ********************************//

platform_err_t bsp_battery_adc_init(bsp_battery_adc_t *battery);
platform_err_t bsp_battery_adc_deinit(bsp_battery_adc_t *battery);

/*
 * Take one sample and update the cached voltage. The platform ADC exposes the
 * raw count and the millivolt conversion as two calls, so this performs two
 * conversions a few microseconds apart; the raw value is for diagnostics only.
 * It blocks for the conversion time and must not run in an ISR.
 */
platform_err_t bsp_battery_adc_read(bsp_battery_adc_t *battery);

uint32_t bsp_battery_adc_get_mv(const bsp_battery_adc_t *battery);

#endif /* __BSP_BATTERY_ADC_H__ */
