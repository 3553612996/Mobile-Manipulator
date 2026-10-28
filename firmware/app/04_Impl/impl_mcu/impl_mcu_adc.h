/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_mcu_adc.h
 *
 * @par dependencies
 * - platform_mcu_adc.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Declare the ADC implementation adapter.
 *
 * These functions contain the raw-to-millivolt translation and the vendor API
 * calls. The Impl wrapper is the only caller. Do not include this header from
 * 03_Platform or any layer above it.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __IMPL_MCU_ADC_H__
#define __IMPL_MCU_ADC_H__

//******************************** Includes *********************************//

#include "platform_mcu_adc.h"

//******************************** Declaring ********************************//

platform_err_t impl_adc_init(const platform_adc_cfg_t *cfg);
platform_err_t impl_adc_deinit(platform_adc_channel_t channel);

platform_err_t impl_adc_read_raw(platform_adc_channel_t channel,
                                 uint16_t *raw_value);
platform_err_t impl_adc_read_mv(platform_adc_channel_t channel,
                                uint32_t *millivolts);

#endif /* __IMPL_MCU_ADC_H__ */
