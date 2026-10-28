/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_mcu_adc.c
 *
 * @par dependencies
 * - platform_mcu_adc.h
 * - impl_wrapper/impl_mcu_wrapper.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Forward the platform ADC contract to the Impl wrapper.
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

#include "platform_mcu_adc.h"
#include "impl_wrapper/impl_mcu_wrapper.h"

//******************************** Functions ********************************//

platform_err_t platform_adc_init(const platform_adc_cfg_t *cfg)
{
    return impl_adc_init_wrap(cfg);
}

platform_err_t platform_adc_deinit(platform_adc_channel_t channel)
{
    return impl_adc_deinit_wrap(channel);
}

platform_err_t platform_adc_read_raw(platform_adc_channel_t channel,
                                     uint16_t *raw_value)
{
    return impl_adc_read_raw_wrap(channel, raw_value);
}

platform_err_t platform_adc_read_mv(platform_adc_channel_t channel,
                                    uint32_t *millivolts)
{
    return impl_adc_read_mv_wrap(channel, millivolts);
}
