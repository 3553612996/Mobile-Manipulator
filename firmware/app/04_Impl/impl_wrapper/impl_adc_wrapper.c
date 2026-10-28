/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_adc_wrapper.c
 *
 * @par dependencies
 * - impl_wrapper/impl_mcu_wrapper.h
 * - impl_wrapper/impl_trace.h
 * - impl_mcu/impl_mcu_adc.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Add trace hooks around the ADC implementation adapter.
 *
 * The wrapper owns instrumentation only. It must not validate parameters,
 * translate units or call vendor APIs; those belong to impl_mcu_adc.c.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_wrapper/impl_mcu_wrapper.h"
#include "impl_wrapper/impl_trace.h"
#include "impl_mcu/impl_mcu_adc.h"

//******************************** Functions ********************************//

platform_err_t impl_adc_init_wrap(const platform_adc_cfg_t *cfg)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_adc_init");
    err = impl_adc_init(cfg);
    IMPL_TRACE_EXIT("platform_adc_init", err);

    return err;
}

platform_err_t impl_adc_deinit_wrap(platform_adc_channel_t channel)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_adc_deinit");
    err = impl_adc_deinit(channel);
    IMPL_TRACE_EXIT("platform_adc_deinit", err);

    return err;
}

platform_err_t impl_adc_read_raw_wrap(platform_adc_channel_t channel,
                                      uint16_t *raw_value)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_adc_read_raw");
    err = impl_adc_read_raw(channel, raw_value);
    IMPL_TRACE_EXIT("platform_adc_read_raw", err);

    return err;
}

platform_err_t impl_adc_read_mv_wrap(platform_adc_channel_t channel,
                                     uint32_t *millivolts)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_adc_read_mv");
    err = impl_adc_read_mv(channel, millivolts);
    IMPL_TRACE_EXIT("platform_adc_read_mv", err);

    return err;
}
