/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_mcu_encoder.c
 *
 * @par dependencies
 * - platform_mcu_encoder.h
 * - impl_wrapper/impl_mcu_wrapper.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Forward the platform encoder contract to the Impl wrapper.
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

#include "platform_mcu_encoder.h"
#include "impl_wrapper/impl_mcu_wrapper.h"

//******************************** Functions ********************************//

platform_err_t platform_encoder_init(const platform_encoder_cfg_t *cfg)
{
    return impl_encoder_init_wrap(cfg);
}

platform_err_t platform_encoder_deinit(platform_encoder_channel_t channel)
{
    return impl_encoder_deinit_wrap(channel);
}

platform_err_t platform_encoder_read_delta(platform_encoder_channel_t channel,
                                           int16_t *delta)
{
    return impl_encoder_read_delta_wrap(channel, delta);
}

platform_err_t platform_encoder_get_count(platform_encoder_channel_t channel,
                                          int16_t *count)
{
    return impl_encoder_get_count_wrap(channel, count);
}

platform_err_t platform_encoder_reset(platform_encoder_channel_t channel)
{
    return impl_encoder_reset_wrap(channel);
}
