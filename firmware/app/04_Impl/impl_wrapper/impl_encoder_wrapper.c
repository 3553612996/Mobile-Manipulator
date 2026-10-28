/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_encoder_wrapper.c
 *
 * @par dependencies
 * - impl_wrapper/impl_mcu_wrapper.h
 * - impl_wrapper/impl_trace.h
 * - impl_mcu/impl_mcu_encoder.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Add trace hooks around the encoder implementation adapter.
 *
 * The wrapper owns instrumentation only. It must not validate parameters,
 * translate units or call vendor APIs; those belong to impl_mcu_encoder.c.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_wrapper/impl_mcu_wrapper.h"
#include "impl_wrapper/impl_trace.h"
#include "impl_mcu/impl_mcu_encoder.h"

//******************************** Functions ********************************//

platform_err_t impl_encoder_init_wrap(const platform_encoder_cfg_t *cfg)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_encoder_init");
    err = impl_encoder_init(cfg);
    IMPL_TRACE_EXIT("platform_encoder_init", err);

    return err;
}

platform_err_t impl_encoder_deinit_wrap(platform_encoder_channel_t channel)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_encoder_deinit");
    err = impl_encoder_deinit(channel);
    IMPL_TRACE_EXIT("platform_encoder_deinit", err);

    return err;
}

platform_err_t impl_encoder_read_delta_wrap(platform_encoder_channel_t channel,
                                            int16_t *delta)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_encoder_read_delta");
    err = impl_encoder_read_delta(channel, delta);
    IMPL_TRACE_EXIT("platform_encoder_read_delta", err);

    return err;
}

platform_err_t impl_encoder_get_count_wrap(platform_encoder_channel_t channel,
                                           int16_t *count)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_encoder_get_count");
    err = impl_encoder_get_count(channel, count);
    IMPL_TRACE_EXIT("platform_encoder_get_count", err);

    return err;
}

platform_err_t impl_encoder_reset_wrap(platform_encoder_channel_t channel)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_encoder_reset");
    err = impl_encoder_reset(channel);
    IMPL_TRACE_EXIT("platform_encoder_reset", err);

    return err;
}
