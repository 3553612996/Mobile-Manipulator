/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_pwm_wrapper.c
 *
 * @par dependencies
 * - impl_wrapper/impl_mcu_wrapper.h
 * - impl_wrapper/impl_trace.h
 * - impl_mcu/impl_mcu_pwm.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Add trace hooks around the PWM implementation adapter.
 *
 * The wrapper owns instrumentation only. It must not validate parameters,
 * translate units or call vendor APIs; those belong to impl_mcu_pwm.c.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_wrapper/impl_mcu_wrapper.h"
#include "impl_wrapper/impl_trace.h"
#include "impl_mcu/impl_mcu_pwm.h"

//******************************** Functions ********************************//

platform_err_t impl_pwm_init_wrap(const platform_pwm_cfg_t *cfg)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_pwm_init");
    err = impl_pwm_init(cfg);
    IMPL_TRACE_EXIT("platform_pwm_init", err);

    return err;
}

platform_err_t impl_pwm_deinit_wrap(platform_pwm_channel_t channel)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_pwm_deinit");
    err = impl_pwm_deinit(channel);
    IMPL_TRACE_EXIT("platform_pwm_deinit", err);

    return err;
}

platform_err_t impl_pwm_start_wrap(platform_pwm_channel_t channel)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_pwm_start");
    err = impl_pwm_start(channel);
    IMPL_TRACE_EXIT("platform_pwm_start", err);

    return err;
}

platform_err_t impl_pwm_stop_wrap(platform_pwm_channel_t channel)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_pwm_stop");
    err = impl_pwm_stop(channel);
    IMPL_TRACE_EXIT("platform_pwm_stop", err);

    return err;
}

platform_err_t impl_pwm_set_duty_wrap(platform_pwm_channel_t channel,
                                      uint16_t duty_permille)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_pwm_set_duty");
    err = impl_pwm_set_duty(channel, duty_permille);
    IMPL_TRACE_EXIT("platform_pwm_set_duty", err);

    return err;
}

platform_err_t impl_pwm_set_frequency_wrap(platform_pwm_channel_t channel,
                                            uint32_t frequency_hz)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_pwm_set_frequency");
    err = impl_pwm_set_frequency(channel, frequency_hz);
    IMPL_TRACE_EXIT("platform_pwm_set_frequency", err);

    return err;
}

platform_err_t impl_pwm_get_duty_wrap(platform_pwm_channel_t channel,
                                      uint16_t *duty_permille)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_pwm_get_duty");
    err = impl_pwm_get_duty(channel, duty_permille);
    IMPL_TRACE_EXIT("platform_pwm_get_duty", err);

    return err;
}
