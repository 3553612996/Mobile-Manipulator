/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_mcu_pwm.c
 *
 * @par dependencies
 * - platform_mcu_pwm.h
 * - impl_wrapper/impl_mcu_wrapper.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Forward the platform PWM contract to the Impl wrapper.
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

#include "platform_mcu_pwm.h"
#include "impl_wrapper/impl_mcu_wrapper.h"

//******************************** Functions ********************************//

platform_err_t platform_pwm_init(const platform_pwm_cfg_t *cfg)
{
    return impl_pwm_init_wrap(cfg);
}

platform_err_t platform_pwm_deinit(platform_pwm_channel_t channel)
{
    return impl_pwm_deinit_wrap(channel);
}

platform_err_t platform_pwm_start(platform_pwm_channel_t channel)
{
    return impl_pwm_start_wrap(channel);
}

platform_err_t platform_pwm_stop(platform_pwm_channel_t channel)
{
    return impl_pwm_stop_wrap(channel);
}

platform_err_t platform_pwm_set_duty(platform_pwm_channel_t channel,
                                     uint16_t duty_permille)
{
    return impl_pwm_set_duty_wrap(channel, duty_permille);
}

platform_err_t platform_pwm_set_frequency(platform_pwm_channel_t channel,
                                          uint32_t frequency_hz)
{
    return impl_pwm_set_frequency_wrap(channel, frequency_hz);
}

platform_err_t platform_pwm_get_duty(platform_pwm_channel_t channel,
                                     uint16_t *duty_permille)
{
    return impl_pwm_get_duty_wrap(channel, duty_permille);
}
