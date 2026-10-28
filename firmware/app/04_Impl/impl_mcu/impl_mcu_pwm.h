/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_mcu_pwm.h
 *
 * @par dependencies
 * - platform_mcu_pwm.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Declare the PWM implementation adapter.
 *
 * These functions contain the permille-to-CCR translation and the vendor API
 * calls. The Impl wrapper is the only caller. Do not include this header from
 * 03_Platform or any layer above it.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __IMPL_MCU_PWM_H__
#define __IMPL_MCU_PWM_H__

//******************************** Includes *********************************//

#include "platform_mcu_pwm.h"

//******************************** Declaring ********************************//

platform_err_t impl_pwm_init(const platform_pwm_cfg_t *cfg);
platform_err_t impl_pwm_deinit(platform_pwm_channel_t channel);

platform_err_t impl_pwm_start(platform_pwm_channel_t channel);
platform_err_t impl_pwm_stop(platform_pwm_channel_t channel);

platform_err_t impl_pwm_set_duty(platform_pwm_channel_t channel,
                                 uint16_t duty_permille);
platform_err_t impl_pwm_set_frequency(platform_pwm_channel_t channel,
                                      uint32_t frequency_hz);
platform_err_t impl_pwm_get_duty(platform_pwm_channel_t channel,
                                 uint16_t *duty_permille);

#endif /* __IMPL_MCU_PWM_H__ */
