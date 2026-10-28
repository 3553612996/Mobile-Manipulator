/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_mcu_pwm.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform PWM interface.
 *
 * A PWM channel is a logical resource, not a timer number. 04_Impl maps it to
 * (TIMx, channel, pin) and owns the clock/alternate-function setup.
 *
 * Duty is expressed in permille (0..1000) instead of float so that the same
 * value is exact on every compiler and easy to print in debug logs.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_MCU_PWM_H__
#define __PLATFORM_MCU_PWM_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"

//******************************** Defines **********************************//

#define PLATFORM_PWM_CHANNEL_0      ((platform_pwm_channel_t)0u)
#define PLATFORM_PWM_CHANNEL_1      ((platform_pwm_channel_t)1u)
#define PLATFORM_PWM_CHANNEL_2      ((platform_pwm_channel_t)2u)
#define PLATFORM_PWM_CHANNEL_3      ((platform_pwm_channel_t)3u)
#define PLATFORM_PWM_CHANNEL_4      ((platform_pwm_channel_t)4u)
#define PLATFORM_PWM_CHANNEL_5      ((platform_pwm_channel_t)5u)
#define PLATFORM_PWM_CHANNEL_6      ((platform_pwm_channel_t)6u)
#define PLATFORM_PWM_CHANNEL_7      ((platform_pwm_channel_t)7u)
#define PLATFORM_PWM_CHANNEL_INVALID ((platform_pwm_channel_t)0xFFu)

#define PLATFORM_PWM_DUTY_PERMILLE_MIN  ((uint16_t)0u)
#define PLATFORM_PWM_DUTY_PERMILLE_MAX  ((uint16_t)1000u)

//******************************** Declaring ********************************//

typedef uint8_t platform_pwm_channel_t;

typedef struct
{
    platform_pwm_channel_t  channel;
    uint32_t                frequency_hz;
    uint16_t                duty_permille;      /* 0..1000. */
    platform_bool_t         start_immediately;
} platform_pwm_cfg_t;

platform_err_t platform_pwm_init(const platform_pwm_cfg_t *cfg);
platform_err_t platform_pwm_deinit(platform_pwm_channel_t channel);

platform_err_t platform_pwm_start(platform_pwm_channel_t channel);
platform_err_t platform_pwm_stop(platform_pwm_channel_t channel);

platform_err_t platform_pwm_set_duty(platform_pwm_channel_t channel,
                                     uint16_t duty_permille);
platform_err_t platform_pwm_set_frequency(platform_pwm_channel_t channel,
                                          uint32_t frequency_hz);
platform_err_t platform_pwm_get_duty(platform_pwm_channel_t channel,
                                     uint16_t *duty_permille);

#endif /* __PLATFORM_MCU_PWM_H__ */
