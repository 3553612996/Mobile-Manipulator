/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_mcu_encoder.h
 *
 * @par dependencies
 * - platform_mcu_encoder.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Declare the quadrature-encoder implementation adapter.
 *
 * These functions own the timer-to-channel mapping and the vendor API calls.
 * The Impl wrapper is the only caller. Do not include this header from
 * 03_Platform or any layer above it.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __IMPL_MCU_ENCODER_H__
#define __IMPL_MCU_ENCODER_H__

//******************************** Includes *********************************//

#include "platform_mcu_encoder.h"

//******************************** Declaring ********************************//

platform_err_t impl_encoder_init(const platform_encoder_cfg_t *cfg);
platform_err_t impl_encoder_deinit(platform_encoder_channel_t channel);

platform_err_t impl_encoder_read_delta(platform_encoder_channel_t channel,
                                       int16_t *delta);
platform_err_t impl_encoder_get_count(platform_encoder_channel_t channel,
                                      int16_t *count);
platform_err_t impl_encoder_reset(platform_encoder_channel_t channel);

#endif /* __IMPL_MCU_ENCODER_H__ */
