/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_mcu_adc.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform ADC interface.
 *
 * Raw values are right-aligned and use the MCU's native resolution. The
 * millivolt conversion uses reference_mv from the channel configuration, so
 * a board with a different reference (internal/external) only changes 04_Impl
 * configuration, not upper-layer code.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_MCU_ADC_H__
#define __PLATFORM_MCU_ADC_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"

//******************************** Defines **********************************//

#define PLATFORM_ADC_CHANNEL_0      ((platform_adc_channel_t)0u)
#define PLATFORM_ADC_CHANNEL_1      ((platform_adc_channel_t)1u)
#define PLATFORM_ADC_CHANNEL_2      ((platform_adc_channel_t)2u)
#define PLATFORM_ADC_CHANNEL_3      ((platform_adc_channel_t)3u)
#define PLATFORM_ADC_CHANNEL_4      ((platform_adc_channel_t)4u)
#define PLATFORM_ADC_CHANNEL_5      ((platform_adc_channel_t)5u)
#define PLATFORM_ADC_CHANNEL_6      ((platform_adc_channel_t)6u)
#define PLATFORM_ADC_CHANNEL_7      ((platform_adc_channel_t)7u)
#define PLATFORM_ADC_CHANNEL_INVALID ((platform_adc_channel_t)0xFFu)

#define PLATFORM_ADC_REFERENCE_MV_DEFAULT 3300u

//******************************** Declaring ********************************//

typedef uint8_t platform_adc_channel_t;

typedef struct
{
    platform_adc_channel_t  channel;
    uint32_t                reference_mv;   /* 0 -> use board default. */
} platform_adc_cfg_t;

platform_err_t platform_adc_init(const platform_adc_cfg_t *cfg);
platform_err_t platform_adc_deinit(platform_adc_channel_t channel);

platform_err_t platform_adc_read_raw(platform_adc_channel_t channel,
                                     uint16_t *raw_value);
platform_err_t platform_adc_read_mv(platform_adc_channel_t channel,
                                    uint32_t *millivolts);

#endif /* __PLATFORM_MCU_ADC_H__ */
