/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_mcu_adc.c
 *
 * @par dependencies
 * - impl_mcu/impl_mcu_adc.h
 * - adc.h (CubeMX-generated handle)
 * - stm32f1xx_hal.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Adapt the platform ADC contract to the STM32F1 ADC in polling mode.
 *
 * The board only measures the battery divider (ADC1_IN13 on PC3), so there is
 * one hardware channel and one platform channel mapping to it. CubeMX owns the
 * pin, the conversion mode (single, software trigger), the sampling time and
 * the ADC prescaler (PCLK2 / 6 = 12 MHz, below the 14 MHz limit).
 *
 * A read is start/convert/stop because the platform contract is synchronous;
 * the vendor's continuous-DMA scheme is not needed for a value that is
 * sampled by a low-rate status task.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_mcu/impl_mcu_adc.h"
#include "stm32f1xx_hal.h"
#include "adc.h"

//******************************** Defines **********************************//

#define IMPL_ADC_CHANNEL_COUNT      1u
#define IMPL_ADC_FULL_SCALE         4095u
#define IMPL_ADC_TIMEOUT_MS         5u

//******************************** Declaring ********************************//

typedef struct
{
    ADC_HandleTypeDef  *hadc;
    void              (*cube_init)(void);
    uint32_t            reference_mv;
    platform_bool_t     initialized;
} impl_adc_ctx_t;

//******************************** Variables ********************************//

static impl_adc_ctx_t s_adc[IMPL_ADC_CHANNEL_COUNT] =
{
    { &hadc1, MX_ADC1_Init, PLATFORM_ADC_REFERENCE_MV_DEFAULT, PLATFORM_FALSE },
};

//******************************** Functions ********************************//

static impl_adc_ctx_t *impl_adc_get(platform_adc_channel_t channel)
{
    if (channel >= IMPL_ADC_CHANNEL_COUNT)
    {
        return NULL;
    }

    return &s_adc[channel];
}

platform_err_t impl_adc_init(const platform_adc_cfg_t *cfg)
{
    impl_adc_ctx_t *ctx;

    if (cfg == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    ctx = impl_adc_get(cfg->channel);
    if (ctx == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (ctx->initialized == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    if (ctx->hadc->State == HAL_ADC_STATE_RESET)
    {
        ctx->cube_init();
    }

    /*
     * An F1 ADC needs a calibration after power-up and before the first
     * conversion; it is cheap enough to repeat whenever the channel is
     * initialized.
     */
    (void)HAL_ADCEx_Calibration_Start(ctx->hadc);

    ctx->reference_mv = (cfg->reference_mv == 0u)
                        ? PLATFORM_ADC_REFERENCE_MV_DEFAULT
                        : cfg->reference_mv;
    ctx->initialized  = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t impl_adc_deinit(platform_adc_channel_t channel)
{
    impl_adc_ctx_t *ctx = impl_adc_get(channel);

    if (ctx == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (ctx->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    (void)HAL_ADC_Stop(ctx->hadc);
    (void)HAL_ADC_DeInit(ctx->hadc);

    ctx->initialized = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

platform_err_t impl_adc_read_raw(platform_adc_channel_t channel,
                                 uint16_t *raw_value)
{
    impl_adc_ctx_t *ctx = impl_adc_get(channel);

    if ((ctx == NULL) || (raw_value == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (ctx->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (HAL_ADC_Start(ctx->hadc) != HAL_OK)
    {
        return PLATFORM_ERR_FAIL;
    }

    if (HAL_ADC_PollForConversion(ctx->hadc, IMPL_ADC_TIMEOUT_MS) != HAL_OK)
    {
        (void)HAL_ADC_Stop(ctx->hadc);
        return PLATFORM_ERR_TIMEOUT;
    }

    *raw_value = (uint16_t)HAL_ADC_GetValue(ctx->hadc);

    (void)HAL_ADC_Stop(ctx->hadc);

    return PLATFORM_ERR_OK;
}

platform_err_t impl_adc_read_mv(platform_adc_channel_t channel,
                                uint32_t *millivolts)
{
    impl_adc_ctx_t *ctx = impl_adc_get(channel);
    uint16_t        raw;
    platform_err_t  err;

    if (ctx == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (millivolts == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    err = impl_adc_read_raw(channel, &raw);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    *millivolts = ((uint32_t)raw * ctx->reference_mv) / IMPL_ADC_FULL_SCALE;

    return PLATFORM_ERR_OK;
}
