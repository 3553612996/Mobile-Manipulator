/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_mcu_encoder.c
 *
 * @par dependencies
 * - impl_mcu/impl_mcu_encoder.h
 * - tim.h (CubeMX-generated handles)
 * - stm32f1xx_hal.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Adapt the platform quadrature-encoder contract to the STM32 timer
 *        encoder interface.
 *
 * CubeMX owns everything that is fixed at build time: the TIM clock, the two
 * input pins in alternate-function input mode and the NVIC line are generated
 * in tim.c/tim.h. This adapter only applies what the platform contract makes
 * run-time configurable (input filter, counting direction), starts the counter
 * and translates the 16-bit hardware counter into a signed delta.
 *
 * Channel map: 0 -> TIM2, 1 -> TIM3, 2 -> TIM4, 3 -> TIM5.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_mcu/impl_mcu_encoder.h"
#include "stm32f1xx_hal.h"
#include "tim.h"

//******************************** Defines **********************************//

#define IMPL_ENCODER_CHANNEL_COUNT  4u

//******************************** Declaring ********************************//

typedef struct
{
    TIM_HandleTypeDef  *htim;
    void              (*cube_init)(void);
    uint8_t             filter;
    platform_bool_t     invert;
    platform_bool_t     initialized;
} impl_encoder_ctx_t;

//******************************** Variables ********************************//

/*
 * The handles are owned by CubeMX (tim.c). Keeping them in a table lets the
 * rest of this file stay channel-numbered, exactly like the platform contract.
 */
static impl_encoder_ctx_t s_encoder[IMPL_ENCODER_CHANNEL_COUNT] =
{
    { &htim2, MX_TIM2_Init, 0u, PLATFORM_FALSE, PLATFORM_FALSE },
    { &htim3, MX_TIM3_Init, 0u, PLATFORM_FALSE, PLATFORM_FALSE },
    { &htim4, MX_TIM4_Init, 0u, PLATFORM_FALSE, PLATFORM_FALSE },
    { &htim5, MX_TIM5_Init, 0u, PLATFORM_FALSE, PLATFORM_FALSE },
};

//******************************** Functions ********************************//

static impl_encoder_ctx_t *impl_encoder_get(platform_encoder_channel_t channel)
{
    if (channel >= IMPL_ENCODER_CHANNEL_COUNT)
    {
        return NULL;
    }

    return &s_encoder[channel];
}

platform_err_t impl_encoder_init(const platform_encoder_cfg_t *cfg)
{
    TIM_Encoder_InitTypeDef       s_config = { 0 };
    impl_encoder_ctx_t           *ctx;
    uint32_t                      polarity;

    if (cfg == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    ctx = impl_encoder_get(cfg->channel);
    if ((ctx == NULL) || (cfg->filter > PLATFORM_ENCODER_FILTER_MAX))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (ctx->initialized == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    /* Clock, GPIO alternate function and GPIO pull-ups are CubeMX-owned. */
    ctx->cube_init();

    polarity                  = (cfg->invert == PLATFORM_TRUE)
                              ? TIM_ICPOLARITY_FALLING
                              : TIM_ICPOLARITY_RISING;
    s_config.EncoderMode      = TIM_ENCODERMODE_TI12;
    s_config.IC1Polarity      = polarity;
    s_config.IC1Selection     = TIM_ICSELECTION_DIRECTTI;
    s_config.IC1Prescaler     = TIM_ICPSC_DIV1;
    s_config.IC1Filter        = cfg->filter;
    s_config.IC2Polarity      = polarity;
    s_config.IC2Selection     = TIM_ICSELECTION_DIRECTTI;
    s_config.IC2Prescaler     = TIM_ICPSC_DIV1;
    s_config.IC2Filter        = cfg->filter;

    /*
     * The F1 HAL has no per-channel encoder configuration: the filter and the
     * polarity of both inputs are applied by HAL_TIM_Encoder_Init(), which also
     * reprograms the base from htim->Init (CubeMX values survive).
     */
    if (HAL_TIM_Encoder_Init(ctx->htim, &s_config) != HAL_OK)
    {
        return PLATFORM_ERR_FAIL;
    }

    __HAL_TIM_SET_COUNTER(ctx->htim, 0u);

    if (HAL_TIM_Encoder_Start(ctx->htim, TIM_CHANNEL_ALL) != HAL_OK)
    {
        return PLATFORM_ERR_FAIL;
    }

    ctx->filter      = cfg->filter;
    ctx->invert      = cfg->invert;
    ctx->initialized = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t impl_encoder_deinit(platform_encoder_channel_t channel)
{
    impl_encoder_ctx_t *ctx = impl_encoder_get(channel);

    if (ctx == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (ctx->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    (void)HAL_TIM_Encoder_Stop(ctx->htim, TIM_CHANNEL_ALL);
    (void)HAL_TIM_Encoder_DeInit(ctx->htim);

    ctx->initialized = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

platform_err_t impl_encoder_read_delta(platform_encoder_channel_t channel,
                                       int16_t *delta)
{
    impl_encoder_ctx_t *ctx = impl_encoder_get(channel);
    uint16_t            count;

    if ((ctx == NULL) || (delta == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (ctx->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    /*
     * The counter is 16 bit. Casting it to int16_t turns a wrap into the
     * signed distance the encoder really travelled, as long as the caller
     * reads often enough to stay inside +-32767 counts.
     */
    count = (uint16_t)__HAL_TIM_GET_COUNTER(ctx->htim);
    __HAL_TIM_SET_COUNTER(ctx->htim, 0u);

    *delta = (int16_t)count;

    return PLATFORM_ERR_OK;
}

platform_err_t impl_encoder_get_count(platform_encoder_channel_t channel,
                                      int16_t *count)
{
    impl_encoder_ctx_t *ctx = impl_encoder_get(channel);

    if ((ctx == NULL) || (count == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (ctx->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    *count = (int16_t)(uint16_t)__HAL_TIM_GET_COUNTER(ctx->htim);

    return PLATFORM_ERR_OK;
}

platform_err_t impl_encoder_reset(platform_encoder_channel_t channel)
{
    impl_encoder_ctx_t *ctx = impl_encoder_get(channel);

    if (ctx == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (ctx->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    __HAL_TIM_SET_COUNTER(ctx->htim, 0u);

    return PLATFORM_ERR_OK;
}
