/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_mcu_pwm.c
 *
 * @par dependencies
 * - impl_mcu/impl_mcu_pwm.h
 * - tim.h (CubeMX-generated handles)
 * - stm32f1xx_hal.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Adapt the platform PWM contract to the STM32F1 general-purpose timers.
 *
 * Eight hardware channels drive the four H-bridges (two PWM inputs each):
 *
 *   0 -> TIM8 CH1 (PC6)     4 -> TIM1 CH1  (PA8)
 *   1 -> TIM8 CH2 (PC7)     5 -> TIM1 CH2N (PB0)
 *   2 -> TIM8 CH3 (PC8)     6 -> TIM1 CH3N (PB1)
 *   3 -> TIM8 CH4 (PC9)     7 -> TIM1 CH4  (PA11)
 *
 * The channel layout follows the vendor board: TIM1 channels 2 and 3 are only
 * routed to their complementary pins, so the "channel" here is the CCR number
 * and the adapter remembers whether the active output is the normal or the
 * inverted one.
 *
 * CubeMX owns the pins, the timer clocks and the period/prescaler defaults
 * (PSC 1, ARR 2000 -> about 18 kHz). set_frequency() recomputes PSC/ARR for the
 * requested rate and re-applies the duty of every channel on that timer,
 * because all channels of a timer share one time base.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_mcu/impl_mcu_pwm.h"
#include "stm32f1xx_hal.h"
#include "tim.h"

//******************************** Defines **********************************//

#define IMPL_PWM_CHANNEL_COUNT      8u

/* TIM1 and TIM8 hang off APB2, whose timer clock is 72 MHz with this clock tree. */
#define IMPL_PWM_TIMER_CLOCK_HZ     72000000u
#define IMPL_PWM_ARR_MAX            65535u
#define IMPL_PWM_PSC_MAX            65535u

//******************************** Declaring ********************************//

typedef struct
{
    TIM_HandleTypeDef  *htim;
    void              (*cube_init)(void);
    uint32_t            hal_channel;    /* TIM_CHANNEL_1 .. TIM_CHANNEL_4 */
    platform_bool_t     complementary;  /* Drive CHxN instead of CHx. */
    uint16_t            duty_permille;
    platform_bool_t     running;
    platform_bool_t     initialized;
} impl_pwm_ctx_t;

/* One entry per timer, kept so that frequency changes can be applied once. */
typedef struct
{
    TIM_HandleTypeDef  *htim;
    void              (*cube_init)(void);
    platform_bool_t     initialized;
    uint32_t            period_ticks;   /* ARR + 1 */
} impl_pwm_timer_ctx_t;

//******************************** Variables ********************************//

static impl_pwm_timer_ctx_t s_pwm_timer[2] =
{
    { &htim1, MX_TIM1_Init, PLATFORM_FALSE, 2001u },
    { &htim8, MX_TIM8_Init, PLATFORM_FALSE, 2001u },
};

static impl_pwm_ctx_t s_pwm[IMPL_PWM_CHANNEL_COUNT] =
{
    { &htim8, MX_TIM8_Init, TIM_CHANNEL_1, PLATFORM_FALSE, 0u, PLATFORM_FALSE, PLATFORM_FALSE },
    { &htim8, MX_TIM8_Init, TIM_CHANNEL_2, PLATFORM_FALSE, 0u, PLATFORM_FALSE, PLATFORM_FALSE },
    { &htim8, MX_TIM8_Init, TIM_CHANNEL_3, PLATFORM_FALSE, 0u, PLATFORM_FALSE, PLATFORM_FALSE },
    { &htim8, MX_TIM8_Init, TIM_CHANNEL_4, PLATFORM_FALSE, 0u, PLATFORM_FALSE, PLATFORM_FALSE },
    { &htim1, MX_TIM1_Init, TIM_CHANNEL_1, PLATFORM_FALSE, 0u, PLATFORM_FALSE, PLATFORM_FALSE },
    { &htim1, MX_TIM1_Init, TIM_CHANNEL_2, PLATFORM_TRUE,  0u, PLATFORM_FALSE, PLATFORM_FALSE },
    { &htim1, MX_TIM1_Init, TIM_CHANNEL_3, PLATFORM_TRUE,  0u, PLATFORM_FALSE, PLATFORM_FALSE },
    { &htim1, MX_TIM1_Init, TIM_CHANNEL_4, PLATFORM_FALSE, 0u, PLATFORM_FALSE, PLATFORM_FALSE },
};

//******************************** Functions ********************************//

static impl_pwm_ctx_t *impl_pwm_get(platform_pwm_channel_t channel)
{
    if (channel >= IMPL_PWM_CHANNEL_COUNT)
    {
        return NULL;
    }

    return &s_pwm[channel];
}

static impl_pwm_timer_ctx_t *impl_pwm_timer_get(TIM_HandleTypeDef *htim)
{
    uint32_t i;

    for (i = 0u; i < ARRAY_SIZE(s_pwm_timer); i++)
    {
        if (s_pwm_timer[i].htim == htim)
        {
            return &s_pwm_timer[i];
        }
    }

    return NULL;
}

/* Make sure the HAL handle exists and the timer clock/pins are configured. */
static void impl_pwm_timer_ensure(TIM_HandleTypeDef *htim,
                                  void (*cube_init)(void))
{
    impl_pwm_timer_ctx_t *timer;

    if (htim->State != HAL_TIM_STATE_RESET)
    {
        return;
    }

    cube_init();

    timer = impl_pwm_timer_get(htim);
    if (timer != NULL)
    {
        timer->initialized  = PLATFORM_TRUE;
        timer->period_ticks = (uint32_t)__HAL_TIM_GET_AUTORELOAD(htim) + 1u;
    }
}

/*
 * The CCR value is the same for the normal and the complementary output; only
 * the start/stop call differs, which is why this helper does not branch.
 */
static void impl_pwm_write_compare(impl_pwm_ctx_t *ctx, uint32_t ticks)
{
    __HAL_TIM_SET_COMPARE(ctx->htim, ctx->hal_channel, ticks);
}

static platform_err_t impl_pwm_apply_duty(impl_pwm_ctx_t *ctx,
                                          uint16_t duty_permille)
{
    impl_pwm_timer_ctx_t *timer = impl_pwm_timer_get(ctx->htim);
    uint32_t              period;
    uint32_t              ticks;

    period = (timer != NULL) ? timer->period_ticks
                             : ((uint32_t)__HAL_TIM_GET_AUTORELOAD(ctx->htim) + 1u);

    ticks = ((uint32_t)duty_permille * period) / PLATFORM_PWM_DUTY_PERMILLE_MAX;

    /*
     * A CCR equal to the period would keep the output high level for a whole
     * cycle; clamp it to period - 1 so 100% is represented the way the HAL
     * expects and the counter still generates the update event.
     */
    if (ticks >= period)
    {
        ticks = period - 1u;
    }

    impl_pwm_write_compare(ctx, ticks);
    ctx->duty_permille = duty_permille;

    return PLATFORM_ERR_OK;
}

platform_err_t impl_pwm_init(const platform_pwm_cfg_t *cfg)
{
    impl_pwm_ctx_t *ctx;

    if (cfg == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    ctx = impl_pwm_get(cfg->channel);
    if ((ctx == NULL) ||
        (cfg->duty_permille > PLATFORM_PWM_DUTY_PERMILLE_MAX))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (ctx->initialized == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    impl_pwm_timer_ensure(ctx->htim, ctx->cube_init);

    if (cfg->frequency_hz != 0u)
    {
        (void)impl_pwm_set_frequency(cfg->channel, cfg->frequency_hz);
    }

    (void)impl_pwm_apply_duty(ctx, cfg->duty_permille);

    ctx->initialized = PLATFORM_TRUE;

    if (cfg->start_immediately == PLATFORM_TRUE)
    {
        return impl_pwm_start(cfg->channel);
    }

    return PLATFORM_ERR_OK;
}

platform_err_t impl_pwm_deinit(platform_pwm_channel_t channel)
{
    impl_pwm_ctx_t *ctx = impl_pwm_get(channel);

    if (ctx == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (ctx->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    (void)impl_pwm_stop(channel);
    ctx->initialized   = PLATFORM_FALSE;
    ctx->duty_permille = 0u;

    return PLATFORM_ERR_OK;
}

platform_err_t impl_pwm_start(platform_pwm_channel_t channel)
{
    impl_pwm_ctx_t *ctx = impl_pwm_get(channel);

    if (ctx == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (ctx->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (ctx->running == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_OK;
    }

    if (ctx->complementary == PLATFORM_TRUE)
    {
        if (HAL_TIMEx_PWMN_Start(ctx->htim, ctx->hal_channel) != HAL_OK)
        {
            return PLATFORM_ERR_FAIL;
        }
    }
    else
    {
        if (HAL_TIM_PWM_Start(ctx->htim, ctx->hal_channel) != HAL_OK)
        {
            return PLATFORM_ERR_FAIL;
        }
    }

    ctx->running = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t impl_pwm_stop(platform_pwm_channel_t channel)
{
    impl_pwm_ctx_t *ctx = impl_pwm_get(channel);

    if (ctx == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (ctx->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (ctx->running != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_OK;
    }

    if (ctx->complementary == PLATFORM_TRUE)
    {
        (void)HAL_TIMEx_PWMN_Stop(ctx->htim, ctx->hal_channel);
    }
    else
    {
        (void)HAL_TIM_PWM_Stop(ctx->htim, ctx->hal_channel);
    }

    ctx->running = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

platform_err_t impl_pwm_set_duty(platform_pwm_channel_t channel,
                                 uint16_t duty_permille)
{
    impl_pwm_ctx_t *ctx = impl_pwm_get(channel);

    if (ctx == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (ctx->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (duty_permille > PLATFORM_PWM_DUTY_PERMILLE_MAX)
    {
        return PLATFORM_ERR_PARAM;
    }

    return impl_pwm_apply_duty(ctx, duty_permille);
}

platform_err_t impl_pwm_set_frequency(platform_pwm_channel_t channel,
                                      uint32_t frequency_hz)
{
    impl_pwm_ctx_t       *ctx = impl_pwm_get(channel);
    impl_pwm_timer_ctx_t *timer;
    uint32_t              prescaler;
    uint32_t              period;
    uint32_t              i;

    if (ctx == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if ((frequency_hz == 0u) || (frequency_hz > (IMPL_PWM_TIMER_CLOCK_HZ / 2u)))
    {
        return PLATFORM_ERR_PARAM;
    }

    /* The timer may not be initialized yet when init() applies the frequency. */
    if (ctx->initialized != PLATFORM_TRUE)
    {
        impl_pwm_timer_ensure(ctx->htim, ctx->cube_init);
    }

    timer = impl_pwm_timer_get(ctx->htim);
    if (timer == NULL)
    {
        return PLATFORM_ERR_FAIL;
    }

    /*
     * Keep 1 tick of resolution by choosing the smallest prescaler that still
     * fits the 16-bit auto-reload register.
     */
    prescaler = IMPL_PWM_TIMER_CLOCK_HZ / (frequency_hz * 65536u);
    if (prescaler == 0u)
    {
        prescaler = 1u;
    }
    else if (prescaler > IMPL_PWM_PSC_MAX)
    {
        return PLATFORM_ERR_PARAM;
    }

    period = IMPL_PWM_TIMER_CLOCK_HZ / (frequency_hz * prescaler);
    if ((period == 0u) || (period > IMPL_PWM_ARR_MAX + 1u))
    {
        return PLATFORM_ERR_PARAM;
    }

    __HAL_TIM_DISABLE(ctx->htim);
    __HAL_TIM_SET_PRESCALER(ctx->htim, prescaler - 1u);
    __HAL_TIM_SET_AUTORELOAD(ctx->htim, period - 1u);
    __HAL_TIM_SET_COUNTER(ctx->htim, 0u);
    timer->period_ticks = period;

    /* All channels of this timer share the time base, so refresh them all. */
    for (i = 0u; i < IMPL_PWM_CHANNEL_COUNT; i++)
    {
        if ((s_pwm[i].htim == ctx->htim) &&
            (s_pwm[i].initialized == PLATFORM_TRUE))
        {
            (void)impl_pwm_apply_duty(&s_pwm[i], s_pwm[i].duty_permille);
        }
    }

    __HAL_TIM_ENABLE(ctx->htim);

    return PLATFORM_ERR_OK;
}

platform_err_t impl_pwm_get_duty(platform_pwm_channel_t channel,
                                 uint16_t *duty_permille)
{
    impl_pwm_ctx_t *ctx = impl_pwm_get(channel);

    if ((ctx == NULL) || (duty_permille == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (ctx->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    *duty_permille = ctx->duty_permille;

    return PLATFORM_ERR_OK;
}
