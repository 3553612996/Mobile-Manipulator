/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_mcu_timer.c
 *
 * @par dependencies
 * - impl_mcu/impl_mcu_timer.h
 * - tim.h (CubeMX-generated handles)
 * - stm32f1xx_hal.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Adapt the platform one-shot interval-timer contract to a basic timer.
 *
 * TIM7 is a basic timer, so it has no one-pulse bit: the adapter emulates the
 * one-shot behavior by disabling the counter at the start of the update
 * interrupt, before the platform callback runs. Once the callback (or any other
 * context) calls set_next_interval_us(), the adapter reprograms the prescaler
 * and auto-reload for the new interval and resumes counting.
 *
 * Resolution is 1 us for intervals up to 65.535 ms. Longer intervals use a
 * coarser tick (ceil(us / 65535)) because the auto-reload register is 16 bit;
 * the servo waveform only ever needs sub-20 ms steps, so it always gets 1 us.
 *
 * Timer map: id 0 -> TIM7.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_mcu/impl_mcu_timer.h"
#include "stm32f1xx_hal.h"
#include "tim.h"

//******************************** Defines **********************************//

#define IMPL_TIMER_COUNT               1u

/* TIM2..TIM5 come from APB1, whose timer clock is doubled by the prescaler. */
#define IMPL_TIMER_CLOCK_HZ            72000000u
#define IMPL_TIMER_TICKS_MAX           65535u

//******************************** Declaring ********************************//

typedef struct
{
    TIM_HandleTypeDef   *htim;
    void               (*cube_init)(void);
    platform_timer_cb_t  callback;
    void                *user_data;
    uint32_t             interval_us;
    platform_bool_t      initialized;
    platform_bool_t      running;
} impl_timer_ctx_t;

//******************************** Variables ********************************//

static impl_timer_ctx_t s_timer[IMPL_TIMER_COUNT] =
{
    { &htim7, MX_TIM7_Init, NULL, NULL, 0u, PLATFORM_FALSE, PLATFORM_FALSE },
};

//******************************** Functions ********************************//

static impl_timer_ctx_t *impl_timer_get(platform_timer_id_t timer)
{
    if (timer >= IMPL_TIMER_COUNT)
    {
        return NULL;
    }

    return &s_timer[timer];
}

/*
 * Translate microseconds into (prescaler, auto-reload) and load them while the
 * counter is stopped. Loading the prescaler needs a software update event
 * because the PSC register itself is buffered.
 */
static void impl_timer_program(impl_timer_ctx_t *ctx, uint32_t interval_us)
{
    TIM_TypeDef *tim      = ctx->htim->Instance;
    uint32_t     tick_us;
    uint32_t     ticks;
    uint32_t     prescaler;

    tick_us   = (interval_us + IMPL_TIMER_TICKS_MAX - 1u) / IMPL_TIMER_TICKS_MAX;
    if (tick_us == 0u)
    {
        tick_us = 1u;
    }
    ticks     = (interval_us + tick_us - 1u) / tick_us;
    prescaler = (IMPL_TIMER_CLOCK_HZ / 1000000u) * tick_us;

    if (prescaler == 0u)
    {
        prescaler = 1u;
    }

    tim->CR1 &= ~(uint32_t)TIM_CR1_CEN;
    tim->PSC  = prescaler - 1u;
    tim->ARR  = ticks - 1u;

    /* FREERTOS owns SysTick; EGR is a plain register write, safe here. */
    tim->EGR  = TIM_EGR_UG;
    tim->SR   = ~(uint32_t)TIM_SR_UIF;
    tim->CNT  = 0u;
    tim->SR   = ~(uint32_t)TIM_SR_UIF;

    ctx->interval_us = interval_us;
}

platform_err_t impl_timer_init(const platform_timer_cfg_t *cfg)
{
    impl_timer_ctx_t *ctx;

    if ((cfg == NULL) || (cfg->callback == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    if ((cfg->first_interval_us == 0u) ||
        (cfg->first_interval_us > PLATFORM_TIMER_INTERVAL_US_MAX))
    {
        return PLATFORM_ERR_PARAM;
    }

    ctx = impl_timer_get(cfg->timer);
    if (ctx == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (ctx->initialized == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    /*
     * main() already calls MX_TIM7_Init() during CubeMX startup, so only
     * initialize the HAL handle when the reset state is still visible.
     */
    if (ctx->htim->State == HAL_TIM_STATE_RESET)
    {
        ctx->cube_init();
    }

    ctx->callback    = cfg->callback;
    ctx->user_data   = cfg->user_data;
    ctx->running     = PLATFORM_FALSE;
    ctx->initialized = PLATFORM_TRUE;

    impl_timer_program(ctx, cfg->first_interval_us);

    return PLATFORM_ERR_OK;
}

platform_err_t impl_timer_deinit(platform_timer_id_t timer)
{
    impl_timer_ctx_t *ctx = impl_timer_get(timer);

    if (ctx == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (ctx->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    __HAL_TIM_DISABLE_IT(ctx->htim, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE(ctx->htim);

    ctx->callback    = NULL;
    ctx->user_data   = NULL;
    ctx->running     = PLATFORM_FALSE;
    ctx->initialized = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

platform_err_t impl_timer_start(platform_timer_id_t timer)
{
    impl_timer_ctx_t *ctx = impl_timer_get(timer);

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

    ctx->running = PLATFORM_TRUE;
    impl_timer_program(ctx, ctx->interval_us);
    __HAL_TIM_CLEAR_FLAG(ctx->htim, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(ctx->htim, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(ctx->htim);

    return PLATFORM_ERR_OK;
}

platform_err_t impl_timer_stop(platform_timer_id_t timer)
{
    impl_timer_ctx_t *ctx = impl_timer_get(timer);

    if (ctx == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (ctx->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    ctx->running = PLATFORM_FALSE;
    __HAL_TIM_DISABLE_IT(ctx->htim, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE(ctx->htim);

    return PLATFORM_ERR_OK;
}

platform_err_t impl_timer_set_next_interval_us(platform_timer_id_t timer,
                                               uint32_t interval_us)
{
    impl_timer_ctx_t *ctx = impl_timer_get(timer);

    if (ctx == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (ctx->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if ((interval_us == 0u) ||
        (interval_us > PLATFORM_TIMER_INTERVAL_US_MAX))
    {
        return PLATFORM_ERR_PARAM;
    }

    impl_timer_program(ctx, interval_us);

    if (ctx->running == PLATFORM_TRUE)
    {
        __HAL_TIM_CLEAR_FLAG(ctx->htim, TIM_FLAG_UPDATE);
        __HAL_TIM_ENABLE_IT(ctx->htim, TIM_IT_UPDATE);
        __HAL_TIM_ENABLE(ctx->htim);
    }

    return PLATFORM_ERR_OK;
}

/*
 * HAL dispatches every timer update to one global callback, and main.c already
 * defines it to feed the HAL timebase from TIM6. It forwards to this hook for
 * every timer, and only TIM7 belongs to this adapter.
 */
void impl_timer_period_elapsed(TIM_HandleTypeDef *htim)
{
    impl_timer_ctx_t    *ctx;
    platform_timer_cb_t  callback;
    void                *user_data;

    if (htim->Instance != TIM7)
    {
        return;
    }

    ctx = &s_timer[0];
    if ((ctx->initialized != PLATFORM_TRUE) ||
        (ctx->running != PLATFORM_TRUE))
    {
        return;
    }

    /* Emulate the one-shot behavior of a basic timer. */
    __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE(htim);

    callback  = ctx->callback;
    user_data = ctx->user_data;

    if (callback != NULL)
    {
        callback((platform_timer_id_t)0u, user_data);
    }
}
