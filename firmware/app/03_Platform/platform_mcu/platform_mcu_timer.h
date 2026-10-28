/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_mcu_timer.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform one-shot interval timer interface.
 *
 * This service exists for deadlines that software must generate itself: the
 * six-channel servo driver builds its PWM waveform from a chain of "wait N us,
 * then flip a pin" intervals. A generic periodic timer cannot do that because
 * the next interval depends on the state reached in the current interrupt.
 *
 * Semantics: every time the hardware counter expires, the callback runs in
 * interrupt context and the timer stops counting until the callback (or any
 * other code) programs the next interval with set_next_interval_us(). The
 * first interval is supplied by the configuration.
 *
 * The callback must not block, allocate or print. It may call
 * platform_timer_set_next_interval_us().
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_MCU_TIMER_H__
#define __PLATFORM_MCU_TIMER_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"

//******************************** Defines **********************************//

#define PLATFORM_TIMER_ID_0         ((platform_timer_id_t)0u)
#define PLATFORM_TIMER_ID_INVALID   ((platform_timer_id_t)0xFFu)

/* Longest single interval the implementation must accept, in microseconds. */
#define PLATFORM_TIMER_INTERVAL_US_MAX  ((uint32_t)1000000u)

//******************************** Declaring ********************************//

typedef uint8_t platform_timer_id_t;

/*
 * Runs in interrupt context. user_data is the pointer registered with the
 * configuration, never NULL unless the caller passed NULL.
 */
typedef void (*platform_timer_cb_t)(platform_timer_id_t timer, void *user_data);

typedef struct
{
    platform_timer_id_t     timer;
    uint32_t                first_interval_us;  /* 1 .. INTERVAL_US_MAX. */
    platform_timer_cb_t     callback;
    void                   *user_data;
} platform_timer_cfg_t;

platform_err_t platform_timer_init(const platform_timer_cfg_t *cfg);
platform_err_t platform_timer_deinit(platform_timer_id_t timer);

platform_err_t platform_timer_start(platform_timer_id_t timer);
platform_err_t platform_timer_stop(platform_timer_id_t timer);

/*
 * Program the next interval. Safe from the timer callback (interrupt context)
 * and from task context. Values outside 1..INTERVAL_US_MAX are rejected with
 * PLATFORM_ERR_PARAM so that a bad computation cannot stall the waveform.
 */
platform_err_t platform_timer_set_next_interval_us(platform_timer_id_t timer,
                                                   uint32_t interval_us);

#endif /* __PLATFORM_MCU_TIMER_H__ */
