/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file bsp_servo_softpwm.c
 *
 * @par dependencies
 * - 05_BSP/bsp_servo_softpwm.h
 * - platform_mcu_gpio.h
 * - platform_mcu_timer.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Six-channel software-PWM servo driver.
 *
 * The frame state machine runs entirely in the interval-timer callback:
 *
 *   low  phase, slot n : pin n high, wait pulse[n]
 *   high phase, slot n : pin n low,  wait SLOT_US - pulse[n]
 *   after slot 5       : wait the frame gap, wrap to slot 0
 *
 * set_angles() only writes the target/step arrays; the interrupt picks them up
 * at the next frame start, so a task never has to wait for the waveform.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "bsp_servo_softpwm.h"

//******************************** Defines **********************************//

#define BSP_SERVO_FRAME_MS      ((uint32_t)(BSP_SERVO_FRAME_US / 1000u))

//******************************** Declaring ********************************//

static void bsp_servo_softpwm_irq(platform_timer_id_t timer, void *user_data);

//******************************** Functions ********************************//

static platform_err_t bsp_servo_softpwm_pin_write(const bsp_servo_softpwm_t *servo,
                                                  uint8_t joint,
                                                  platform_gpio_level_t level)
{
    return platform_gpio_write(servo->cfg.pin[joint], level);
}

/* Convert a fixed-point angle into a pulse width, clamped to the safe range. */
static uint16_t bsp_servo_softpwm_pulse_from_q8(
                                    const bsp_servo_softpwm_t *servo,
                                    int32_t angle_q8)
{
    int32_t centi_deg = angle_q8 / BSP_SERVO_ANGLE_SCALE;
    int32_t pulse     = (int32_t)servo->cfg.pulse_center_us +
                        (centi_deg * (int32_t)servo->cfg.pulse_span_us) /
                        (int32_t)servo->cfg.angle_limit_centi_deg;

    if (pulse < (int32_t)servo->cfg.pulse_center_us -
                (int32_t)servo->cfg.pulse_span_us)
    {
        pulse = (int32_t)servo->cfg.pulse_center_us -
                (int32_t)servo->cfg.pulse_span_us;
    }

    if (pulse > (int32_t)servo->cfg.pulse_center_us +
                (int32_t)servo->cfg.pulse_span_us)
    {
        pulse = (int32_t)servo->cfg.pulse_center_us +
                (int32_t)servo->cfg.pulse_span_us;
    }

    return (uint16_t)pulse;
}

/*
 * One interpolation step for every joint, executed at the start of a frame.
 * It also refreshes the pulse array, because a target set with move_time_ms
 * equal to zero changes the position without a step.
 */
static void bsp_servo_softpwm_advance(bsp_servo_softpwm_t *servo)
{
    uint8_t i;

    for (i = 0u; i < BSP_SERVO_JOINT_COUNT; i++)
    {
        int32_t step = servo->step_q8[i];

        if (step != 0)
        {
            servo->current_q8[i] += step;

            if (((step > 0) && (servo->current_q8[i] >= servo->target_q8[i])) ||
                ((step < 0) && (servo->current_q8[i] <= servo->target_q8[i])))
            {
                servo->current_q8[i] = servo->target_q8[i];
                servo->step_q8[i]    = 0;
            }
        }

        servo->pulse_us[i] = bsp_servo_softpwm_pulse_from_q8(
                                    servo, servo->current_q8[i]);
    }
}

/* Interval-timer callback: one edge per invocation. */
static void bsp_servo_softpwm_irq(platform_timer_id_t timer, void *user_data)
{
    bsp_servo_softpwm_t *servo = (bsp_servo_softpwm_t *)user_data;
    uint32_t             next_us;

    (void)timer;

    if ((servo == NULL) || (servo->enabled != PLATFORM_TRUE))
    {
        return;
    }

    if (servo->high == PLATFORM_TRUE)
    {
        /* Falling edge: the rest of the slot belongs to the next joint. */
        (void)bsp_servo_softpwm_pin_write(servo, servo->slot,
                                          PLATFORM_GPIO_LEVEL_LOW);
        next_us = BSP_SERVO_SLOT_US - (uint32_t)servo->pulse_us[servo->slot];

        servo->slot++;
        servo->high = PLATFORM_FALSE;

        if (servo->slot >= BSP_SERVO_JOINT_COUNT)
        {
            /* Last joint done: idle for the rest of the 20 ms frame. */
            next_us   += BSP_SERVO_FRAME_GAP_US;
            servo->slot = 0u;
        }
    }
    else
    {
        if (servo->slot == 0u)
        {
            bsp_servo_softpwm_advance(servo);
        }

        /* Rising edge: start this joint's pulse. */
        (void)bsp_servo_softpwm_pin_write(servo, servo->slot,
                                          PLATFORM_GPIO_LEVEL_HIGH);
        next_us     = (uint32_t)servo->pulse_us[servo->slot];
        servo->high = PLATFORM_TRUE;
    }

    (void)platform_timer_set_next_interval_us(servo->cfg.timer, next_us);
}

platform_err_t bsp_servo_softpwm_init(bsp_servo_softpwm_t *servo)
{
    platform_gpio_cfg_t gpio_cfg;
    platform_timer_cfg_t timer_cfg;
    platform_err_t       err;
    uint8_t              i;

    if (servo == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (servo->initialized == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    /* A partially filled cfg still works: fall back to the vendor timing. */
    if (servo->cfg.pulse_center_us == 0u)
    {
        servo->cfg.pulse_center_us = BSP_SERVO_PULSE_CENTER_US;
    }

    if (servo->cfg.pulse_span_us == 0u)
    {
        servo->cfg.pulse_span_us = BSP_SERVO_PULSE_SPAN_US;
    }

    if (servo->cfg.angle_limit_centi_deg <= 0)
    {
        servo->cfg.angle_limit_centi_deg = BSP_SERVO_ANGLE_LIMIT;
    }

    if ((uint32_t)servo->cfg.pulse_center_us +
        (uint32_t)servo->cfg.pulse_span_us > BSP_SERVO_SLOT_US)
    {
        /* A pulse longer than its slot would overlap the next joint. */
        return PLATFORM_ERR_PARAM;
    }

    gpio_cfg.mode          = PLATFORM_GPIO_MODE_OUTPUT_PP;
    gpio_cfg.pull          = PLATFORM_GPIO_PULL_NONE;
    gpio_cfg.initial_level = PLATFORM_GPIO_LEVEL_LOW;

    for (i = 0u; i < BSP_SERVO_JOINT_COUNT; i++)
    {
        gpio_cfg.pin = servo->cfg.pin[i];

        err = platform_gpio_init(&gpio_cfg);
        if (PLATFORM_IS_ERR(err))
        {
            while (i > 0u)
            {
                i--;
                (void)platform_gpio_deinit(servo->cfg.pin[i]);
            }

            return err;
        }

        servo->current_q8[i] = 0;
        servo->target_q8[i]  = 0;
        servo->step_q8[i]    = 0;
        servo->pulse_us[i]   = servo->cfg.pulse_center_us;
    }

    timer_cfg.timer             = servo->cfg.timer;
    timer_cfg.first_interval_us = BSP_SERVO_SLOT_US;
    timer_cfg.callback          = bsp_servo_softpwm_irq;
    timer_cfg.user_data         = servo;

    err = platform_timer_init(&timer_cfg);
    if (PLATFORM_IS_ERR(err))
    {
        for (i = 0u; i < BSP_SERVO_JOINT_COUNT; i++)
        {
            (void)platform_gpio_deinit(servo->cfg.pin[i]);
        }

        return err;
    }

    servo->slot        = 0u;
    servo->high        = PLATFORM_FALSE;
    servo->enabled     = PLATFORM_FALSE;
    servo->initialized = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t bsp_servo_softpwm_deinit(bsp_servo_softpwm_t *servo)
{
    uint8_t i;

    if (servo == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (servo->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    (void)bsp_servo_softpwm_set_enable(servo, PLATFORM_FALSE);

    (void)platform_timer_deinit(servo->cfg.timer);

    for (i = 0u; i < BSP_SERVO_JOINT_COUNT; i++)
    {
        (void)platform_gpio_deinit(servo->cfg.pin[i]);
    }

    servo->initialized = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

platform_err_t bsp_servo_softpwm_set_enable(bsp_servo_softpwm_t *servo,
                                            platform_bool_t enable)
{
    platform_err_t err;
    uint8_t        i;

    if (servo == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (servo->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (enable != PLATFORM_TRUE)
    {
        /* Stop first: the callback must not touch the pins any more. */
        servo->enabled = PLATFORM_FALSE;
        (void)platform_timer_stop(servo->cfg.timer);

        for (i = 0u; i < BSP_SERVO_JOINT_COUNT; i++)
        {
            (void)bsp_servo_softpwm_pin_write(servo, i, PLATFORM_GPIO_LEVEL_LOW);
        }

        return PLATFORM_ERR_OK;
    }

    if (servo->enabled == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_OK;
    }

    servo->slot = 0u;
    servo->high = PLATFORM_FALSE;

    for (i = 0u; i < BSP_SERVO_JOINT_COUNT; i++)
    {
        (void)bsp_servo_softpwm_pin_write(servo, i, PLATFORM_GPIO_LEVEL_LOW);
    }

    (void)platform_timer_set_next_interval_us(servo->cfg.timer,
                                              BSP_SERVO_SLOT_US);

    servo->enabled = PLATFORM_TRUE;

    err = platform_timer_start(servo->cfg.timer);
    if (PLATFORM_IS_ERR(err))
    {
        servo->enabled = PLATFORM_FALSE;
    }

    return err;
}

platform_err_t bsp_servo_softpwm_set_angles(
                        bsp_servo_softpwm_t *servo,
                        const int16_t joint_centi_deg[BSP_SERVO_JOINT_COUNT],
                        uint16_t move_time_ms)
{
    uint32_t frames;
    uint8_t  i;

    if ((servo == NULL) || (joint_centi_deg == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (servo->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (move_time_ms > BSP_SERVO_MOVE_TIME_MAX_MS)
    {
        move_time_ms = BSP_SERVO_MOVE_TIME_MAX_MS;
    }

    frames = (uint32_t)move_time_ms / BSP_SERVO_FRAME_MS;

    for (i = 0u; i < BSP_SERVO_JOINT_COUNT; i++)
    {
        int32_t target;

        if ((joint_centi_deg[i] > servo->cfg.angle_limit_centi_deg) ||
            (joint_centi_deg[i] < -servo->cfg.angle_limit_centi_deg))
        {
            return PLATFORM_ERR_PARAM;
        }

        target = (int32_t)joint_centi_deg[i] * BSP_SERVO_ANGLE_SCALE;

        servo->target_q8[i] = target;

        if ((move_time_ms == 0u) || (frames == 0u))
        {
            /* move_time_ms shorter than one frame: jump, do not interpolate. */
            servo->current_q8[i] = target;
            servo->step_q8[i]    = 0;
        }
        else
        {
            servo->step_q8[i] = (target - servo->current_q8[i]) /
                                (int32_t)frames;
        }
    }

    return PLATFORM_ERR_OK;
}

void bsp_servo_softpwm_get_angles(
                        const bsp_servo_softpwm_t *servo,
                        int16_t joint_centi_deg[BSP_SERVO_JOINT_COUNT])
{
    uint8_t i;

    if ((servo == NULL) || (joint_centi_deg == NULL))
    {
        return;
    }

    for (i = 0u; i < BSP_SERVO_JOINT_COUNT; i++)
    {
        joint_centi_deg[i] =
            (int16_t)(servo->current_q8[i] / BSP_SERVO_ANGLE_SCALE);
    }
}
