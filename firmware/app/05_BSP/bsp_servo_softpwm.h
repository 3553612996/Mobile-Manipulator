/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file bsp_servo_softpwm.h
 *
 * @par dependencies
 * - platform_mcu_gpio.h
 * - platform_mcu_timer.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Drive six analog hobby servos with one GPIO per joint and a software
 *        PWM chain clocked by an interval timer.
 *
 * The vendor board has no servo controller: every joint is a plain GPIO and the
 * waveform is generated in the timer interrupt. One frame is 20 ms long and is
 * split into fixed 2.5 ms slots, one per joint; inside a slot the pin is high
 * for the pulse width (500..2500 us) and low for the rest. A 5 ms gap after the
 * sixth slot fills the frame:
 *
 *   |- slot 0 -|- slot 1 -| ... |- slot 5 -|-- gap --|
 *   |<------------------ 20000 us ------------------>|
 *
 * The interval timer is reprogrammed from its own callback, so no periodic
 * timer with a fixed period can be used: each step lasts a different time.
 *
 * Angle convention: 0.01 degrees, 0 is the center pulse. The default mapping is
 * 1500 us +- 1000 us for +-90 degrees, which matches the vendor's 500..2500 us
 * pulse range. move_time_ms is interpolated once per frame with integer
 * fixed-point arithmetic; there is no float and no FPU dependency.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __BSP_SERVO_SOFTPWM_H__
#define __BSP_SERVO_SOFTPWM_H__

//******************************** Includes *********************************//

#include "platform_mcu_gpio.h"
#include "platform_mcu_timer.h"

//******************************** Defines **********************************//

#define BSP_SERVO_JOINT_COUNT       ((uint8_t)6u)

#define BSP_SERVO_SLOT_US           ((uint32_t)2500u)
#define BSP_SERVO_FRAME_US          ((uint32_t)20000u)
#define BSP_SERVO_FRAME_GAP_US      (BSP_SERVO_FRAME_US - \
                                     (BSP_SERVO_SLOT_US * BSP_SERVO_JOINT_COUNT))

#define BSP_SERVO_PULSE_CENTER_US   ((uint16_t)1500u)
#define BSP_SERVO_PULSE_SPAN_US     ((uint16_t)1000u)

/* +-90 degrees in 0.01 degree units; also the default pulse-range limit. */
#define BSP_SERVO_ANGLE_LIMIT       ((int16_t)9000)

#define BSP_SERVO_MOVE_TIME_MAX_MS  ((uint16_t)10000u)

/* Interpolation is applied once per frame; keep the shifts in one place. */
#define BSP_SERVO_ANGLE_FRAC_BITS   ((uint8_t)8u)
#define BSP_SERVO_ANGLE_SCALE       ((int32_t)1 << BSP_SERVO_ANGLE_FRAC_BITS)

//******************************** Declaring ********************************//

typedef struct
{
    platform_gpio_t      pin[BSP_SERVO_JOINT_COUNT];
    platform_timer_id_t  timer;
    uint16_t             pulse_center_us;
    uint16_t             pulse_span_us;
    int16_t              angle_limit_centi_deg;
} bsp_servo_softpwm_cfg_t;

typedef struct
{
    bsp_servo_softpwm_cfg_t cfg;

    /* Current position and per-frame step, both in 0.01 deg with 8 fraction bits. */
    int32_t         current_q8[BSP_SERVO_JOINT_COUNT];
    int32_t         target_q8[BSP_SERVO_JOINT_COUNT];
    int32_t         step_q8[BSP_SERVO_JOINT_COUNT];

    uint16_t        pulse_us[BSP_SERVO_JOINT_COUNT];
    uint8_t         slot;
    platform_bool_t high;
    platform_bool_t enabled;
    platform_bool_t initialized;
} bsp_servo_softpwm_t;

//******************************** Functions ********************************//

/*
 * Configure the six GPIOs from servo->cfg, start the interval timer in its
 * stopped state and park every joint on the center pulse.
 */
platform_err_t bsp_servo_softpwm_init(bsp_servo_softpwm_t *servo);

platform_err_t bsp_servo_softpwm_deinit(bsp_servo_softpwm_t *servo);

/*
 * Enable starts the frame chain with all pins low; disable stops the timer and
 * leaves every pin low (the servos hold their last mechanical position).
 */
platform_err_t bsp_servo_softpwm_set_enable(bsp_servo_softpwm_t *servo,
                                            platform_bool_t enable);

/*
 * New targets for all six joints in 0.01 degrees. move_time_ms == 0 jumps to
 * the target, otherwise the driver interpolates one step per 20 ms frame.
 * Returns PLATFORM_ERR_PARAM if a joint is outside the configured limit.
 */
platform_err_t bsp_servo_softpwm_set_angles(
                        bsp_servo_softpwm_t *servo,
                        const int16_t joint_centi_deg[BSP_SERVO_JOINT_COUNT],
                        uint16_t move_time_ms);

void bsp_servo_softpwm_get_angles(
                        const bsp_servo_softpwm_t *servo,
                        int16_t joint_centi_deg[BSP_SERVO_JOINT_COUNT]);

#endif /* __BSP_SERVO_SOFTPWM_H__ */
