/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file bsp_motor_dual_pwm.h
 *
 * @par dependencies
 * - platform_mcu_pwm.h
 * - platform_mcu_encoder.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Drive one brushed DC gear motor with a dual-PWM H-bridge.
 *
 * The vendor board does not use a direction pin. Each H-bridge input is fed by
 * one PWM channel and the two inputs are driven in opposite phase:
 *
 *   duty > 0 : input A = 1000 - duty, input B = 1000
 *   duty < 0 : input A = 1000,        input B = 1000 + duty
 *   duty = 0 : input A = 1000,        input B = 1000   (both high -> brake)
 *
 * That reproduces the vendor MOTOR_x_SetSpeed() waveform, where one input is
 * held at full compare and the other is chopped. The whole board is wired so
 * that input B high / input A chopped turns the wheel "forward"; a wheel that
 * is mounted mirrored is corrected with cfg.invert_direction instead of
 * changing the control layer.
 *
 * The driver owns no clock and no pin: PWM and encoder channels are logical
 * platform resources, so the same code would run on another board as long as
 * impl_board maps the channels.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __BSP_MOTOR_DUAL_PWM_H__
#define __BSP_MOTOR_DUAL_PWM_H__

//******************************** Includes *********************************//

#include "platform_mcu_pwm.h"
#include "platform_mcu_encoder.h"

//******************************** Defines **********************************//

#define BSP_MOTOR_DUTY_PERMILLE_MAX     ((int16_t)1000)
#define BSP_MOTOR_PWM_FREQUENCY_HZ      ((uint32_t)18000u)
#define BSP_MOTOR_ENCODER_FILTER        ((uint8_t)10u)

//******************************** Declaring ********************************//

/*
 * Wiring of one motor. The two PWM channels are the H-bridge inputs, the encoder
 * channel is the quadrature timer that counts the wheel.
 */
typedef struct
{
    platform_pwm_channel_t      pwm_forward;    /* Chopped input for duty > 0. */
    platform_pwm_channel_t      pwm_reverse;    /* Chopped input for duty < 0. */
    platform_encoder_channel_t  encoder;
    platform_bool_t             invert_direction;
    uint32_t                    pwm_frequency_hz;
} bsp_motor_dual_pwm_cfg_t;

/*
 * Driver instance. cfg is filled by impl_board before init(). The remaining
 * fields are private state: the 32-bit position is accumulated in software
 * because the hardware counter is 16 bit and wraps.
 */
typedef struct
{
    bsp_motor_dual_pwm_cfg_t    cfg;
    int32_t                     encoder_count;
    int16_t                     duty_permille;
    platform_bool_t             enabled;
    platform_bool_t             initialized;
} bsp_motor_dual_pwm_t;

//******************************** Functions ********************************//

/*
 * Configure the PWM channels and the encoder from motor->cfg, then leave both
 * bridge inputs low (coast) with the driver disabled.
 */
platform_err_t bsp_motor_dual_pwm_init(bsp_motor_dual_pwm_t *motor);

platform_err_t bsp_motor_dual_pwm_deinit(bsp_motor_dual_pwm_t *motor);

/*
 * Enable: start both PWM outputs and re-apply the stored duty.
 * Disable: drive both inputs low (coast) and stop the outputs so that a
 * disabled motor cannot move even if a duty is set by mistake.
 */
platform_err_t bsp_motor_dual_pwm_set_enable(bsp_motor_dual_pwm_t *motor,
                                             platform_bool_t enable);

platform_err_t bsp_motor_dual_pwm_set_duty(bsp_motor_dual_pwm_t *motor,
                                           int16_t duty_permille);

/*
 * Read the encoder delta since the previous call and add it to the 32-bit
 * position. Call once per control period; it is the only place that consumes
 * the hardware counter, so the count cannot be lost by another reader.
 */
platform_err_t bsp_motor_dual_pwm_update_encoder(bsp_motor_dual_pwm_t *motor);

int32_t bsp_motor_dual_pwm_get_encoder(const bsp_motor_dual_pwm_t *motor);

#endif /* __BSP_MOTOR_DUAL_PWM_H__ */
