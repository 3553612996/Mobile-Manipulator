/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file bsp_motor_dual_pwm.c
 *
 * @par dependencies
 * - 06_BSP/bsp_motor_dual_pwm.h
 * - platform_mcu_pwm.h
 * - platform_mcu_encoder.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Dual-PWM H-bridge motor driver.
 *
 * See the header for the waveform convention. Only platform_mcu_pwm /
 * platform_mcu_encoder are used here, so this file has no vendor dependency.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "bsp_motor_dual_pwm.h"

//******************************** Defines **********************************//

/* Full compare: both bridge inputs high, which shorts the motor (brake). */
#define BSP_MOTOR_DUTY_FULL     ((uint16_t)1000u)
#define BSP_MOTOR_DUTY_NONE     ((uint16_t)0u)

//******************************** Functions ********************************//

/*
 * Set the compare value of both bridge inputs. The controller shape is
 * "one input full, the other chopped"; the sign already picked which one.
 */
static platform_err_t bsp_motor_dual_pwm_write(bsp_motor_dual_pwm_t *motor,
                                               uint16_t forward_permille,
                                               uint16_t reverse_permille)
{
    platform_err_t err;

    err = platform_pwm_set_duty(motor->cfg.pwm_forward, forward_permille);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    return platform_pwm_set_duty(motor->cfg.pwm_reverse, reverse_permille);
}

/*
 * Translate the signed duty into the two bridge compare values and push them.
 * brake (both inputs high) is the vendor's behavior at zero speed.
 */
static platform_err_t bsp_motor_dual_pwm_apply(bsp_motor_dual_pwm_t *motor)
{
    int16_t  duty = motor->duty_permille;
    uint16_t forward_permille;
    uint16_t reverse_permille;

    if (motor->cfg.invert_direction == PLATFORM_TRUE)
    {
        duty = (int16_t)-duty;
    }

    if (duty > 0)
    {
        forward_permille = (uint16_t)(BSP_MOTOR_DUTY_FULL - (uint16_t)duty);
        reverse_permille = BSP_MOTOR_DUTY_FULL;
    }
    else if (duty < 0)
    {
        forward_permille = BSP_MOTOR_DUTY_FULL;
        reverse_permille = (uint16_t)(BSP_MOTOR_DUTY_FULL + (uint16_t)duty);
    }
    else
    {
        forward_permille = BSP_MOTOR_DUTY_FULL;
        reverse_permille = BSP_MOTOR_DUTY_FULL;
    }

    return bsp_motor_dual_pwm_write(motor, forward_permille, reverse_permille);
}

platform_err_t bsp_motor_dual_pwm_init(bsp_motor_dual_pwm_t *motor)
{
    platform_pwm_cfg_t     pwm_cfg;
    platform_encoder_cfg_t encoder_cfg;
    platform_err_t         err;

    if (motor == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (motor->initialized == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    pwm_cfg.frequency_hz      = (motor->cfg.pwm_frequency_hz == 0u)
                                ? BSP_MOTOR_PWM_FREQUENCY_HZ
                                : motor->cfg.pwm_frequency_hz;
    pwm_cfg.duty_permille     = BSP_MOTOR_DUTY_NONE;
    pwm_cfg.start_immediately = PLATFORM_FALSE;

    pwm_cfg.channel = motor->cfg.pwm_forward;
    err = platform_pwm_init(&pwm_cfg);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    pwm_cfg.channel = motor->cfg.pwm_reverse;
    err = platform_pwm_init(&pwm_cfg);
    if (PLATFORM_IS_ERR(err))
    {
        (void)platform_pwm_deinit(motor->cfg.pwm_forward);
        return err;
    }

    encoder_cfg.channel = motor->cfg.encoder;
    encoder_cfg.filter  = BSP_MOTOR_ENCODER_FILTER;
    /*
     * invert_encoder corrects the A/B pair, invert_direction corrects the
     * mounting. Between them the control layer always sees one rule: positive
     * duty and positive count are the same physical direction.
     */
    encoder_cfg.invert  = motor->cfg.invert_encoder;

    err = platform_encoder_init(&encoder_cfg);
    if (PLATFORM_IS_ERR(err))
    {
        (void)platform_pwm_deinit(motor->cfg.pwm_reverse);
        (void)platform_pwm_deinit(motor->cfg.pwm_forward);
        return err;
    }

    motor->encoder_count = 0;
    motor->duty_permille = 0;
    motor->enabled       = PLATFORM_FALSE;
    motor->initialized   = PLATFORM_TRUE;

    /* Outputs are still stopped, so both bridge inputs read low (coast). */
    (void)bsp_motor_dual_pwm_write(motor, BSP_MOTOR_DUTY_NONE,
                                   BSP_MOTOR_DUTY_NONE);

    return PLATFORM_ERR_OK;
}

platform_err_t bsp_motor_dual_pwm_deinit(bsp_motor_dual_pwm_t *motor)
{
    platform_err_t err;

    if (motor == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (motor->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = bsp_motor_dual_pwm_set_enable(motor, PLATFORM_FALSE);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    (void)platform_pwm_deinit(motor->cfg.pwm_forward);
    (void)platform_pwm_deinit(motor->cfg.pwm_reverse);
    (void)platform_encoder_deinit(motor->cfg.encoder);

    motor->initialized = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

platform_err_t bsp_motor_dual_pwm_set_enable(bsp_motor_dual_pwm_t *motor,
                                             platform_bool_t enable)
{
    platform_err_t err;

    if (motor == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (motor->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (enable == PLATFORM_TRUE)
    {
        err = platform_pwm_start(motor->cfg.pwm_forward);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }

        err = platform_pwm_start(motor->cfg.pwm_reverse);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }

        motor->enabled = PLATFORM_TRUE;

        return bsp_motor_dual_pwm_apply(motor);
    }

    /* Coast first so the bridge is not left with a stale compare value. */
    (void)bsp_motor_dual_pwm_write(motor, BSP_MOTOR_DUTY_NONE,
                                   BSP_MOTOR_DUTY_NONE);
    (void)platform_pwm_stop(motor->cfg.pwm_forward);
    (void)platform_pwm_stop(motor->cfg.pwm_reverse);

    motor->enabled = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

platform_err_t bsp_motor_dual_pwm_set_duty(bsp_motor_dual_pwm_t *motor,
                                           int16_t duty_permille)
{
    if (motor == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (motor->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if ((duty_permille > BSP_MOTOR_DUTY_PERMILLE_MAX) ||
        (duty_permille < -BSP_MOTOR_DUTY_PERMILLE_MAX))
    {
        return PLATFORM_ERR_PARAM;
    }

    motor->duty_permille = duty_permille;

    if (motor->enabled != PLATFORM_TRUE)
    {
        /* Remember the value; the bridge stays coasting until enable. */
        return PLATFORM_ERR_OK;
    }

    return bsp_motor_dual_pwm_apply(motor);
}

platform_err_t bsp_motor_dual_pwm_update_encoder(bsp_motor_dual_pwm_t *motor)
{
    platform_err_t err;
    int16_t        delta;

    if (motor == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (motor->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = platform_encoder_read_delta(motor->cfg.encoder, &delta);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    /* Widen before negating: -INT16_MIN does not fit in 16 bits. */
    motor->encoder_count += (motor->cfg.invert_direction == PLATFORM_TRUE)
                            ? -(int32_t)delta : (int32_t)delta;

    return PLATFORM_ERR_OK;
}

int32_t bsp_motor_dual_pwm_get_encoder(const bsp_motor_dual_pwm_t *motor)
{
    return (motor == NULL) ? 0 : motor->encoder_count;
}
