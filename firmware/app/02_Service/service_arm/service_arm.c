/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file service_arm.c
 *
 * @par dependencies
 * - service_arm.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement the six-joint arm service.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "service_arm.h"

//******************************** Variables ********************************//

static struct
{
    platform_bool_t     initialized;
    platform_bool_t     enabled;
    service_arm_cfg_t   cfg;
    platform_servo_angles_t target;
} s_arm;

//******************************** Functions ********************************//

platform_err_t service_arm_init(const service_arm_cfg_t *cfg)
{
    uint8_t i;

    if (cfg == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    for (i = 0u; i < SERVICE_ARM_JOINT_COUNT; i++)
    {
        if (cfg->min_centi_deg[i] > cfg->max_centi_deg[i])
        {
            return PLATFORM_ERR_PARAM;
        }

        if ((cfg->home_centi_deg[i] < cfg->min_centi_deg[i]) ||
            (cfg->home_centi_deg[i] > cfg->max_centi_deg[i]))
        {
            return PLATFORM_ERR_PARAM;
        }

        s_arm.target.joint_centi_deg[i] = cfg->home_centi_deg[i];
    }

    s_arm.cfg = *cfg;

    if (s_arm.cfg.default_move_time_ms == 0u)
    {
        s_arm.cfg.default_move_time_ms = SERVICE_ARM_MOVE_TIME_MS;
    }

    /*
     * board_init() leaves the soft-PWM running at the center pulse, so the
     * arm is physically enabled at this point. Mirror that state instead of
     * reporting "disabled" while the servos hold torque.
     */
    s_arm.enabled = PLATFORM_TRUE;
    s_arm.initialized = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t service_arm_enable(platform_bool_t enable)
{
    platform_err_t err;

    if (s_arm.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = platform_servo_set_enable(enable);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    s_arm.enabled = enable;

    return PLATFORM_ERR_OK;
}

platform_bool_t service_arm_is_enabled(void)
{
    return s_arm.enabled;
}

platform_err_t service_arm_set_joints(const int16_t joint_centi_deg
                                          [SERVICE_ARM_JOINT_COUNT],
                                      uint16_t move_time_ms)
{
    platform_servo_angles_t target;
    uint8_t i;

    if (s_arm.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (joint_centi_deg == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (s_arm.enabled == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_FAIL;
    }

    /* Validate the whole frame before touching any joint. */
    for (i = 0u; i < SERVICE_ARM_JOINT_COUNT; i++)
    {
        if ((joint_centi_deg[i] < s_arm.cfg.min_centi_deg[i]) ||
            (joint_centi_deg[i] > s_arm.cfg.max_centi_deg[i]))
        {
            return PLATFORM_ERR_PARAM;
        }

        target.joint_centi_deg[i] = joint_centi_deg[i];
    }

    if (move_time_ms == 0u)
    {
        move_time_ms = s_arm.cfg.default_move_time_ms;
    }

    if (PLATFORM_IS_ERR(platform_servo_set_angles(&target, move_time_ms)))
    {
        return PLATFORM_ERR_FAIL;
    }

    s_arm.target = target;

    return PLATFORM_ERR_OK;
}

platform_err_t service_arm_get_targets(platform_servo_angles_t *out)
{
    if (out == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    *out = s_arm.target;

    return PLATFORM_ERR_OK;
}

platform_err_t service_arm_get_states(platform_servo_states_t *out)
{
    if (out == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (s_arm.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return platform_servo_get_states(out);
}
