/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_bsp_servo.c
 *
 * @par dependencies
 * - impl_bsp/impl_bsp_servo.h
 * - 05_BSP/bsp_servo_softpwm.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement platform_bsp_servo.h on top of bsp_servo_softpwm.
 *
 * The only real adaptation is the shape of the angle array: the platform
 * contract uses platform_servo_angles_t, the driver uses a plain int16_t[6].
 * Neither the platform layer nor the application sees the soft-PWM timer.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_bsp/impl_bsp_servo.h"
#include "bsp_servo_softpwm.h"

//******************************** Variables ********************************//

static const platform_servo_ops_t *s_servo_ops;
static void                       *s_servo_context;
static platform_bool_t             s_servo_registered;
static platform_bool_t             s_servo_initialized;

//******************************** Functions ********************************//

static platform_err_t impl_bsp_servo_op_init(void *context)
{
    return bsp_servo_softpwm_init((bsp_servo_softpwm_t *)context);
}

static platform_err_t impl_bsp_servo_op_deinit(void *context)
{
    return bsp_servo_softpwm_deinit((bsp_servo_softpwm_t *)context);
}

static platform_err_t impl_bsp_servo_op_set_enable(void *context,
                                                   platform_bool_t enable)
{
    return bsp_servo_softpwm_set_enable((bsp_servo_softpwm_t *)context,
                                        enable);
}

static platform_err_t impl_bsp_servo_op_set_angles(
                                        void *context,
                                        const platform_servo_angles_t *angles,
                                        uint16_t move_time_ms)
{
    if (angles == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    return bsp_servo_softpwm_set_angles((bsp_servo_softpwm_t *)context,
                                        angles->joint_centi_deg,
                                        move_time_ms);
}

static platform_err_t impl_bsp_servo_op_get_states(
                                        void *context,
                                        platform_servo_states_t *states)
{
    int16_t angles_centi_deg[BSP_SERVO_JOINT_COUNT];
    uint8_t i;

    if (states == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    bsp_servo_softpwm_get_angles((const bsp_servo_softpwm_t *)context,
                                 angles_centi_deg);

    for (i = 0u; i < PLATFORM_SERVO_JOINT_COUNT; i++)
    {
        states->joint[i].angle_centi_deg = angles_centi_deg[i];
        states->joint[i].load_permille   = PLATFORM_SERVO_VALUE_INVALID;
        states->joint[i].temperature_c   = PLATFORM_SERVO_VALUE_INVALID;
        states->joint[i].fault           = PLATFORM_FALSE;
    }

    return PLATFORM_ERR_OK;
}

const platform_servo_ops_t impl_bsp_servo_ops =
{
    impl_bsp_servo_op_init,
    impl_bsp_servo_op_deinit,
    impl_bsp_servo_op_set_enable,
    impl_bsp_servo_op_set_angles,
    impl_bsp_servo_op_get_states,
};

platform_err_t platform_servo_register(const platform_servo_ops_t *ops,
                                       void *context)
{
    if ((ops == NULL) || (context == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (s_servo_registered == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    s_servo_ops      = ops;
    s_servo_context  = context;
    s_servo_registered = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_servo_init(void)
{
    platform_err_t err;

    if (s_servo_registered != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (s_servo_initialized == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    err = s_servo_ops->init(s_servo_context);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    s_servo_initialized = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_servo_deinit(void)
{
    platform_err_t err;

    if (s_servo_initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = s_servo_ops->deinit(s_servo_context);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    s_servo_initialized = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_servo_set_enable(platform_bool_t enable)
{
    if (s_servo_initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return s_servo_ops->set_enable(s_servo_context, enable);
}

platform_err_t platform_servo_set_angles(
                                        const platform_servo_angles_t *angles,
                                        uint16_t move_time_ms)
{
    if (s_servo_initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return s_servo_ops->set_angles(s_servo_context, angles, move_time_ms);
}

platform_err_t platform_servo_get_states(platform_servo_states_t *states)
{
    if (s_servo_initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return s_servo_ops->get_states(s_servo_context, states);
}
