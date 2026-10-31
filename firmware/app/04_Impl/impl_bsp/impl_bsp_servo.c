/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_bsp_servo.c
 *
 * @par dependencies
 * - impl_bsp/impl_bsp_servo.h
 * - 06_BSP/bsp_servo_bus.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement platform_bsp_servo.h on top of bsp_servo_bus.
 *
 * The adapter is thin on purpose: the bus driver already speaks centimetre
 * degrees and owns the measurement cache, so the only real work here is
 * reshaping arrays and forwarding the configuration calls.
 *
 * @version V2.0 2026-10-31
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_bsp/impl_bsp_servo.h"
#include "bsp_servo_bus.h"

//******************************** Variables ********************************//

static const platform_servo_ops_t *s_servo_ops;
static void                       *s_servo_context;
static platform_bool_t             s_servo_registered;
static platform_bool_t             s_servo_initialized;

//******************************** Functions ********************************//

static platform_err_t impl_bsp_servo_op_init(void *context)
{
    return bsp_servo_bus_init((bsp_servo_bus_t *)context);
}

static platform_err_t impl_bsp_servo_op_deinit(void *context)
{
    return bsp_servo_bus_deinit((bsp_servo_bus_t *)context);
}

static platform_err_t impl_bsp_servo_op_set_enable(void *context,
                                                   platform_bool_t enable)
{
    return bsp_servo_bus_set_enable((bsp_servo_bus_t *)context, enable);
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

    return bsp_servo_bus_request_move((bsp_servo_bus_t *)context,
                                      angles->joint_centi_deg,
                                      move_time_ms);
}

static platform_err_t impl_bsp_servo_op_get_states(
                                        void *context,
                                        platform_servo_states_t *states)
{
    int16_t         angles[BSP_SERVO_BUS_JOINT_MAX];
    int16_t         temperatures[BSP_SERVO_BUS_JOINT_MAX];
    uint16_t        voltages[BSP_SERVO_BUS_JOINT_MAX];
    uint8_t         faults[BSP_SERVO_BUS_JOINT_MAX];
    platform_bool_t online[BSP_SERVO_BUS_JOINT_MAX];
    uint8_t         i;

    if (states == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    bsp_servo_bus_get_states((const bsp_servo_bus_t *)context,
                             angles,
                             temperatures,
                             voltages,
                             faults,
                             online);

    for (i = 0u; i < PLATFORM_SERVO_JOINT_COUNT; i++)
    {
        states->joint[i].angle_centi_deg = angles[i];

        /*
         * The YH protocol has no load channel; report it as unmeasured so a
         * host plot shows a gap instead of a fake zero.
         */
        states->joint[i].load_permille = PLATFORM_SERVO_VALUE_INVALID;

        states->joint[i].temperature_c =
            (temperatures[i] == BSP_SERVO_BUS_TEMP_INVALID)
                ? PLATFORM_SERVO_TEMP_INVALID
                : temperatures[i];

        states->joint[i].voltage_mv = voltages[i];
        states->joint[i].fault_bits = faults[i];
        states->joint[i].online     = online[i];
    }

    return PLATFORM_ERR_OK;
}

static platform_err_t impl_bsp_servo_op_poll(void *context, uint32_t now_ms)
{
    bsp_servo_bus_poll((bsp_servo_bus_t *)context, now_ms);

    return PLATFORM_ERR_OK;
}

static platform_err_t impl_bsp_servo_op_set_joint_map(
                        void *context,
                        const platform_servo_joint_cfg_t
                            map[PLATFORM_SERVO_JOINT_COUNT])
{
    uint8_t servo_id[BSP_SERVO_BUS_JOINT_MAX];
    int16_t offset_centi_deg[BSP_SERVO_BUS_JOINT_MAX];
    uint8_t i;
    uint8_t joint;

    if (map == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    for (i = 0u; i < BSP_SERVO_BUS_JOINT_MAX; i++)
    {
        servo_id[i] = (uint8_t)i;
        offset_centi_deg[i] = 0;
    }

    for (i = 0u; i < PLATFORM_SERVO_JOINT_COUNT; i++)
    {
        joint = map[i].joint;

        if (joint >= BSP_SERVO_BUS_JOINT_MAX)
        {
            return PLATFORM_ERR_PARAM;
        }

        servo_id[joint] = map[i].servo_id;
        offset_centi_deg[joint] = map[i].offset_centi_deg;
    }

    bsp_servo_bus_set_map((bsp_servo_bus_t *)context,
                          servo_id,
                          offset_centi_deg);

    return PLATFORM_ERR_OK;
}

static platform_err_t impl_bsp_servo_op_config(
                                        void *context,
                                        const platform_servo_cfg_req_t *req)
{
    if (req == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    return bsp_servo_bus_request_config((bsp_servo_bus_t *)context,
                                        req->joint,
                                        req->op,
                                        req->value);
}

static platform_bool_t impl_bsp_servo_op_config_ready(void *context)
{
    return bsp_servo_bus_config_ready((const bsp_servo_bus_t *)context);
}

static platform_err_t impl_bsp_servo_op_config_take(void *context,
                                                    int32_t *out_value)
{
    return bsp_servo_bus_config_take((bsp_servo_bus_t *)context, out_value);
}

static void impl_bsp_servo_op_config_cancel(void *context)
{
    bsp_servo_bus_config_cancel((bsp_servo_bus_t *)context);
}

const platform_servo_ops_t impl_bsp_servo_ops =
{
    impl_bsp_servo_op_init,
    impl_bsp_servo_op_deinit,
    impl_bsp_servo_op_set_enable,
    impl_bsp_servo_op_set_angles,
    impl_bsp_servo_op_get_states,
    impl_bsp_servo_op_poll,
    impl_bsp_servo_op_set_joint_map,
    impl_bsp_servo_op_config,
    impl_bsp_servo_op_config_ready,
    impl_bsp_servo_op_config_take,
    impl_bsp_servo_op_config_cancel,
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

platform_err_t platform_servo_poll(uint32_t now_ms)
{
    if (s_servo_initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return s_servo_ops->poll(s_servo_context, now_ms);
}

platform_err_t platform_servo_set_joint_map(
                    const platform_servo_joint_cfg_t
                        map[PLATFORM_SERVO_JOINT_COUNT])
{
    if (s_servo_registered != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (s_servo_initialized == PLATFORM_TRUE)
    {
        /* The task may already be polling; a hot map change would race it. */
        return PLATFORM_ERR_BUSY;
    }

    return s_servo_ops->set_joint_map(s_servo_context, map);
}

platform_err_t platform_servo_config(const platform_servo_cfg_req_t *req)
{
    if (s_servo_initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return s_servo_ops->config(s_servo_context, req);
}

platform_bool_t platform_servo_config_ready(void)
{
    if (s_servo_initialized != PLATFORM_TRUE)
    {
        return PLATFORM_FALSE;
    }

    return s_servo_ops->config_ready(s_servo_context);
}

platform_err_t platform_servo_config_take(int32_t *out_value)
{
    if (s_servo_initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return s_servo_ops->config_take(s_servo_context, out_value);
}

void platform_servo_config_cancel(void)
{
    if (s_servo_initialized == PLATFORM_TRUE)
    {
        s_servo_ops->config_cancel(s_servo_context);
    }
}
