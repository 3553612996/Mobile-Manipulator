/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_bsp_motor.c
 *
 * @par dependencies
 * - impl_bsp/impl_bsp_motor.h
 * - 06_BSP/bsp_motor_dual_pwm.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement platform_bsp_motor.h on top of bsp_motor_dual_pwm.
 *
 * The file holds the registration table and the four platform entry points
 * (register / init / deinit). Every operation is forwarded to the shared ops
 * table, which in turn calls the concrete driver with the instance pointer.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_bsp/impl_bsp_motor.h"
#include "bsp_motor_dual_pwm.h"

//******************************** Defines **********************************//

#define IMPL_BSP_MOTOR_COUNT    PLATFORM_MOTOR_ID_MAX

//******************************** Declaring ********************************//

typedef struct
{
    const platform_motor_ops_t *ops;
    void                       *context;
    platform_bool_t             registered;
    platform_bool_t             initialized;
} impl_bsp_motor_slot_t;

//******************************** Variables ********************************//

static impl_bsp_motor_slot_t s_motor[IMPL_BSP_MOTOR_COUNT];

//******************************** Functions ********************************//

static platform_err_t impl_bsp_motor_op_init(void *context)
{
    return bsp_motor_dual_pwm_init((bsp_motor_dual_pwm_t *)context);
}

static platform_err_t impl_bsp_motor_op_deinit(void *context)
{
    return bsp_motor_dual_pwm_deinit((bsp_motor_dual_pwm_t *)context);
}

static platform_err_t impl_bsp_motor_op_set_enable(void *context,
                                                   platform_bool_t enable)
{
    return bsp_motor_dual_pwm_set_enable((bsp_motor_dual_pwm_t *)context,
                                         enable);
}

static platform_err_t impl_bsp_motor_op_set_duty(void *context,
                                                 int16_t duty_permille)
{
    return bsp_motor_dual_pwm_set_duty((bsp_motor_dual_pwm_t *)context,
                                       duty_permille);
}

static platform_err_t impl_bsp_motor_op_get_encoder(void *context,
                                                    int32_t *encoder_count)
{
    bsp_motor_dual_pwm_t *motor = (bsp_motor_dual_pwm_t *)context;
    platform_err_t        err;

    if (encoder_count == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    err = bsp_motor_dual_pwm_update_encoder(motor);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    *encoder_count = bsp_motor_dual_pwm_get_encoder(motor);

    return PLATFORM_ERR_OK;
}

static platform_err_t impl_bsp_motor_op_get_state(void *context,
                                                  platform_motor_state_t *state)
{
    bsp_motor_dual_pwm_t *motor = (bsp_motor_dual_pwm_t *)context;
    platform_err_t        err;

    if (state == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    err = bsp_motor_dual_pwm_update_encoder(motor);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    state->encoder_count = bsp_motor_dual_pwm_get_encoder(motor);
    state->speed_mm_s    = PLATFORM_MOTOR_VALUE_INVALID;
    state->current_ma    = PLATFORM_MOTOR_VALUE_INVALID;
    state->temperature_c = PLATFORM_MOTOR_TEMPERATURE_INVALID;
    state->enabled       = motor->enabled;
    state->fault         = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

const platform_motor_ops_t impl_bsp_motor_ops =
{
    impl_bsp_motor_op_init,
    impl_bsp_motor_op_deinit,
    impl_bsp_motor_op_set_enable,
    impl_bsp_motor_op_set_duty,
    impl_bsp_motor_op_get_encoder,
    impl_bsp_motor_op_get_state,
};

static impl_bsp_motor_slot_t *impl_bsp_motor_get(platform_motor_id_t id)
{
    if (id >= IMPL_BSP_MOTOR_COUNT)
    {
        return NULL;
    }

    return &s_motor[id];
}

platform_err_t platform_motor_register(platform_motor_id_t id,
                                       const platform_motor_ops_t *ops,
                                       void *context)
{
    impl_bsp_motor_slot_t *slot = impl_bsp_motor_get(id);

    if ((slot == NULL) || (ops == NULL) || (context == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (slot->registered == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    slot->ops        = ops;
    slot->context    = context;
    slot->registered = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_motor_init(platform_motor_id_t id)
{
    impl_bsp_motor_slot_t *slot = impl_bsp_motor_get(id);
    platform_err_t         err;

    if (slot == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (slot->registered != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (slot->initialized == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    err = slot->ops->init(slot->context);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    slot->initialized = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_motor_deinit(platform_motor_id_t id)
{
    impl_bsp_motor_slot_t *slot = impl_bsp_motor_get(id);
    platform_err_t         err;

    if (slot == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (slot->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = slot->ops->deinit(slot->context);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    slot->initialized = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_motor_set_enable(platform_motor_id_t id,
                                         platform_bool_t enable)
{
    impl_bsp_motor_slot_t *slot = impl_bsp_motor_get(id);

    if (slot == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (slot->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return slot->ops->set_enable(slot->context, enable);
}

platform_err_t platform_motor_set_duty(platform_motor_id_t id,
                                       int16_t duty_permille)
{
    impl_bsp_motor_slot_t *slot = impl_bsp_motor_get(id);

    if (slot == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (slot->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return slot->ops->set_duty(slot->context, duty_permille);
}

platform_err_t platform_motor_get_encoder(platform_motor_id_t id,
                                          int32_t *encoder_count)
{
    impl_bsp_motor_slot_t *slot = impl_bsp_motor_get(id);

    if (slot == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (slot->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return slot->ops->get_encoder(slot->context, encoder_count);
}

platform_err_t platform_motor_get_state(platform_motor_id_t id,
                                        platform_motor_state_t *state)
{
    impl_bsp_motor_slot_t *slot = impl_bsp_motor_get(id);

    if (slot == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (slot->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return slot->ops->get_state(slot->context, state);
}

platform_err_t platform_motor_stop_all(void)
{
    uint8_t        i;
    platform_err_t first_err = PLATFORM_ERR_OK;

    for (i = 0u; i < IMPL_BSP_MOTOR_COUNT; i++)
    {
        if (s_motor[i].initialized == PLATFORM_TRUE)
        {
            platform_err_t err = s_motor[i].ops->set_duty(s_motor[i].context,
                                                          0);

            if (PLATFORM_IS_ERR(err) && PLATFORM_IS_OK(first_err))
            {
                first_err = err;
            }
        }
    }

    return first_err;
}
