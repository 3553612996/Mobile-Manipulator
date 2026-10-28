/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_bsp_battery.c
 *
 * @par dependencies
 * - impl_bsp/impl_bsp_battery.h
 * - 06_BSP/bsp_battery_adc.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement platform_bsp_battery.h on top of bsp_battery_adc.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_bsp/impl_bsp_battery.h"
#include "bsp_battery_adc.h"

//******************************** Variables ********************************//

static const platform_battery_ops_t *s_battery_ops;
static void                         *s_battery_context;
static platform_bool_t               s_battery_registered;
static platform_bool_t               s_battery_initialized;

//******************************** Functions ********************************//

static platform_err_t impl_bsp_battery_op_init(void *context)
{
    return bsp_battery_adc_init((bsp_battery_adc_t *)context);
}

static platform_err_t impl_bsp_battery_op_deinit(void *context)
{
    return bsp_battery_adc_deinit((bsp_battery_adc_t *)context);
}

static platform_err_t impl_bsp_battery_op_read(void *context,
                                               platform_battery_state_t *state)
{
    bsp_battery_adc_t *battery = (bsp_battery_adc_t *)context;
    platform_err_t     err;

    if (state == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    err = bsp_battery_adc_read(battery);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    state->voltage_mv = bsp_battery_adc_get_mv(battery);
    state->raw        = battery->raw;
    state->valid      = battery->valid;

    return PLATFORM_ERR_OK;
}

const platform_battery_ops_t impl_bsp_battery_ops =
{
    impl_bsp_battery_op_init,
    impl_bsp_battery_op_deinit,
    impl_bsp_battery_op_read,
};

platform_err_t platform_battery_register(const platform_battery_ops_t *ops,
                                         void *context)
{
    if ((ops == NULL) || (context == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (s_battery_registered == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    s_battery_ops        = ops;
    s_battery_context    = context;
    s_battery_registered = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_battery_init(void)
{
    platform_err_t err;

    if (s_battery_registered != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (s_battery_initialized == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    err = s_battery_ops->init(s_battery_context);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    s_battery_initialized = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_battery_deinit(void)
{
    platform_err_t err;

    if (s_battery_initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = s_battery_ops->deinit(s_battery_context);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    s_battery_initialized = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_battery_read(platform_battery_state_t *state)
{
    if (s_battery_initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return s_battery_ops->read(s_battery_context, state);
}
