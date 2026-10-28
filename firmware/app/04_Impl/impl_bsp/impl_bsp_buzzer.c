/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_bsp_buzzer.c
 *
 * @par dependencies
 * - impl_bsp/impl_bsp_buzzer.h
 * - 06_BSP/bsp_buzzer_gpio.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement platform_bsp_buzzer.h on top of bsp_buzzer_gpio.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_bsp/impl_bsp_buzzer.h"
#include "bsp_buzzer_gpio.h"

//******************************** Variables ********************************//

static const platform_buzzer_ops_t *s_buzzer_ops;
static void                        *s_buzzer_context;
static platform_bool_t              s_buzzer_registered;
static platform_bool_t              s_buzzer_initialized;

//******************************** Functions ********************************//

static platform_err_t impl_bsp_buzzer_op_init(void *context)
{
    return bsp_buzzer_gpio_init((bsp_buzzer_gpio_t *)context);
}

static platform_err_t impl_bsp_buzzer_op_deinit(void *context)
{
    return bsp_buzzer_gpio_deinit((bsp_buzzer_gpio_t *)context);
}

static platform_err_t impl_bsp_buzzer_op_set(void *context,
                                             platform_bool_t on)
{
    return bsp_buzzer_gpio_set((bsp_buzzer_gpio_t *)context, on);
}

const platform_buzzer_ops_t impl_bsp_buzzer_ops =
{
    impl_bsp_buzzer_op_init,
    impl_bsp_buzzer_op_deinit,
    impl_bsp_buzzer_op_set,
};

platform_err_t platform_buzzer_register(const platform_buzzer_ops_t *ops,
                                        void *context)
{
    if ((ops == NULL) || (context == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (s_buzzer_registered == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    s_buzzer_ops        = ops;
    s_buzzer_context    = context;
    s_buzzer_registered = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_buzzer_init(void)
{
    platform_err_t err;

    if (s_buzzer_registered != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (s_buzzer_initialized == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    err = s_buzzer_ops->init(s_buzzer_context);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    s_buzzer_initialized = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_buzzer_deinit(void)
{
    platform_err_t err;

    if (s_buzzer_initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = s_buzzer_ops->deinit(s_buzzer_context);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    s_buzzer_initialized = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_buzzer_set(platform_bool_t on)
{
    if (s_buzzer_initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return s_buzzer_ops->set(s_buzzer_context, on);
}
