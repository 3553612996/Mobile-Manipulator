/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_bsp_led.c
 *
 * @par dependencies
 * - impl_bsp/impl_bsp_led.h
 * - 06_BSP/bsp_led_gpio.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement platform_bsp_led.h on top of bsp_led_gpio.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_bsp/impl_bsp_led.h"
#include "bsp_led_gpio.h"

//******************************** Variables ********************************//

static const platform_led_ops_t *s_led_ops;
static void                     *s_led_context;
static platform_bool_t           s_led_registered;
static platform_bool_t           s_led_initialized;

//******************************** Functions ********************************//

static platform_err_t impl_bsp_led_op_init(void *context)
{
    return bsp_led_gpio_init((bsp_led_gpio_t *)context);
}

static platform_err_t impl_bsp_led_op_deinit(void *context)
{
    return bsp_led_gpio_deinit((bsp_led_gpio_t *)context);
}

static platform_err_t impl_bsp_led_op_set(void *context, platform_bool_t on)
{
    return bsp_led_gpio_set((bsp_led_gpio_t *)context, on);
}

const platform_led_ops_t impl_bsp_led_ops =
{
    impl_bsp_led_op_init,
    impl_bsp_led_op_deinit,
    impl_bsp_led_op_set,
};

platform_err_t platform_led_register(const platform_led_ops_t *ops,
                                     void *context)
{
    if ((ops == NULL) || (context == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (s_led_registered == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    s_led_ops        = ops;
    s_led_context    = context;
    s_led_registered = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_led_init(void)
{
    platform_err_t err;

    if (s_led_registered != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (s_led_initialized == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    err = s_led_ops->init(s_led_context);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    s_led_initialized = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_led_deinit(void)
{
    platform_err_t err;

    if (s_led_initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = s_led_ops->deinit(s_led_context);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    s_led_initialized = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_led_set(platform_bool_t on)
{
    if (s_led_initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return s_led_ops->set(s_led_context, on);
}
