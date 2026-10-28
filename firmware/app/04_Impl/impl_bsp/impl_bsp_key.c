/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_bsp_key.c
 *
 * @par dependencies
 * - impl_bsp/impl_bsp_key.h
 * - 05_BSP/bsp_key_gpio.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement platform_bsp_key.h on top of bsp_key_gpio.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_bsp/impl_bsp_key.h"
#include "bsp_key_gpio.h"

//******************************** Defines **********************************//

#define IMPL_BSP_KEY_COUNT      PLATFORM_KEY_ID_MAX

//******************************** Declaring ********************************//

typedef struct
{
    const platform_key_ops_t *ops;
    void                     *context;
    platform_bool_t           registered;
    platform_bool_t           initialized;
} impl_bsp_key_slot_t;

//******************************** Variables ********************************//

static impl_bsp_key_slot_t s_key[IMPL_BSP_KEY_COUNT];

//******************************** Functions ********************************//

static platform_err_t impl_bsp_key_op_init(void *context)
{
    return bsp_key_gpio_init((bsp_key_gpio_t *)context);
}

static platform_err_t impl_bsp_key_op_deinit(void *context)
{
    return bsp_key_gpio_deinit((bsp_key_gpio_t *)context);
}

static platform_err_t impl_bsp_key_op_poll(void *context, uint32_t elapsed_ms)
{
    return bsp_key_gpio_poll((bsp_key_gpio_t *)context, elapsed_ms);
}

static platform_err_t impl_bsp_key_op_get_state(void *context,
                                                platform_key_state_t *state)
{
    bsp_key_gpio_t *key = (bsp_key_gpio_t *)context;

    if (state == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    state->pressed     = bsp_key_gpio_is_pressed(key);
    state->raw_pressed = key->raw_pressed;
    state->pressed_ms  = bsp_key_gpio_pressed_ms(key);

    return PLATFORM_ERR_OK;
}

const platform_key_ops_t impl_bsp_key_ops =
{
    impl_bsp_key_op_init,
    impl_bsp_key_op_deinit,
    impl_bsp_key_op_poll,
    impl_bsp_key_op_get_state,
};

static impl_bsp_key_slot_t *impl_bsp_key_get(platform_key_id_t id)
{
    if (id >= IMPL_BSP_KEY_COUNT)
    {
        return NULL;
    }

    return &s_key[id];
}

platform_err_t platform_key_register(platform_key_id_t id,
                                     const platform_key_ops_t *ops,
                                     void *context)
{
    impl_bsp_key_slot_t *slot = impl_bsp_key_get(id);

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

platform_err_t platform_key_init(platform_key_id_t id)
{
    impl_bsp_key_slot_t *slot = impl_bsp_key_get(id);
    platform_err_t       err;

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

platform_err_t platform_key_deinit(platform_key_id_t id)
{
    impl_bsp_key_slot_t *slot = impl_bsp_key_get(id);
    platform_err_t       err;

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

platform_err_t platform_key_poll(platform_key_id_t id, uint32_t elapsed_ms)
{
    impl_bsp_key_slot_t *slot = impl_bsp_key_get(id);

    if (slot == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (slot->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return slot->ops->poll(slot->context, elapsed_ms);
}

platform_err_t platform_key_get_state(platform_key_id_t id,
                                      platform_key_state_t *state)
{
    impl_bsp_key_slot_t *slot = impl_bsp_key_get(id);

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
