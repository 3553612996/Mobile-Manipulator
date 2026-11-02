/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_bsp_flash.c
 *
 * @par dependencies
 * - impl_bsp/impl_bsp_flash.h
 * - 06_BSP/bsp_flash_w25q64.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement platform_bsp_flash.h on top of bsp_flash_w25q64.
 *
 * The dispatchers are the only entry points the platform layers use; the ops
 * table lets a future board swap the chip without touching them.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_bsp/impl_bsp_flash.h"
#include "bsp_flash_w25q64.h"

//******************************** Variables ********************************//

static const platform_flash_ops_t *s_flash_ops;
static void                       *s_flash_context;
static platform_bool_t             s_flash_registered;
static platform_bool_t             s_flash_initialized;

//******************************** Functions ********************************//

static platform_err_t impl_bsp_flash_op_init(void *context)
{
    return bsp_flash_w25q64_init((bsp_flash_w25q64_t *)context);
}

static platform_err_t impl_bsp_flash_op_deinit(void *context)
{
    return bsp_flash_w25q64_deinit((bsp_flash_w25q64_t *)context);
}

static platform_err_t impl_bsp_flash_op_read(void *context,
                                             uint32_t address,
                                             uint8_t *data,
                                             uint32_t length)
{
    return bsp_flash_w25q64_read((bsp_flash_w25q64_t *)context,
                                 address, data, length);
}

static platform_err_t impl_bsp_flash_op_write(void *context,
                                              uint32_t address,
                                              const uint8_t *data,
                                              uint32_t length)
{
    return bsp_flash_w25q64_write((bsp_flash_w25q64_t *)context,
                                  address, data, length);
}

static platform_err_t impl_bsp_flash_op_erase(void *context,
                                              uint32_t address,
                                              uint32_t length)
{
    return bsp_flash_w25q64_erase((bsp_flash_w25q64_t *)context,
                                  address, length);
}

static platform_err_t impl_bsp_flash_op_get_info(void *context,
                                                 platform_flash_info_t *info)
{
    return bsp_flash_w25q64_get_info((const bsp_flash_w25q64_t *)context,
                                     info);
}

static platform_err_t impl_bsp_flash_op_is_busy(void *context,
                                                platform_bool_t *busy)
{
    return bsp_flash_w25q64_is_busy((bsp_flash_w25q64_t *)context, busy);
}

const platform_flash_ops_t impl_bsp_flash_ops =
{
    impl_bsp_flash_op_init,
    impl_bsp_flash_op_deinit,
    impl_bsp_flash_op_read,
    impl_bsp_flash_op_write,
    impl_bsp_flash_op_erase,
    impl_bsp_flash_op_get_info,
    impl_bsp_flash_op_is_busy,
};

platform_err_t platform_flash_register(const platform_flash_ops_t *ops,
                                       void *context)
{
    if ((ops == NULL) || (context == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (s_flash_registered == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    s_flash_ops        = ops;
    s_flash_context    = context;
    s_flash_registered = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_flash_init(void)
{
    platform_err_t err;

    if (s_flash_registered != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (s_flash_initialized == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    err = s_flash_ops->init(s_flash_context);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    s_flash_initialized = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_flash_deinit(void)
{
    platform_err_t err;

    if (s_flash_initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = s_flash_ops->deinit(s_flash_context);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    s_flash_initialized = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_flash_read(uint32_t address,
                                   uint8_t *data,
                                   uint32_t length)
{
    if (s_flash_initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return s_flash_ops->read(s_flash_context, address, data, length);
}

platform_err_t platform_flash_write(uint32_t address,
                                    const uint8_t *data,
                                    uint32_t length)
{
    if (s_flash_initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return s_flash_ops->write(s_flash_context, address, data, length);
}

platform_err_t platform_flash_erase(uint32_t address, uint32_t length)
{
    if (s_flash_initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return s_flash_ops->erase(s_flash_context, address, length);
}

platform_err_t platform_flash_get_info(platform_flash_info_t *info)
{
    if (s_flash_initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return s_flash_ops->get_info(s_flash_context, info);
}

platform_err_t platform_flash_is_busy(platform_bool_t *busy)
{
    if (s_flash_initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return s_flash_ops->is_busy(s_flash_context, busy);
}
