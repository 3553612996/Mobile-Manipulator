/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_mcu_i2c.c
 *
 * @par dependencies
 * - impl_mcu/impl_mcu_i2c.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Adapt the platform I2C contract to the MCU (skeleton).
 *
 * This is the only layer allowed to include stm32f4xx_hal.h once the
 * adaptation is written. The wrapper and the platform layer must never see
 * vendor headers.
 *
 * TODO: replace each PLATFORM_ERR_NOT_SUPPORTED body with the HAL/LL
 * implementation. Keep the signatures unchanged because the wrapper depends
 * on them.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_mcu/impl_mcu_i2c.h"

//******************************** Functions ********************************//

platform_err_t impl_i2c_init(const platform_i2c_cfg_t *cfg)
{
    (void)cfg;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_i2c_deinit(platform_i2c_bus_t bus)
{
    (void)bus;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_i2c_write(platform_i2c_bus_t bus,
                              uint16_t dev_addr,
                              const uint8_t *data,
                              uint32_t len,
                              uint32_t timeout_ms)
{
    (void)bus;
    (void)dev_addr;
    (void)data;
    (void)len;
    (void)timeout_ms;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_i2c_read(platform_i2c_bus_t bus,
                             uint16_t dev_addr,
                             uint8_t *data,
                             uint32_t len,
                             uint32_t timeout_ms)
{
    (void)bus;
    (void)dev_addr;
    (void)data;
    (void)len;
    (void)timeout_ms;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_i2c_mem_write(platform_i2c_bus_t bus,
                                  uint16_t dev_addr,
                                  uint16_t mem_addr,
                                  uint8_t mem_addr_size_bytes,
                                  const uint8_t *data,
                                  uint32_t len,
                                  uint32_t timeout_ms)
{
    (void)bus;
    (void)dev_addr;
    (void)mem_addr;
    (void)mem_addr_size_bytes;
    (void)data;
    (void)len;
    (void)timeout_ms;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_i2c_mem_read(platform_i2c_bus_t bus,
                                 uint16_t dev_addr,
                                 uint16_t mem_addr,
                                 uint8_t mem_addr_size_bytes,
                                 uint8_t *data,
                                 uint32_t len,
                                 uint32_t timeout_ms)
{
    (void)bus;
    (void)dev_addr;
    (void)mem_addr;
    (void)mem_addr_size_bytes;
    (void)data;
    (void)len;
    (void)timeout_ms;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_i2c_is_ready(platform_i2c_bus_t bus,
                                 uint16_t dev_addr,
                                 uint32_t trials,
                                 uint32_t timeout_ms,
                                 platform_bool_t *ready)
{
    (void)bus;
    (void)dev_addr;
    (void)trials;
    (void)timeout_ms;

    if (ready != NULL)
    {
        *ready = PLATFORM_FALSE;
    }

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_i2c_lock(platform_i2c_bus_t bus, uint32_t timeout_ms)
{
    (void)bus;
    (void)timeout_ms;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_i2c_unlock(platform_i2c_bus_t bus)
{
    (void)bus;

    return PLATFORM_ERR_NOT_SUPPORTED;
}
