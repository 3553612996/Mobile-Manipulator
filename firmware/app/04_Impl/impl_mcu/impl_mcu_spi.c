/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_mcu_spi.c
 *
 * @par dependencies
 * - impl_mcu/impl_mcu_spi.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Adapt the platform SPI contract to the MCU (skeleton).
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

#include "impl_mcu/impl_mcu_spi.h"

//******************************** Functions ********************************//

platform_err_t impl_spi_init(const platform_spi_cfg_t *cfg)
{
    (void)cfg;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_spi_deinit(platform_spi_bus_t bus)
{
    (void)bus;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_spi_transfer(platform_spi_bus_t bus,
                                 const uint8_t *tx_data,
                                 uint8_t *rx_data,
                                 uint32_t len,
                                 uint32_t timeout_ms)
{
    (void)bus;
    (void)tx_data;
    (void)rx_data;
    (void)len;
    (void)timeout_ms;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_spi_write(platform_spi_bus_t bus,
                              const uint8_t *tx_data,
                              uint32_t len,
                              uint32_t timeout_ms)
{
    (void)bus;
    (void)tx_data;
    (void)len;
    (void)timeout_ms;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_spi_read(platform_spi_bus_t bus,
                             uint8_t *rx_data,
                             uint32_t len,
                             uint32_t timeout_ms)
{
    (void)bus;
    (void)rx_data;
    (void)len;
    (void)timeout_ms;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_spi_write_async(platform_spi_bus_t bus,
                                    const uint8_t *tx_data,
                                    uint32_t len)
{
    (void)bus;
    (void)tx_data;
    (void)len;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_spi_register_complete_cb(
                                        platform_spi_bus_t bus,
                                        platform_spi_complete_cb_t callback,
                                        void *user_data)
{
    (void)bus;
    (void)callback;
    (void)user_data;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_spi_cs_set(platform_spi_bus_t bus,
                               platform_gpio_level_t level)
{
    (void)bus;
    (void)level;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_spi_lock(platform_spi_bus_t bus, uint32_t timeout_ms)
{
    (void)bus;
    (void)timeout_ms;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_spi_unlock(platform_spi_bus_t bus)
{
    (void)bus;

    return PLATFORM_ERR_NOT_SUPPORTED;
}
