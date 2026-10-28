/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_mcu_spi.c
 *
 * @par dependencies
 * - platform_mcu_spi.h
 * - impl_wrapper/impl_mcu_wrapper.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Forward the platform SPI contract to the Impl wrapper.
 *
 * This file contains no validation, adaptation or tracing. The platform layer
 * owns only the stable names; call-side behavior belongs to the Impl wrapper.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "platform_mcu_spi.h"
#include "impl_wrapper/impl_mcu_wrapper.h"

//******************************** Functions ********************************//

platform_err_t platform_spi_init(const platform_spi_cfg_t *cfg)
{
    return impl_spi_init_wrap(cfg);
}

platform_err_t platform_spi_deinit(platform_spi_bus_t bus)
{
    return impl_spi_deinit_wrap(bus);
}

platform_err_t platform_spi_transfer(platform_spi_bus_t bus,
                                     const uint8_t *tx_data,
                                     uint8_t *rx_data,
                                     uint32_t len,
                                     uint32_t timeout_ms)
{
    return impl_spi_transfer_wrap(bus, tx_data, rx_data, len, timeout_ms);
}

platform_err_t platform_spi_write(platform_spi_bus_t bus,
                                  const uint8_t *tx_data,
                                  uint32_t len,
                                  uint32_t timeout_ms)
{
    return impl_spi_write_wrap(bus, tx_data, len, timeout_ms);
}

platform_err_t platform_spi_read(platform_spi_bus_t bus,
                                 uint8_t *rx_data,
                                 uint32_t len,
                                 uint32_t timeout_ms)
{
    return impl_spi_read_wrap(bus, rx_data, len, timeout_ms);
}

platform_err_t platform_spi_write_async(platform_spi_bus_t bus,
                                        const uint8_t *tx_data,
                                        uint32_t len)
{
    return impl_spi_write_async_wrap(bus, tx_data, len);
}

platform_err_t platform_spi_register_complete_cb(
                                        platform_spi_bus_t bus,
                                        platform_spi_complete_cb_t callback,
                                        void *user_data)
{
    return impl_spi_register_complete_cb_wrap(bus, callback, user_data);
}

platform_err_t platform_spi_cs_set(platform_spi_bus_t bus,
                                   platform_gpio_level_t level)
{
    return impl_spi_cs_set_wrap(bus, level);
}

platform_err_t platform_spi_lock(platform_spi_bus_t bus, uint32_t timeout_ms)
{
    return impl_spi_lock_wrap(bus, timeout_ms);
}

platform_err_t platform_spi_unlock(platform_spi_bus_t bus)
{
    return impl_spi_unlock_wrap(bus);
}
