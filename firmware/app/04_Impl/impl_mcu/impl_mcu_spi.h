/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_mcu_spi.h
 *
 * @par dependencies
 * - platform_mcu_spi.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Declare the SPI implementation adapter.
 *
 * These functions contain the parameter translation and the vendor API calls.
 * The Impl wrapper is the only caller. Do not include this header from
 * 03_Platform or any layer above it.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __IMPL_MCU_SPI_H__
#define __IMPL_MCU_SPI_H__

//******************************** Includes *********************************//

#include "platform_mcu_spi.h"

//******************************** Declaring ********************************//

platform_err_t impl_spi_init(const platform_spi_cfg_t *cfg);
platform_err_t impl_spi_deinit(platform_spi_bus_t bus);

platform_err_t impl_spi_transfer(platform_spi_bus_t bus,
                                 const uint8_t *tx_data,
                                 uint8_t *rx_data,
                                 uint32_t len,
                                 uint32_t timeout_ms);
platform_err_t impl_spi_write(platform_spi_bus_t bus,
                              const uint8_t *tx_data,
                              uint32_t len,
                              uint32_t timeout_ms);
platform_err_t impl_spi_read(platform_spi_bus_t bus,
                             uint8_t *rx_data,
                             uint32_t len,
                             uint32_t timeout_ms);

platform_err_t impl_spi_write_async(platform_spi_bus_t bus,
                                    const uint8_t *tx_data,
                                    uint32_t len);
platform_err_t impl_spi_register_complete_cb(
                                        platform_spi_bus_t bus,
                                        platform_spi_complete_cb_t callback,
                                        void *user_data);

platform_err_t impl_spi_cs_set(platform_spi_bus_t bus,
                               platform_gpio_level_t level);

platform_err_t impl_spi_lock(platform_spi_bus_t bus, uint32_t timeout_ms);
platform_err_t impl_spi_unlock(platform_spi_bus_t bus);

#endif /* __IMPL_MCU_SPI_H__ */
