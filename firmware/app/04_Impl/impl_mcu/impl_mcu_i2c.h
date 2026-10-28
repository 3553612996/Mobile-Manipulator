/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_mcu_i2c.h
 *
 * @par dependencies
 * - platform_mcu_i2c.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Declare the I2C implementation adapter.
 *
 * These functions contain the parameter translation (including the 7-bit to
 * 8-bit address shift) and the vendor API calls. The Impl wrapper is the only
 * caller. Do not include this header from 03_Platform or any layer above it.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __IMPL_MCU_I2C_H__
#define __IMPL_MCU_I2C_H__

//******************************** Includes *********************************//

#include "platform_mcu_i2c.h"

//******************************** Declaring ********************************//

platform_err_t impl_i2c_init(const platform_i2c_cfg_t *cfg);
platform_err_t impl_i2c_deinit(platform_i2c_bus_t bus);

platform_err_t impl_i2c_write(platform_i2c_bus_t bus,
                              uint16_t dev_addr,
                              const uint8_t *data,
                              uint32_t len,
                              uint32_t timeout_ms);
platform_err_t impl_i2c_read(platform_i2c_bus_t bus,
                             uint16_t dev_addr,
                             uint8_t *data,
                             uint32_t len,
                             uint32_t timeout_ms);

platform_err_t impl_i2c_mem_write(platform_i2c_bus_t bus,
                                  uint16_t dev_addr,
                                  uint16_t mem_addr,
                                  uint8_t mem_addr_size_bytes,
                                  const uint8_t *data,
                                  uint32_t len,
                                  uint32_t timeout_ms);
platform_err_t impl_i2c_mem_read(platform_i2c_bus_t bus,
                                 uint16_t dev_addr,
                                 uint16_t mem_addr,
                                 uint8_t mem_addr_size_bytes,
                                 uint8_t *data,
                                 uint32_t len,
                                 uint32_t timeout_ms);

platform_err_t impl_i2c_is_ready(platform_i2c_bus_t bus,
                                 uint16_t dev_addr,
                                 uint32_t trials,
                                 uint32_t timeout_ms,
                                 platform_bool_t *ready);

platform_err_t impl_i2c_lock(platform_i2c_bus_t bus, uint32_t timeout_ms);
platform_err_t impl_i2c_unlock(platform_i2c_bus_t bus);

#endif /* __IMPL_MCU_I2C_H__ */
