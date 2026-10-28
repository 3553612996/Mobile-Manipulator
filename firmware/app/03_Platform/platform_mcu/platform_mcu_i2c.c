/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_mcu_i2c.c
 *
 * @par dependencies
 * - platform_mcu_i2c.h
 * - impl_wrapper/impl_mcu_wrapper.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Forward the platform I2C contract to the Impl wrapper.
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

#include "platform_mcu_i2c.h"
#include "impl_wrapper/impl_mcu_wrapper.h"

//******************************** Functions ********************************//

platform_err_t platform_i2c_init(const platform_i2c_cfg_t *cfg)
{
    return impl_i2c_init_wrap(cfg);
}

platform_err_t platform_i2c_deinit(platform_i2c_bus_t bus)
{
    return impl_i2c_deinit_wrap(bus);
}

platform_err_t platform_i2c_write(platform_i2c_bus_t bus,
                                  uint16_t dev_addr,
                                  const uint8_t *data,
                                  uint32_t len,
                                  uint32_t timeout_ms)
{
    return impl_i2c_write_wrap(bus, dev_addr, data, len, timeout_ms);
}

platform_err_t platform_i2c_read(platform_i2c_bus_t bus,
                                 uint16_t dev_addr,
                                 uint8_t *data,
                                 uint32_t len,
                                 uint32_t timeout_ms)
{
    return impl_i2c_read_wrap(bus, dev_addr, data, len, timeout_ms);
}

platform_err_t platform_i2c_mem_write(platform_i2c_bus_t bus,
                                      uint16_t dev_addr,
                                      uint16_t mem_addr,
                                      uint8_t mem_addr_size_bytes,
                                      const uint8_t *data,
                                      uint32_t len,
                                      uint32_t timeout_ms)
{
    return impl_i2c_mem_write_wrap(bus, dev_addr, mem_addr,
                                   mem_addr_size_bytes, data, len, timeout_ms);
}

platform_err_t platform_i2c_mem_read(platform_i2c_bus_t bus,
                                     uint16_t dev_addr,
                                     uint16_t mem_addr,
                                     uint8_t mem_addr_size_bytes,
                                     uint8_t *data,
                                     uint32_t len,
                                     uint32_t timeout_ms)
{
    return impl_i2c_mem_read_wrap(bus, dev_addr, mem_addr,
                                  mem_addr_size_bytes, data, len, timeout_ms);
}

platform_err_t platform_i2c_is_ready(platform_i2c_bus_t bus,
                                     uint16_t dev_addr,
                                     uint32_t trials,
                                     uint32_t timeout_ms,
                                     platform_bool_t *ready)
{
    return impl_i2c_is_ready_wrap(bus, dev_addr, trials, timeout_ms, ready);
}

platform_err_t platform_i2c_lock(platform_i2c_bus_t bus, uint32_t timeout_ms)
{
    return impl_i2c_lock_wrap(bus, timeout_ms);
}

platform_err_t platform_i2c_unlock(platform_i2c_bus_t bus)
{
    return impl_i2c_unlock_wrap(bus);
}
