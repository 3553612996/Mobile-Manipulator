/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_i2c_wrapper.c
 *
 * @par dependencies
 * - impl_wrapper/impl_mcu_wrapper.h
 * - impl_wrapper/impl_trace.h
 * - impl_mcu/impl_mcu_i2c.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Add trace hooks around the I2C implementation adapter.
 *
 * The wrapper owns instrumentation only. It must not validate parameters,
 * translate units or call vendor APIs; those belong to impl_mcu_i2c.c.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_wrapper/impl_mcu_wrapper.h"
#include "impl_wrapper/impl_trace.h"
#include "impl_mcu/impl_mcu_i2c.h"

//******************************** Functions ********************************//

platform_err_t impl_i2c_init_wrap(const platform_i2c_cfg_t *cfg)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_i2c_init");
    err = impl_i2c_init(cfg);
    IMPL_TRACE_EXIT("platform_i2c_init", err);

    return err;
}

platform_err_t impl_i2c_deinit_wrap(platform_i2c_bus_t bus)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_i2c_deinit");
    err = impl_i2c_deinit(bus);
    IMPL_TRACE_EXIT("platform_i2c_deinit", err);

    return err;
}

platform_err_t impl_i2c_write_wrap(platform_i2c_bus_t bus,
                                   uint16_t dev_addr,
                                   const uint8_t *data,
                                   uint32_t len,
                                   uint32_t timeout_ms)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_i2c_write");
    err = impl_i2c_write(bus, dev_addr, data, len, timeout_ms);
    IMPL_TRACE_EXIT("platform_i2c_write", err);

    return err;
}

platform_err_t impl_i2c_read_wrap(platform_i2c_bus_t bus,
                                  uint16_t dev_addr,
                                  uint8_t *data,
                                  uint32_t len,
                                  uint32_t timeout_ms)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_i2c_read");
    err = impl_i2c_read(bus, dev_addr, data, len, timeout_ms);
    IMPL_TRACE_EXIT("platform_i2c_read", err);

    return err;
}

platform_err_t impl_i2c_mem_write_wrap(platform_i2c_bus_t bus,
                                       uint16_t dev_addr,
                                       uint16_t mem_addr,
                                       uint8_t mem_addr_size_bytes,
                                       const uint8_t *data,
                                       uint32_t len,
                                       uint32_t timeout_ms)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_i2c_mem_write");
    err = impl_i2c_mem_write(bus, dev_addr, mem_addr,
                             mem_addr_size_bytes, data, len, timeout_ms);
    IMPL_TRACE_EXIT("platform_i2c_mem_write", err);

    return err;
}

platform_err_t impl_i2c_mem_read_wrap(platform_i2c_bus_t bus,
                                      uint16_t dev_addr,
                                      uint16_t mem_addr,
                                      uint8_t mem_addr_size_bytes,
                                      uint8_t *data,
                                      uint32_t len,
                                      uint32_t timeout_ms)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_i2c_mem_read");
    err = impl_i2c_mem_read(bus, dev_addr, mem_addr,
                            mem_addr_size_bytes, data, len, timeout_ms);
    IMPL_TRACE_EXIT("platform_i2c_mem_read", err);

    return err;
}

platform_err_t impl_i2c_is_ready_wrap(platform_i2c_bus_t bus,
                                      uint16_t dev_addr,
                                      uint32_t trials,
                                      uint32_t timeout_ms,
                                      platform_bool_t *ready)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_i2c_is_ready");
    err = impl_i2c_is_ready(bus, dev_addr, trials, timeout_ms, ready);
    IMPL_TRACE_EXIT("platform_i2c_is_ready", err);

    return err;
}

platform_err_t impl_i2c_lock_wrap(platform_i2c_bus_t bus,
                                  uint32_t timeout_ms)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_i2c_lock");
    err = impl_i2c_lock(bus, timeout_ms);
    IMPL_TRACE_EXIT("platform_i2c_lock", err);

    return err;
}

platform_err_t impl_i2c_unlock_wrap(platform_i2c_bus_t bus)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_i2c_unlock");
    err = impl_i2c_unlock(bus);
    IMPL_TRACE_EXIT("platform_i2c_unlock", err);

    return err;
}
