/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_spi_wrapper.c
 *
 * @par dependencies
 * - impl_wrapper/impl_mcu_wrapper.h
 * - impl_wrapper/impl_trace.h
 * - impl_mcu/impl_mcu_spi.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Add trace hooks around the SPI implementation adapter.
 *
 * The wrapper owns instrumentation only. It must not validate parameters,
 * translate units or call vendor APIs; those belong to impl_mcu_spi.c.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_wrapper/impl_mcu_wrapper.h"
#include "impl_wrapper/impl_trace.h"
#include "impl_mcu/impl_mcu_spi.h"

//******************************** Functions ********************************//

platform_err_t impl_spi_init_wrap(const platform_spi_cfg_t *cfg)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_spi_init");
    err = impl_spi_init(cfg);
    IMPL_TRACE_EXIT("platform_spi_init", err);

    return err;
}

platform_err_t impl_spi_deinit_wrap(platform_spi_bus_t bus)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_spi_deinit");
    err = impl_spi_deinit(bus);
    IMPL_TRACE_EXIT("platform_spi_deinit", err);

    return err;
}

platform_err_t impl_spi_transfer_wrap(platform_spi_bus_t bus,
                                      const uint8_t *tx_data,
                                      uint8_t *rx_data,
                                      uint32_t len,
                                      uint32_t timeout_ms)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_spi_transfer");
    err = impl_spi_transfer(bus, tx_data, rx_data, len, timeout_ms);
    IMPL_TRACE_EXIT("platform_spi_transfer", err);

    return err;
}

platform_err_t impl_spi_write_wrap(platform_spi_bus_t bus,
                                   const uint8_t *tx_data,
                                   uint32_t len,
                                   uint32_t timeout_ms)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_spi_write");
    err = impl_spi_write(bus, tx_data, len, timeout_ms);
    IMPL_TRACE_EXIT("platform_spi_write", err);

    return err;
}

platform_err_t impl_spi_read_wrap(platform_spi_bus_t bus,
                                  uint8_t *rx_data,
                                  uint32_t len,
                                  uint32_t timeout_ms)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_spi_read");
    err = impl_spi_read(bus, rx_data, len, timeout_ms);
    IMPL_TRACE_EXIT("platform_spi_read", err);

    return err;
}

platform_err_t impl_spi_write_async_wrap(platform_spi_bus_t bus,
                                         const uint8_t *tx_data,
                                         uint32_t len)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_spi_write_async");
    err = impl_spi_write_async(bus, tx_data, len);
    IMPL_TRACE_EXIT("platform_spi_write_async", err);

    return err;
}

platform_err_t impl_spi_register_complete_cb_wrap(
                                        platform_spi_bus_t bus,
                                        platform_spi_complete_cb_t callback,
                                        void *user_data)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_spi_register_complete_cb");
    err = impl_spi_register_complete_cb(bus, callback, user_data);
    IMPL_TRACE_EXIT("platform_spi_register_complete_cb", err);

    return err;
}

platform_err_t impl_spi_cs_set_wrap(platform_spi_bus_t bus,
                                    platform_gpio_level_t level)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_spi_cs_set");
    err = impl_spi_cs_set(bus, level);
    IMPL_TRACE_EXIT("platform_spi_cs_set", err);

    return err;
}

platform_err_t impl_spi_lock_wrap(platform_spi_bus_t bus, uint32_t timeout_ms)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_spi_lock");
    err = impl_spi_lock(bus, timeout_ms);
    IMPL_TRACE_EXIT("platform_spi_lock", err);

    return err;
}

platform_err_t impl_spi_unlock_wrap(platform_spi_bus_t bus)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_spi_unlock");
    err = impl_spi_unlock(bus);
    IMPL_TRACE_EXIT("platform_spi_unlock", err);

    return err;
}
