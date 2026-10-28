/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_uart_wrapper.c
 *
 * @par dependencies
 * - impl_wrapper/impl_mcu_wrapper.h
 * - impl_wrapper/impl_trace.h
 * - impl_mcu/impl_mcu_uart.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Add trace hooks around the UART implementation adapter.
 *
 * The wrapper owns instrumentation only. It must not validate parameters,
 * translate units or call vendor APIs; those belong to impl_mcu_uart.c.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_wrapper/impl_mcu_wrapper.h"
#include "impl_wrapper/impl_trace.h"
#include "impl_mcu/impl_mcu_uart.h"

//******************************** Functions ********************************//

platform_err_t impl_uart_init_wrap(const platform_uart_cfg_t *cfg)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_uart_init");
    err = impl_uart_init(cfg);
    IMPL_TRACE_EXIT("platform_uart_init", err);

    return err;
}

platform_err_t impl_uart_deinit_wrap(platform_uart_bus_t bus)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_uart_deinit");
    err = impl_uart_deinit(bus);
    IMPL_TRACE_EXIT("platform_uart_deinit", err);

    return err;
}

platform_err_t impl_uart_write_wrap(platform_uart_bus_t bus,
                                    const uint8_t *data,
                                    uint32_t len,
                                    uint32_t timeout_ms)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_uart_write");
    err = impl_uart_write(bus, data, len, timeout_ms);
    IMPL_TRACE_EXIT("platform_uart_write", err);

    return err;
}

platform_err_t impl_uart_read_wrap(platform_uart_bus_t bus,
                                   uint8_t *data,
                                   uint32_t len,
                                   uint32_t timeout_ms)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_uart_read");
    err = impl_uart_read(bus, data, len, timeout_ms);
    IMPL_TRACE_EXIT("platform_uart_read", err);

    return err;
}

platform_err_t impl_uart_write_async_wrap(platform_uart_bus_t bus,
                                          const uint8_t *data,
                                          uint32_t len)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_uart_write_async");
    err = impl_uart_write_async(bus, data, len);
    IMPL_TRACE_EXIT("platform_uart_write_async", err);

    return err;
}

platform_err_t impl_uart_read_async_wrap(platform_uart_bus_t bus,
                                         uint8_t *data,
                                         uint32_t max_len)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_uart_read_async");
    err = impl_uart_read_async(bus, data, max_len);
    IMPL_TRACE_EXIT("platform_uart_read_async", err);

    return err;
}

platform_err_t impl_uart_get_rx_count_wrap(platform_uart_bus_t bus,
                                           uint32_t *rx_len)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_uart_get_rx_count");
    err = impl_uart_get_rx_count(bus, rx_len);
    IMPL_TRACE_EXIT("platform_uart_get_rx_count", err);

    return err;
}

platform_err_t impl_uart_rx_read_wrap(platform_uart_bus_t bus,
                                      uint8_t *data,
                                      uint32_t max_len,
                                      uint32_t *out_len)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_uart_rx_read");
    err = impl_uart_rx_read(bus, data, max_len, out_len);
    IMPL_TRACE_EXIT("platform_uart_rx_read", err);

    return err;
}

platform_err_t impl_uart_rx_available_wrap(platform_uart_bus_t bus,
                                           uint32_t *count)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_uart_rx_available");
    err = impl_uart_rx_available(bus, count);
    IMPL_TRACE_EXIT("platform_uart_rx_available", err);

    return err;
}

platform_err_t impl_uart_register_event_cb_wrap(
                                        platform_uart_bus_t bus,
                                        platform_uart_event_cb_t callback,
                                        void *user_data)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_uart_register_event_cb");
    err = impl_uart_register_event_cb(bus, callback, user_data);
    IMPL_TRACE_EXIT("platform_uart_register_event_cb", err);

    return err;
}

platform_err_t impl_uart_flush_rx_wrap(platform_uart_bus_t bus)
{
    platform_err_t err;

    IMPL_TRACE_ENTER("platform_uart_flush_rx");
    err = impl_uart_flush_rx(bus);
    IMPL_TRACE_EXIT("platform_uart_flush_rx", err);

    return err;
}
