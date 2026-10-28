/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_mcu_uart.c
 *
 * @par dependencies
 * - platform_mcu_uart.h
 * - impl_wrapper/impl_mcu_wrapper.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Forward the platform UART contract to the Impl wrapper.
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

#include "platform_mcu_uart.h"
#include "impl_wrapper/impl_mcu_wrapper.h"

//******************************** Functions ********************************//

platform_err_t platform_uart_init(const platform_uart_cfg_t *cfg)
{
    return impl_uart_init_wrap(cfg);
}

platform_err_t platform_uart_deinit(platform_uart_bus_t bus)
{
    return impl_uart_deinit_wrap(bus);
}

platform_err_t platform_uart_write(platform_uart_bus_t bus,
                                   const uint8_t *data,
                                   uint32_t len,
                                   uint32_t timeout_ms)
{
    return impl_uart_write_wrap(bus, data, len, timeout_ms);
}

platform_err_t platform_uart_read(platform_uart_bus_t bus,
                                  uint8_t *data,
                                  uint32_t len,
                                  uint32_t timeout_ms)
{
    return impl_uart_read_wrap(bus, data, len, timeout_ms);
}

platform_err_t platform_uart_write_async(platform_uart_bus_t bus,
                                         const uint8_t *data,
                                         uint32_t len)
{
    return impl_uart_write_async_wrap(bus, data, len);
}

platform_err_t platform_uart_read_async(platform_uart_bus_t bus,
                                        uint8_t *data,
                                        uint32_t max_len)
{
    return impl_uart_read_async_wrap(bus, data, max_len);
}

platform_err_t platform_uart_get_rx_count(platform_uart_bus_t bus,
                                          uint32_t *rx_len)
{
    return impl_uart_get_rx_count_wrap(bus, rx_len);
}

platform_err_t platform_uart_rx_read(platform_uart_bus_t bus,
                                     uint8_t *data,
                                     uint32_t max_len,
                                     uint32_t *out_len)
{
    return impl_uart_rx_read_wrap(bus, data, max_len, out_len);
}

platform_err_t platform_uart_rx_available(platform_uart_bus_t bus,
                                          uint32_t *count)
{
    return impl_uart_rx_available_wrap(bus, count);
}

platform_err_t platform_uart_register_event_cb(
                                        platform_uart_bus_t bus,
                                        platform_uart_event_cb_t callback,
                                        void *user_data)
{
    return impl_uart_register_event_cb_wrap(bus, callback, user_data);
}

platform_err_t platform_uart_flush_rx(platform_uart_bus_t bus)
{
    return impl_uart_flush_rx_wrap(bus);
}
