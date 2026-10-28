/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_mcu_uart.c
 *
 * @par dependencies
 * - impl_mcu/impl_mcu_uart.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Adapt the platform UART contract to the MCU (skeleton).
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

#include "impl_mcu/impl_mcu_uart.h"

//******************************** Functions ********************************//

platform_err_t impl_uart_init(const platform_uart_cfg_t *cfg)
{
    (void)cfg;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_uart_deinit(platform_uart_bus_t bus)
{
    (void)bus;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_uart_write(platform_uart_bus_t bus,
                               const uint8_t *data,
                               uint32_t len,
                               uint32_t timeout_ms)
{
    (void)bus;
    (void)data;
    (void)len;
    (void)timeout_ms;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_uart_read(platform_uart_bus_t bus,
                              uint8_t *data,
                              uint32_t len,
                              uint32_t timeout_ms)
{
    (void)bus;
    (void)data;
    (void)len;
    (void)timeout_ms;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_uart_write_async(platform_uart_bus_t bus,
                                     const uint8_t *data,
                                     uint32_t len)
{
    (void)bus;
    (void)data;
    (void)len;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_uart_read_async(platform_uart_bus_t bus,
                                    uint8_t *data,
                                    uint32_t max_len)
{
    (void)bus;
    (void)data;
    (void)max_len;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_uart_get_rx_count(platform_uart_bus_t bus,
                                      uint32_t *rx_len)
{
    (void)bus;

    if (rx_len != NULL)
    {
        *rx_len = 0u;
    }

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_uart_register_event_cb(
                                        platform_uart_bus_t bus,
                                        platform_uart_event_cb_t callback,
                                        void *user_data)
{
    (void)bus;
    (void)callback;
    (void)user_data;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_uart_flush_rx(platform_uart_bus_t bus)
{
    (void)bus;

    return PLATFORM_ERR_NOT_SUPPORTED;
}
