/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_mcu_uart.h
 *
 * @par dependencies
 * - platform_mcu_uart.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Declare the UART implementation adapter.
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

#ifndef __IMPL_MCU_UART_H__
#define __IMPL_MCU_UART_H__

//******************************** Includes *********************************//

#include "platform_mcu_uart.h"

//******************************** Declaring ********************************//

platform_err_t impl_uart_init(const platform_uart_cfg_t *cfg);
platform_err_t impl_uart_deinit(platform_uart_bus_t bus);

platform_err_t impl_uart_write(platform_uart_bus_t bus,
                               const uint8_t *data,
                               uint32_t len,
                               uint32_t timeout_ms);
platform_err_t impl_uart_read(platform_uart_bus_t bus,
                              uint8_t *data,
                              uint32_t len,
                              uint32_t timeout_ms);

platform_err_t impl_uart_write_async(platform_uart_bus_t bus,
                                     const uint8_t *data,
                                     uint32_t len);
platform_err_t impl_uart_read_async(platform_uart_bus_t bus,
                                    uint8_t *data,
                                    uint32_t max_len);

platform_err_t impl_uart_get_rx_count(platform_uart_bus_t bus,
                                      uint32_t *rx_len);

/* Non-blocking drain of the DMA RX ring buffer (see platform_mcu_uart.h). */
platform_err_t impl_uart_rx_read(platform_uart_bus_t bus,
                                 uint8_t *data,
                                 uint32_t max_len,
                                 uint32_t *out_len);
platform_err_t impl_uart_rx_available(platform_uart_bus_t bus,
                                      uint32_t *count);

/*
 * Call from USART3_IRQHandler (USER CODE section). Detects the IDLE flag,
 * clears it and moves the newly received DMA bytes into the ring buffer.
 * Kept free of HAL types so Core does not need vendor headers for it.
 */
void impl_uart_rx_poll_isr(void);

platform_err_t impl_uart_register_event_cb(
                                        platform_uart_bus_t bus,
                                        platform_uart_event_cb_t callback,
                                        void *user_data);
platform_err_t impl_uart_flush_rx(platform_uart_bus_t bus);

#endif /* __IMPL_MCU_UART_H__ */
