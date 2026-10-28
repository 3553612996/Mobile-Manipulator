/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_mcu_uart.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform UART interface.
 *
 * Logical bus IDs are mapped to physical USART instances in 04_Impl. The
 * upper layer never sees USART_HandleTypeDef or HAL_UART_*.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_MCU_UART_H__
#define __PLATFORM_MCU_UART_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"

//******************************** Defines **********************************//

#define PLATFORM_UART_BUS_0         ((platform_uart_bus_t)0u)
#define PLATFORM_UART_BUS_1         ((platform_uart_bus_t)1u)
#define PLATFORM_UART_BUS_2         ((platform_uart_bus_t)2u)
#define PLATFORM_UART_BUS_3         ((platform_uart_bus_t)3u)
#define PLATFORM_UART_BUS_INVALID   ((platform_uart_bus_t)0xFFu)

#define PLATFORM_UART_TIMEOUT_NONE  0u

//******************************** Declaring ********************************//

typedef uint8_t platform_uart_bus_t;

typedef enum
{
    PLATFORM_UART_PARITY_NONE = 0,
    PLATFORM_UART_PARITY_EVEN,
    PLATFORM_UART_PARITY_ODD
} platform_uart_parity_t;

typedef enum
{
    PLATFORM_UART_STOP_BITS_1 = 0,
    PLATFORM_UART_STOP_BITS_2
} platform_uart_stop_bits_t;

typedef enum
{
    PLATFORM_UART_DUPLEX_FULL = 0,
    PLATFORM_UART_DUPLEX_HALF       /* Single-wire / RS-485 style bus. */
} platform_uart_duplex_t;

typedef enum
{
    PLATFORM_UART_EVENT_RX_IDLE = 0,    /* Idle line detected. */
    PLATFORM_UART_EVENT_TX_DONE,        /* Async TX finished. */
    PLATFORM_UART_EVENT_RX_ERROR        /* Overrun/framing/noise/parity. */
} platform_uart_event_t;

typedef struct
{
    platform_uart_bus_t         bus;
    uint32_t                    baudrate;
    uint8_t                     data_bits;      /* 8 or 9. */
    platform_uart_parity_t      parity;
    platform_uart_stop_bits_t   stop_bits;
    platform_uart_duplex_t      duplex;
    platform_bool_t             use_dma;        /* Async transfers use DMA. */
} platform_uart_cfg_t;

typedef void (*platform_uart_event_cb_t)(platform_uart_bus_t bus,
                                         platform_uart_event_t event,
                                         void *user_data);

/*
 * Blocking transfer. timeout_ms == PLATFORM_UART_TIMEOUT_NONE means no wait;
 * the function then returns PLATFORM_ERR_BUSY if the transfer cannot start.
 */
platform_err_t platform_uart_init(const platform_uart_cfg_t *cfg);
platform_err_t platform_uart_deinit(platform_uart_bus_t bus);

platform_err_t platform_uart_write(platform_uart_bus_t bus,
                                   const uint8_t *data,
                                   uint32_t len,
                                   uint32_t timeout_ms);
platform_err_t platform_uart_read(platform_uart_bus_t bus,
                                  uint8_t *data,
                                  uint32_t len,
                                  uint32_t timeout_ms);

/*
 * Asynchronous transfers. The caller's buffer must stay valid until the
 * corresponding TX_DONE / RX_IDLE event is reported.
 */
platform_err_t platform_uart_write_async(platform_uart_bus_t bus,
                                        const uint8_t *data,
                                        uint32_t len);
platform_err_t platform_uart_read_async(platform_uart_bus_t bus,
                                       uint8_t *data,
                                       uint32_t max_len);

/*
 * Number of bytes received by the last platform_uart_read_async().
 * Valid after RX_IDLE, before the next read_async call.
 */
platform_err_t platform_uart_get_rx_count(platform_uart_bus_t bus,
                                         uint32_t *rx_len);

platform_err_t platform_uart_register_event_cb(platform_uart_bus_t bus,
                                              platform_uart_event_cb_t callback,
                                              void *user_data);
platform_err_t platform_uart_flush_rx(platform_uart_bus_t bus);

#endif /* __PLATFORM_MCU_UART_H__ */
