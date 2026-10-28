/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_mcu_spi.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 * - platform_mcu_gpio.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform SPI interface.
 *
 * Logical bus IDs are mapped to physical SPI instances in 04_Impl. Chip-select
 * handling is explicit so the caller controls transaction boundaries.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_MCU_SPI_H__
#define __PLATFORM_MCU_SPI_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"
#include "platform_mcu_gpio.h"

//******************************** Defines **********************************//

#define PLATFORM_SPI_BUS_0          ((platform_spi_bus_t)0u)
#define PLATFORM_SPI_BUS_1          ((platform_spi_bus_t)1u)
#define PLATFORM_SPI_BUS_2          ((platform_spi_bus_t)2u)
#define PLATFORM_SPI_BUS_3          ((platform_spi_bus_t)3u)
#define PLATFORM_SPI_BUS_INVALID    ((platform_spi_bus_t)0xFFu)

//******************************** Declaring ********************************//

typedef uint8_t platform_spi_bus_t;

typedef enum
{
    PLATFORM_SPI_MODE_0 = 0,
    PLATFORM_SPI_MODE_1,
    PLATFORM_SPI_MODE_2,
    PLATFORM_SPI_MODE_3
} platform_spi_mode_t;

typedef enum
{
    PLATFORM_SPI_DATA_8BIT = 0,
    PLATFORM_SPI_DATA_16BIT
} platform_spi_data_width_t;

typedef enum
{
    PLATFORM_SPI_CS_NONE = 0,       /* No chip-select managed by the platform. */
    PLATFORM_SPI_CS_SOFTWARE,       /* Use platform_spi_cs_set(). */
    PLATFORM_SPI_CS_HARDWARE        /* Peripheral drives NSS automatically. */
} platform_spi_cs_mode_t;

typedef struct
{
    platform_spi_bus_t          bus;
    uint32_t                    max_frequency_hz;
    platform_spi_mode_t         mode;
    platform_spi_data_width_t   data_width;
    platform_bool_t             msb_first;
    platform_spi_cs_mode_t      cs_mode;
    platform_gpio_t             cs_pin;     /* Valid when cs_mode == SOFTWARE. */
} platform_spi_cfg_t;

/*
 * Completion callback for platform_spi_write_async(). Runs in ISR context.
 * The caller's TX buffer must stay valid until this callback fires.
 */
typedef void (*platform_spi_complete_cb_t)(platform_spi_bus_t bus,
                                           void *user_data);

/*
 * Blocking transfers. tx_data or rx_data may be NULL for a write-only or
 * read-only transfer. When cs_mode == SOFTWARE the caller is responsible for
 * asserting/deasserting CS around the transfer with platform_spi_cs_set().
 */
platform_err_t platform_spi_init(const platform_spi_cfg_t *cfg);
platform_err_t platform_spi_deinit(platform_spi_bus_t bus);

platform_err_t platform_spi_transfer(platform_spi_bus_t bus,
                                     const uint8_t *tx_data,
                                     uint8_t *rx_data,
                                     uint32_t len,
                                     uint32_t timeout_ms);
platform_err_t platform_spi_write(platform_spi_bus_t bus,
                                  const uint8_t *tx_data,
                                  uint32_t len,
                                  uint32_t timeout_ms);
platform_err_t platform_spi_read(platform_spi_bus_t bus,
                                 uint8_t *rx_data,
                                 uint32_t len,
                                 uint32_t timeout_ms);

platform_err_t platform_spi_write_async(platform_spi_bus_t bus,
                                       const uint8_t *tx_data,
                                       uint32_t len);
platform_err_t platform_spi_register_complete_cb(
                                        platform_spi_bus_t bus,
                                        platform_spi_complete_cb_t callback,
                                        void *user_data);

platform_err_t platform_spi_cs_set(platform_spi_bus_t bus,
                                   platform_gpio_level_t level);

/*
 * Optional bus lock for multi-transfer transactions. The implementation may
 * use an OS mutex; callers must not hold it in an ISR.
 */
platform_err_t platform_spi_lock(platform_spi_bus_t bus,
                                 uint32_t timeout_ms);
platform_err_t platform_spi_unlock(platform_spi_bus_t bus);

#endif /* __PLATFORM_MCU_SPI_H__ */
