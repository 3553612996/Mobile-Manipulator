/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_mcu_spi.c
 *
 * @par dependencies
 * - impl_mcu/impl_mcu_spi.h
 * - impl_mcu/impl_mcu_gpio.h
 * - spi.h (CubeMX handle)
 * - FreeRTOS.h / semphr.h (bus lock only)
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Adapt the platform SPI contract to SPI2 on STM32F103.
 *
 * Mapping
 * -------
 * The board wires SPI2 to the W25Q64 (PB13 SCK, PB14 MISO, PB15 MOSI) and
 * drives chip-select from PB12 as a plain GPIO, one bus and one device, so
 * logical bus 0 maps to SPI2 and every other bus returns
 * PLATFORM_ERR_NOT_SUPPORTED.
 *
 * Transfer model
 * --------------
 * Blocking HAL calls only. The flash driver moves at most one 256-byte page or
 * a 64-bit status word per call and the storage task is the only user, so DMA
 * would add a channel, an interrupt and a completion state machine without
 * buying latency that matters. platform_spi_write_async() is therefore left
 * unimplemented; it has no caller in this firmware.
 *
 * Chip select
 * -----------
 * cs_mode == PLATFORM_SPI_CS_SOFTWARE: the pin named in cfg->cs_pin is
 * configured as a push-pull output and left deasserted (high) at init. The
 * caller brackets each transaction with platform_spi_cs_set(). cs_mode ==
 * PLATFORM_SPI_CS_NONE is accepted because the bus itself does not care;
 * PLATFORM_SPI_CS_HARDWARE returns NOT_SUPPORTED (NSS is not wired).
 *
 * Bus lock
 * --------
 * A static FreeRTOS mutex created on the first init. The lock is a no-op
 * before the scheduler starts, so a bring-up path in main() can still read the
 * JEDEC ID. It only coordinates whole transactions (command + data under one
 * CS pulse), not individual transfers.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_mcu/impl_mcu_spi.h"
#include "impl_mcu/impl_mcu_gpio.h"

#include "stm32f1xx_hal.h"
#include "spi.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

//******************************** Defines **********************************//

#define IMPL_SPI_BUS                ((platform_spi_bus_t)0u)  /* SPI2 only. */

//******************************** Declaring ********************************//

typedef struct
{
    platform_bool_t         initialized;
    platform_spi_cs_mode_t  cs_mode;
    platform_gpio_t         cs_pin;
} impl_spi_ctx_t;

//******************************** Variables ********************************//

static impl_spi_ctx_t       s_spi;
static StaticSemaphore_t    s_spi_lock_buffer;
static SemaphoreHandle_t    s_spi_lock = NULL;

//******************************** Functions ********************************//

static platform_bool_t impl_spi_bus_ok(platform_spi_bus_t bus)
{
    return (bus == IMPL_SPI_BUS) ? PLATFORM_TRUE : PLATFORM_FALSE;
}

static platform_err_t impl_spi_hal_status(HAL_StatusTypeDef status)
{
    switch (status)
    {
    case HAL_OK:
        return PLATFORM_ERR_OK;
    case HAL_TIMEOUT:
        return PLATFORM_ERR_TIMEOUT;
    case HAL_BUSY:
        return PLATFORM_ERR_BUSY;
    default:
        return PLATFORM_ERR_FAIL;
    }
}

/*
 * Smallest prescaler that keeps the clock at or below max_frequency_hz. SPI2
 * runs from PCLK1 (36 MHz at the current 72 MHz SYSCLK). A max of 0 keeps the
 * CubeMX default.
 */
static uint32_t impl_spi_prescaler(uint32_t max_frequency_hz)
{
    static const uint32_t s_divider[8] =
    {
        2u, 4u, 8u, 16u, 32u, 64u, 128u, 256u
    };
    static const uint32_t s_prescaler[8] =
    {
        SPI_BAUDRATEPRESCALER_2,
        SPI_BAUDRATEPRESCALER_4,
        SPI_BAUDRATEPRESCALER_8,
        SPI_BAUDRATEPRESCALER_16,
        SPI_BAUDRATEPRESCALER_32,
        SPI_BAUDRATEPRESCALER_64,
        SPI_BAUDRATEPRESCALER_128,
        SPI_BAUDRATEPRESCALER_256
    };
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t i;

    if (max_frequency_hz == 0u)
    {
        return SPI_BAUDRATEPRESCALER_8;
    }

    for (i = 0u; i < 8u; i++)
    {
        if ((pclk1 / s_divider[i]) <= max_frequency_hz)
        {
            return s_prescaler[i];
        }
    }

    return SPI_BAUDRATEPRESCALER_256;
}

static platform_err_t impl_spi_apply_cfg(const platform_spi_cfg_t *cfg)
{
    if (cfg->data_width != PLATFORM_SPI_DATA_8BIT)
    {
        /* The HAL counts 16-bit frames, the platform contract counts bytes. */
        return PLATFORM_ERR_NOT_SUPPORTED;
    }

    switch (cfg->mode)
    {
    case PLATFORM_SPI_MODE_0:
        hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
        hspi2.Init.CLKPhase    = SPI_PHASE_1EDGE;
        break;
    case PLATFORM_SPI_MODE_1:
        hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
        hspi2.Init.CLKPhase    = SPI_PHASE_2EDGE;
        break;
    case PLATFORM_SPI_MODE_2:
        hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
        hspi2.Init.CLKPhase    = SPI_PHASE_1EDGE;
        break;
    case PLATFORM_SPI_MODE_3:
        hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
        hspi2.Init.CLKPhase    = SPI_PHASE_2EDGE;
        break;
    default:
        return PLATFORM_ERR_PARAM;
    }

    hspi2.Init.Mode        = SPI_MODE_MASTER;
    hspi2.Init.Direction   = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize    = SPI_DATASIZE_8BIT;
    hspi2.Init.NSS         = SPI_NSS_SOFT;
    hspi2.Init.FirstBit    = (cfg->msb_first == PLATFORM_TRUE)
                                 ? SPI_FIRSTBIT_MSB : SPI_FIRSTBIT_LSB;
    hspi2.Init.BaudRatePrescaler = impl_spi_prescaler(cfg->max_frequency_hz);
    hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial     = 10;

    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        return PLATFORM_ERR_FAIL;
    }

    return PLATFORM_ERR_OK;
}

platform_err_t impl_spi_init(const platform_spi_cfg_t *cfg)
{
    platform_err_t err;

    if (cfg == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (impl_spi_bus_ok(cfg->bus) == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_SUPPORTED;
    }

    if (cfg->cs_mode == PLATFORM_SPI_CS_HARDWARE)
    {
        return PLATFORM_ERR_NOT_SUPPORTED;
    }

    if (s_spi.initialized == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    err = impl_spi_apply_cfg(cfg);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    s_spi.cs_mode = cfg->cs_mode;

    if (cfg->cs_mode == PLATFORM_SPI_CS_SOFTWARE)
    {
        platform_gpio_cfg_t cs_cfg;

        cs_cfg.pin           = cfg->cs_pin;
        cs_cfg.mode          = PLATFORM_GPIO_MODE_OUTPUT_PP;
        cs_cfg.pull          = PLATFORM_GPIO_PULL_NONE;
        cs_cfg.initial_level = PLATFORM_GPIO_LEVEL_HIGH;

        err = impl_gpio_init(&cs_cfg);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }

        s_spi.cs_pin = cfg->cs_pin;
    }

    /*
     * Do not create the mutex here: init runs from board_init() before the
     * scheduler starts, and xSemaphoreCreateMutexStatic enters a kernel
     * critical section that (on this port) leaves BASEPRI masked until the
     * first SVC. The flash init that follows immediately uses HAL_Delay(),
     * which then never returns. impl_spi_lock() creates it on first use after
     * the scheduler is running instead.
     */

    s_spi.initialized = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t impl_spi_deinit(platform_spi_bus_t bus)
{
    if (impl_spi_bus_ok(bus) == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_SUPPORTED;
    }

    if (s_spi.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    (void)HAL_SPI_DeInit(&hspi2);

    if (s_spi.cs_mode == PLATFORM_SPI_CS_SOFTWARE)
    {
        (void)impl_gpio_write(s_spi.cs_pin, PLATFORM_GPIO_LEVEL_HIGH);
    }

    s_spi.cs_mode     = PLATFORM_SPI_CS_NONE;
    s_spi.initialized = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

platform_err_t impl_spi_transfer(platform_spi_bus_t bus,
                                 const uint8_t *tx_data,
                                 uint8_t *rx_data,
                                 uint32_t len,
                                 uint32_t timeout_ms)
{
    HAL_StatusTypeDef status;

    if (impl_spi_bus_ok(bus) == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_SUPPORTED;
    }
    if (s_spi.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }
    if ((tx_data == NULL) || (rx_data == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }
    if (len == 0u)
    {
        return PLATFORM_ERR_OK;
    }
    if (len > 0xFFFFu)
    {
        return PLATFORM_ERR_PARAM;
    }

    status = HAL_SPI_TransmitReceive(&hspi2,
                                     (uint8_t *)tx_data,
                                     rx_data,
                                     (uint16_t)len,
                                     timeout_ms);

    return impl_spi_hal_status(status);
}

platform_err_t impl_spi_write(platform_spi_bus_t bus,
                              const uint8_t *tx_data,
                              uint32_t len,
                              uint32_t timeout_ms)
{
    HAL_StatusTypeDef status;

    if (impl_spi_bus_ok(bus) == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_SUPPORTED;
    }
    if (s_spi.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }
    if (tx_data == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }
    if (len == 0u)
    {
        return PLATFORM_ERR_OK;
    }
    if (len > 0xFFFFu)
    {
        return PLATFORM_ERR_PARAM;
    }

    status = HAL_SPI_Transmit(&hspi2, (uint8_t *)tx_data, (uint16_t)len,
                              timeout_ms);

    return impl_spi_hal_status(status);
}

platform_err_t impl_spi_read(platform_spi_bus_t bus,
                             uint8_t *rx_data,
                             uint32_t len,
                             uint32_t timeout_ms)
{
    HAL_StatusTypeDef status;

    if (impl_spi_bus_ok(bus) == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_SUPPORTED;
    }
    if (s_spi.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }
    if (rx_data == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }
    if (len == 0u)
    {
        return PLATFORM_ERR_OK;
    }
    if (len > 0xFFFFu)
    {
        return PLATFORM_ERR_PARAM;
    }

    /* HAL clocks 0xFF out while receiving, which is what a NOR device wants. */
    status = HAL_SPI_Receive(&hspi2, rx_data, (uint16_t)len, timeout_ms);

    return impl_spi_hal_status(status);
}

platform_err_t impl_spi_write_async(platform_spi_bus_t bus,
                                    const uint8_t *tx_data,
                                    uint32_t len)
{
    (void)bus;
    (void)tx_data;
    (void)len;

    /* No caller: the W25Q64 driver only uses the blocking path. */
    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_spi_register_complete_cb(
                                        platform_spi_bus_t bus,
                                        platform_spi_complete_cb_t callback,
                                        void *user_data)
{
    (void)bus;
    (void)callback;
    (void)user_data;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_spi_cs_set(platform_spi_bus_t bus,
                               platform_gpio_level_t level)
{
    if (impl_spi_bus_ok(bus) == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_SUPPORTED;
    }
    if (s_spi.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }
    if (s_spi.cs_mode != PLATFORM_SPI_CS_SOFTWARE)
    {
        return PLATFORM_ERR_NOT_SUPPORTED;
    }

    return impl_gpio_write(s_spi.cs_pin, level);
}

platform_err_t impl_spi_lock(platform_spi_bus_t bus, uint32_t timeout_ms)
{
    TickType_t ticks;

    if (impl_spi_bus_ok(bus) == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_SUPPORTED;
    }
    if (s_spi.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        /* Bring-up path in main(); nobody else can be using the bus yet. */
        return PLATFORM_ERR_OK;
    }
    if (s_spi_lock == NULL)
    {
        /*
         * Created on the first post-scheduler use: only the storage task
         * touches this bus, so there is no second caller to race with.
         */
        s_spi_lock = xSemaphoreCreateMutexStatic(&s_spi_lock_buffer);
        if (s_spi_lock == NULL)
        {
            return PLATFORM_ERR_NO_MEMORY;
        }
    }

    ticks = (timeout_ms == 0u) ? 0u : pdMS_TO_TICKS(timeout_ms);
    if ((timeout_ms != 0u) && (ticks == 0u))
    {
        ticks = 1u;
    }

    return (xSemaphoreTake(s_spi_lock, ticks) == pdTRUE)
               ? PLATFORM_ERR_OK : PLATFORM_ERR_TIMEOUT;
}

platform_err_t impl_spi_unlock(platform_spi_bus_t bus)
{
    if (impl_spi_bus_ok(bus) == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_SUPPORTED;
    }
    if (s_spi.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }
    if ((s_spi_lock == NULL) ||
        (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED))
    {
        return PLATFORM_ERR_OK;
    }

    return (xSemaphoreGive(s_spi_lock) == pdTRUE)
               ? PLATFORM_ERR_OK : PLATFORM_ERR_FAIL;
}
