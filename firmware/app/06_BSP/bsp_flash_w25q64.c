/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file bsp_flash_w25q64.c
 *
 * @par dependencies
 * - bsp_flash_w25q64.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief W25Q64 command set over the platform SPI contract.
 *
 * Timing constants are the datasheet maxima rounded up: tPP 3 ms -> 50 ms,
 * tSE 400 ms -> 1000 ms, tWEL and tPUW are sub-millisecond and 5 ms.
 *
 * The status register is polled instead of using the suspend/resume or the
 * ready/busy pin, because neither is wired on this board.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "bsp_flash_w25q64.h"

//******************************** Defines **********************************//

#define W25Q_CMD_WRITE_ENABLE       ((uint8_t)0x06u)
#define W25Q_CMD_READ_STATUS1       ((uint8_t)0x05u)
#define W25Q_CMD_READ_DATA          ((uint8_t)0x03u)
#define W25Q_CMD_PAGE_PROGRAM       ((uint8_t)0x02u)
#define W25Q_CMD_SECTOR_ERASE       ((uint8_t)0x20u)
#define W25Q_CMD_JEDEC_ID           ((uint8_t)0x9Fu)

#define W25Q_STATUS_BUSY            ((uint8_t)0x01u)
#define W25Q_STATUS_WEL             ((uint8_t)0x02u)

#define W25Q_JEDEC_LENGTH           (3u)

#define W25Q_POWER_UP_MS            ((uint32_t)5u)
#define W25Q_READY_AFTER_RESET_MS   ((uint32_t)5000u)
#define W25Q_WRITE_ENABLE_MS        ((uint32_t)10u)
#define W25Q_PAGE_PROGRAM_MS        ((uint32_t)50u)
#define W25Q_SECTOR_ERASE_MS        ((uint32_t)1000u)
#define W25Q_BUS_LOCK_MS            ((uint32_t)1000u)
#define W25Q_TRANSFER_TIMEOUT_MS    ((uint32_t)100u)

//******************************** Functions ********************************//

static uint32_t w25q_transfer_timeout(const bsp_flash_w25q64_t *flash)
{
    return (flash->cfg.transfer_timeout_ms == 0u)
               ? W25Q_TRANSFER_TIMEOUT_MS : flash->cfg.transfer_timeout_ms;
}

static platform_bool_t w25q_range_ok(const bsp_flash_w25q64_t *flash,
                                     uint32_t address,
                                     uint32_t length)
{
    if (address >= flash->info.capacity_bytes)
    {
        return PLATFORM_FALSE;
    }

    if (length > (flash->info.capacity_bytes - address))
    {
        return PLATFORM_FALSE;
    }

    return PLATFORM_TRUE;
}

/* CS is already asserted by the caller. */
static platform_err_t w25q_send_header(bsp_flash_w25q64_t *flash,
                                       uint8_t command,
                                       uint32_t address,
                                       platform_bool_t with_address)
{
    uint8_t  header[4];
    uint32_t length = 1u;

    header[0] = command;

    if (with_address == PLATFORM_TRUE)
    {
        header[1] = (uint8_t)((address >> 16) & 0xFFu);
        header[2] = (uint8_t)((address >> 8) & 0xFFu);
        header[3] = (uint8_t)(address & 0xFFu);
        length    = 4u;
    }

    return platform_spi_write(flash->cfg.bus, header, length,
                              w25q_transfer_timeout(flash));
}

static platform_err_t w25q_read_status(bsp_flash_w25q64_t *flash,
                                       uint8_t *status)
{
    platform_err_t err;

    *status = 0xFFu;

    err = platform_spi_cs_set(flash->cfg.bus, PLATFORM_GPIO_LEVEL_LOW);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = w25q_send_header(flash, W25Q_CMD_READ_STATUS1, 0u, PLATFORM_FALSE);
    if (PLATFORM_IS_OK(err))
    {
        err = platform_spi_read(flash->cfg.bus, status, 1u,
                                w25q_transfer_timeout(flash));
    }

    (void)platform_spi_cs_set(flash->cfg.bus, PLATFORM_GPIO_LEVEL_HIGH);

    return err;
}

static platform_err_t w25q_wait_ready(bsp_flash_w25q64_t *flash,
                                      uint32_t timeout_ms)
{
    platform_err_t err;
    uint8_t        status;
    uint32_t       waited;

    for (waited = 0u; ; waited++)
    {
        err = w25q_read_status(flash, &status);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }

        if ((status & W25Q_STATUS_BUSY) == 0u)
        {
            return PLATFORM_ERR_OK;
        }

        if (waited >= timeout_ms)
        {
            return PLATFORM_ERR_TIMEOUT;
        }

        PLATFORM_DELAY_MS(1u);
    }
}

static platform_err_t w25q_write_enable(bsp_flash_w25q64_t *flash)
{
    platform_err_t err;
    uint8_t        status;
    uint32_t       waited;

    err = platform_spi_cs_set(flash->cfg.bus, PLATFORM_GPIO_LEVEL_LOW);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = w25q_send_header(flash, W25Q_CMD_WRITE_ENABLE, 0u, PLATFORM_FALSE);

    (void)platform_spi_cs_set(flash->cfg.bus, PLATFORM_GPIO_LEVEL_HIGH);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    /* A dead bus latches nothing; do not let the program/erase run blind. */
    for (waited = 0u; waited < W25Q_WRITE_ENABLE_MS; waited++)
    {
        err = w25q_read_status(flash, &status);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }
        if ((status & W25Q_STATUS_WEL) != 0u)
        {
            return PLATFORM_ERR_OK;
        }

        PLATFORM_DELAY_MS(1u);
    }

    return PLATFORM_ERR_TIMEOUT;
}

static platform_err_t w25q_read_jedec(bsp_flash_w25q64_t *flash)
{
    platform_err_t err;

    err = platform_spi_cs_set(flash->cfg.bus, PLATFORM_GPIO_LEVEL_LOW);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = w25q_send_header(flash, W25Q_CMD_JEDEC_ID, 0u, PLATFORM_FALSE);
    if (PLATFORM_IS_OK(err))
    {
        err = platform_spi_read(flash->cfg.bus, flash->jedec_id,
                                W25Q_JEDEC_LENGTH,
                                w25q_transfer_timeout(flash));
    }

    (void)platform_spi_cs_set(flash->cfg.bus, PLATFORM_GPIO_LEVEL_HIGH);

    return err;
}

static platform_err_t w25q_read_block(bsp_flash_w25q64_t *flash,
                                      uint32_t address,
                                      uint8_t *data,
                                      uint32_t length)
{
    platform_err_t err;

    err = platform_spi_cs_set(flash->cfg.bus, PLATFORM_GPIO_LEVEL_LOW);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = w25q_send_header(flash, W25Q_CMD_READ_DATA, address, PLATFORM_TRUE);
    if (PLATFORM_IS_OK(err))
    {
        err = platform_spi_read(flash->cfg.bus, data, length,
                                w25q_transfer_timeout(flash));
    }

    (void)platform_spi_cs_set(flash->cfg.bus, PLATFORM_GPIO_LEVEL_HIGH);

    return err;
}

static platform_err_t w25q_page_program(bsp_flash_w25q64_t *flash,
                                        uint32_t address,
                                        const uint8_t *data,
                                        uint32_t length)
{
    platform_err_t err;

    err = w25q_write_enable(flash);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = platform_spi_cs_set(flash->cfg.bus, PLATFORM_GPIO_LEVEL_LOW);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = w25q_send_header(flash, W25Q_CMD_PAGE_PROGRAM, address,
                           PLATFORM_TRUE);
    if (PLATFORM_IS_OK(err))
    {
        err = platform_spi_write(flash->cfg.bus, data, length,
                                 w25q_transfer_timeout(flash));
    }

    (void)platform_spi_cs_set(flash->cfg.bus, PLATFORM_GPIO_LEVEL_HIGH);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    return w25q_wait_ready(flash, W25Q_PAGE_PROGRAM_MS);
}

static platform_err_t w25q_sector_erase(bsp_flash_w25q64_t *flash,
                                        uint32_t address)
{
    platform_err_t err;

    err = w25q_write_enable(flash);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = platform_spi_cs_set(flash->cfg.bus, PLATFORM_GPIO_LEVEL_LOW);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = w25q_send_header(flash, W25Q_CMD_SECTOR_ERASE, address,
                           PLATFORM_TRUE);

    (void)platform_spi_cs_set(flash->cfg.bus, PLATFORM_GPIO_LEVEL_HIGH);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    return w25q_wait_ready(flash, W25Q_SECTOR_ERASE_MS);
}

platform_err_t bsp_flash_w25q64_init(bsp_flash_w25q64_t *flash)
{
    platform_spi_cfg_t cfg;
    platform_err_t     err;
    uint16_t           manufacturer;
    uint16_t           device;

    if (flash == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (flash->initialized == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    if (flash->cfg.max_frequency_hz == 0u)
    {
        flash->cfg.max_frequency_hz = BSP_FLASH_W25Q64_DEFAULT_HZ;
    }
    if (flash->cfg.transfer_timeout_ms == 0u)
    {
        flash->cfg.transfer_timeout_ms = W25Q_TRANSFER_TIMEOUT_MS;
    }

    cfg.bus              = flash->cfg.bus;
    cfg.max_frequency_hz = flash->cfg.max_frequency_hz;
    cfg.mode             = PLATFORM_SPI_MODE_0;
    cfg.data_width       = PLATFORM_SPI_DATA_8BIT;
    cfg.msb_first        = PLATFORM_TRUE;
    cfg.cs_mode          = PLATFORM_SPI_CS_SOFTWARE;
    cfg.cs_pin           = flash->cfg.cs_pin;

    err = platform_spi_init(&cfg);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    /* tPUW: no command is accepted until the internal supply is stable. */
    PLATFORM_DELAY_MS(W25Q_POWER_UP_MS);

    /*
     * A warm reset can land in the middle of an erase. JEDEC reads are ignored
     * while BUSY, so wait for the device before asking it who it is.
     */
    err = w25q_wait_ready(flash, W25Q_READY_AFTER_RESET_MS);
    if (PLATFORM_IS_OK(err))
    {
        err = w25q_read_jedec(flash);
    }

    if (PLATFORM_IS_OK(err))
    {
        /* JEDEC: byte 0 manufacturer, byte 1 memory type, byte 2 capacity. */
        manufacturer = (uint16_t)flash->jedec_id[0];
        device       = (uint16_t)(((uint16_t)flash->jedec_id[1] << 8) |
                                  flash->jedec_id[2]);

        if ((manufacturer != BSP_FLASH_W25Q64_MANUFACTURER_ID) ||
            (device != BSP_FLASH_W25Q64_DEVICE_ID))
        {
            err = PLATFORM_ERR_FAIL;
        }
    }

    if (PLATFORM_IS_ERR(err))
    {
        (void)platform_spi_deinit(flash->cfg.bus);
        return err;
    }

    flash->info.capacity_bytes    = BSP_FLASH_W25Q64_CAPACITY_BYTES;
    flash->info.sector_size_bytes = BSP_FLASH_W25Q64_SECTOR_SIZE;
    flash->info.page_size_bytes   = BSP_FLASH_W25Q64_PAGE_SIZE;
    flash->info.manufacturer_id   = BSP_FLASH_W25Q64_MANUFACTURER_ID;
    flash->info.device_id         = BSP_FLASH_W25Q64_DEVICE_ID;

    flash->initialized = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t bsp_flash_w25q64_deinit(bsp_flash_w25q64_t *flash)
{
    platform_err_t err;

    if (flash == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (flash->initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = platform_spi_deinit(flash->cfg.bus);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    flash->initialized = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

platform_err_t bsp_flash_w25q64_read(bsp_flash_w25q64_t *flash,
                                     uint32_t address,
                                     uint8_t *data,
                                     uint32_t length)
{
    platform_err_t err;

    if ((flash == NULL) || (data == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (flash->initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (w25q_range_ok(flash, address, length) == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (length == 0u)
    {
        return PLATFORM_ERR_OK;
    }

    err = platform_spi_lock(flash->cfg.bus, W25Q_BUS_LOCK_MS);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = w25q_read_block(flash, address, data, length);

    (void)platform_spi_unlock(flash->cfg.bus);

    return err;
}

platform_err_t bsp_flash_w25q64_write(bsp_flash_w25q64_t *flash,
                                      uint32_t address,
                                      const uint8_t *data,
                                      uint32_t length)
{
    platform_err_t err;

    if ((flash == NULL) || (data == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (flash->initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (w25q_range_ok(flash, address, length) == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (length == 0u)
    {
        return PLATFORM_ERR_OK;
    }

    if (((address % BSP_FLASH_W25Q64_PAGE_SIZE) + length) >
        BSP_FLASH_W25Q64_PAGE_SIZE)
    {
        return PLATFORM_ERR_PARAM;
    }

    err = platform_spi_lock(flash->cfg.bus, W25Q_BUS_LOCK_MS);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = w25q_wait_ready(flash, W25Q_PAGE_PROGRAM_MS);
    if (PLATFORM_IS_OK(err))
    {
        err = w25q_page_program(flash, address, data, length);
    }

    (void)platform_spi_unlock(flash->cfg.bus);

    return err;
}

platform_err_t bsp_flash_w25q64_erase(bsp_flash_w25q64_t *flash,
                                      uint32_t address,
                                      uint32_t length)
{
    platform_err_t err;
    uint32_t       sector = address;

    if (flash == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (flash->initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if ((length == 0u) ||
        ((address % BSP_FLASH_W25Q64_SECTOR_SIZE) != 0u) ||
        ((length % BSP_FLASH_W25Q64_SECTOR_SIZE) != 0u) ||
        (w25q_range_ok(flash, address, length) == PLATFORM_FALSE))
    {
        return PLATFORM_ERR_PARAM;
    }

    err = platform_spi_lock(flash->cfg.bus, W25Q_BUS_LOCK_MS);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    while ((length > 0u) && PLATFORM_IS_OK(err))
    {
        err = w25q_sector_erase(flash, sector);
        sector += BSP_FLASH_W25Q64_SECTOR_SIZE;
        length -= BSP_FLASH_W25Q64_SECTOR_SIZE;
    }

    (void)platform_spi_unlock(flash->cfg.bus);

    return err;
}

platform_err_t bsp_flash_w25q64_get_info(const bsp_flash_w25q64_t *flash,
                                         platform_flash_info_t *info)
{
    if ((flash == NULL) || (info == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (flash->initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    *info = flash->info;

    return PLATFORM_ERR_OK;
}

platform_err_t bsp_flash_w25q64_is_busy(bsp_flash_w25q64_t *flash,
                                        platform_bool_t *busy)
{
    platform_err_t err;
    uint8_t        status;

    if ((flash == NULL) || (busy == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (flash->initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = platform_spi_lock(flash->cfg.bus, W25Q_BUS_LOCK_MS);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = w25q_read_status(flash, &status);
    if (PLATFORM_IS_OK(err))
    {
        *busy = ((status & W25Q_STATUS_BUSY) != 0u)
                    ? PLATFORM_TRUE : PLATFORM_FALSE;
    }

    (void)platform_spi_unlock(flash->cfg.bus);

    return err;
}
