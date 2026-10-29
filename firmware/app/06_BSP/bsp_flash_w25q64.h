/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file bsp_flash_w25q64.h
 *
 * @par dependencies
 * - platform_mcu_spi.h
 * - platform_bsp_flash.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief W25Q64 (8 MiB SPI NOR) driver over platform_spi.
 *
 * Scope
 * -----
 * Command framing, chip-select timing and the BUSY/WEL polling that a NOR flash
 * needs. No cache, no address translation, no erase tracking: the partition
 * map and the wear policy live in service_storage.
 *
 * Geometry enforced here
 * ----------------------
 *   capacity 8 MiB, page 256 B, sector 4 KiB (the only erase unit used).
 * A write must stay inside one page; a cross-page write is rejected rather
 * than split, because a silent split hides a bug in the caller. Erase requires
 * a sector-aligned address and length.
 *
 * Bus behaviour
 * -------------
 * Mode 0, MSB first, 8 bits, software chip-select on cfg.cs_pin. Every public
 * call takes the platform bus lock around its whole transaction (CS asserted
 * to CS released), so reads from two tasks cannot interleave. The lock is a
 * plain (non-recursive) mutex: never call this driver while holding the same
 * platform SPI lock, or the second acquire blocks until it times out.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __BSP_FLASH_W25Q64_H__
#define __BSP_FLASH_W25Q64_H__

//******************************** Includes *********************************//

#include "platform_mcu_spi.h"
#include "platform_bsp_flash.h"

//******************************** Defines **********************************//

#define BSP_FLASH_W25Q64_CAPACITY_BYTES     ((uint32_t)0x800000u)
#define BSP_FLASH_W25Q64_SECTOR_SIZE        ((uint32_t)4096u)
#define BSP_FLASH_W25Q64_PAGE_SIZE          ((uint32_t)256u)

#define BSP_FLASH_W25Q64_MANUFACTURER_ID    ((uint16_t)0x00EFu)
#define BSP_FLASH_W25Q64_DEVICE_ID          ((uint16_t)0x4017u)

#define BSP_FLASH_W25Q64_DEFAULT_HZ         ((uint32_t)4500000u)

//******************************** Declaring ********************************//

typedef struct
{
    platform_spi_bus_t  bus;
    platform_gpio_t     cs_pin;
    uint32_t            max_frequency_hz;   /* 0 -> BSP_FLASH_W25Q64_DEFAULT_HZ */
    uint32_t            transfer_timeout_ms;
} bsp_flash_w25q64_cfg_t;

typedef struct
{
    bsp_flash_w25q64_cfg_t cfg;
    platform_flash_info_t  info;
    uint8_t                jedec_id[3];
    platform_bool_t        initialized;
} bsp_flash_w25q64_t;

//******************************** Functions ********************************//

/*
 * Open the bus, wait tPUW, read the JEDEC ID and fill info. Returns
 * PLATFORM_ERR_FAIL when the device does not answer with EF 40 17, and
 * PLATFORM_ERR_TIMEOUT when it stays busy.
 */
platform_err_t bsp_flash_w25q64_init(bsp_flash_w25q64_t *flash);
platform_err_t bsp_flash_w25q64_deinit(bsp_flash_w25q64_t *flash);

platform_err_t bsp_flash_w25q64_read(bsp_flash_w25q64_t *flash,
                                     uint32_t address,
                                     uint8_t *data,
                                     uint32_t length);

/* length must not cross a 256-byte page and the target must be erased. */
platform_err_t bsp_flash_w25q64_write(bsp_flash_w25q64_t *flash,
                                      uint32_t address,
                                      const uint8_t *data,
                                      uint32_t length);

/* address and length must be multiples of the 4 KiB sector size. */
platform_err_t bsp_flash_w25q64_erase(bsp_flash_w25q64_t *flash,
                                      uint32_t address,
                                      uint32_t length);

platform_err_t bsp_flash_w25q64_get_info(const bsp_flash_w25q64_t *flash,
                                         platform_flash_info_t *info);
platform_err_t bsp_flash_w25q64_is_busy(bsp_flash_w25q64_t *flash,
                                        platform_bool_t *busy);

#endif /* __BSP_FLASH_W25Q64_H__ */
