/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_bsp_flash.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform external-flash device interface.
 *
 * Addresses are zero-based byte offsets inside the device. The interface does
 * not assume a particular chip: page/sector geometry is reported by
 * platform_flash_get_info() and 04_Impl enforces the chip's alignment rules.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_BSP_FLASH_H__
#define __PLATFORM_BSP_FLASH_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"

//******************************** Declaring ********************************//

typedef struct
{
    uint32_t        capacity_bytes;
    uint32_t        sector_size_bytes;
    uint32_t        page_size_bytes;
    uint16_t        manufacturer_id;
    uint16_t        device_id;
} platform_flash_info_t;

typedef struct
{
    platform_err_t (*init)(void *context);
    platform_err_t (*deinit)(void *context);
    platform_err_t (*read)(void *context,
                           uint32_t address,
                           uint8_t *data,
                           uint32_t length);
    platform_err_t (*write)(void *context,
                            uint32_t address,
                            const uint8_t *data,
                            uint32_t length);
    platform_err_t (*erase)(void *context,
                            uint32_t address,
                            uint32_t length);
    platform_err_t (*get_info)(void *context, platform_flash_info_t *info);
    platform_err_t (*is_busy)(void *context, platform_bool_t *busy);
} platform_flash_ops_t;

/*
 * Register the flash implementation. Must be called before
 * platform_flash_init(). Only one external flash device is assumed.
 */
platform_err_t platform_flash_register(const platform_flash_ops_t *ops,
                                       void *context);

platform_err_t platform_flash_init(void);
platform_err_t platform_flash_deinit(void);

platform_err_t platform_flash_read(uint32_t address,
                                   uint8_t *data,
                                   uint32_t length);

/*
 * Write requires the target region to be erased first. Writes that cross a
 * page boundary return PLATFORM_ERR_PARAM instead of silently splitting, so a
 * caller cannot accidentally corrupt the next page.
 */
platform_err_t platform_flash_write(uint32_t address,
                                    const uint8_t *data,
                                    uint32_t length);

/* Erase the sectors covering [address, address + length). */
platform_err_t platform_flash_erase(uint32_t address, uint32_t length);

platform_err_t platform_flash_get_info(platform_flash_info_t *info);
platform_err_t platform_flash_is_busy(platform_bool_t *busy);

#endif /* __PLATFORM_BSP_FLASH_H__ */
