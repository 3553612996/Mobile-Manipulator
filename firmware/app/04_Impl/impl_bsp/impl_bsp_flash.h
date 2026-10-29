/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_bsp_flash.h
 *
 * @par dependencies
 * - platform_bsp_flash.h
 * - 06_BSP/bsp_flash_w25q64.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Ops adapter for the platform external-flash contract.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __IMPL_BSP_FLASH_H__
#define __IMPL_BSP_FLASH_H__

//******************************** Includes *********************************//

#include "platform_bsp_flash.h"

//******************************** Declaring ********************************//

/*
 * Pass this table and a bsp_flash_w25q64_t * to platform_flash_register().
 * The instance's cfg (bus, chip-select pin, frequency) must be filled first.
 */
extern const platform_flash_ops_t impl_bsp_flash_ops;

#endif /* __IMPL_BSP_FLASH_H__ */
