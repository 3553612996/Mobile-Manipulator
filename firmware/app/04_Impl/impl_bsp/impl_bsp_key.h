/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_bsp_key.h
 *
 * @par dependencies
 * - platform_bsp_key.h
 * - 06_BSP/bsp_key_gpio.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Device table and ops adapters for the platform key contract.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __IMPL_BSP_KEY_H__
#define __IMPL_BSP_KEY_H__

//******************************** Includes *********************************//

#include "platform_bsp_key.h"

//******************************** Declaring ********************************//

/*
 * Pass this table and a bsp_key_gpio_t * to platform_key_register().
 * The instance's cfg (pin, pull, polarity, debounce) must be filled first.
 */
extern const platform_key_ops_t impl_bsp_key_ops;

#endif /* __IMPL_BSP_KEY_H__ */
