/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_bsp_buzzer.h
 *
 * @par dependencies
 * - platform_bsp_buzzer.h
 * - 05_BSP/bsp_buzzer_gpio.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Ops adapter for the platform buzzer contract.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __IMPL_BSP_BUZZER_H__
#define __IMPL_BSP_BUZZER_H__

//******************************** Includes *********************************//

#include "platform_bsp_buzzer.h"

//******************************** Declaring ********************************//

/*
 * Pass this table and a bsp_buzzer_gpio_t * to platform_buzzer_register().
 * The instance's cfg (pin and polarity) must be filled first.
 */
extern const platform_buzzer_ops_t impl_bsp_buzzer_ops;

#endif /* __IMPL_BSP_BUZZER_H__ */
