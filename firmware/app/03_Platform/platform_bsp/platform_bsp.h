/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_bsp.h
 *
 * @par dependencies
 * - platform_bsp_motor.h
 * - platform_bsp_servo.h
 * - platform_bsp_flash.h
 * - platform_bsp_temphumi.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Aggregate header for the platform BSP device interfaces.
 *
 * Upper layers include this file only. They must not include a concrete chip
 * driver header. 04_Impl registers the ops tables at board init time.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_BSP_H__
#define __PLATFORM_BSP_H__

//******************************** Includes *********************************//

#include "platform_bsp_motor.h"
#include "platform_bsp_servo.h"
#include "platform_bsp_key.h"
#include "platform_bsp_led.h"
#include "platform_bsp_buzzer.h"
#include "platform_bsp_battery.h"
#include "platform_bsp_flash.h"
#include "platform_bsp_temphumi.h"

#endif /* __PLATFORM_BSP_H__ */
