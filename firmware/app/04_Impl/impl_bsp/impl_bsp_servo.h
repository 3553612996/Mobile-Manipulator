/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_bsp_servo.h
 *
 * @par dependencies
 * - platform_bsp_servo.h
 * - 06_BSP/bsp_servo_bus.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Device table and ops adapters for the platform servo contract.
 *
 * @version V2.0 2026-10-31
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __IMPL_BSP_SERVO_H__
#define __IMPL_BSP_SERVO_H__

//******************************** Includes *********************************//

#include "platform_bsp_servo.h"

//******************************** Declaring ********************************//

/*
 * Pass this table and a bsp_servo_bus_t * to platform_servo_register(). The
 * bus cfg (UART, joint count, pulse range, gap) must be filled first; the
 * ID/zero map follows via platform_servo_set_joint_map().
 */
extern const platform_servo_ops_t impl_bsp_servo_ops;

#endif /* __IMPL_BSP_SERVO_H__ */
