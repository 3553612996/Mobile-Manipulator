/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_bsp_motor.h
 *
 * @par dependencies
 * - platform_bsp_motor.h
 * - 05_BSP/bsp_motor_dual_pwm.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Device table and ops adapters for the platform motor contract.
 *
 * There is one shared ops table: every motor on this board is the same
 * dual-PWM bridge, and the wiring differences (PWM channels, encoder channel,
 * mirroring) live in the bsp_motor_dual_pwm_t instance that is passed as the
 * ops context. impl_board fills those instances and registers them.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __IMPL_BSP_MOTOR_H__
#define __IMPL_BSP_MOTOR_H__

//******************************** Includes *********************************//

#include "platform_bsp_motor.h"

//******************************** Declaring ********************************//

/*
 * Pass this table and a bsp_motor_dual_pwm_t * to platform_motor_register().
 * The instance's cfg must be filled before the id is initialized.
 */
extern const platform_motor_ops_t impl_bsp_motor_ops;

#endif /* __IMPL_BSP_MOTOR_H__ */
