/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file board_init.h
 *
 * @par dependencies
 * - platform_error.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Board composition root: bind concrete devices to platform contracts.
 *
 * main() calls this once after the CubeMX peripheral init and before the
 * scheduler starts. Everything above 04_Impl talks to the devices through
 * platform_bsp_*.h only; the wiring in this directory is the single place that
 * knows which PWM channel or GPIO belongs to which wheel, joint or LED.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __BOARD_INIT_H__
#define __BOARD_INIT_H__

//******************************** Includes *********************************//

#include "platform_error.h"

//******************************** Declaring ********************************//

/*
 * Register every board device and initialize the ones that must be live before
 * the tasks start (servos hold the center pulse, motor bridges stay coasting,
 * the ADC is calibrated).
 *
 * Returns the first error; a device that is not registered makes the matching
 * platform_*_init() return PLATFORM_ERR_NOT_INITIALIZED later.
 */
platform_err_t board_init(void);

#endif /* __BOARD_INIT_H__ */
