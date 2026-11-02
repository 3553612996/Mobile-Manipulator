/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_motion_test.h
 *
 * @par dependencies
 * - platform_def.h
 * - service_key.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief On-board key motion test for untethered odometry calibration.
 *
 * The test uses the single user key and the chassis odometry, so the robot can
 * drive a known distance and a known angle without a laptop, USB cable or
 * host:
 *
 *   SHORT_PRESS  drive forward APP_MOTION_TEST_STRAIGHT_MM, then stop
 *   LONG_PRESS   rotate APP_MOTION_TEST_SPIN_CDEG in place, then stop
 *   SHORT_PRESS  while moving: abort and stop
 *
 * The distance/angle are taken from the reported odometry, not from a timer,
 * so the result can be compared with a tape measure afterwards. The test
 * enables the chassis itself; if a host had already enabled it, the test
 * leaves it enabled and only stops the motion.
 *
 * This module has no obstacle detection. Clear the floor before pressing the
 * key, and keep the power switch within reach. In test mode the key's long
 * press no longer latches the emergency stop; the protocol ESTOP still does.
 *
 * @version V1.0 2026-11-02
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __APP_MOTION_TEST_H__
#define __APP_MOTION_TEST_H__

//******************************** Includes *********************************//

#include "platform_def.h"
#include "service_key/service_key.h"

//******************************** Declaring ********************************//

typedef enum
{
    APP_MOTION_TEST_NONE = 0,
    APP_MOTION_TEST_STRAIGHT,
    APP_MOTION_TEST_SPIN
} app_motion_test_mode_t;

/*
 * service_key callback shim. Call from the control task. A press while a move
 * is already running requests an abort instead of starting a second move.
 */
void app_motion_test_on_key(service_key_event_t event, uint32_t now_ms);

/*
 * Advance the state machine. Call every control period, after the chassis
 * odometry has been integrated, so the distance/angle check sees this tick.
 */
void app_motion_test_tick(uint32_t now_ms);

/* Current mode, for tests and for a future status report. */
app_motion_test_mode_t app_motion_test_mode(void);

/* TRUE from the tick that starts a move until the tick that stops it. */
platform_bool_t app_motion_test_running(void);

#endif /* __APP_MOTION_TEST_H__ */
