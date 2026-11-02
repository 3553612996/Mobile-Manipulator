/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_motion_test.c
 *
 * @par dependencies
 * - app_motion_test.h
 * - app_state.h
 * - service_chassis.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement the key-driven straight/spin calibration moves.
 *
 * Time is injected from the control task, and every service call stays in that
 * task: the key service, the chassis apply() and this module share the same
 * context. The start position/heading are sampled on the first tick after a
 * press, not inside the key callback, so the odometry snapshot belongs to a
 * completed 10 ms interval.
 *
 * @version V1.0 2026-11-02
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "app_robot/app_motion_test.h"
#include "app_robot/app_state.h"
#include "app_robot/app_robot_config.h"

#include "service_chassis/service_chassis.h"

//******************************** Defines **********************************//

/*
 * A move that takes this much longer than expected is stopped. Two times the
 * estimate plus 3 s still covers a low battery or a loaded chassis, while not
 * pushing against an obstacle for half a minute.
 */
#define APP_MOTION_TEST_TIMEOUT_FACTOR      ((uint32_t)2u)
#define APP_MOTION_TEST_TIMEOUT_MARGIN_MS   ((uint32_t)3000u)

/* Full turn in centi-degrees around the heading wrap point. */
#define APP_MOTION_TEST_HEADING_HALF_TURN   ((int32_t)18000)
#define APP_MOTION_TEST_HEADING_FULL_TURN   ((int32_t)36000)

/* One degree in milli-radians, scaled by 1000, for the spin timeout. */
#define APP_MOTION_TEST_MRAD_PER_DEGREE_X1000 ((int32_t)17453)

//******************************** Variables ********************************//

static struct
{
    app_motion_test_mode_t mode;
    app_motion_test_mode_t pending;
    platform_bool_t        abort_requested;
    platform_bool_t        enabled_by_test;
    uint32_t               start_ms;
    uint32_t               timeout_ms;
    int32_t                start_x_mm;
    int32_t                start_y_mm;
    int16_t                prev_heading_cdeg;
    int32_t                accum_cdeg;
} s_motion;

//******************************** Functions ********************************//

static void app_motion_test_finish(uint32_t now_ms)
{
    service_chassis_stop();

    if (s_motion.enabled_by_test == PLATFORM_TRUE)
    {
        (void)service_chassis_enable(PLATFORM_FALSE, now_ms);
        app_state_on_disabled();
    }

    s_motion.mode = APP_MOTION_TEST_NONE;
    s_motion.pending = APP_MOTION_TEST_NONE;
    s_motion.abort_requested = PLATFORM_FALSE;
    s_motion.enabled_by_test = PLATFORM_FALSE;
    s_motion.accum_cdeg = 0;
}

static platform_bool_t app_motion_test_start(app_motion_test_mode_t mode,
                                             uint32_t now_ms)
{
    service_chassis_odometry_t odom;
    uint32_t expected_ms;

    if (service_chassis_is_estopped() == PLATFORM_TRUE)
    {
        return PLATFORM_FALSE;
    }

    if (app_state_enable_allowed() == PLATFORM_FALSE)
    {
        return PLATFORM_FALSE;
    }

    /*
     * A host may have enabled the chassis for teleoperation. Do not switch it
     * off when the test ends in that case; only clean up what we turned on.
     */
    s_motion.enabled_by_test =
        (service_chassis_is_enabled() == PLATFORM_TRUE) ? PLATFORM_FALSE
                                                        : PLATFORM_TRUE;

    if (s_motion.enabled_by_test == PLATFORM_TRUE)
    {
        if (PLATFORM_IS_ERR(service_chassis_enable(PLATFORM_TRUE, now_ms)))
        {
            s_motion.enabled_by_test = PLATFORM_FALSE;
            return PLATFORM_FALSE;
        }

        app_state_on_enabled();
    }

    service_chassis_get_odometry(&odom);
    s_motion.start_x_mm = odom.x_mm;
    s_motion.start_y_mm = odom.y_mm;
    s_motion.prev_heading_cdeg = odom.heading_centi_deg;
    s_motion.accum_cdeg = 0;
    s_motion.start_ms = now_ms;
    s_motion.mode = mode;

    if (mode == APP_MOTION_TEST_STRAIGHT)
    {
        expected_ms = (uint32_t)(((int64_t)APP_MOTION_TEST_STRAIGHT_MM * 1000)
                                 / APP_MOTION_TEST_STRAIGHT_MM_S);
    }
    else
    {
        /*
         * 1 degree = 100 centi-degrees = 17.453 mrad; the x1000 scaling
         * cancels against the millisecond conversion below.
         */
        expected_ms = (uint32_t)(
            ((int64_t)APP_MOTION_TEST_SPIN_CDEG *
             APP_MOTION_TEST_MRAD_PER_DEGREE_X1000) /
            ((int64_t)100 * APP_MOTION_TEST_SPIN_MRAD_S));
    }

    s_motion.timeout_ms = (expected_ms * APP_MOTION_TEST_TIMEOUT_FACTOR) +
                          APP_MOTION_TEST_TIMEOUT_MARGIN_MS;

    return PLATFORM_TRUE;
}

static void app_motion_test_tick_straight(uint32_t now_ms)
{
    service_chassis_odometry_t odom;
    int64_t dx;
    int64_t dy;
    int64_t distance_sq;
    int64_t target_sq;

    service_chassis_get_odometry(&odom);

    dx = (int64_t)odom.x_mm - (int64_t)s_motion.start_x_mm;
    dy = (int64_t)odom.y_mm - (int64_t)s_motion.start_y_mm;
    distance_sq = (dx * dx) + (dy * dy);
    target_sq = (int64_t)APP_MOTION_TEST_STRAIGHT_MM *
                (int64_t)APP_MOTION_TEST_STRAIGHT_MM;

    if (distance_sq >= target_sq)
    {
        app_motion_test_finish(now_ms);
        return;
    }

    (void)service_chassis_set_velocity(APP_MOTION_TEST_STRAIGHT_MM_S,
                                       0, 0, now_ms);
}

static void app_motion_test_tick_spin(uint32_t now_ms)
{
    service_chassis_odometry_t odom;
    int32_t delta;

    service_chassis_get_odometry(&odom);

    /*
     * heading_centi_deg wraps at +/-18000, so accumulate the shortest delta
     * instead of comparing the absolute heading with the start value.
     */
    delta = (int32_t)odom.heading_centi_deg -
            (int32_t)s_motion.prev_heading_cdeg;
    if (delta > APP_MOTION_TEST_HEADING_HALF_TURN)
    {
        delta -= APP_MOTION_TEST_HEADING_FULL_TURN;
    }
    else if (delta < -APP_MOTION_TEST_HEADING_HALF_TURN)
    {
        delta += APP_MOTION_TEST_HEADING_FULL_TURN;
    }

    s_motion.accum_cdeg += delta;
    s_motion.prev_heading_cdeg = odom.heading_centi_deg;

    if ((s_motion.accum_cdeg >= APP_MOTION_TEST_SPIN_CDEG) ||
        (s_motion.accum_cdeg <= -APP_MOTION_TEST_SPIN_CDEG))
    {
        app_motion_test_finish(now_ms);
        return;
    }

    (void)service_chassis_set_velocity(0, 0, APP_MOTION_TEST_SPIN_MRAD_S,
                                       now_ms);
}

void app_motion_test_on_key(service_key_event_t event, uint32_t now_ms)
{
    (void)now_ms;

    if (s_motion.mode != APP_MOTION_TEST_NONE)
    {
        /* Any press while moving is a stop, including a second short press. */
        s_motion.abort_requested = PLATFORM_TRUE;
        return;
    }

    if (event == SERVICE_KEY_EVENT_SHORT_PRESS)
    {
        s_motion.pending = APP_MOTION_TEST_STRAIGHT;
    }
    else if (event == SERVICE_KEY_EVENT_LONG_PRESS)
    {
        s_motion.pending = APP_MOTION_TEST_SPIN;
    }
    else
    {
        /* No other event is defined; keep the pending move unchanged. */
    }
}

void app_motion_test_tick(uint32_t now_ms)
{
    app_motion_test_mode_t pending;

    if (s_motion.abort_requested == PLATFORM_TRUE)
    {
        if (s_motion.mode != APP_MOTION_TEST_NONE)
        {
            app_motion_test_finish(now_ms);
        }

        s_motion.abort_requested = PLATFORM_FALSE;
        s_motion.pending = APP_MOTION_TEST_NONE;
        return;
    }

    if (s_motion.mode == APP_MOTION_TEST_NONE)
    {
        if (s_motion.pending != APP_MOTION_TEST_NONE)
        {
            pending = s_motion.pending;
            s_motion.pending = APP_MOTION_TEST_NONE;
            (void)app_motion_test_start(pending, now_ms);
        }

        return;
    }

    /*
     * A host ESTOP, a fault policy, or a host disable must win over the test.
     * Do not call emergency_stop() here: the test is a convenience, not the
     * safety layer.
     */
    if ((service_chassis_is_estopped() == PLATFORM_TRUE) ||
        (service_chassis_is_enabled() == PLATFORM_FALSE))
    {
        app_motion_test_finish(now_ms);
        return;
    }

    /* Signed difference so a 32-bit millisecond wrap cannot look like a timeout. */
    if ((int32_t)(now_ms - s_motion.start_ms) >= (int32_t)s_motion.timeout_ms)
    {
        app_motion_test_finish(now_ms);
        return;
    }

    if (s_motion.mode == APP_MOTION_TEST_STRAIGHT)
    {
        app_motion_test_tick_straight(now_ms);
    }
    else
    {
        app_motion_test_tick_spin(now_ms);
    }
}

app_motion_test_mode_t app_motion_test_mode(void)
{
    return s_motion.mode;
}

platform_bool_t app_motion_test_running(void)
{
    return (s_motion.mode != APP_MOTION_TEST_NONE) ? PLATFORM_TRUE
                                                   : PLATFORM_FALSE;
}
