/* L1 unit test for 01_APP/app_robot/app_motion_test.c.
 *
 * The module is a small state machine: a key press requests a move, each
 * control tick streams a chassis velocity and checks the odometry, and the
 * move stops on distance/angle or on an abort. The test supplies the chassis
 * service symbols and feeds odometry samples, so the 1 m and 360 degree
 * thresholds, the heading wrap and the enable/disable ownership can be
 * checked without a board.
 *
 * Build:
 *   gcc -std=c99 -Wall -Wextra -Werror \
 *       -I firmware/app/03_Platform/platform_common \
 *       -I firmware/app/03_Platform/platform_mcu \
 *       -I firmware/app/04_Impl \
 *       -I firmware/app/03_Platform/platform_bsp \
 *       -I firmware/app/05_Middlewares/protocol \
 *       -I firmware/app/02_Service \
 *       -I firmware/app/01_APP \
 *       tests/unit/app/test_app_motion_test.c \
 *       firmware/app/01_APP/app_robot/app_motion_test.c \
 *       firmware/app/01_APP/app_robot/app_state.c \
 *       firmware/app/05_Middlewares/protocol/protocol_def.c \
 *       -o build/test_app_motion_test
 */

#include <stdio.h>

#include "app_robot/app_motion_test.h"
#include "app_robot/app_state.h"
#include "app_robot/app_robot_config.h"

#include "service_chassis/service_chassis.h"

static int g_failed = 0;

#define CHECK(cond, msg)                                                  \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);        \
            g_failed++;                                                   \
        }                                                                 \
    } while (0)

/* ------------------------------ stubs --------------------------------- */

static platform_bool_t           g_enabled;
static platform_bool_t           g_estopped;
static int                       g_enable_calls;
static int                       g_stop_calls;
static int                       g_velocity_calls;
static int16_t                   g_last_vx;
static int16_t                   g_last_wz;
static service_chassis_odometry_t g_odom;

platform_err_t service_chassis_enable(platform_bool_t enable, uint32_t now_ms)
{
    (void)now_ms;

    g_enable_calls++;

    if ((enable == PLATFORM_TRUE) && (g_estopped == PLATFORM_TRUE))
    {
        return PLATFORM_ERR_FAIL;
    }

    g_enabled = enable;
    return PLATFORM_ERR_OK;
}

platform_bool_t service_chassis_is_enabled(void)
{
    return g_enabled;
}

platform_bool_t service_chassis_is_estopped(void)
{
    return g_estopped;
}

void service_chassis_stop(void)
{
    g_stop_calls++;
    g_last_vx = 0;
    g_last_wz = 0;
}

platform_err_t service_chassis_set_velocity(int16_t vx_mm_s,
                                            int16_t vy_mm_s,
                                            int16_t wz_mrad_s,
                                            uint32_t now_ms)
{
    (void)vy_mm_s;
    (void)now_ms;

    g_velocity_calls++;
    g_last_vx = vx_mm_s;
    g_last_wz = wz_mrad_s;
    return PLATFORM_ERR_OK;
}

void service_chassis_get_odometry(service_chassis_odometry_t *out)
{
    *out = g_odom;
}

/* ------------------------------ helpers -------------------------------- */

static void motion_reset(void)
{
    if (app_motion_test_running() == PLATFORM_TRUE)
    {
        app_motion_test_on_key(SERVICE_KEY_EVENT_SHORT_PRESS, 0u);
        app_motion_test_tick(0u);
    }

    app_state_init();
    app_state_ready();

    g_enabled = PLATFORM_FALSE;
    g_estopped = PLATFORM_FALSE;
    g_enable_calls = 0;
    g_stop_calls = 0;
    g_velocity_calls = 0;
    g_last_vx = 0;
    g_last_wz = 0;

    g_odom.x_mm = 0;
    g_odom.y_mm = 0;
    g_odom.heading_centi_deg = 0;
    g_odom.vl_mm_s = 0;
    g_odom.vr_mm_s = 0;
}

static void test_short_press_drives_one_metre(void)
{
    motion_reset();

    app_motion_test_on_key(SERVICE_KEY_EVENT_SHORT_PRESS, 1000u);
    CHECK(app_motion_test_mode() == APP_MOTION_TEST_NONE,
          "the callback only queues the move");

    app_motion_test_tick(1000u);

    CHECK(app_motion_test_mode() == APP_MOTION_TEST_STRAIGHT,
          "short press starts the straight move");
    CHECK(g_enable_calls == 1, "the test enables the chassis");
    CHECK(g_enabled == PLATFORM_TRUE, "chassis is enabled");
    CHECK(app_state_get() == APP_STATE_MANUAL, "state follows the enable");
    CHECK(g_velocity_calls == 0, "the start tick only takes the snapshot");

    g_odom.x_mm = 999;
    app_motion_test_tick(1010u);

    CHECK(app_motion_test_running() == PLATFORM_TRUE, "999 mm is not done");
    CHECK(g_last_vx == APP_MOTION_TEST_STRAIGHT_MM_S,
          "straight speed is streamed");
    CHECK(g_last_wz == 0, "straight move has no yaw");

    g_odom.x_mm = 1000;
    app_motion_test_tick(1020u);

    CHECK(app_motion_test_running() == PLATFORM_FALSE, "1 m stops the move");
    CHECK(g_stop_calls == 1, "the chassis target is zeroed");
    CHECK(g_enabled == PLATFORM_FALSE, "the test disables what it enabled");
    CHECK(app_state_get() == APP_STATE_IDLE, "state returns to idle");
}

static void test_long_press_spins_full_turn_across_wrap(void)
{
    motion_reset();

    g_odom.heading_centi_deg = 17900;
    app_motion_test_on_key(SERVICE_KEY_EVENT_LONG_PRESS, 2000u);
    app_motion_test_tick(2000u);

    CHECK(app_motion_test_mode() == APP_MOTION_TEST_SPIN,
          "long press starts the spin");

    g_odom.heading_centi_deg = -17900;   /* +200 cdeg across the wrap */
    app_motion_test_tick(2010u);
    CHECK(g_last_wz == APP_MOTION_TEST_SPIN_MRAD_S,
          "spin speed is streamed");

    g_odom.heading_centi_deg = -8900;    /* +9000 */
    app_motion_test_tick(2020u);

    g_odom.heading_centi_deg = 100;      /* +9000 */
    app_motion_test_tick(2030u);

    g_odom.heading_centi_deg = 9100;     /* +9000 */
    app_motion_test_tick(2040u);

    CHECK(app_motion_test_running() == PLATFORM_TRUE,
          "270 degrees is not a full turn");

    g_odom.heading_centi_deg = 17900;    /* +8800 -> 36000 */
    app_motion_test_tick(2050u);

    CHECK(app_motion_test_running() == PLATFORM_FALSE,
          "360 degrees stops the spin");
    CHECK(g_stop_calls == 1, "spin target is zeroed");
    CHECK(g_enabled == PLATFORM_FALSE, "spin disables the chassis again");
}

static void test_short_press_aborts_a_move(void)
{
    motion_reset();

    app_motion_test_on_key(SERVICE_KEY_EVENT_SHORT_PRESS, 3000u);
    app_motion_test_tick(3000u);

    app_motion_test_on_key(SERVICE_KEY_EVENT_SHORT_PRESS, 3010u);
    app_motion_test_tick(3010u);

    CHECK(app_motion_test_running() == PLATFORM_FALSE,
          "a press while moving aborts");
    CHECK(g_stop_calls == 1, "abort zeroes the target");
    CHECK(g_enabled == PLATFORM_FALSE, "abort disables the chassis");
}

static void test_estop_refuses_to_start(void)
{
    motion_reset();

    g_estopped = PLATFORM_TRUE;
    app_motion_test_on_key(SERVICE_KEY_EVENT_SHORT_PRESS, 4000u);
    app_motion_test_tick(4000u);

    CHECK(app_motion_test_running() == PLATFORM_FALSE,
          "a latched estop refuses the move");
    CHECK(g_enable_calls == 0, "the test does not fight the estop");
}

static void test_fault_state_refuses_to_start(void)
{
    motion_reset();

    app_state_on_fault();
    app_motion_test_on_key(SERVICE_KEY_EVENT_SHORT_PRESS, 5000u);
    app_motion_test_tick(5000u);

    CHECK(app_motion_test_running() == PLATFORM_FALSE,
          "a fault state refuses the move");
    CHECK(g_enable_calls == 0, "the test does not enable in fault");
}

static void test_timeout_stops_a_stuck_move(void)
{
    motion_reset();

    app_motion_test_on_key(SERVICE_KEY_EVENT_SHORT_PRESS, 6000u);
    app_motion_test_tick(6000u);

    /*
     * 1000 mm at 150 mm/s is 6666 ms; the timeout is 2x plus 3 s, so
     * 16332 ms. Cross it without moving the odometry.
     */
    app_motion_test_tick(6000u + 16333u);

    CHECK(app_motion_test_running() == PLATFORM_FALSE,
          "a move that never reaches the target times out");
    CHECK(g_stop_calls == 1, "timeout zeroes the target");
    CHECK(g_enabled == PLATFORM_FALSE, "timeout disables the chassis");
}

static void test_host_enabled_chassis_stays_enabled(void)
{
    motion_reset();

    g_enabled = PLATFORM_TRUE;
    app_state_on_enabled();

    app_motion_test_on_key(SERVICE_KEY_EVENT_SHORT_PRESS, 7000u);
    app_motion_test_tick(7000u);

    CHECK(app_motion_test_running() == PLATFORM_TRUE,
          "the test runs on a host-enabled chassis");

    g_odom.x_mm = 1000;
    app_motion_test_tick(7010u);

    CHECK(app_motion_test_running() == PLATFORM_FALSE, "move completes");
    CHECK(g_enable_calls == 0, "the test did not enable the chassis");
    CHECK(g_enabled == PLATFORM_TRUE, "the host's enable is left alone");
    CHECK(app_state_get() == APP_STATE_MANUAL, "state stays manual");
}

int main(void)
{
    test_short_press_drives_one_metre();
    test_long_press_spins_full_turn_across_wrap();
    test_short_press_aborts_a_move();
    test_estop_refuses_to_start();
    test_fault_state_refuses_to_start();
    test_timeout_stops_a_stuck_move();
    test_host_enabled_chassis_stays_enabled();

    if (g_failed == 0)
    {
        printf("PASS: app_motion_test tests\n");
        return 0;
    }

    printf("FAILED: %d check(s)\n", g_failed);
    return 1;
}
