/* L1 unit test for 01_APP/app_robot/app_state.c. Pure logic, no stubs.
 *
 * Build:
 *   gcc -std=c99 -Wall -Wextra -Werror \
 *       -I firmware/app/03_Platform/platform_common \
 *       -I firmware/app/04_Impl \
 *       -I firmware/app/05_Middlewares/protocol \
 *       -I firmware/app/01_APP \
 *       tests/unit/app/test_app_state.c \
 *       firmware/app/01_APP/app_robot/app_state.c \
 *       -o build/test_app_state
 */

#include <stdio.h>

#include "app_robot/app_state.h"

static int g_failed = 0;

#define CHECK(cond, msg)                                                  \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);        \
            g_failed++;                                                   \
        }                                                                 \
    } while (0)

static void test_power_up(void)
{
    app_state_init();

    CHECK(app_state_get() == APP_STATE_INIT, "boots in INIT");
    CHECK(app_state_motion_allowed() == PLATFORM_FALSE, "no motion in INIT");
    CHECK(app_state_enable_allowed() == PLATFORM_FALSE,
          "cannot enable in INIT");

    app_state_ready();
    CHECK(app_state_get() == APP_STATE_IDLE, "INIT -> IDLE");
    CHECK(app_state_enable_allowed() == PLATFORM_TRUE,
          "enable allowed in IDLE");
    CHECK(app_state_motion_allowed() == PLATFORM_FALSE,
          "no motion before enable");
}

static void test_manual_round_trip(void)
{
    app_state_init();
    app_state_ready();
    app_state_on_enabled();

    CHECK(app_state_get() == APP_STATE_MANUAL, "IDLE -> MANUAL");
    CHECK(app_state_motion_allowed() == PLATFORM_TRUE, "manual can move");

    /* Enabling again is a no-op, not a state rollback. */
    app_state_on_enabled();
    CHECK(app_state_get() == APP_STATE_MANUAL, "repeat enable stays MANUAL");

    app_state_on_disabled();
    CHECK(app_state_get() == APP_STATE_IDLE, "MANUAL -> IDLE");
    CHECK(app_state_motion_allowed() == PLATFORM_FALSE, "idle cannot move");
}

static void test_fault_and_clear(void)
{
    app_state_init();
    app_state_ready();
    app_state_on_enabled();

    app_state_on_fault();
    CHECK(app_state_get() == APP_STATE_FAULT, "MANUAL -> FAULT");
    CHECK(app_state_enable_allowed() == PLATFORM_FALSE,
          "cannot enable in FAULT");

    app_state_on_cleared();
    CHECK(app_state_get() == APP_STATE_IDLE, "FAULT -> IDLE");

    /* ESTOP outranks a later FAULT, and clearing returns to IDLE. */
    app_state_on_estop();
    CHECK(app_state_get() == APP_STATE_ESTOP, "-> ESTOP");
    app_state_on_fault();
    CHECK(app_state_get() == APP_STATE_ESTOP, "FAULT cannot override ESTOP");
    app_state_on_cleared();
    CHECK(app_state_get() == APP_STATE_IDLE, "ESTOP -> IDLE");
}

static void test_clear_warn_keeps_manual(void)
{
    app_state_init();
    app_state_ready();
    app_state_on_enabled();

    /*
     * A WARN only reports. Clearing it while driving must not drop the
     * chassis back to IDLE or require another SET_ENABLE.
     */
    app_state_on_cleared();
    CHECK(app_state_get() == APP_STATE_MANUAL,
          "clearing a warning keeps MANUAL");
}

int main(void)
{
    test_power_up();
    test_manual_round_trip();
    test_fault_and_clear();
    test_clear_warn_keeps_manual();

    if (g_failed != 0)
    {
        printf("FAIL: app_state tests (%d failure(s))\n", g_failed);
        return 1;
    }

    printf("PASS: app_state tests\n");
    return 0;
}
