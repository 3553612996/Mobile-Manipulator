/* L1 unit test for 01_APP/app_robot/app_fault.c.
 *
 * The fault policy is pure decision logic plus calls into the chassis/arm
 * services. The test supplies those few symbols and records what the policy
 * asked them to do, so severity arbitration and the WARN/ERROR/FATAL split can
 * be checked without a board.
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
 *       tests/unit/app/test_app_fault.c \
 *       firmware/app/01_APP/app_robot/app_fault.c \
 *       firmware/app/01_APP/app_robot/app_state.c \
 *       firmware/app/05_Middlewares/protocol/protocol_def.c \
 *       -o build/test_app_fault
 */

#include <stdio.h>

#include "app_robot/app_fault.h"
#include "app_robot/app_state.h"

#include "service_chassis/service_chassis.h"
#include "service_arm/service_arm.h"
#include "service_proto/service_proto.h"

static int g_failed = 0;

#define CHECK(cond, msg)                                                  \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);        \
            g_failed++;                                                   \
        }                                                                 \
    } while (0)

/* ------------------------------ stubs --------------------------------- */

static int             g_chassis_stops;
static int             g_chassis_stop_targets;
static int             g_chassis_enable_calls;
static platform_bool_t g_chassis_enable_last;
static int             g_arm_disable_calls;
static int             g_event_count;
static uint16_t        g_last_event;
static platform_bool_t g_estop;
static platform_bool_t g_timed_out;
static platform_bool_t g_crc_alarm;
static platform_bool_t g_battery_low;

platform_err_t service_chassis_enable(platform_bool_t enable, uint32_t now_ms)
{
    (void)now_ms;
    g_chassis_enable_calls++;
    g_chassis_enable_last = enable;
    return PLATFORM_ERR_OK;
}

void service_chassis_emergency_stop(void)
{
    g_chassis_stops++;
    g_estop = PLATFORM_TRUE;
}

void service_chassis_stop(void)
{
    g_chassis_stop_targets++;
}

platform_err_t service_chassis_clear_estop(void)
{
    g_estop = PLATFORM_FALSE;
    g_timed_out = PLATFORM_FALSE;
    return PLATFORM_ERR_OK;
}

platform_bool_t service_chassis_cmd_timed_out(void)
{
    return g_timed_out;
}

platform_err_t service_arm_enable(platform_bool_t enable)
{
    if (enable == PLATFORM_FALSE)
    {
        g_arm_disable_calls++;
    }
    return PLATFORM_ERR_OK;
}

platform_err_t service_proto_send_event(uint16_t event_code,
                                        uint32_t timestamp_ms)
{
    (void)timestamp_ms;
    g_event_count++;
    g_last_event = event_code;
    return PLATFORM_ERR_OK;
}

platform_bool_t service_proto_crc_alarm(void)
{
    return g_crc_alarm;
}

platform_bool_t app_battery_low(void)
{
    return g_battery_low;
}

/* ------------------------------ fixtures ------------------------------ */

static void reset_all(void)
{
    g_chassis_stops = 0;
    g_chassis_stop_targets = 0;
    g_chassis_enable_calls = 0;
    g_chassis_enable_last = PLATFORM_TRUE;
    g_arm_disable_calls = 0;
    g_event_count = 0;
    g_last_event = 0u;
    g_estop = PLATFORM_FALSE;
    g_timed_out = PLATFORM_FALSE;
    g_crc_alarm = PLATFORM_FALSE;
    g_battery_low = PLATFORM_FALSE;

    app_state_init();
    app_state_ready();
    app_state_on_enabled();
    app_fault_init();
}

/* ------------------------------- tests -------------------------------- */

static void test_scope_table(void)
{
    CHECK(app_fault_scope_get(PROTOCOL_FAULT_OVERCURRENT) ==
          APP_FAULT_SCOPE_CHASSIS, "overcurrent is a chassis fault");
    CHECK(app_fault_scope_get(PROTOCOL_FAULT_JOINT_LIMIT) ==
          APP_FAULT_SCOPE_ARM, "joint limit is an arm fault");
    CHECK(app_fault_scope_get(PROTOCOL_FAULT_BATTERY_LOW) ==
          APP_FAULT_SCOPE_NONE, "battery low has no stop scope");
    CHECK(app_fault_scope_get(PROTOCOL_FAULT_COMM_TIMEOUT) ==
          APP_FAULT_SCOPE_ALL, "timeout stops everything");
    CHECK(app_fault_scope_get(0x9999u) == APP_FAULT_SCOPE_ALL,
          "unknown code is treated as a full stop");
}

static void test_warn_only_reports(void)
{
    reset_all();

    CHECK(app_fault_raise(PROTOCOL_FAULT_BATTERY_LOW, 100u,
                          PLATFORM_FALSE) == PLATFORM_TRUE,
          "warning is latched");
    CHECK(app_fault_code() == PROTOCOL_FAULT_BATTERY_LOW, "warning code");
    CHECK(app_state_get() == APP_STATE_MANUAL, "warning keeps MANUAL");
    CHECK(g_chassis_enable_calls == 0, "warning does not disable chassis");
    CHECK(g_chassis_stops == 0, "warning does not stop the chassis");
    CHECK(g_arm_disable_calls == 0, "warning does not disable the arm");
    CHECK(g_event_count == 1 && g_last_event == PROTOCOL_FAULT_BATTERY_LOW,
          "warning sends one event");

    /* Equal severity keeps the first latch. */
    CHECK(app_fault_raise(PROTOCOL_FAULT_CRC_LIMIT, 110u,
                          PLATFORM_FALSE) == PLATFORM_FALSE,
          "equal severity is ignored");
    CHECK(app_fault_code() == PROTOCOL_FAULT_BATTERY_LOW,
          "first warning stays latched");
}

static void test_error_stops_one_subsystem(void)
{
    reset_all();

    CHECK(app_fault_raise(PROTOCOL_FAULT_OVERCURRENT, 200u,
                          PLATFORM_FALSE) == PLATFORM_TRUE,
          "error is latched");
    CHECK(app_state_get() == APP_STATE_FAULT, "error enters FAULT");
    CHECK(g_chassis_enable_calls == 1, "chassis disabled once");
    CHECK(g_chassis_enable_last == PLATFORM_FALSE, "chassis disabled");
    CHECK(g_chassis_stops == 0, "error is not an emergency stop");
    CHECK(g_arm_disable_calls == 0, "unrelated arm is untouched");

    /* A WARN must not replace this ERROR. */
    CHECK(app_fault_raise(PROTOCOL_FAULT_BATTERY_LOW, 210u,
                          PLATFORM_FALSE) == PLATFORM_FALSE,
          "warning cannot replace an error");
    CHECK(app_fault_code() == PROTOCOL_FAULT_OVERCURRENT,
          "error stays latched");
}

static void test_arm_error_stops_chassis_target(void)
{
    reset_all();

    CHECK(app_fault_raise(PROTOCOL_FAULT_JOINT_LIMIT, 250u,
                          PLATFORM_FALSE) == PLATFORM_TRUE,
          "arm error is latched");
    CHECK(app_state_get() == APP_STATE_FAULT, "arm error enters FAULT");
    CHECK(g_arm_disable_calls == 1, "arm disabled");
    CHECK(g_chassis_enable_calls == 0, "chassis bridges stay enabled");
    CHECK(g_chassis_stop_targets == 1,
          "chassis target zeroed while it brakes");
}

static void test_fatal_stops_everything(void)
{
    reset_all();

    CHECK(app_fault_raise(PROTOCOL_FAULT_COMM_TIMEOUT, 300u,
                          PLATFORM_FALSE) == PLATFORM_TRUE,
          "fatal is latched");
    CHECK(app_state_get() == APP_STATE_ESTOP, "fatal enters ESTOP");
    CHECK(g_chassis_stops == 1, "fatal stops the chassis");
    CHECK(g_arm_disable_calls == 1, "fatal disables the arm");
    CHECK(g_estop == PLATFORM_TRUE, "fatal latches the stop");

    CHECK(app_fault_clear(PROTOCOL_CLEAR_ALL_FAULTS, 310u) ==
          PROTOCOL_RESULT_OK, "clear fatal");
    CHECK(app_fault_code() == 0u, "code cleared");
    CHECK(app_state_get() == APP_STATE_IDLE, "ESTOP -> IDLE");
    CHECK(g_estop == PLATFORM_FALSE, "stop released");
}

static void test_clear_refuses_live_condition(void)
{
    reset_all();
    g_battery_low = PLATFORM_TRUE;

    (void)app_fault_raise(PROTOCOL_FAULT_BATTERY_LOW, 400u, PLATFORM_FALSE);
    CHECK(app_fault_clear(PROTOCOL_FAULT_BATTERY_LOW, 410u) ==
          PROTOCOL_RESULT_STATE, "cannot clear a live low battery");
    CHECK(app_fault_code() == PROTOCOL_FAULT_BATTERY_LOW, "code stays");

    g_battery_low = PLATFORM_FALSE;
    CHECK(app_fault_clear(PROTOCOL_FAULT_BATTERY_LOW, 420u) ==
          PROTOCOL_RESULT_OK, "clear after recovery");
    CHECK(app_fault_code() == 0u, "code gone after recovery");
    CHECK(app_state_get() == APP_STATE_MANUAL,
          "clearing a warning keeps MANUAL");
}

static void test_poll_edges(void)
{
    reset_all();

    g_timed_out = PLATFORM_TRUE;
    app_fault_poll(500u);
    CHECK(app_fault_code() == PROTOCOL_FAULT_COMM_TIMEOUT,
          "timeout edge raises 0x0202");
    CHECK(app_state_get() == APP_STATE_ESTOP, "timeout enters ESTOP");

    /* The real service clears timed_out on clear_estop; the stub does too. */
    CHECK(app_fault_clear(PROTOCOL_CLEAR_ALL_FAULTS, 510u) ==
          PROTOCOL_RESULT_OK, "clear timeout");
    app_fault_poll(520u);
    CHECK(app_fault_code() == 0u, "no repeat timeout without a new command");

    /* The host must re-enable after a FATAL; a later WARN keeps MANUAL. */
    app_state_on_enabled();
    g_crc_alarm = PLATFORM_TRUE;
    app_fault_poll(530u);
    CHECK(app_fault_code() == PROTOCOL_FAULT_CRC_LIMIT,
          "CRC alarm raises 0x0201");
    CHECK(app_state_get() == APP_STATE_MANUAL, "CRC warning keeps MANUAL");

    /* Seen once per boot. */
    (void)app_fault_clear(PROTOCOL_CLEAR_ALL_FAULTS, 540u);
    app_fault_poll(550u);
    CHECK(app_fault_code() == 0u, "CRC alarm is reported once");
}

int main(void)
{
    test_scope_table();
    test_warn_only_reports();
    test_error_stops_one_subsystem();
    test_arm_error_stops_chassis_target();
    test_fatal_stops_everything();
    test_clear_refuses_live_condition();
    test_poll_edges();

    if (g_failed != 0)
    {
        printf("FAIL: app_fault tests (%d failure(s))\n", g_failed);
        return 1;
    }

    printf("PASS: app_fault tests\n");
    return 0;
}
