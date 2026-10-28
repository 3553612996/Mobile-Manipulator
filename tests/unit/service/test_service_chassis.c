/* L1 unit test for 02_Service/service_chassis. Runs on the PC, no hardware.
 *
 * The service depends on the platform motor device only, so the test provides
 * those four symbols and records the duty each wheel receives. Everything the
 * test checks (inverse kinematics, the 200 ms watchdog, the latched stop) is
 * pure logic that must not need a board to verify.
 *
 * Build (MinGW or any host gcc):
 *   gcc -std=c99 -Wall -Wextra \
 *       -I firmware/app/03_Platform/platform_common \
 *       -I firmware/app/04_Impl \
 *       -I firmware/app/03_Platform/platform_bsp \
 *       -I firmware/app/02_Service/service_chassis \
 *       tests/unit/service/test_service_chassis.c \
 *       firmware/app/02_Service/service_chassis/service_chassis.c \
 *       -o build/test_service_chassis -lm
 */

#include <stdio.h>
#include <string.h>

#include "service_chassis.h"

static int g_failed = 0;

#define CHECK(cond, msg)                                                  \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);        \
            g_failed++;                                                   \
        }                                                                 \
    } while (0)

/* ------------------------- platform motor stubs ------------------------ */

static int16_t g_duty[PLATFORM_MOTOR_ID_MAX];
static int32_t g_encoder[PLATFORM_MOTOR_ID_MAX];
static platform_bool_t g_bridge[PLATFORM_MOTOR_ID_MAX];
static int     g_stop_all_calls;

platform_err_t platform_motor_set_enable(platform_motor_id_t id,
                                         platform_bool_t enable)
{
    if (id >= PLATFORM_MOTOR_ID_MAX)
    {
        return PLATFORM_ERR_PARAM;
    }

    g_bridge[id] = enable;
    return PLATFORM_ERR_OK;
}

platform_err_t platform_motor_stop_all(void)
{
    g_stop_all_calls++;
    memset(g_duty, 0, sizeof(g_duty));
    return PLATFORM_ERR_OK;
}

platform_err_t platform_motor_set_duty(platform_motor_id_t id,
                                       int16_t duty_permille)
{
    if (id >= PLATFORM_MOTOR_ID_MAX)
    {
        return PLATFORM_ERR_PARAM;
    }

    g_duty[id] = duty_permille;
    return PLATFORM_ERR_OK;
}

platform_err_t platform_motor_get_encoder(platform_motor_id_t id,
                                          int32_t *encoder_count)
{
    if ((id >= PLATFORM_MOTOR_ID_MAX) || (encoder_count == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    /* Mimic the driver: cumulative, never cleared by reading. */
    *encoder_count = g_encoder[id];

    return PLATFORM_ERR_OK;
}

/* ------------------------------ fixtures ------------------------------ */

#define TEST_TIMEOUT_MS     ((uint32_t)200u)
#define TEST_MAX_WHEEL      ((int16_t)500)

static service_chassis_cfg_t make_cfg(void)
{
    service_chassis_cfg_t cfg;

    cfg.motor[SERVICE_CHASSIS_WHEEL_FL] = PLATFORM_MOTOR_ID_0;
    cfg.motor[SERVICE_CHASSIS_WHEEL_FR] = PLATFORM_MOTOR_ID_1;
    cfg.motor[SERVICE_CHASSIS_WHEEL_RL] = PLATFORM_MOTOR_ID_2;
    cfg.motor[SERVICE_CHASSIS_WHEEL_RR] = PLATFORM_MOTOR_ID_3;
    cfg.half_length_mm = 150;
    cfg.half_width_mm = 120;
    cfg.max_vx_mm_s = 500;
    cfg.max_vy_mm_s = 500;
    cfg.max_wz_mrad_s = 1500;
    cfg.max_wheel_mm_s = TEST_MAX_WHEEL;
    cfg.counts_per_mm_x1000 = 22918;
    cfg.cmd_timeout_ms = TEST_TIMEOUT_MS;

    return cfg;
}

/* ---------------------------- kinematics ------------------------------ */

static void test_inverse_kinematics(void)
{
    int16_t wheel[SERVICE_CHASSIS_WHEEL_COUNT];
    int16_t k = 270;   /* 150 + 120 from the fixture. */

    service_chassis_inverse_kinematics(100, 0, 0, k, wheel);
    CHECK(wheel[0] == 100 && wheel[1] == 100 &&
          wheel[2] == 100 && wheel[3] == 100, "pure forward");

    service_chassis_inverse_kinematics(0, 100, 0, k, wheel);
    CHECK(wheel[0] == -100 && wheel[1] == 100 &&
          wheel[2] == 100 && wheel[3] == -100, "strafe left");

    /* k * 1000 / 1000 = 270 mm/s at the wheel. */
    service_chassis_inverse_kinematics(0, 0, 1000, k, wheel);
    CHECK(wheel[0] == -270 && wheel[1] == 270 &&
          wheel[2] == -270 && wheel[3] == 270, "rotate CCW");

    /* Saturation must clamp, not wrap: FL/RR = 30000-30000 = 0,
       FR/RL = 30000+30000 = 60000 -> 32767. */
    service_chassis_inverse_kinematics(30000, 30000, 0, k, wheel);
    CHECK(wheel[0] == 0 && wheel[1] == 32767 &&
          wheel[2] == 32767 && wheel[3] == 0, "clamp to int16");
}

/* ------------------------------- service ------------------------------- */

static void test_enable_and_duty(void)
{
    service_chassis_cfg_t cfg = make_cfg();

    CHECK(service_chassis_init(&cfg) == PLATFORM_ERR_OK, "init");
    CHECK(service_chassis_is_enabled() == PLATFORM_FALSE, "disabled at boot");

    /* Moving while disabled must not produce duty. */
    CHECK(service_chassis_set_velocity(200, 0, 0, 10u) == PLATFORM_ERR_OK,
          "set velocity while disabled");
    CHECK(service_chassis_apply(20u) == PLATFORM_FALSE, "no timeout edge");
    CHECK(g_duty[0] == 0 && g_duty[1] == 0 &&
          g_duty[2] == 0 && g_duty[3] == 0, "no duty while disabled");

    CHECK(service_chassis_enable(PLATFORM_TRUE, 100u) == PLATFORM_ERR_OK,
          "enable");
    /* The BSP coasts until its bridge is enabled; duty alone moves nothing. */
    CHECK(g_bridge[0] == PLATFORM_TRUE && g_bridge[1] == PLATFORM_TRUE &&
          g_bridge[2] == PLATFORM_TRUE && g_bridge[3] == PLATFORM_TRUE,
          "enable turns on every bridge");
    CHECK(service_chassis_set_velocity(200, 0, 0, 110u) == PLATFORM_ERR_OK,
          "forward target");

    (void)service_chassis_apply(120u);
    /* 200 mm/s of 500 mm/s full scale -> 400 permille. */
    CHECK(g_duty[0] == 400 && g_duty[1] == 400 &&
          g_duty[2] == 400 && g_duty[3] == 400, "duty from target");
}

static void test_velocity_limits(void)
{
    CHECK(service_chassis_set_velocity(501, 0, 0, 200u) ==
          PLATFORM_ERR_PARAM, "vx over limit");
    CHECK(service_chassis_set_velocity(0, -501, 0, 200u) ==
          PLATFORM_ERR_PARAM, "vy over limit");
    CHECK(service_chassis_set_velocity(0, 0, -1501, 200u) ==
          PLATFORM_ERR_PARAM, "wz over limit");

    /* A rejected frame must leave the previous target untouched. */
    (void)service_chassis_apply(210u);
    CHECK(g_duty[0] == 400, "previous target kept after reject");
}

static void test_timeout(void)
{
    CHECK(service_chassis_cmd_timed_out() == PLATFORM_FALSE, "not timed out");

    /* 110 ms + 200 ms + 1: the watchdog fires on this apply. */
    CHECK(service_chassis_apply(311u) == PLATFORM_TRUE, "timeout edge");
    CHECK(service_chassis_cmd_timed_out() == PLATFORM_TRUE, "timeout latched");
    CHECK(g_duty[0] == 0 && g_duty[1] == 0 &&
          g_duty[2] == 0 && g_duty[3] == 0, "wheels stopped");

    /* The edge must not repeat every control tick. */
    CHECK(service_chassis_apply(321u) == PLATFORM_FALSE, "edge once");

    /* A fresh frame clears the timeout. */
    CHECK(service_chassis_set_velocity(0, 0, 0, 330u) == PLATFORM_ERR_OK,
          "refresh");
    CHECK(service_chassis_cmd_timed_out() == PLATFORM_FALSE, "timeout cleared");
}

static void test_emergency_stop(void)
{
    CHECK(service_chassis_enable(PLATFORM_TRUE, 400u) == PLATFORM_ERR_OK,
          "re-enable");

    service_chassis_emergency_stop();
    CHECK(service_chassis_is_estopped() == PLATFORM_TRUE, "estop latched");
    CHECK(service_chassis_is_enabled() == PLATFORM_FALSE, "estop disables");
    CHECK(g_stop_all_calls > 0, "estop stops motors");
    CHECK(g_bridge[0] == PLATFORM_FALSE && g_bridge[3] == PLATFORM_FALSE,
          "estop coasts the bridges");

    CHECK(service_chassis_enable(PLATFORM_TRUE, 500u) == PLATFORM_ERR_FAIL,
          "enable refused while latched");

    CHECK(service_chassis_clear_estop() == PLATFORM_ERR_OK, "clear");
    CHECK(service_chassis_is_estopped() == PLATFORM_FALSE, "latch released");
    CHECK(service_chassis_is_enabled() == PLATFORM_FALSE,
          "clear does not re-enable");
}

/* ------------------------------ odometry ------------------------------- */

static void test_odometry(void)
{
    service_chassis_cfg_t cfg = make_cfg();
    service_chassis_odometry_t odom;

    (void)service_chassis_init(&cfg);

    /* Four wheels, 22918 counts = 1000 mm of travel each. */
    g_encoder[0] = 22918;
    g_encoder[1] = 22918;
    g_encoder[2] = 22918;
    g_encoder[3] = 22918;
    service_chassis_update_odometry(1000u);

    service_chassis_get_odometry(&odom);
    CHECK(odom.x_mm == 1000, "one metre forward");
    CHECK(odom.y_mm == 0, "no lateral drift");
    CHECK(odom.heading_centi_deg == 0, "no heading change");
    /* 1000 mm in 1000 ms. */
    CHECK(odom.vl_mm_s == 1000 && odom.vr_mm_s == 1000, "measured speed");

    /*
     * The platform reports a cumulative count. A second sample that adds no
     * counts must read 0 mm/s and leave the position alone -- treating the
     * total as a delta is what made the real chassis report 32767 mm/s.
     */
    service_chassis_update_odometry(1000u);
    service_chassis_get_odometry(&odom);
    CHECK(odom.x_mm == 1000, "position holds without new counts");
    CHECK(odom.vl_mm_s == 0 && odom.vr_mm_s == 0,
          "no speed without new counts");
}

int main(void)
{
    test_inverse_kinematics();
    test_enable_and_duty();
    test_velocity_limits();
    test_timeout();
    test_emergency_stop();
    test_odometry();

    if (g_failed != 0)
    {
        printf("FAIL: service_chassis tests (%d failure(s))\n", g_failed);
        return 1;
    }

    printf("PASS: service_chassis tests\n");
    return 0;
}
