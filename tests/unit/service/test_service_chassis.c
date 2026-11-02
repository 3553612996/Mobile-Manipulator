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
 *       -I firmware/app/03_Platform/platform_middleware \
 *       -I firmware/app/02_Service/service_chassis \
 *       tests/unit/service/test_service_chassis.c \
 *       firmware/app/02_Service/service_chassis/service_chassis.c \
 *       firmware/app/03_Platform/platform_middleware/platform_pid.c \
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

#define TEST_TIMEOUT_MS         ((uint32_t)200u)
#define TEST_MAX_WHEEL          ((int16_t)500)
#define TEST_COUNTS_PER_MM_X1000 ((int32_t)24828)

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
    cfg.counts_per_mm_x1000 = TEST_COUNTS_PER_MM_X1000;
    cfg.cmd_timeout_ms = TEST_TIMEOUT_MS;

    /*
     * Pure P here, so the expected duty is exactly Kp * error and the test
     * does not depend on the application's tuning. Kp = 2 maps a 200 mm/s
     * error to the same 400 permille the old open-loop mapping produced.
     */
    for (uint8_t i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
    {
        cfg.pid_kp[i] = 2.0f;
        cfg.pid_ki[i] = 0.0f;
        cfg.pid_kd[i] = 0.0f;
    }

    return cfg;
}

static void reset_motor_stubs(void)
{
    memset(g_duty, 0, sizeof(g_duty));
    memset(g_encoder, 0, sizeof(g_encoder));
    memset(g_bridge, 0, sizeof(g_bridge));
    g_stop_all_calls = 0;
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

/* ------------------------- encoder scale ------------------------------ */

static void test_counts_per_mm(void)
{
    /* 4680 counts/rev on a 60 mm wheel: 4680 / (pi * 60) = 24.828. */
    CHECK(service_chassis_counts_per_mm_x1000(4680, 60) == 24828,
          "60 mm wheel gives 24.828 counts/mm");

    /* 65 mm is the case the x1000 integer exists for: 22.918. */
    CHECK(service_chassis_counts_per_mm_x1000(4680, 65) == 22918,
          "65 mm wheel gives 22.918 counts/mm");

    /* 59 mm is 25248.99: rounding gives 25249, a plain cast would give 25248. */
    CHECK(service_chassis_counts_per_mm_x1000(4680, 59) == 25249,
          "rounds instead of truncating");

    /* A zero diameter must not divide by zero. */
    CHECK(service_chassis_counts_per_mm_x1000(4680, 0) == 0,
          "zero diameter returns 0");
}

/* ------------------------------- service ------------------------------- */

static void test_enable_and_duty(void)
{
    service_chassis_cfg_t cfg = make_cfg();

    reset_motor_stubs();
    CHECK(service_chassis_init(&cfg) == PLATFORM_ERR_OK, "init");
    CHECK(service_chassis_is_enabled() == PLATFORM_FALSE, "disabled at boot");

    /* Moving while disabled must not produce duty. */
    CHECK(service_chassis_set_velocity(200, 0, 0, 10u) == PLATFORM_ERR_OK,
          "set velocity while disabled");
    CHECK(service_chassis_apply(20u, 10u) == PLATFORM_FALSE, "no timeout edge");
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

    (void)service_chassis_apply(120u, 10u);
    /* Measured speed is zero, Kp = 2, target 200 -> 400 permille. */
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
    (void)service_chassis_apply(210u, 10u);
    CHECK(g_duty[0] == 400, "previous target kept after reject");
}

static void test_timeout(void)
{
    CHECK(service_chassis_cmd_timed_out() == PLATFORM_FALSE, "not timed out");

    /* 110 ms + 200 ms + 1: the watchdog fires on this apply. */
    CHECK(service_chassis_apply(311u, 10u) == PLATFORM_TRUE, "timeout edge");
    CHECK(service_chassis_cmd_timed_out() == PLATFORM_TRUE, "timeout latched");
    CHECK(g_duty[0] == 0 && g_duty[1] == 0 &&
          g_duty[2] == 0 && g_duty[3] == 0, "wheels stopped");

    /* The edge must not repeat every control tick. */
    CHECK(service_chassis_apply(321u, 10u) == PLATFORM_FALSE, "edge once");

    /* A fresh frame clears the timeout. */
    CHECK(service_chassis_set_velocity(0, 0, 0, 330u) == PLATFORM_ERR_OK,
          "refresh");
    CHECK(service_chassis_cmd_timed_out() == PLATFORM_FALSE, "timeout cleared");

    /* The loops restart from a fresh encoder baseline, not a stale delta. */
    CHECK(service_chassis_set_velocity(200, 0, 0, 330u) == PLATFORM_ERR_OK,
          "target after timeout");
    CHECK(service_chassis_apply(330u, 10u) == PLATFORM_FALSE,
          "no second timeout edge");
    CHECK(g_duty[0] == 400, "PID restarts from zero measured speed");
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

/* ----------------------------- speed loop ------------------------------ */

static void test_speed_feedback(void)
{
    service_chassis_cfg_t cfg = make_cfg();
    int16_t speeds[SERVICE_CHASSIS_WHEEL_COUNT];
    uint8_t i;

    reset_motor_stubs();
    CHECK(service_chassis_init(&cfg) == PLATFORM_ERR_OK, "speed init");
    CHECK(service_chassis_enable(PLATFORM_TRUE, 0u) == PLATFORM_ERR_OK,
          "speed enable");

    /* Consume the enable transition before feeding motion. */
    (void)service_chassis_apply(0u, 10u);
    CHECK(g_duty[0] == 0, "zero target before the first frame");

    CHECK(service_chassis_set_velocity(200, 0, 0, 10u) == PLATFORM_ERR_OK,
          "speed target");
    (void)service_chassis_apply(10u, 10u);
    CHECK(g_duty[0] == 400, "no feedback yet: P drives 400");

    /*
     * 100 mm/s for 10 ms = 1 mm = 24828 / 1000 counts on every wheel.
     * The measured speed is therefore about 100.7 mm/s, so the error drops
     * and the P output must drop from 400 to about 199, not stay open-loop.
     */
    for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
    {
        g_encoder[i] += 25;
    }

    (void)service_chassis_apply(20u, 10u);
    CHECK((g_duty[0] >= 197) && (g_duty[0] <= 201),
          "feedback reduces the duty");
    CHECK(g_duty[1] == g_duty[0] && g_duty[2] == g_duty[0] &&
          g_duty[3] == g_duty[0], "all four loops see the same motion");

    service_chassis_get_wheel_speeds(speeds);
    CHECK((speeds[0] >= 100) && (speeds[0] <= 101),
          "per-wheel speed is reported");
    CHECK(speeds[1] == speeds[0] && speeds[2] == speeds[0] &&
          speeds[3] == speeds[0], "per-wheel speeds match");
}

static void test_pid_gain_update(void)
{
    service_chassis_cfg_t cfg = make_cfg();
    float_t gain;

    reset_motor_stubs();
    CHECK(service_chassis_init(&cfg) == PLATFORM_ERR_OK, "gain init");

    CHECK(service_chassis_get_pid_gain(SERVICE_CHASSIS_WHEEL_FL,
                                       SERVICE_CHASSIS_PID_KP,
                                       &gain) == PLATFORM_ERR_OK,
          "gain read");
    CHECK(gain == 2.0f, "initial gain");

    /* Only FL changes; the other three keep the startup value. */
    CHECK(service_chassis_set_pid_gain(SERVICE_CHASSIS_WHEEL_FL,
                                       SERVICE_CHASSIS_PID_KP,
                                       4.0f) == PLATFORM_ERR_OK,
          "gain write");
    CHECK(service_chassis_get_pid_gain(SERVICE_CHASSIS_WHEEL_FL,
                                       SERVICE_CHASSIS_PID_KP,
                                       &gain) == PLATFORM_ERR_OK,
          "gain read back");
    CHECK(gain == 4.0f, "FL gain updated");
    CHECK(service_chassis_get_pid_gain(SERVICE_CHASSIS_WHEEL_FR,
                                       SERVICE_CHASSIS_PID_KP,
                                       &gain) == PLATFORM_ERR_OK,
          "FR gain read");
    CHECK(gain == 2.0f, "FR gain untouched");

    CHECK(service_chassis_set_pid_gain(SERVICE_CHASSIS_WHEEL_COUNT,
                                       SERVICE_CHASSIS_PID_KP,
                                       1.0f) == PLATFORM_ERR_PARAM,
          "wheel out of range");
    CHECK(service_chassis_set_pid_gain(SERVICE_CHASSIS_WHEEL_FL,
                                       (service_chassis_pid_gain_t)3,
                                       1.0f) == PLATFORM_ERR_PARAM,
          "gain index out of range");
    CHECK(service_chassis_set_pid_gain(SERVICE_CHASSIS_WHEEL_FL,
                                       SERVICE_CHASSIS_PID_KP,
                                       -1.0f) == PLATFORM_ERR_PARAM,
          "negative gain rejected");

    /* The new gain is used by the next tick, from a clean integral. */
    CHECK(service_chassis_enable(PLATFORM_TRUE, 0u) == PLATFORM_ERR_OK,
          "gain enable");
    (void)service_chassis_apply(0u, 10u);
    CHECK(service_chassis_set_velocity(200, 0, 0, 10u) == PLATFORM_ERR_OK,
          "gain target");
    (void)service_chassis_apply(10u, 10u);
    CHECK(g_duty[0] == 800, "FL runs at the new Kp");
    CHECK(g_duty[1] == 400, "FR keeps the startup Kp");
}

static void test_pid_reset_on_enable(void)
{
    service_chassis_cfg_t cfg = make_cfg();
    uint8_t i;

    reset_motor_stubs();

    for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
    {
        cfg.pid_kp[i] = 0.0f;
        cfg.pid_ki[i] = 1.0f;
        cfg.pid_kd[i] = 0.0f;
    }

    CHECK(service_chassis_init(&cfg) == PLATFORM_ERR_OK, "integral init");
    CHECK(service_chassis_enable(PLATFORM_TRUE, 0u) == PLATFORM_ERR_OK,
          "integral enable");

    (void)service_chassis_apply(0u, 10u);
    CHECK(service_chassis_set_velocity(200, 0, 0, 10u) == PLATFORM_ERR_OK,
          "integral target");

    (void)service_chassis_apply(10u, 10u);
    CHECK(g_duty[0] == 2, "first integral step");
    (void)service_chassis_apply(20u, 10u);
    CHECK(g_duty[0] == 4, "integral accumulates");

    /* A zero target at rest must drop the integral instead of creeping. */
    CHECK(service_chassis_set_velocity(0, 0, 0, 25u) == PLATFORM_ERR_OK,
          "zero target");
    (void)service_chassis_apply(25u, 10u);
    CHECK(g_duty[0] == 0, "zero target at rest clears the integral");

    /* Disable/re-enable must not resume from the old integral. */
    CHECK(service_chassis_enable(PLATFORM_FALSE, 30u) == PLATFORM_ERR_OK,
          "disable");
    (void)service_chassis_apply(30u, 10u);
    CHECK(service_chassis_enable(PLATFORM_TRUE, 40u) == PLATFORM_ERR_OK,
          "re-enable");
    CHECK(service_chassis_set_velocity(200, 0, 0, 40u) == PLATFORM_ERR_OK,
          "target after re-enable");
    (void)service_chassis_apply(40u, 10u);
    CHECK(g_duty[0] == 2, "enable transition clears the integral");
}

static void test_open_loop_fallback(void)
{
    service_chassis_cfg_t cfg = make_cfg();
    int16_t speeds[SERVICE_CHASSIS_WHEEL_COUNT];

    reset_motor_stubs();
    cfg.counts_per_mm_x1000 = 0;

    CHECK(service_chassis_init(&cfg) == PLATFORM_ERR_OK, "fallback init");
    CHECK(service_chassis_enable(PLATFORM_TRUE, 0u) == PLATFORM_ERR_OK,
          "fallback enable");
    (void)service_chassis_apply(0u, 10u);

    CHECK(service_chassis_set_velocity(200, 0, 0, 10u) == PLATFORM_ERR_OK,
          "fallback target");
    (void)service_chassis_apply(10u, 10u);
    CHECK(g_duty[0] == 400, "open-loop duty when counts_per_mm is 0");

    service_chassis_get_wheel_speeds(speeds);
    CHECK(speeds[0] == 0, "no wheel speed without feedback");
}

/* ----------------------- runtime calibration --------------------------- */

/*
 * Geometry feeds the mecanum k term. With the open-loop fallback the duty is
 * proportional to the wheel target, so a rotate command exposes k directly:
 * k = 270 -> 540 permille, k = 290 -> 580 permille.
 */
static void test_runtime_geometry(void)
{
    service_chassis_cfg_t cfg = make_cfg();

    reset_motor_stubs();
    cfg.counts_per_mm_x1000 = 0;

    CHECK(service_chassis_init(&cfg) == PLATFORM_ERR_OK, "geometry init");
    CHECK(service_chassis_set_geometry(0, 120) == PLATFORM_ERR_PARAM,
          "zero half length refused");
    CHECK(service_chassis_set_geometry(150, 1001) == PLATFORM_ERR_PARAM,
          "geometry above the sanity bound refused");

    CHECK(service_chassis_enable(PLATFORM_TRUE, 0u) == PLATFORM_ERR_OK,
          "geometry enable");
    (void)service_chassis_apply(0u, 10u);

    CHECK(service_chassis_set_velocity(0, 0, 1000, 10u) == PLATFORM_ERR_OK,
          "geometry rotate");
    (void)service_chassis_apply(10u, 10u);
    CHECK(g_duty[0] == -540, "rotate uses the startup k");

    CHECK(service_chassis_set_geometry(160, 130) == PLATFORM_ERR_OK,
          "geometry write");
    CHECK(service_chassis_set_velocity(0, 0, 1000, 20u) == PLATFORM_ERR_OK,
          "geometry rotate again");
    (void)service_chassis_apply(20u, 10u);
    CHECK(g_duty[0] == -580, "rotate uses the new k");
    CHECK(g_duty[1] == 580, "the mirrored wheel follows");
}

/*
 * Scale and direction change the ruler the loop and the odometry use. A
 * mirrored motor (dir = -1) must report positive speed for a positive command,
 * and the odometry must move the robot the other way for the same counts.
 */
static void test_runtime_wheel_scale(void)
{
    service_chassis_cfg_t cfg = make_cfg();
    service_chassis_odometry_t odom;
    uint8_t i;

    reset_motor_stubs();
    CHECK(service_chassis_init(&cfg) == PLATFORM_ERR_OK, "scale init");

    CHECK(service_chassis_set_wheel_scale(SERVICE_CHASSIS_WHEEL_FL, 24828,
                                          -1) == PLATFORM_ERR_OK,
          "mirrored wheel accepted");
    CHECK(service_chassis_set_wheel_scale(SERVICE_CHASSIS_WHEEL_FL, 24828,
                                          0) == PLATFORM_ERR_PARAM,
          "direction 0 refused");
    CHECK(service_chassis_set_wheel_scale(SERVICE_CHASSIS_WHEEL_COUNT, 24828,
                                          1) == PLATFORM_ERR_PARAM,
          "wheel index checked");

    CHECK(service_chassis_enable(PLATFORM_TRUE, 0u) == PLATFORM_ERR_OK,
          "scale enable");
    (void)service_chassis_apply(0u, 10u);
    CHECK(service_chassis_set_velocity(100, 0, 0, 10u) == PLATFORM_ERR_OK,
          "scale target");
    (void)service_chassis_apply(10u, 10u);
    CHECK(g_duty[0] == 200 && g_duty[1] == 200, "both loops open-loop first");

    /* FL sees +25 counts, but it is mirrored, so it reads -100.7 mm/s. */
    g_encoder[0] += 25;
    (void)service_chassis_apply(20u, 10u);
    CHECK((g_duty[0] >= 399) && (g_duty[0] <= 403),
          "mirrored wheel speed has the reversed sign");
    CHECK(g_duty[1] == 200, "the other wheels are unaffected");

    /* All four mirrored: the same counts now drive the robot backwards. */
    for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
    {
        g_encoder[i] = TEST_COUNTS_PER_MM_X1000;
        CHECK(service_chassis_set_wheel_scale(i, TEST_COUNTS_PER_MM_X1000,
                                              -1) == PLATFORM_ERR_OK,
              "odometry scale write");
    }

    service_chassis_update_odometry(1000u);
    service_chassis_get_odometry(&odom);
    CHECK(odom.x_mm == 0,
          "re-baselined odometry ignores counts from before the change");

    for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
    {
        g_encoder[i] += TEST_COUNTS_PER_MM_X1000;
    }

    service_chassis_update_odometry(1000u);
    service_chassis_get_odometry(&odom);
    CHECK(odom.x_mm == -1000, "a mirrored wheel drives the odometry back");
}

/*
 * Reliable start duty lifts the loop output only while the wheel is at rest.
 * The fixture is pure P with Kp = 2: a 100 mm/s target is 200 permille.
 */
static void test_runtime_start_duty(void)
{
    service_chassis_cfg_t cfg = make_cfg();
    uint8_t i;

    reset_motor_stubs();
    CHECK(service_chassis_init(&cfg) == PLATFORM_ERR_OK, "duty init");

    CHECK(service_chassis_set_start_duty(SERVICE_CHASSIS_WHEEL_FL, 300, 250) ==
          PLATFORM_ERR_OK, "start duty write");
    CHECK(service_chassis_set_start_duty(SERVICE_CHASSIS_WHEEL_FL, -1, 0) ==
          PLATFORM_ERR_PARAM, "negative start duty refused");
    CHECK(service_chassis_set_start_duty(SERVICE_CHASSIS_WHEEL_FL, 0, 1001) ==
          PLATFORM_ERR_PARAM, "start duty above the bound refused");
    CHECK(service_chassis_set_start_duty(SERVICE_CHASSIS_WHEEL_COUNT, 0, 0) ==
          PLATFORM_ERR_PARAM, "wheel index checked");

    CHECK(service_chassis_enable(PLATFORM_TRUE, 0u) == PLATFORM_ERR_OK,
          "duty enable");
    (void)service_chassis_apply(0u, 10u);

    CHECK(service_chassis_set_velocity(100, 0, 0, 10u) == PLATFORM_ERR_OK,
          "duty target");
    (void)service_chassis_apply(10u, 10u);
    CHECK(g_duty[0] == 300, "forward start duty lifts the FL output");
    for (i = 1u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
    {
        CHECK(g_duty[i] == 200, "uncalibrated wheels keep the P output");
    }

    /* The wheel turns: measured speed is about 100.7, so the floor is gone. */
    g_encoder[0] += 25;
    (void)service_chassis_apply(20u, 10u);
    CHECK((g_duty[0] >= -3) && (g_duty[0] <= 0),
          "a turning wheel may brake below the start duty");

    CHECK(service_chassis_set_velocity(-100, 0, 0, 30u) == PLATFORM_ERR_OK,
          "duty reverse target");
    (void)service_chassis_apply(40u, 10u);
    CHECK(g_duty[0] == -250, "reverse start duty uses its own value");
}

static void test_runtime_limits(void)
{
    service_chassis_cfg_t cfg = make_cfg();

    reset_motor_stubs();
    CHECK(service_chassis_init(&cfg) == PLATFORM_ERR_OK, "limits init");

    CHECK(service_chassis_set_limits(0u, 500, 500, 1500) == PLATFORM_ERR_PARAM,
          "zero timeout refused");
    CHECK(service_chassis_set_limits(200u, 0, 500, 1500) == PLATFORM_ERR_PARAM,
          "zero linear limit refused");
    CHECK(service_chassis_set_limits(300u, 100, 100, 800) == PLATFORM_ERR_OK,
          "limits write");

    CHECK(service_chassis_set_velocity(150, 0, 0, 10u) == PLATFORM_ERR_PARAM,
          "the new linear limit rejects 150 mm/s");
    CHECK(service_chassis_set_velocity(100, 0, 0, 10u) == PLATFORM_ERR_OK,
          "the new linear limit accepts 100 mm/s");
    CHECK(service_chassis_set_velocity(100, 0, 900, 10u) == PLATFORM_ERR_PARAM,
          "the new angular limit rejects 900 mrad/s");
}

/* ---------------------------- test duty -------------------------------- */

static void test_test_duty(void)
{
    service_chassis_cfg_t cfg = make_cfg();
    int16_t duty[SERVICE_CHASSIS_WHEEL_COUNT];

    reset_motor_stubs();
    (void)service_chassis_init(&cfg);

    /* Refused until the bridges are on: a duty with nowhere to go would look
       like an encoder fault on the bench. */
    CHECK(service_chassis_set_test_duty(SERVICE_CHASSIS_WHEEL_FL, 100, 0u) ==
          PLATFORM_ERR_FAIL, "test duty needs enable");
    CHECK(service_chassis_is_test_mode() == PLATFORM_FALSE, "not in test mode");

    CHECK(service_chassis_enable(PLATFORM_TRUE, 0u) == PLATFORM_ERR_OK,
          "test enable");
    CHECK(service_chassis_set_test_duty(SERVICE_CHASSIS_WHEEL_COUNT, 100, 0u) ==
          PLATFORM_ERR_PARAM, "wheel out of range");
    CHECK(service_chassis_set_test_duty(SERVICE_CHASSIS_WHEEL_FL,
                                        SERVICE_CHASSIS_TEST_DUTY_MAX + 1, 0u) ==
          PLATFORM_ERR_PARAM, "cap enforced, not clamped");
    CHECK(service_chassis_set_test_duty(SERVICE_CHASSIS_WHEEL_FL, 150, 1000u) ==
          PLATFORM_ERR_OK, "test duty accepted");
    CHECK(service_chassis_is_test_mode() == PLATFORM_TRUE, "test mode entered");

    /* Only the commanded wheel moves; the other three are explicitly zero. */
    (void)service_chassis_apply(1010u, 10u);
    CHECK(g_duty[0] == 150 && g_duty[1] == 0 && g_duty[2] == 0 &&
          g_duty[3] == 0, "only the test wheel is driven");

    service_chassis_get_wheel_duty(duty);
    CHECK(duty[0] == 150 && duty[1] == 0, "duty is readable back");

    /* Refreshing keeps it going past the original expiry. */
    CHECK(service_chassis_set_test_duty(SERVICE_CHASSIS_WHEEL_FL, 150, 1200u) ==
          PLATFORM_ERR_OK, "test duty refresh");
    (void)service_chassis_apply(1290u, 10u);
    CHECK(g_duty[0] == 150, "refresh holds the duty");

    /* No refresh for > 300 ms: the wheel coasts and test mode ends by itself. */
    (void)service_chassis_apply(1600u, 10u);
    CHECK(g_duty[0] == 0 && g_duty[1] == 0 && g_duty[2] == 0 &&
          g_duty[3] == 0, "stale test duty coasts");
    CHECK(service_chassis_is_test_mode() == PLATFORM_FALSE,
          "all wheels stale ends test mode");

    /* A velocity command exits test mode at once. */
    CHECK(service_chassis_set_test_duty(SERVICE_CHASSIS_WHEEL_FR, -200, 1700u) ==
          PLATFORM_ERR_OK, "negative test duty");
    (void)service_chassis_apply(1710u, 10u);
    CHECK(g_duty[1] == -200, "reverse test duty");
    CHECK(service_chassis_set_velocity(200, 0, 0, 1720u) == PLATFORM_ERR_OK,
          "velocity after test");
    CHECK(service_chassis_is_test_mode() == PLATFORM_FALSE,
          "velocity leaves test mode");
    (void)service_chassis_apply(1730u, 10u);
    CHECK(g_duty[1] == 400, "PID owns the wheel again");

    /* Disabling also leaves test mode. */
    CHECK(service_chassis_set_test_duty(SERVICE_CHASSIS_WHEEL_RR, 100, 1740u) ==
          PLATFORM_ERR_OK, "test duty before disable");
    CHECK(service_chassis_enable(PLATFORM_FALSE, 1750u) == PLATFORM_ERR_OK,
          "test disable");
    CHECK(service_chassis_is_test_mode() == PLATFORM_FALSE,
          "disable leaves test mode");
}

/* ------------------------------ odometry ------------------------------- */

static void test_odometry(void)
{
    service_chassis_cfg_t cfg = make_cfg();
    service_chassis_odometry_t odom;

    reset_motor_stubs();
    (void)service_chassis_init(&cfg);

    /* Four wheels, 24828 counts = 1000 mm of travel each. */
    g_encoder[0] = TEST_COUNTS_PER_MM_X1000;
    g_encoder[1] = TEST_COUNTS_PER_MM_X1000;
    g_encoder[2] = TEST_COUNTS_PER_MM_X1000;
    g_encoder[3] = TEST_COUNTS_PER_MM_X1000;
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
    test_counts_per_mm();
    test_enable_and_duty();
    test_velocity_limits();
    test_timeout();
    test_emergency_stop();
    test_speed_feedback();
    test_pid_gain_update();
    test_pid_reset_on_enable();
    test_open_loop_fallback();
    test_runtime_geometry();
    test_runtime_wheel_scale();
    test_runtime_start_duty();
    test_runtime_limits();
    test_test_duty();
    test_odometry();

    if (g_failed != 0)
    {
        printf("FAIL: service_chassis tests (%d failure(s))\n", g_failed);
        return 1;
    }

    printf("PASS: service_chassis tests\n");
    return 0;
}
