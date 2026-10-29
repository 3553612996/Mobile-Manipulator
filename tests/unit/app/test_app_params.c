/* L1 unit test for 01_APP/app_robot/app_params.c.
 *
 * The module owns three things: the RAM copy of the parameters, the defaults
 * from app_robot_config.h, and the on-flash byte layout. The test checks all
 * three plus the one rule that matters on load: a blob that fails validation
 * must leave RAM untouched.
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
 *       tests/unit/app/test_app_params.c \
 *       firmware/app/01_APP/app_robot/app_params.c \
 *       firmware/app/05_Middlewares/protocol/protocol_def.c \
 *       -o build/test_app_params
 */

#include <stdio.h>
#include <string.h>

#include "app_robot/app_params.h"
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

/* ------------------------------ chassis stub --------------------------- */

static int    g_gain_calls;
static float_t g_last_gain;
static int16_t g_half_length;
static int16_t g_half_width;
static int32_t g_wheel_counts;
static int8_t  g_wheel_dir;
static int16_t g_duty_fwd;
static int16_t g_duty_rev;
static uint16_t g_timeout;
static int16_t g_max_vx;
static int16_t g_max_vy;
static int16_t g_max_wz;
static int     g_limits_calls;

platform_err_t service_chassis_set_pid_gain(uint8_t wheel,
                                            service_chassis_pid_gain_t gain,
                                            float_t value)
{
    (void)wheel;
    (void)gain;

    g_gain_calls++;
    g_last_gain = value;

    return PLATFORM_ERR_OK;
}

platform_err_t service_chassis_set_geometry(int16_t half_length_mm,
                                            int16_t half_width_mm)
{
    g_half_length = half_length_mm;
    g_half_width = half_width_mm;

    return PLATFORM_ERR_OK;
}

platform_err_t service_chassis_set_wheel_scale(uint8_t wheel,
                                               int32_t counts_per_mm_x1000,
                                               int8_t dir)
{
    (void)wheel;

    g_wheel_counts = counts_per_mm_x1000;
    g_wheel_dir = dir;

    return PLATFORM_ERR_OK;
}

platform_err_t service_chassis_set_start_duty(uint8_t wheel,
                                              int16_t fwd_permille,
                                              int16_t rev_permille)
{
    (void)wheel;

    g_duty_fwd = fwd_permille;
    g_duty_rev = rev_permille;

    return PLATFORM_ERR_OK;
}

platform_err_t service_chassis_set_limits(uint16_t cmd_timeout_ms,
                                          int16_t max_vx_mm_s,
                                          int16_t max_vy_mm_s,
                                          int16_t max_wz_mrad_s)
{
    g_limits_calls++;
    g_timeout = cmd_timeout_ms;
    g_max_vx = max_vx_mm_s;
    g_max_vy = max_vy_mm_s;
    g_max_wz = max_wz_mrad_s;

    return PLATFORM_ERR_OK;
}

/* -------------------------------- helpers ------------------------------ */

static int32_t expected_default_gain(uint8_t wheel, uint8_t gain)
{
    platform_bool_t left = (wheel == 0u) || (wheel == 2u) ? PLATFORM_TRUE
                                                          : PLATFORM_FALSE;
    float_t value;

    if (gain == 0u)
    {
        value = (left == PLATFORM_TRUE) ? APP_CHASSIS_LEFT_PID_KP
                                        : APP_CHASSIS_RIGHT_PID_KP;
    }
    else if (gain == 1u)
    {
        value = (left == PLATFORM_TRUE) ? APP_CHASSIS_LEFT_PID_KI
                                        : APP_CHASSIS_RIGHT_PID_KI;
    }
    else
    {
        value = (left == PLATFORM_TRUE) ? APP_CHASSIS_LEFT_PID_KD
                                        : APP_CHASSIS_RIGHT_PID_KD;
    }

    return (int32_t)((value * 1000.0f) + 0.5f);
}

/* -------------------------------- tests -------------------------------- */

static void test_defaults(void)
{
    app_params_init();

    CHECK(app_params_log_level() == 1u, "default log level is INFO");
    CHECK(app_params_half_length_mm() == APP_CHASSIS_HALF_LENGTH_MM,
          "default half length");
    CHECK(app_params_half_width_mm() == APP_CHASSIS_HALF_WIDTH_MM,
          "default half width");
    CHECK(app_params_counts_per_mm_x1000() == APP_CHASSIS_COUNTS_PER_MM_X1000,
          "default counts per mm");
    CHECK(app_params_pid_gain_x1000(0u, 0u) == expected_default_gain(0u, 0u),
          "left wheel default Kp");
    CHECK(app_params_pid_gain_x1000(3u, 2u) == expected_default_gain(3u, 2u),
          "right wheel default Kd");
    CHECK(app_params_dirty() == PLATFORM_FALSE, "init is clean");
}

static void test_writer_limits(void)
{
    app_params_init();

    CHECK(PLATFORM_IS_ERR(app_params_set_pid_gain(0u, 0u, -1)),
          "negative gain refused");
    CHECK(PLATFORM_IS_ERR(
              app_params_set_pid_gain(0u, 0u, APP_PID_GAIN_MAX_X1000 + 1)),
          "gain above the Kp/Ki limit refused");
    CHECK(PLATFORM_IS_ERR(
              app_params_set_pid_gain(0u, 2u, APP_PID_KD_MAX_X1000 + 1)),
          "Kd has its own, lower limit");
    CHECK(PLATFORM_IS_OK(app_params_set_pid_gain(1u, 2u,
                                                 APP_PID_KD_MAX_X1000)),
          "Kd at the limit accepted");
    CHECK(PLATFORM_IS_ERR(app_params_set_pid_gain(4u, 0u, 100)),
          "wheel index checked");

    CHECK(PLATFORM_IS_ERR(app_params_set_log_level(4u)),
          "log level above ERROR refused");
    CHECK(PLATFORM_IS_OK(app_params_set_log_level(0u)), "DEBUG accepted");

    CHECK(app_params_dirty() == PLATFORM_TRUE, "writes set the dirty bit");
}

static void test_log_filter(void)
{
    app_params_init();
    (void)app_params_set_log_level(2u);     /* WARN */

    CHECK(app_params_log_enabled(0u) == PLATFORM_FALSE, "DEBUG filtered");
    CHECK(app_params_log_enabled(1u) == PLATFORM_FALSE, "INFO filtered");
    CHECK(app_params_log_enabled(2u) == PLATFORM_TRUE, "WARN kept");
    CHECK(app_params_log_enabled(3u) == PLATFORM_TRUE, "ERROR kept");
}

static void test_param_roundtrip(void)
{
    uint8_t blob[APP_PARAMS_BLOB_PARAM_SIZE];
    uint16_t length;

    app_params_init();
    (void)app_params_set_pid_gain(2u, 1u, 1234);
    (void)app_params_set_pid_gain(3u, 2u, 99);
    (void)app_params_set_log_level(3u);

    length = app_params_serialize(APP_PARAMS_SCOPE_PARAM, blob,
                                  (uint16_t)sizeof(blob));
    CHECK(length == APP_PARAMS_BLOB_PARAM_SIZE, "param blob length");
    CHECK(blob[0] == (uint8_t)(APP_PARAMS_VERSION & 0xFFu) &&
          blob[1] == (uint8_t)(APP_PARAMS_VERSION >> 8),
          "version is little-endian at offset 0");
    CHECK(blob[2] == (uint8_t)(length & 0xFFu) &&
          blob[3] == (uint8_t)(length >> 8),
          "length is little-endian at offset 2");

    /* Start over, then load what was saved. */
    app_params_init();
    CHECK(app_params_parse(APP_PARAMS_SCOPE_PARAM, blob, length) ==
          PLATFORM_TRUE, "saved blob parses");
    CHECK(app_params_pid_gain_x1000(2u, 1u) == 1234, "gain restored");
    CHECK(app_params_pid_gain_x1000(3u, 2u) == 99, "Kd restored");
    CHECK(app_params_log_level() == 3u, "log level restored");
    CHECK(app_params_dirty() == PLATFORM_FALSE, "loading does not dirty");
}

static void put_i32(uint8_t *p, int32_t value)
{
    p[0] = (uint8_t)(value & 0xFF);
    p[1] = (uint8_t)((value >> 8) & 0xFF);
    p[2] = (uint8_t)((value >> 16) & 0xFF);
    p[3] = (uint8_t)((value >> 24) & 0xFF);
}

static void test_calib_roundtrip_and_bounds(void)
{
    uint8_t blob[APP_PARAMS_BLOB_CALIB_SIZE];
    uint16_t length;

    app_params_init();
    CHECK(PLATFORM_IS_OK(app_params_set_track_mm(250)),
          "track in range accepted");
    CHECK(PLATFORM_IS_OK(app_params_set_wheel_diameter_mm(65)),
          "diameter in range accepted");
    CHECK(PLATFORM_IS_OK(app_params_set_wheel_counts(0u, 23000)),
          "per-wheel trim accepted");
    CHECK(PLATFORM_IS_OK(app_params_set_wheel_dir(1u, -1)),
          "negative direction accepted");
    CHECK(PLATFORM_IS_OK(app_params_set_wheel_deadzone(2u, 120, 140)),
          "start duty accepted");

    length = app_params_serialize(APP_PARAMS_SCOPE_CALIB, blob,
                                  (uint16_t)sizeof(blob));
    CHECK(length == APP_PARAMS_BLOB_CALIB_SIZE, "calib blob length");

    app_params_init();
    CHECK(app_params_parse(APP_PARAMS_SCOPE_CALIB, blob, length) ==
          PLATFORM_TRUE, "saved calib parses");
    CHECK(app_params_half_width_mm() == 125, "track restored as half width");
    CHECK(app_params_wheel_diameter_x10_mm() == 650, "diameter restored");
    CHECK(app_params_wheel_counts_per_mm_x1000(0u) == 23000,
          "per-wheel trim restored");
    CHECK(app_params_wheel_dir(1u) == -1, "direction restored");
    CHECK(app_params_deadzone_fwd_permille(2u) == 120, "fwd duty restored");
    CHECK(app_params_deadzone_rev_permille(2u) == 140, "rev duty restored");
    CHECK(app_params_wheel_dir(0u) == 1,
          "a wheel never touched keeps +1");

    /* Zero track (offset 8) must be refused and leave RAM untouched. */
    put_i32(&blob[8], 0);
    CHECK(app_params_parse(APP_PARAMS_SCOPE_CALIB, blob, length) ==
          PLATFORM_FALSE, "zero track refused");
    CHECK(app_params_half_width_mm() == 125,
          "a rejected blob leaves RAM untouched");

    /* A direction of 0 is never a valid wheel direction. */
    put_i32(&blob[8], 250);
    put_i32(&blob[32], 0);
    CHECK(app_params_parse(APP_PARAMS_SCOPE_CALIB, blob, length) ==
          PLATFORM_FALSE, "direction 0 refused");
    CHECK(app_params_wheel_dir(0u) == 1, "still untouched after reject");
}

static void test_calib_writer_limits(void)
{
    app_params_init();

    CHECK(PLATFORM_IS_ERR(app_params_set_track_mm(49)),
          "track below the minimum refused");
    CHECK(PLATFORM_IS_ERR(app_params_set_wheel_diameter_mm(0)),
          "zero diameter refused");
    CHECK(PLATFORM_IS_ERR(app_params_set_wheel_counts(0u, 999)),
          "counts below the minimum refused");
    CHECK(PLATFORM_IS_ERR(app_params_set_wheel_dir(0u, 0)),
          "direction must be +1 or -1");
    CHECK(PLATFORM_IS_ERR(app_params_set_wheel_deadzone(0u, -1, 0)),
          "negative duty refused");
    CHECK(PLATFORM_IS_ERR(app_params_set_wheel_deadzone(4u, 0, 0)),
          "wheel index checked");
    CHECK(PLATFORM_IS_ERR(app_params_set_cmd_timeout_ms(0)),
          "zero timeout refused");
    CHECK(PLATFORM_IS_ERR(app_params_set_max_linear(0)),
          "zero linear limit refused");
    CHECK(PLATFORM_IS_ERR(app_params_set_max_angular(0)),
          "zero angular limit refused");
}

static void test_v1_blobs_still_load(void)
{
    uint8_t param[APP_PARAMS_BLOB_PARAM_SIZE_V1];
    uint8_t calib[APP_PARAMS_BLOB_CALIB_SIZE_V1];
    uint8_t version = 1u;
    uint16_t size;
    int32_t gain;
    uint8_t i;

    memset(param, 0, sizeof(param));
    memset(calib, 0, sizeof(calib));

    /* v1 param: header, 12 gains, log level. */
    param[0] = version;
    param[1] = 0u;
    param[2] = (uint8_t)(sizeof(param) & 0xFFu);
    param[3] = (uint8_t)(sizeof(param) >> 8);
    for (i = 0u; i < 12u; i++)
    {
        put_i32(&param[4u + (i * 4u)], 1500);
    }
    param[52] = 2u;

    app_params_init();
    CHECK(app_params_parse(APP_PARAMS_SCOPE_PARAM, param,
                           (uint16_t)sizeof(param)) == PLATFORM_TRUE,
          "v1 param blob accepted");
    gain = app_params_pid_gain_x1000(0u, 0u);
    CHECK(gain == 1500, "v1 gain restored");
    CHECK(app_params_log_level() == 2u, "v1 log level restored");
    CHECK(app_params_cmd_timeout_ms() == APP_CHASSIS_CMD_TIMEOUT_MS,
          "v1 blob keeps the compile-time timeout");
    CHECK(app_params_max_linear_mm_s() == APP_CHASSIS_MAX_VX_MM_S,
          "v1 blob keeps the compile-time linear limit");
    CHECK(app_params_max_angular_mrad_s() == APP_CHASSIS_MAX_WZ_MRAD_S,
          "v1 blob keeps the compile-time angular limit");

    /* v1 calib: header, half length, half width, four counts. */
    calib[0] = version;
    calib[1] = 0u;
    calib[2] = (uint8_t)(sizeof(calib) & 0xFFu);
    calib[3] = (uint8_t)(sizeof(calib) >> 8);
    put_i32(&calib[4], 160);
    put_i32(&calib[8], 130);
    for (i = 0u; i < 4u; i++)
    {
        put_i32(&calib[12u + (i * 4u)], 24000);
    }

    size = (uint16_t)sizeof(calib);
    CHECK(app_params_parse(APP_PARAMS_SCOPE_CALIB, calib, size) ==
          PLATFORM_TRUE, "v1 calib blob accepted");
    CHECK(app_params_half_length_mm() == 160, "v1 half length doubled");
    CHECK(app_params_half_width_mm() == 130, "v1 half width doubled");
    CHECK(app_params_wheel_counts_per_mm_x1000(3u) == 24000,
          "v1 counts applied to every wheel");
    CHECK(app_params_wheel_dir(0u) == 1, "v1 has no direction, defaults to +1");
    /* v1 has no deadzone, so the per-wheel defaults stay (FL = 75/67). */
    CHECK(app_params_deadzone_fwd_permille(0u) == 75,
          "v1 keeps the default forward start duty");
    CHECK(app_params_deadzone_rev_permille(0u) == 67,
          "v1 keeps the default reverse start duty");
}

static void test_parse_rejects_bad_blobs(void)
{
    uint8_t blob[APP_PARAMS_BLOB_PARAM_SIZE];
    uint16_t length;
    int32_t before;

    app_params_init();
    (void)app_params_set_pid_gain(0u, 0u, 777);

    length = app_params_serialize(APP_PARAMS_SCOPE_PARAM, blob,
                                  (uint16_t)sizeof(blob));
    before = app_params_pid_gain_x1000(0u, 0u);

    /* Wrong version. */
    blob[0] = (uint8_t)(APP_PARAMS_VERSION + 1u);
    CHECK(app_params_parse(APP_PARAMS_SCOPE_PARAM, blob, length) ==
          PLATFORM_FALSE, "wrong version refused");
    CHECK(app_params_pid_gain_x1000(0u, 0u) == before, "RAM untouched");

    /* Wrong declared length. */
    length = app_params_serialize(APP_PARAMS_SCOPE_PARAM, blob,
                                  (uint16_t)sizeof(blob));
    CHECK(app_params_parse(APP_PARAMS_SCOPE_PARAM, blob,
                           (uint16_t)(length - 1u)) == PLATFORM_FALSE,
          "truncated blob refused");

    /* Out-of-range log level inside an otherwise valid blob. */
    blob[52] = 4u;
    CHECK(app_params_parse(APP_PARAMS_SCOPE_PARAM, blob, length) ==
          PLATFORM_FALSE, "bad log level refused");
    CHECK(app_params_pid_gain_x1000(0u, 0u) == before, "RAM still untouched");

    /* A whole-blob scope value is not an encoding scope. */
    CHECK(app_params_parse(APP_PARAMS_SCOPE_ALL, blob, length) ==
          PLATFORM_FALSE, "scope ALL is not serialized/parsed");
    CHECK(app_params_serialize(APP_PARAMS_SCOPE_ALL, blob,
                               (uint16_t)sizeof(blob)) == 0u,
          "scope ALL cannot be serialized as one blob");
}

static void test_dirty_per_scope(void)
{
    app_params_init();

    app_params_mark_dirty(APP_PARAMS_SCOPE_PARAM);
    CHECK(app_params_dirty() == PLATFORM_TRUE, "param scope dirties");

    app_params_mark_saved(APP_PARAMS_SCOPE_PARAM);
    CHECK(app_params_dirty() == PLATFORM_FALSE, "param save cleans");

    app_params_mark_dirty(APP_PARAMS_SCOPE_CALIB);
    app_params_mark_saved(APP_PARAMS_SCOPE_PARAM);
    CHECK(app_params_dirty() == PLATFORM_TRUE,
          "saving the other scope does not clean this one");

    app_params_mark_saved(APP_PARAMS_SCOPE_CALIB);
    CHECK(app_params_dirty() == PLATFORM_FALSE, "each scope saves itself");
}

static void test_reset_marks_dirty_and_reloads(void)
{
    app_params_init();
    (void)app_params_set_pid_gain(0u, 0u, 4321);

    app_params_reset(APP_PARAMS_SCOPE_PARAM);
    CHECK(app_params_pid_gain_x1000(0u, 0u) == expected_default_gain(0u, 0u),
          "reset restores defaults in RAM");
    CHECK(app_params_dirty() == PLATFORM_TRUE, "reset is only RAM, so dirty");

    g_gain_calls = 0;
    g_limits_calls = 0;
    app_params_apply_to_chassis();
    CHECK(g_gain_calls == (int)(APP_PARAMS_WHEEL_COUNT *
                                APP_PARAMS_GAIN_COUNT),
          "apply pushes every wheel/gain pair");
    CHECK(g_last_gain >= 0.0f, "gain pushed in unscaled units");
    CHECK(g_half_length == (APP_CHASSIS_HALF_LENGTH_MM),
          "geometry pushed as half length");
    CHECK(g_half_width == (APP_CHASSIS_HALF_WIDTH_MM),
          "geometry pushed as half width");
    CHECK(g_wheel_counts == APP_CHASSIS_COUNTS_PER_MM_X1000,
          "per-wheel scale pushed");
    CHECK(g_wheel_dir == 1, "per-wheel direction pushed");
    /* The stub keeps the last call, so this is the RR wheel's defaults. */
    CHECK((g_duty_fwd == 80) && (g_duty_rev == 100),
          "per-wheel start duty pushed");
    CHECK(g_limits_calls == 1, "limits pushed once");
    CHECK(g_timeout == (uint16_t)APP_CHASSIS_CMD_TIMEOUT_MS,
          "timeout pushed");
    CHECK(g_max_vx == APP_CHASSIS_MAX_VX_MM_S, "linear limit pushed");
    CHECK(g_max_wz == APP_CHASSIS_MAX_WZ_MRAD_S, "angular limit pushed");

    /* The limits-only reload must not touch the PID state. */
    g_gain_calls = 0;
    g_limits_calls = 0;
    app_params_apply_limits_to_chassis();
    CHECK(g_gain_calls == 0, "limits reload leaves the gains alone");
    CHECK(g_limits_calls == 1, "limits reload pushes the limits");
}

int main(void)
{
    test_defaults();
    test_writer_limits();
    test_log_filter();
    test_param_roundtrip();
    test_calib_roundtrip_and_bounds();
    test_calib_writer_limits();
    test_v1_blobs_still_load();
    test_parse_rejects_bad_blobs();
    test_dirty_per_scope();
    test_reset_marks_dirty_and_reloads();

    if (g_failed != 0)
    {
        printf("FAIL: app_params tests (%d checks)\n", g_failed);
        return 1;
    }

    printf("PASS: app_params tests\n");
    return 0;
}
