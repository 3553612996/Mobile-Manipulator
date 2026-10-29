/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_params.c
 *
 * @par dependencies
 * - app_params.h
 * - app_robot_config.h
 * - service_chassis.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement the parameter RAM copy, defaults and on-flash encoding.
 *
 * @version V1.1 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "app_robot/app_params.h"
#include "app_robot/app_robot_config.h"

#include "service_chassis/service_chassis.h"

#include <math.h>

//******************************** Defines **********************************//

#define APP_PARAMS_DIRTY_PARAM      ((uint8_t)0x01u)
#define APP_PARAMS_DIRTY_CALIB      ((uint8_t)0x02u)

#define APP_PARAMS_PI               (3.14159265358979323846f)

/* ------------------------- version 2 parameter blob ---------------------- */
#define APP_PARAMS_OFF_VERSION      (0u)
#define APP_PARAMS_OFF_LENGTH       (2u)
#define APP_PARAMS_OFF_PID          (4u)
#define APP_PARAMS_OFF_LOG_LEVEL    ((uint16_t)(APP_PARAMS_OFF_PID +          \
                                    (APP_PARAMS_WHEEL_COUNT *               \
                                     APP_PARAMS_GAIN_COUNT * 4u)))
#define APP_PARAMS_OFF_CMD_TIMEOUT  ((uint16_t)(APP_PARAMS_OFF_LOG_LEVEL + 2u))
#define APP_PARAMS_OFF_MAX_LINEAR   ((uint16_t)(APP_PARAMS_OFF_CMD_TIMEOUT + 2u))
#define APP_PARAMS_OFF_MAX_ANGULAR  ((uint16_t)(APP_PARAMS_OFF_MAX_LINEAR + 2u))

/* -------------------------- version 2 calibration blob ------------------- */
#define APP_PARAMS_OFF_CAL_WHEELBASE     (4u)
#define APP_PARAMS_OFF_CAL_TRACK         (8u)
#define APP_PARAMS_OFF_CAL_DIAMETER_X10  (12u)
#define APP_PARAMS_OFF_CAL_COUNTS        (16u)   /* 4 x i32. */
#define APP_PARAMS_OFF_CAL_DIR           (32u)   /* 4 x i32. */
#define APP_PARAMS_OFF_CAL_DZ_FWD        (48u)   /* 4 x i32. */
#define APP_PARAMS_OFF_CAL_DZ_REV        (64u)   /* 4 x i32. */

/* ------------------------------ version 1 offsets ------------------------ */
#define APP_PARAMS_OFF_V1_HALF_LENGTH    (4u)
#define APP_PARAMS_OFF_V1_HALF_WIDTH     (8u)
#define APP_PARAMS_OFF_V1_COUNTS         (12u)

/* Sanity bounds, shared by the setters and the parser. */
#define APP_PARAMS_WHEELBASE_MIN_MM      ((int32_t)50)
#define APP_PARAMS_WHEELBASE_MAX_MM      ((int32_t)2000)
#define APP_PARAMS_TRACK_MIN_MM          ((int32_t)50)
#define APP_PARAMS_TRACK_MAX_MM          ((int32_t)2000)
#define APP_PARAMS_HALF_MAX_MM           ((int32_t)1000)
#define APP_PARAMS_DIAMETER_MIN_MM       ((int32_t)10)
#define APP_PARAMS_DIAMETER_MAX_MM       ((int32_t)500)
#define APP_PARAMS_COUNTS_MIN            ((int32_t)1000)
#define APP_PARAMS_COUNTS_MAX            ((int32_t)100000)
#define APP_PARAMS_DEADZONE_MAX_PERMILLE ((int32_t)1000)
#define APP_PARAMS_TIMEOUT_MIN_MS        ((int32_t)10)
#define APP_PARAMS_TIMEOUT_MAX_MS        ((int32_t)5000)
#define APP_PARAMS_MAX_LINEAR_MIN        ((int32_t)10)
#define APP_PARAMS_MAX_LINEAR_MAX        ((int32_t)500)
#define APP_PARAMS_MAX_ANGULAR_MIN       ((int32_t)10)
#define APP_PARAMS_MAX_ANGULAR_MAX       ((int32_t)1500)

//******************************** Variables ********************************//

static struct
{
    int32_t     pid[APP_PARAMS_WHEEL_COUNT][APP_PARAMS_GAIN_COUNT];

    /* Calibration area. */
    int32_t     wheelbase_mm;
    int32_t     track_mm;
    int32_t     wheel_diameter_x10_mm;
    int32_t     wheel_counts_per_mm_x1000[APP_PARAMS_WHEEL_COUNT];
    int8_t      wheel_dir[APP_PARAMS_WHEEL_COUNT];
    int16_t     deadzone_fwd_permille[APP_PARAMS_WHEEL_COUNT];
    int16_t     deadzone_rev_permille[APP_PARAMS_WHEEL_COUNT];

    /* Parameter area. */
    int32_t     cmd_timeout_ms;
    int16_t     max_linear_mm_s;
    int16_t     max_angular_mrad_s;
    uint8_t     log_level;

    uint8_t     dirty;
} s_params;

/* Bench-measured deadzone defaults; see app_robot_config.h for the source. */
static const int16_t s_default_deadzone_fwd[APP_PARAMS_WHEEL_COUNT] =
    APP_CHASSIS_DEADZONE_FWD_PERMILLE;
static const int16_t s_default_deadzone_rev[APP_PARAMS_WHEEL_COUNT] =
    APP_CHASSIS_DEADZONE_REV_PERMILLE;

//******************************** Functions ********************************//

static void s_put_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFFu);
    p[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static uint16_t s_get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void s_put_i32(uint8_t *p, int32_t value)
{
    uint32_t raw = (uint32_t)value;

    p[0] = (uint8_t)(raw & 0xFFu);
    p[1] = (uint8_t)((raw >> 8) & 0xFFu);
    p[2] = (uint8_t)((raw >> 16) & 0xFFu);
    p[3] = (uint8_t)((raw >> 24) & 0xFFu);
}

static void s_put_i16(uint8_t *p, int16_t value)
{
    s_put_u16(p, (uint16_t)value);
}

static int16_t s_get_i16(const uint8_t *p)
{
    return (int16_t)s_get_u16(p);
}

static int32_t s_get_i32(const uint8_t *p)
{
    uint32_t raw;

    raw = (uint32_t)p[0] |
          ((uint32_t)p[1] << 8) |
          ((uint32_t)p[2] << 16) |
          ((uint32_t)p[3] << 24);

    return (int32_t)raw;
}

static int32_t s_gain_limit(uint8_t gain)
{
    return (gain == (uint8_t)APP_PARAMS_GAIN_COUNT - 1u)
               ? APP_PID_KD_MAX_X1000
               : APP_PID_GAIN_MAX_X1000;
}

static int32_t s_default_gain(uint8_t wheel, uint8_t gain)
{
    platform_bool_t left = ((wheel == 0u) || (wheel == 2u)) ? PLATFORM_TRUE
                                                            : PLATFORM_FALSE;
    float_t value;

    if (gain == (uint8_t)SERVICE_CHASSIS_PID_KP)
    {
        value = (left == PLATFORM_TRUE) ? APP_CHASSIS_LEFT_PID_KP
                                        : APP_CHASSIS_RIGHT_PID_KP;
    }
    else if (gain == (uint8_t)SERVICE_CHASSIS_PID_KI)
    {
        value = (left == PLATFORM_TRUE) ? APP_CHASSIS_LEFT_PID_KI
                                        : APP_CHASSIS_RIGHT_PID_KI;
    }
    else
    {
        value = (left == PLATFORM_TRUE) ? APP_CHASSIS_LEFT_PID_KD
                                        : APP_CHASSIS_RIGHT_PID_KD;
    }

    /* The compile-time constants are floats; the blob stores x1000 integers. */
    return (int32_t)((value * 1000.0f) + 0.5f);
}

/* counts_per_mm from the wheel diameter; see APP_CHASSIS_COUNTS_PER_REV. */
static int32_t s_counts_for_diameter(int32_t diameter_mm)
{
    return (int32_t)(((float_t)APP_CHASSIS_COUNTS_PER_REV * 1000.0f) /
                     (APP_PARAMS_PI * (float_t)diameter_mm) + 0.5f);
}

static void s_reset_param(void)
{
    uint8_t wheel;
    uint8_t gain;

    for (wheel = 0u; wheel < APP_PARAMS_WHEEL_COUNT; wheel++)
    {
        for (gain = 0u; gain < APP_PARAMS_GAIN_COUNT; gain++)
        {
            s_params.pid[wheel][gain] = s_default_gain(wheel, gain);
        }
    }

    s_params.cmd_timeout_ms = (int32_t)APP_CHASSIS_CMD_TIMEOUT_MS;
    s_params.max_linear_mm_s = APP_CHASSIS_MAX_VX_MM_S;
    s_params.max_angular_mrad_s = APP_CHASSIS_MAX_WZ_MRAD_S;
    s_params.log_level = 1u;    /* INFO. */
}

static void s_reset_calib(void)
{
    uint8_t wheel;

    s_params.wheelbase_mm = 2 * (int32_t)APP_CHASSIS_HALF_LENGTH_MM;
    s_params.track_mm = 2 * (int32_t)APP_CHASSIS_HALF_WIDTH_MM;
    s_params.wheel_diameter_x10_mm =
        (int32_t)APP_CHASSIS_WHEEL_DIAMETER_MM * 10;

    for (wheel = 0u; wheel < APP_PARAMS_WHEEL_COUNT; wheel++)
    {
        s_params.wheel_counts_per_mm_x1000[wheel] =
            APP_CHASSIS_COUNTS_PER_MM_X1000;
        s_params.wheel_dir[wheel] = 1;
        s_params.deadzone_fwd_permille[wheel] =
            s_default_deadzone_fwd[wheel];
        s_params.deadzone_rev_permille[wheel] =
            s_default_deadzone_rev[wheel];
    }
}

void app_params_init(void)
{
    s_reset_param();
    s_reset_calib();
    s_params.dirty = 0u;
}

void app_params_reset(uint8_t scope)
{
    if ((scope == APP_PARAMS_SCOPE_PARAM) || (scope == APP_PARAMS_SCOPE_ALL))
    {
        s_reset_param();
        s_params.dirty |= APP_PARAMS_DIRTY_PARAM;
    }

    if ((scope == APP_PARAMS_SCOPE_CALIB) || (scope == APP_PARAMS_SCOPE_ALL))
    {
        s_reset_calib();
        s_params.dirty |= APP_PARAMS_DIRTY_CALIB;
    }
}

//******************************** Readers **********************************//

int32_t app_params_pid_gain_x1000(uint8_t wheel, uint8_t gain)
{
    if ((wheel >= APP_PARAMS_WHEEL_COUNT) || (gain >= APP_PARAMS_GAIN_COUNT))
    {
        return 0;
    }

    return s_params.pid[wheel][gain];
}

int32_t app_params_wheelbase_mm(void)
{
    return s_params.wheelbase_mm;
}

int32_t app_params_track_mm(void)
{
    return s_params.track_mm;
}

int32_t app_params_wheel_diameter_x10_mm(void)
{
    return s_params.wheel_diameter_x10_mm;
}

int32_t app_params_half_length_mm(void)
{
    return s_params.wheelbase_mm / 2;
}

int32_t app_params_half_width_mm(void)
{
    return s_params.track_mm / 2;
}

int32_t app_params_counts_per_mm_x1000(void)
{
    return s_params.wheel_counts_per_mm_x1000[0];
}

int32_t app_params_wheel_counts_per_mm_x1000(uint8_t wheel)
{
    if (wheel >= APP_PARAMS_WHEEL_COUNT)
    {
        return 0;
    }

    return s_params.wheel_counts_per_mm_x1000[wheel];
}

int8_t app_params_wheel_dir(uint8_t wheel)
{
    if (wheel >= APP_PARAMS_WHEEL_COUNT)
    {
        return 1;
    }

    return (s_params.wheel_dir[wheel] < 0) ? (int8_t)-1 : (int8_t)1;
}

int16_t app_params_deadzone_fwd_permille(uint8_t wheel)
{
    if (wheel >= APP_PARAMS_WHEEL_COUNT)
    {
        return 0;
    }

    return s_params.deadzone_fwd_permille[wheel];
}

int16_t app_params_deadzone_rev_permille(uint8_t wheel)
{
    if (wheel >= APP_PARAMS_WHEEL_COUNT)
    {
        return 0;
    }

    return s_params.deadzone_rev_permille[wheel];
}

uint32_t app_params_cmd_timeout_ms(void)
{
    return (uint32_t)s_params.cmd_timeout_ms;
}

int16_t app_params_max_linear_mm_s(void)
{
    return s_params.max_linear_mm_s;
}

int16_t app_params_max_angular_mrad_s(void)
{
    return s_params.max_angular_mrad_s;
}

uint8_t app_params_log_level(void)
{
    return s_params.log_level;
}

platform_bool_t app_params_log_enabled(uint8_t level)
{
    return (level >= s_params.log_level) ? PLATFORM_TRUE : PLATFORM_FALSE;
}

//******************************** Writers **********************************//

platform_err_t app_params_set_pid_gain(uint8_t wheel, uint8_t gain,
                                        int32_t value_x1000)
{
    if ((wheel >= APP_PARAMS_WHEEL_COUNT) || (gain >= APP_PARAMS_GAIN_COUNT))
    {
        return PLATFORM_ERR_PARAM;
    }

    if ((value_x1000 < 0) || (value_x1000 > s_gain_limit(gain)))
    {
        return PLATFORM_ERR_PARAM;
    }

    s_params.pid[wheel][gain] = value_x1000;
    s_params.dirty |= APP_PARAMS_DIRTY_PARAM;

    return PLATFORM_ERR_OK;
}

platform_err_t app_params_set_log_level(uint8_t level)
{
    if (level > 3u)
    {
        return PLATFORM_ERR_PARAM;
    }

    s_params.log_level = level;
    s_params.dirty |= APP_PARAMS_DIRTY_PARAM;

    return PLATFORM_ERR_OK;
}

platform_err_t app_params_set_track_mm(int32_t value)
{
    if ((value < APP_PARAMS_TRACK_MIN_MM) || (value > APP_PARAMS_TRACK_MAX_MM))
    {
        return PLATFORM_ERR_PARAM;
    }

    s_params.track_mm = value;
    s_params.dirty |= APP_PARAMS_DIRTY_CALIB;

    return PLATFORM_ERR_OK;
}

platform_err_t app_params_set_wheel_diameter_mm(int32_t value)
{
    int32_t counts;
    uint8_t wheel;

    if ((value < APP_PARAMS_DIAMETER_MIN_MM) ||
        (value > APP_PARAMS_DIAMETER_MAX_MM))
    {
        return PLATFORM_ERR_PARAM;
    }

    counts = s_counts_for_diameter(value);

    if ((counts < APP_PARAMS_COUNTS_MIN) || (counts > APP_PARAMS_COUNTS_MAX))
    {
        return PLATFORM_ERR_PARAM;
    }

    s_params.wheel_diameter_x10_mm = value * 10;

    /*
     * The diameter is the coarse scale: changing it re-derives every wheel's
     * counts_per_mm from the fixed 4680 counts/rev. A per-wheel trim done with
     * 0x50~0x53 is deliberately lost here, so the host must write the counts
     * after the diameter when it wants both.
     */
    for (wheel = 0u; wheel < APP_PARAMS_WHEEL_COUNT; wheel++)
    {
        s_params.wheel_counts_per_mm_x1000[wheel] = counts;
    }

    s_params.dirty |= APP_PARAMS_DIRTY_CALIB;

    return PLATFORM_ERR_OK;
}

platform_err_t app_params_set_wheel_counts(uint8_t wheel, int32_t value)
{
    if (wheel >= APP_PARAMS_WHEEL_COUNT)
    {
        return PLATFORM_ERR_PARAM;
    }

    if ((value < APP_PARAMS_COUNTS_MIN) || (value > APP_PARAMS_COUNTS_MAX))
    {
        return PLATFORM_ERR_PARAM;
    }

    s_params.wheel_counts_per_mm_x1000[wheel] = value;
    s_params.dirty |= APP_PARAMS_DIRTY_CALIB;

    return PLATFORM_ERR_OK;
}

platform_err_t app_params_set_wheel_dir(uint8_t wheel, int32_t value)
{
    if (wheel >= APP_PARAMS_WHEEL_COUNT)
    {
        return PLATFORM_ERR_PARAM;
    }

    if ((value != 1) && (value != -1))
    {
        return PLATFORM_ERR_PARAM;
    }

    s_params.wheel_dir[wheel] = (int8_t)value;
    s_params.dirty |= APP_PARAMS_DIRTY_CALIB;

    return PLATFORM_ERR_OK;
}

platform_err_t app_params_set_wheel_deadzone(uint8_t wheel, int32_t fwd,
                                             int32_t rev)
{
    if (wheel >= APP_PARAMS_WHEEL_COUNT)
    {
        return PLATFORM_ERR_PARAM;
    }

    if ((fwd < 0) || (rev < 0) ||
        (fwd > APP_PARAMS_DEADZONE_MAX_PERMILLE) ||
        (rev > APP_PARAMS_DEADZONE_MAX_PERMILLE))
    {
        return PLATFORM_ERR_PARAM;
    }

    s_params.deadzone_fwd_permille[wheel] = (int16_t)fwd;
    s_params.deadzone_rev_permille[wheel] = (int16_t)rev;
    s_params.dirty |= APP_PARAMS_DIRTY_CALIB;

    return PLATFORM_ERR_OK;
}

platform_err_t app_params_set_cmd_timeout_ms(int32_t value)
{
    if ((value < APP_PARAMS_TIMEOUT_MIN_MS) ||
        (value > APP_PARAMS_TIMEOUT_MAX_MS))
    {
        return PLATFORM_ERR_PARAM;
    }

    s_params.cmd_timeout_ms = value;
    s_params.dirty |= APP_PARAMS_DIRTY_PARAM;

    return PLATFORM_ERR_OK;
}

platform_err_t app_params_set_max_linear(int32_t value)
{
    if ((value < APP_PARAMS_MAX_LINEAR_MIN) ||
        (value > APP_PARAMS_MAX_LINEAR_MAX))
    {
        return PLATFORM_ERR_PARAM;
    }

    s_params.max_linear_mm_s = (int16_t)value;
    s_params.dirty |= APP_PARAMS_DIRTY_PARAM;

    return PLATFORM_ERR_OK;
}

platform_err_t app_params_set_max_angular(int32_t value)
{
    if ((value < APP_PARAMS_MAX_ANGULAR_MIN) ||
        (value > APP_PARAMS_MAX_ANGULAR_MAX))
    {
        return PLATFORM_ERR_PARAM;
    }

    s_params.max_angular_mrad_s = (int16_t)value;
    s_params.dirty |= APP_PARAMS_DIRTY_PARAM;

    return PLATFORM_ERR_OK;
}

void app_params_apply_to_chassis(void)
{
    uint8_t wheel;
    uint8_t gain;

    for (wheel = 0u; wheel < APP_PARAMS_WHEEL_COUNT; wheel++)
    {
        for (gain = 0u; gain < APP_PARAMS_GAIN_COUNT; gain++)
        {
            (void)service_chassis_set_pid_gain(
                      wheel,
                      (service_chassis_pid_gain_t)gain,
                      (float_t)s_params.pid[wheel][gain] / 1000.0f);
        }

        (void)service_chassis_set_wheel_scale(
                  wheel,
                  s_params.wheel_counts_per_mm_x1000[wheel],
                  s_params.wheel_dir[wheel]);
        (void)service_chassis_set_start_duty(
                  wheel,
                  s_params.deadzone_fwd_permille[wheel],
                  s_params.deadzone_rev_permille[wheel]);
    }

    (void)service_chassis_set_geometry(
              (int16_t)(s_params.wheelbase_mm / 2),
              (int16_t)(s_params.track_mm / 2));

    app_params_apply_limits_to_chassis();
}

void app_params_apply_limits_to_chassis(void)
{
    /* Protocol v1 has no lateral limit: max_linear bounds both vx and vy. */
    (void)service_chassis_set_limits(
              (uint16_t)s_params.cmd_timeout_ms,
              s_params.max_linear_mm_s,
              s_params.max_linear_mm_s,
              s_params.max_angular_mrad_s);
}

//****************************** Dirtiness **********************************//

platform_bool_t app_params_dirty(void)
{
    return (s_params.dirty != 0u) ? PLATFORM_TRUE : PLATFORM_FALSE;
}

void app_params_mark_dirty(uint8_t scope)
{
    if ((scope == APP_PARAMS_SCOPE_PARAM) || (scope == APP_PARAMS_SCOPE_ALL))
    {
        s_params.dirty |= APP_PARAMS_DIRTY_PARAM;
    }

    if ((scope == APP_PARAMS_SCOPE_CALIB) || (scope == APP_PARAMS_SCOPE_ALL))
    {
        s_params.dirty |= APP_PARAMS_DIRTY_CALIB;
    }
}

void app_params_mark_saved(uint8_t scope)
{
    if ((scope == APP_PARAMS_SCOPE_PARAM) || (scope == APP_PARAMS_SCOPE_ALL))
    {
        s_params.dirty &= (uint8_t)~APP_PARAMS_DIRTY_PARAM;
    }

    if ((scope == APP_PARAMS_SCOPE_CALIB) || (scope == APP_PARAMS_SCOPE_ALL))
    {
        s_params.dirty &= (uint8_t)~APP_PARAMS_DIRTY_CALIB;
    }
}

//*************************** Serialization *********************************//

uint16_t app_params_serialize(uint8_t scope,
                              uint8_t *buffer,
                              uint16_t capacity)
{
    uint16_t length;
    uint8_t wheel;
    uint8_t gain;
    uint16_t offset;

    if (buffer == NULL)
    {
        return 0u;
    }

    if (scope == APP_PARAMS_SCOPE_PARAM)
    {
        length = APP_PARAMS_BLOB_PARAM_SIZE;

        if (capacity < length)
        {
            return 0u;
        }

        s_put_u16(&buffer[APP_PARAMS_OFF_VERSION], APP_PARAMS_VERSION);
        s_put_u16(&buffer[APP_PARAMS_OFF_LENGTH], length);

        offset = APP_PARAMS_OFF_PID;

        for (wheel = 0u; wheel < APP_PARAMS_WHEEL_COUNT; wheel++)
        {
            for (gain = 0u; gain < APP_PARAMS_GAIN_COUNT; gain++)
            {
                s_put_i32(&buffer[offset], s_params.pid[wheel][gain]);
                offset = (uint16_t)(offset + 4u);
            }
        }

        buffer[APP_PARAMS_OFF_LOG_LEVEL] = s_params.log_level;
        buffer[APP_PARAMS_OFF_LOG_LEVEL + 1u] = 0u;
        s_put_u16(&buffer[APP_PARAMS_OFF_CMD_TIMEOUT],
                  (uint16_t)s_params.cmd_timeout_ms);
        s_put_i16(&buffer[APP_PARAMS_OFF_MAX_LINEAR],
                  s_params.max_linear_mm_s);
        s_put_i16(&buffer[APP_PARAMS_OFF_MAX_ANGULAR],
                  s_params.max_angular_mrad_s);

        return length;
    }

    if (scope == APP_PARAMS_SCOPE_CALIB)
    {
        length = APP_PARAMS_BLOB_CALIB_SIZE;

        if (capacity < length)
        {
            return 0u;
        }

        s_put_u16(&buffer[APP_PARAMS_OFF_VERSION], APP_PARAMS_VERSION);
        s_put_u16(&buffer[APP_PARAMS_OFF_LENGTH], length);
        s_put_i32(&buffer[APP_PARAMS_OFF_CAL_WHEELBASE],
                  s_params.wheelbase_mm);
        s_put_i32(&buffer[APP_PARAMS_OFF_CAL_TRACK], s_params.track_mm);
        s_put_i32(&buffer[APP_PARAMS_OFF_CAL_DIAMETER_X10],
                  s_params.wheel_diameter_x10_mm);

        for (wheel = 0u; wheel < APP_PARAMS_WHEEL_COUNT; wheel++)
        {
            offset = (uint16_t)(APP_PARAMS_OFF_CAL_COUNTS + (wheel * 4u));
            s_put_i32(&buffer[offset],
                      s_params.wheel_counts_per_mm_x1000[wheel]);

            offset = (uint16_t)(APP_PARAMS_OFF_CAL_DIR + (wheel * 4u));
            s_put_i32(&buffer[offset], (int32_t)s_params.wheel_dir[wheel]);

            offset = (uint16_t)(APP_PARAMS_OFF_CAL_DZ_FWD + (wheel * 4u));
            s_put_i32(&buffer[offset],
                      (int32_t)s_params.deadzone_fwd_permille[wheel]);

            offset = (uint16_t)(APP_PARAMS_OFF_CAL_DZ_REV + (wheel * 4u));
            s_put_i32(&buffer[offset],
                      (int32_t)s_params.deadzone_rev_permille[wheel]);
        }

        return length;
    }

    return 0u;
}

static platform_bool_t s_parse_param(const uint8_t *data, uint16_t length,
                                     uint16_t version)
{
    int32_t values[APP_PARAMS_WHEEL_COUNT][APP_PARAMS_GAIN_COUNT];
    int32_t timeout_ms;
    int32_t max_linear;
    int32_t max_angular;
    uint8_t log_level;
    uint8_t wheel;
    uint8_t gain;
    uint16_t offset;

    if ((version == 1u) && (length != APP_PARAMS_BLOB_PARAM_SIZE_V1))
    {
        return PLATFORM_FALSE;
    }

    if ((version == 2u) && (length != APP_PARAMS_BLOB_PARAM_SIZE))
    {
        return PLATFORM_FALSE;
    }

    offset = APP_PARAMS_OFF_PID;

    for (wheel = 0u; wheel < APP_PARAMS_WHEEL_COUNT; wheel++)
    {
        for (gain = 0u; gain < APP_PARAMS_GAIN_COUNT; gain++)
        {
            values[wheel][gain] = s_get_i32(&data[offset]);

            if ((values[wheel][gain] < 0) ||
                (values[wheel][gain] > s_gain_limit(gain)))
            {
                return PLATFORM_FALSE;
            }

            offset = (uint16_t)(offset + 4u);
        }
    }

    log_level = data[APP_PARAMS_OFF_LOG_LEVEL];

    if (log_level > 3u)
    {
        return PLATFORM_FALSE;
    }

    if (version == 2u)
    {
        timeout_ms = (int32_t)s_get_u16(&data[APP_PARAMS_OFF_CMD_TIMEOUT]);
        max_linear = (int32_t)s_get_i16(&data[APP_PARAMS_OFF_MAX_LINEAR]);
        max_angular = (int32_t)s_get_i16(&data[APP_PARAMS_OFF_MAX_ANGULAR]);

        if ((timeout_ms < APP_PARAMS_TIMEOUT_MIN_MS) ||
            (timeout_ms > APP_PARAMS_TIMEOUT_MAX_MS) ||
            (max_linear < APP_PARAMS_MAX_LINEAR_MIN) ||
            (max_linear > APP_PARAMS_MAX_LINEAR_MAX) ||
            (max_angular < APP_PARAMS_MAX_ANGULAR_MIN) ||
            (max_angular > APP_PARAMS_MAX_ANGULAR_MAX))
        {
            return PLATFORM_FALSE;
        }
    }
    else
    {
        /* Version 1 had no limit fields: keep the compile-time values. */
        timeout_ms = (int32_t)APP_CHASSIS_CMD_TIMEOUT_MS;
        max_linear = APP_CHASSIS_MAX_VX_MM_S;
        max_angular = APP_CHASSIS_MAX_WZ_MRAD_S;
    }

    /* Everything validated: commit. A bad blob must leave RAM untouched. */
    for (wheel = 0u; wheel < APP_PARAMS_WHEEL_COUNT; wheel++)
    {
        for (gain = 0u; gain < APP_PARAMS_GAIN_COUNT; gain++)
        {
            s_params.pid[wheel][gain] = values[wheel][gain];
        }
    }

    s_params.log_level = log_level;
    s_params.cmd_timeout_ms = timeout_ms;
    s_params.max_linear_mm_s = (int16_t)max_linear;
    s_params.max_angular_mrad_s = (int16_t)max_angular;

    return PLATFORM_TRUE;
}

static platform_bool_t s_parse_calib(const uint8_t *data, uint16_t length,
                                     uint16_t version)
{
    int32_t wheelbase;
    int32_t track;
    int32_t diameter_x10;
    int32_t counts[APP_PARAMS_WHEEL_COUNT];
    int32_t deadzone_fwd[APP_PARAMS_WHEEL_COUNT];
    int32_t deadzone_rev[APP_PARAMS_WHEEL_COUNT];
    int8_t dir[APP_PARAMS_WHEEL_COUNT];
    uint8_t wheel;
    uint16_t offset;

    if ((version == 1u) && (length != APP_PARAMS_BLOB_CALIB_SIZE_V1))
    {
        return PLATFORM_FALSE;
    }

    if ((version == 2u) && (length != APP_PARAMS_BLOB_CALIB_SIZE))
    {
        return PLATFORM_FALSE;
    }

    if (version == 1u)
    {
        int32_t half_length =
            s_get_i32(&data[APP_PARAMS_OFF_V1_HALF_LENGTH]);
        int32_t half_width =
            s_get_i32(&data[APP_PARAMS_OFF_V1_HALF_WIDTH]);

        if ((half_length <= 0) || (half_length > APP_PARAMS_HALF_MAX_MM) ||
            (half_width <= 0) || (half_width > APP_PARAMS_HALF_MAX_MM))
        {
            return PLATFORM_FALSE;
        }

        wheelbase = half_length * 2;
        track = half_width * 2;
        diameter_x10 = (int32_t)APP_CHASSIS_WHEEL_DIAMETER_MM * 10;

        for (wheel = 0u; wheel < APP_PARAMS_WHEEL_COUNT; wheel++)
        {
            counts[wheel] = s_get_i32(&data[APP_PARAMS_OFF_V1_COUNTS]);

            /* Version 1 allowed any positive scale; keep that contract. */
            if ((counts[wheel] <= 0) ||
                (counts[wheel] > APP_PARAMS_COUNTS_MAX))
            {
                return PLATFORM_FALSE;
            }

            dir[wheel] = 1;
            deadzone_fwd[wheel] = s_default_deadzone_fwd[wheel];
            deadzone_rev[wheel] = s_default_deadzone_rev[wheel];
        }
    }
    else
    {
        wheelbase = s_get_i32(&data[APP_PARAMS_OFF_CAL_WHEELBASE]);
        track = s_get_i32(&data[APP_PARAMS_OFF_CAL_TRACK]);
        diameter_x10 = s_get_i32(&data[APP_PARAMS_OFF_CAL_DIAMETER_X10]);

        if ((wheelbase < APP_PARAMS_WHEELBASE_MIN_MM) ||
            (wheelbase > APP_PARAMS_WHEELBASE_MAX_MM) ||
            (track < APP_PARAMS_TRACK_MIN_MM) ||
            (track > APP_PARAMS_TRACK_MAX_MM) ||
            (diameter_x10 < (APP_PARAMS_DIAMETER_MIN_MM * 10)) ||
            (diameter_x10 > (APP_PARAMS_DIAMETER_MAX_MM * 10)))
        {
            return PLATFORM_FALSE;
        }

        for (wheel = 0u; wheel < APP_PARAMS_WHEEL_COUNT; wheel++)
        {
            offset = (uint16_t)(APP_PARAMS_OFF_CAL_COUNTS + (wheel * 4u));
            counts[wheel] = s_get_i32(&data[offset]);

            offset = (uint16_t)(APP_PARAMS_OFF_CAL_DIR + (wheel * 4u));
            dir[wheel] = (int8_t)s_get_i32(&data[offset]);

            offset = (uint16_t)(APP_PARAMS_OFF_CAL_DZ_FWD + (wheel * 4u));
            deadzone_fwd[wheel] = s_get_i32(&data[offset]);

            offset = (uint16_t)(APP_PARAMS_OFF_CAL_DZ_REV + (wheel * 4u));
            deadzone_rev[wheel] = s_get_i32(&data[offset]);

            if ((counts[wheel] < APP_PARAMS_COUNTS_MIN) ||
                (counts[wheel] > APP_PARAMS_COUNTS_MAX) ||
                ((dir[wheel] != 1) && (dir[wheel] != -1)) ||
                (deadzone_fwd[wheel] < 0) ||
                (deadzone_fwd[wheel] > APP_PARAMS_DEADZONE_MAX_PERMILLE) ||
                (deadzone_rev[wheel] < 0) ||
                (deadzone_rev[wheel] > APP_PARAMS_DEADZONE_MAX_PERMILLE))
            {
                return PLATFORM_FALSE;
            }
        }
    }

    s_params.wheelbase_mm = wheelbase;
    s_params.track_mm = track;
    s_params.wheel_diameter_x10_mm = diameter_x10;

    for (wheel = 0u; wheel < APP_PARAMS_WHEEL_COUNT; wheel++)
    {
        s_params.wheel_counts_per_mm_x1000[wheel] = counts[wheel];
        s_params.wheel_dir[wheel] = dir[wheel];
        s_params.deadzone_fwd_permille[wheel] = (int16_t)deadzone_fwd[wheel];
        s_params.deadzone_rev_permille[wheel] = (int16_t)deadzone_rev[wheel];
    }

    return PLATFORM_TRUE;
}

platform_bool_t app_params_parse(uint8_t scope,
                                 const uint8_t *data,
                                 uint16_t length)
{
    uint16_t version;

    if ((data == NULL) || (length < 4u))
    {
        return PLATFORM_FALSE;
    }

    version = s_get_u16(&data[APP_PARAMS_OFF_VERSION]);

    if ((version != 1u) && (version != APP_PARAMS_VERSION))
    {
        return PLATFORM_FALSE;
    }

    if (s_get_u16(&data[APP_PARAMS_OFF_LENGTH]) != length)
    {
        return PLATFORM_FALSE;
    }

    if (scope == APP_PARAMS_SCOPE_PARAM)
    {
        return s_parse_param(data, length, version);
    }

    if (scope == APP_PARAMS_SCOPE_CALIB)
    {
        return s_parse_calib(data, length, version);
    }

    return PLATFORM_FALSE;
}
