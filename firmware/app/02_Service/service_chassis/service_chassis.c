/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file service_chassis.c
 *
 * @par dependencies
 * - service_chassis.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement mecanum kinematics, four speed loops, the watchdog and
 *        dead reckoning.
 *
 * Kinematics (x forward, y left, w CCW):
 *
 *   FL = vx - vy - k*w      FR = vx + vy + k*w
 *   RL = vx + vy - k*w      RR = vx - vy + k*w
 *
 * with k = half_length + half_width (mm) and w in mrad/s, so k*w/1000 is a
 * wheel speed in mm/s and the whole calculation stays integral.
 *
 * Each wheel has its own PID (platform_middleware/platform_pid). Encoder
 * counts are sampled once per control tick and divided by the measured dt to
 * obtain that wheel's speed; the PID output is the signed duty itself.
 *
 * The 10 ms odometry uses float internally: four divisions per tick are cheap
 * on the M3, and integer accumulation of sub-millimetre distances would lose
 * the very motion we are trying to integrate. The protocol-facing values are
 * still integers.
 *
 * @version V1.1 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "service_chassis.h"
#include "platform_pid.h"

#include <math.h>

//******************************** Defines **********************************//

#define SERVICE_CHASSIS_PI          (3.14159265358979323846f)
#define SERVICE_CHASSIS_TWO_PI      (2.0f * SERVICE_CHASSIS_PI)
#define SERVICE_CHASSIS_RAD_TO_CD   (18000.0f / SERVICE_CHASSIS_PI)

/*
 * At rest with a zero target, discard the integral instead of letting it hold
 * a small reverse duty. 5 mm/s is below one encoder count per 10 ms (about
 * 4 mm/s), so this only triggers when the wheel is genuinely stopped.
 */
#define SERVICE_CHASSIS_ZERO_SPEED_EPS_MM_S ((float_t)5.0f)

/*
 * Sanity bound for a runtime gain. The application has a tighter policy limit
 * (10.0 for Kp/Ki, 5.0 for Kd); this only keeps NaN/Inf out of the loop.
 */
#define SERVICE_CHASSIS_PID_GAIN_LIMIT ((float_t)1000.0f)

//******************************** Variables ********************************//

static struct
{
    platform_bool_t             initialized;
    service_chassis_cfg_t       cfg;

    platform_bool_t             enabled;
    platform_bool_t             estop;
    platform_bool_t             timed_out;

    int16_t                     target_vx_mm_s;
    int16_t                     target_vy_mm_s;
    int16_t                     target_wz_mrad_s;
    uint32_t                    last_cmd_ms;

    /*
     * The platform hands out the cumulative encoder count. Integration needs
     * the step since the last sample, so keep the previous reading here.
     * Odometry and the speed loops sample independently: both only read the
     * cumulative value, so neither consumes the other's delta.
     */
    int32_t                     prev_counts[SERVICE_CHASSIS_WHEEL_COUNT];

    /*
     * Speed-loop state. apply() owns these fields; the enable/estop setters
     * only flip the flags above, and apply() notices the transition.
     */
    platform_pid_t              pid[SERVICE_CHASSIS_WHEEL_COUNT];
    int32_t                     pid_prev_counts[SERVICE_CHASSIS_WHEEL_COUNT];
    int16_t                     measured_wheel_mm_s[SERVICE_CHASSIS_WHEEL_COUNT];
    int16_t                     applied_duty[SERVICE_CHASSIS_WHEEL_COUNT];
    platform_bool_t             closed_loop;
    platform_bool_t             pid_was_enabled;
    platform_bool_t             pid_was_estopped;
    platform_bool_t             pid_was_timed_out;

    /*
     * Runtime calibration, written by the control task only. A zero scale
     * means "use cfg.counts_per_mm_x1000"; direction is +1 or -1 and is
     * applied to the count delta, not to the commanded speed.
     */
    int32_t                     wheel_counts_per_mm_x1000
                                    [SERVICE_CHASSIS_WHEEL_COUNT];
    int8_t                      wheel_dir[SERVICE_CHASSIS_WHEEL_COUNT];
    int16_t                     start_duty_fwd_permille
                                    [SERVICE_CHASSIS_WHEEL_COUNT];
    int16_t                     start_duty_rev_permille
                                    [SERVICE_CHASSIS_WHEEL_COUNT];

    /*
     * Bench open-loop test. test_active is the switch; the per-wheel values
     * and timestamps are written by the comm task (through the control task's
     * queue) and consumed by apply(). apply() notices the switch transition
     * and resets the loops, so the PID state is never touched off-thread.
     */
    platform_bool_t             test_active;
    platform_bool_t             pid_was_test_active;
    int16_t                     test_duty[SERVICE_CHASSIS_WHEEL_COUNT];
    uint32_t                    test_duty_ms[SERVICE_CHASSIS_WHEEL_COUNT];

    int32_t                     x_mm;
    int32_t                     y_mm;
    float                       heading_rad;
    int16_t                     vl_mm_s;
    int16_t                     vr_mm_s;
} s_chassis;

//******************************** Functions ********************************//

static int16_t service_chassis_clamp_i16(int32_t value)
{
    if (value > 32767)
    {
        return (int16_t)32767;
    }

    if (value < -32768)
    {
        return (int16_t)-32768;
    }

    return (int16_t)value;
}

/* Round half away from zero; a plain cast truncates sub-millimetre motion. */
static int32_t service_chassis_round(float_t value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value + 0.5f);
    }

    return (int32_t)(value - 0.5f);
}

static int16_t service_chassis_clamp_speed(int32_t value, int16_t limit)
{
    if (limit <= 0)
    {
        return (int16_t)0;
    }

    if (value > (int32_t)limit)
    {
        return limit;
    }

    if (value < -(int32_t)limit)
    {
        return (int16_t)(-(int32_t)limit);
    }

    return (int16_t)value;
}

/* Round half away from zero; a truncating cast would add a small dead band. */
static int16_t service_chassis_pid_duty(float_t duty)
{
    if (duty > (float_t)PLATFORM_MOTOR_DUTY_PERMILLE_MAX)
    {
        duty = (float_t)PLATFORM_MOTOR_DUTY_PERMILLE_MAX;
    }
    else if (duty < (float_t)PLATFORM_MOTOR_DUTY_PERMILLE_MIN)
    {
        duty = (float_t)PLATFORM_MOTOR_DUTY_PERMILLE_MIN;
    }
    else
    {
        /* Within range. */
    }

    return (int16_t)service_chassis_round(duty);
}

/*
 * Clear every speed loop and re-read the encoders as the new zero step.
 * Called from apply() and init only, so it cannot race with a PID update.
 */
static void service_chassis_reset_pid_state(void)
{
    uint8_t i;

    for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
    {
        platform_pid_reset(&s_chassis.pid[i]);
        s_chassis.measured_wheel_mm_s[i] = 0;
        s_chassis.pid_prev_counts[i] = 0;
        (void)platform_motor_get_encoder(s_chassis.cfg.motor[i],
                                         &s_chassis.pid_prev_counts[i]);
    }
}

/* Effective encoder scale: the per-wheel value, else the common config one. */
static int32_t service_chassis_wheel_counts(uint8_t wheel)
{
    int32_t counts = s_chassis.wheel_counts_per_mm_x1000[wheel];

    return (counts > 0) ? counts : s_chassis.cfg.counts_per_mm_x1000;
}

/* Millimetres per encoder count, or 0 when this wheel has no scale. */
static float_t service_chassis_wheel_counts_to_mm(uint8_t wheel)
{
    int32_t counts = service_chassis_wheel_counts(wheel);

    return (counts > 0) ? (1000.0f / (float_t)counts) : 0.0f;
}

static int8_t service_chassis_wheel_dir(uint8_t wheel)
{
    return (s_chassis.wheel_dir[wheel] < 0) ? (int8_t)-1 : (int8_t)1;
}

/*
 * Closed loop needs a usable scale on every wheel: the kinematics mix all
 * four, so one missing scale would make three wheels right and one wrong.
 */
static void service_chassis_refresh_closed_loop(void)
{
    uint8_t i;

    s_chassis.closed_loop = PLATFORM_TRUE;

    for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
    {
        if (service_chassis_wheel_counts(i) <= 0)
        {
            s_chassis.closed_loop = PLATFORM_FALSE;
        }
    }
}

/*
 * Sample one encoder and turn the count step into mm/s. Returns the float
 * speed the loop uses; the rounded value goes out in REPORT_WHEEL_SPEED.
 * The per-wheel direction is applied here, so a mirrored motor reports a
 * positive speed for a positive command.
 */
static float_t service_chassis_speed_mm_s(uint8_t wheel, float_t dt_s,
                                          float_t counts_to_mm)
{
    int32_t now_counts = s_chassis.pid_prev_counts[wheel];
    int32_t delta_counts;
    float_t measured_mm_s;

    /*
     * A failed read contributes no motion rather than a fresh baseline,
     * matching the odometry path.
     */
    (void)platform_motor_get_encoder(s_chassis.cfg.motor[wheel],
                                     &now_counts);

    delta_counts = (int32_t)((uint32_t)now_counts -
                             (uint32_t)s_chassis.pid_prev_counts[wheel]);
    s_chassis.pid_prev_counts[wheel] = now_counts;

    measured_mm_s = ((float_t)delta_counts * counts_to_mm *
                     (float_t)service_chassis_wheel_dir(wheel)) / dt_s;
    s_chassis.measured_wheel_mm_s[wheel] =
        service_chassis_clamp_i16(service_chassis_round(measured_mm_s));

    return measured_mm_s;
}

static void service_chassis_zero_duties(void)
{
    uint8_t i;

    for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
    {
        s_chassis.applied_duty[i] = 0;
        (void)platform_motor_set_duty(s_chassis.cfg.motor[i], 0);
    }
}

static int16_t service_chassis_duty(int16_t wheel_mm_s, int16_t max_mm_s)
{
    int32_t duty;

    if (max_mm_s <= 0)
    {
        return (int16_t)0;
    }

    duty = ((int32_t)wheel_mm_s * (int32_t)PLATFORM_MOTOR_DUTY_PERMILLE_MAX)
           / (int32_t)max_mm_s;

    if (duty > (int32_t)PLATFORM_MOTOR_DUTY_PERMILLE_MAX)
    {
        duty = (int32_t)PLATFORM_MOTOR_DUTY_PERMILLE_MAX;
    }
    else if (duty < (int32_t)PLATFORM_MOTOR_DUTY_PERMILLE_MIN)
    {
        duty = (int32_t)PLATFORM_MOTOR_DUTY_PERMILLE_MIN;
    }
    else
    {
        /* Within range. */
    }

    return (int16_t)duty;
}

void service_chassis_inverse_kinematics(int16_t vx_mm_s,
                                        int16_t vy_mm_s,
                                        int16_t wz_mrad_s,
                                        int16_t k_mm,
                                        int16_t wheel_mm_s
                                            [SERVICE_CHASSIS_WHEEL_COUNT])
{
    int32_t turn = ((int32_t)k_mm * (int32_t)wz_mrad_s) / 1000;
    int32_t vx = (int32_t)vx_mm_s;
    int32_t vy = (int32_t)vy_mm_s;

    if (wheel_mm_s == NULL)
    {
        return;
    }

    wheel_mm_s[SERVICE_CHASSIS_WHEEL_FL] =
        service_chassis_clamp_i16(vx - vy - turn);
    wheel_mm_s[SERVICE_CHASSIS_WHEEL_FR] =
        service_chassis_clamp_i16(vx + vy + turn);
    wheel_mm_s[SERVICE_CHASSIS_WHEEL_RL] =
        service_chassis_clamp_i16(vx + vy - turn);
    wheel_mm_s[SERVICE_CHASSIS_WHEEL_RR] =
        service_chassis_clamp_i16(vx - vy + turn);
}

int32_t service_chassis_counts_per_mm_x1000(int32_t counts_per_rev,
                                            int32_t diameter_mm)
{
    if (diameter_mm == 0)
    {
        return 0;
    }

    return service_chassis_round(((float_t)counts_per_rev * 1000.0f) /
                                 (SERVICE_CHASSIS_PI * (float_t)diameter_mm));
}

platform_err_t service_chassis_init(const service_chassis_cfg_t *cfg)
{
    platform_pid_cfg_t pid_cfg;
    uint8_t i;

    if (cfg == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if ((cfg->half_length_mm <= 0) || (cfg->half_width_mm <= 0) ||
        (cfg->max_vx_mm_s <= 0) || (cfg->max_vy_mm_s <= 0) ||
        (cfg->max_wz_mrad_s <= 0) || (cfg->max_wheel_mm_s <= 0) ||
        (cfg->counts_per_mm_x1000 < 0) || (cfg->cmd_timeout_ms == 0u))
    {
        return PLATFORM_ERR_PARAM;
    }

    for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
    {
        if ((cfg->motor[i] >= PLATFORM_MOTOR_ID_MAX) ||
            (cfg->pid_kp[i] < 0.0f) || (cfg->pid_ki[i] < 0.0f) ||
            (cfg->pid_kd[i] < 0.0f))
        {
            return PLATFORM_ERR_PARAM;
        }
    }

    s_chassis.cfg = *cfg;
    s_chassis.enabled = PLATFORM_FALSE;
    s_chassis.estop = PLATFORM_FALSE;
    s_chassis.timed_out = PLATFORM_FALSE;
    s_chassis.target_vx_mm_s = 0;
    s_chassis.target_vy_mm_s = 0;
    s_chassis.target_wz_mrad_s = 0;
    s_chassis.last_cmd_ms = 0u;
    s_chassis.x_mm = 0;
    s_chassis.y_mm = 0;
    s_chassis.heading_rad = 0.0f;
    s_chassis.vl_mm_s = 0;
    s_chassis.vr_mm_s = 0;
    s_chassis.pid_was_enabled = PLATFORM_FALSE;
    s_chassis.pid_was_estopped = PLATFORM_FALSE;
    s_chassis.pid_was_timed_out = PLATFORM_FALSE;
    s_chassis.test_active = PLATFORM_FALSE;
    s_chassis.pid_was_test_active = PLATFORM_FALSE;

    for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
    {
        s_chassis.applied_duty[i] = 0;
        s_chassis.test_duty[i] = 0;
        s_chassis.test_duty_ms[i] = 0u;

        /* Until the application pushes a calibration, the common scale and
         * the unmirrored direction are what the config asked for. */
        s_chassis.wheel_counts_per_mm_x1000[i] = 0;
        s_chassis.wheel_dir[i] = 1;
        s_chassis.start_duty_fwd_permille[i] = 0;
        s_chassis.start_duty_rev_permille[i] = 0;
    }

    service_chassis_refresh_closed_loop();

    /*
     * The PID output is the signed duty itself, so its limits are the motor
     * limits. Per-wheel gains let a later calibration trim one wheel without
     * changing the other three.
     */
    pid_cfg.output_min = (float_t)PLATFORM_MOTOR_DUTY_PERMILLE_MIN;
    pid_cfg.output_max = (float_t)PLATFORM_MOTOR_DUTY_PERMILLE_MAX;

    for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
    {
        pid_cfg.kp = cfg->pid_kp[i];
        pid_cfg.ki = cfg->pid_ki[i];
        pid_cfg.kd = cfg->pid_kd[i];

        if (platform_pid_init(&s_chassis.pid[i], &pid_cfg) != PLATFORM_ERR_OK)
        {
            return PLATFORM_ERR_PARAM;
        }
    }

    /* Prime the odometry baseline so the first step is not the whole count. */
    for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
    {
        s_chassis.prev_counts[i] = 0;
        (void)platform_motor_get_encoder(cfg->motor[i],
                                         &s_chassis.prev_counts[i]);
    }

    service_chassis_reset_pid_state();

    s_chassis.initialized = PLATFORM_TRUE;

    (void)platform_motor_stop_all();

    return PLATFORM_ERR_OK;
}

platform_err_t service_chassis_set_pid_gain(uint8_t wheel,
                                            service_chassis_pid_gain_t gain,
                                            float_t value)
{
    platform_pid_cfg_t pid_cfg;

    if (s_chassis.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (wheel >= SERVICE_CHASSIS_WHEEL_COUNT)
    {
        return PLATFORM_ERR_PARAM;
    }

    if ((gain != SERVICE_CHASSIS_PID_KP) &&
        (gain != SERVICE_CHASSIS_PID_KI) &&
        (gain != SERVICE_CHASSIS_PID_KD))
    {
        return PLATFORM_ERR_PARAM;
    }

    /* The inverted comparison also rejects NaN. */
    if ((!(value >= 0.0f)) || (value > SERVICE_CHASSIS_PID_GAIN_LIMIT))
    {
        return PLATFORM_ERR_PARAM;
    }

    pid_cfg = s_chassis.pid[wheel].cfg;

    if (gain == SERVICE_CHASSIS_PID_KP)
    {
        pid_cfg.kp = value;
    }
    else if (gain == SERVICE_CHASSIS_PID_KI)
    {
        pid_cfg.ki = value;
    }
    else
    {
        pid_cfg.kd = value;
    }

    if (platform_pid_init(&s_chassis.pid[wheel], &pid_cfg) != PLATFORM_ERR_OK)
    {
        return PLATFORM_ERR_PARAM;
    }

    /* Commit the config only after the PID accepted the new gains. */
    if (gain == SERVICE_CHASSIS_PID_KP)
    {
        s_chassis.cfg.pid_kp[wheel] = value;
    }
    else if (gain == SERVICE_CHASSIS_PID_KI)
    {
        s_chassis.cfg.pid_ki[wheel] = value;
    }
    else
    {
        s_chassis.cfg.pid_kd[wheel] = value;
    }

    /*
     * A new gain starts from a clean loop: discard the integral and use the
     * current encoder count as the new baseline, exactly like a timeout
     * recovery. Otherwise the first tick after tuning sees a huge step.
     */
    s_chassis.measured_wheel_mm_s[wheel] = 0;
    s_chassis.pid_prev_counts[wheel] = 0;
    (void)platform_motor_get_encoder(s_chassis.cfg.motor[wheel],
                                     &s_chassis.pid_prev_counts[wheel]);

    return PLATFORM_ERR_OK;
}

platform_err_t service_chassis_get_pid_gain(uint8_t wheel,
                                            service_chassis_pid_gain_t gain,
                                            float_t *value)
{
    if ((s_chassis.initialized == PLATFORM_FALSE) || (value == NULL) ||
        (wheel >= SERVICE_CHASSIS_WHEEL_COUNT))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (gain == SERVICE_CHASSIS_PID_KP)
    {
        *value = s_chassis.pid[wheel].cfg.kp;
    }
    else if (gain == SERVICE_CHASSIS_PID_KI)
    {
        *value = s_chassis.pid[wheel].cfg.ki;
    }
    else if (gain == SERVICE_CHASSIS_PID_KD)
    {
        *value = s_chassis.pid[wheel].cfg.kd;
    }
    else
    {
        return PLATFORM_ERR_PARAM;
    }

    return PLATFORM_ERR_OK;
}

platform_err_t service_chassis_set_geometry(int16_t half_length_mm,
                                            int16_t half_width_mm)
{
    if (s_chassis.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if ((half_length_mm <= 0) || (half_width_mm <= 0) ||
        (half_length_mm > SERVICE_CHASSIS_GEOMETRY_MAX_MM) ||
        (half_width_mm > SERVICE_CHASSIS_GEOMETRY_MAX_MM))
    {
        return PLATFORM_ERR_PARAM;
    }

    /*
     * The next tick's kinematics and yaw term read these; both are 16-bit
     * stores, so a tick that starts in between sees one consistent value or
     * the other, never a torn one. The application keeps the chassis disabled
     * for this reason in any case.
     */
    s_chassis.cfg.half_length_mm = half_length_mm;
    s_chassis.cfg.half_width_mm = half_width_mm;

    return PLATFORM_ERR_OK;
}

platform_err_t service_chassis_set_wheel_scale(uint8_t wheel,
                                               int32_t counts_per_mm_x1000,
                                               int8_t dir)
{
    platform_bool_t was_closed_loop;

    if (s_chassis.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if ((wheel >= SERVICE_CHASSIS_WHEEL_COUNT) ||
        (counts_per_mm_x1000 < 0) ||
        ((dir != (int8_t)1) && (dir != (int8_t)-1)))
    {
        return PLATFORM_ERR_PARAM;
    }

    was_closed_loop = s_chassis.closed_loop;

    s_chassis.wheel_counts_per_mm_x1000[wheel] = counts_per_mm_x1000;
    s_chassis.wheel_dir[wheel] = dir;

    service_chassis_refresh_closed_loop();

    /*
     * A direction flip would otherwise present the next count step as one
     * huge negative speed. Re-baseline this wheel and, if the loop mode
     * changed at all, start every loop from clean state.
     */
    if (was_closed_loop != s_chassis.closed_loop)
    {
        service_chassis_reset_pid_state();
    }
    else
    {
        s_chassis.measured_wheel_mm_s[wheel] = 0;
        s_chassis.pid_prev_counts[wheel] = 0;
        (void)platform_motor_get_encoder(s_chassis.cfg.motor[wheel],
                                         &s_chassis.pid_prev_counts[wheel]);

        /*
         * Re-baseline the odometry too. A scale change is a change of ruler:
         * counts accumulated under the old value must not be re-measured with
         * the new one. The chassis is supposed to be disabled here, but the
         * encoder is still sampling until the next tick.
         */
        s_chassis.prev_counts[wheel] = s_chassis.pid_prev_counts[wheel];
    }

    return PLATFORM_ERR_OK;
}

platform_err_t service_chassis_set_start_duty(uint8_t wheel,
                                              int16_t fwd_permille,
                                              int16_t rev_permille)
{
    if (s_chassis.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if ((wheel >= SERVICE_CHASSIS_WHEEL_COUNT) ||
        (fwd_permille < 0) || (rev_permille < 0) ||
        (fwd_permille > SERVICE_CHASSIS_START_DUTY_MAX) ||
        (rev_permille > SERVICE_CHASSIS_START_DUTY_MAX))
    {
        return PLATFORM_ERR_PARAM;
    }

    s_chassis.start_duty_fwd_permille[wheel] = fwd_permille;
    s_chassis.start_duty_rev_permille[wheel] = rev_permille;

    return PLATFORM_ERR_OK;
}

platform_err_t service_chassis_set_limits(uint16_t cmd_timeout_ms,
                                          int16_t max_vx_mm_s,
                                          int16_t max_vy_mm_s,
                                          int16_t max_wz_mrad_s)
{
    if (s_chassis.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if ((cmd_timeout_ms == 0u) || (max_vx_mm_s <= 0) ||
        (max_vy_mm_s <= 0) || (max_wz_mrad_s <= 0))
    {
        return PLATFORM_ERR_PARAM;
    }

    s_chassis.cfg.cmd_timeout_ms = cmd_timeout_ms;
    s_chassis.cfg.max_vx_mm_s = max_vx_mm_s;
    s_chassis.cfg.max_vy_mm_s = max_vy_mm_s;
    s_chassis.cfg.max_wz_mrad_s = max_wz_mrad_s;

    return PLATFORM_ERR_OK;
}

/**
 * @brief Enable or coast the four H-bridges.
 *
 * Logical enable alone is not enough: the BSP keeps the bridges coasting and
 * only latches the duty until set_enable(TRUE), so a chassis that forgot this
 * call reports "enabled" while the wheels never turn.
 */
static void service_chassis_set_bridges(platform_bool_t enable)
{
    uint8_t i;

    for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
    {
        (void)platform_motor_set_enable(s_chassis.cfg.motor[i], enable);
    }
}

platform_err_t service_chassis_enable(platform_bool_t enable,
                                      uint32_t now_ms)
{
    if (s_chassis.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (enable == PLATFORM_TRUE)
    {
        if (s_chassis.estop == PLATFORM_TRUE)
        {
            return PLATFORM_ERR_FAIL;
        }

        s_chassis.enabled = PLATFORM_TRUE;
        s_chassis.timed_out = PLATFORM_FALSE;
        s_chassis.target_vx_mm_s = 0;
        s_chassis.target_vy_mm_s = 0;
        s_chassis.target_wz_mrad_s = 0;

        /* Give the host a full timeout window to send the next velocity. */
        s_chassis.last_cmd_ms = now_ms;

        service_chassis_set_bridges(PLATFORM_TRUE);
    }
    else
    {
        s_chassis.enabled = PLATFORM_FALSE;
        s_chassis.test_active = PLATFORM_FALSE;
        s_chassis.target_vx_mm_s = 0;
        s_chassis.target_vy_mm_s = 0;
        s_chassis.target_wz_mrad_s = 0;

        /* Zero the compare first, then let the bridge coast. */
        (void)platform_motor_stop_all();
        service_chassis_set_bridges(PLATFORM_FALSE);
        service_chassis_zero_duties();
    }

    return PLATFORM_ERR_OK;
}

platform_bool_t service_chassis_is_enabled(void)
{
    return s_chassis.enabled;
}

platform_bool_t service_chassis_is_estopped(void)
{
    return s_chassis.estop;
}

platform_err_t service_chassis_set_velocity(int16_t vx_mm_s,
                                            int16_t vy_mm_s,
                                            int16_t wz_mrad_s,
                                            uint32_t now_ms)
{
    int32_t abs_vx;
    int32_t abs_vy;
    int32_t abs_wz;

    if (s_chassis.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    abs_vx = (vx_mm_s < 0) ? -(int32_t)vx_mm_s : (int32_t)vx_mm_s;
    abs_vy = (vy_mm_s < 0) ? -(int32_t)vy_mm_s : (int32_t)vy_mm_s;
    abs_wz = (wz_mrad_s < 0) ? -(int32_t)wz_mrad_s : (int32_t)wz_mrad_s;

    if ((abs_vx > (int32_t)s_chassis.cfg.max_vx_mm_s) ||
        (abs_vy > (int32_t)s_chassis.cfg.max_vy_mm_s) ||
        (abs_wz > (int32_t)s_chassis.cfg.max_wz_mrad_s))
    {
        return PLATFORM_ERR_PARAM;
    }

    /*
     * Keep the watchdog alive even while disabled: the host is clearly
     * talking, so a later enable must not inherit a stale timeout.
     */
    s_chassis.last_cmd_ms = now_ms;
    s_chassis.timed_out = PLATFORM_FALSE;

    /* Normal velocity control always wins over a bench open-loop test. */
    s_chassis.test_active = PLATFORM_FALSE;

    if ((s_chassis.enabled == PLATFORM_TRUE) &&
        (s_chassis.estop == PLATFORM_FALSE))
    {
        s_chassis.target_vx_mm_s = vx_mm_s;
        s_chassis.target_vy_mm_s = vy_mm_s;
        s_chassis.target_wz_mrad_s = wz_mrad_s;
    }
    else
    {
        s_chassis.target_vx_mm_s = 0;
        s_chassis.target_vy_mm_s = 0;
        s_chassis.target_wz_mrad_s = 0;
    }

    return PLATFORM_ERR_OK;
}

platform_err_t service_chassis_forward(uint16_t speed_mm_s, uint32_t now_ms)
{
    if (speed_mm_s > (uint16_t)32767u)
    {
        return PLATFORM_ERR_PARAM;
    }

    return service_chassis_set_velocity((int16_t)speed_mm_s, 0, 0, now_ms);
}

platform_err_t service_chassis_backward(uint16_t speed_mm_s, uint32_t now_ms)
{
    if (speed_mm_s > (uint16_t)32767u)
    {
        return PLATFORM_ERR_PARAM;
    }

    return service_chassis_set_velocity((int16_t)(-(int32_t)speed_mm_s),
                                        0, 0, now_ms);
}

platform_err_t service_chassis_strafe(int16_t speed_mm_s, uint32_t now_ms)
{
    return service_chassis_set_velocity(0, speed_mm_s, 0, now_ms);
}

platform_err_t service_chassis_rotate(int16_t wz_mrad_s, uint32_t now_ms)
{
    return service_chassis_set_velocity(0, 0, wz_mrad_s, now_ms);
}

void service_chassis_stop(void)
{
    s_chassis.test_active = PLATFORM_FALSE;
    s_chassis.target_vx_mm_s = 0;
    s_chassis.target_vy_mm_s = 0;
    s_chassis.target_wz_mrad_s = 0;
}

void service_chassis_emergency_stop(void)
{
    s_chassis.estop = PLATFORM_TRUE;
    s_chassis.enabled = PLATFORM_FALSE;
    s_chassis.test_active = PLATFORM_FALSE;
    s_chassis.timed_out = PLATFORM_FALSE;
    s_chassis.target_vx_mm_s = 0;
    s_chassis.target_vy_mm_s = 0;
    s_chassis.target_wz_mrad_s = 0;

    (void)platform_motor_stop_all();
    service_chassis_set_bridges(PLATFORM_FALSE);
    service_chassis_zero_duties();
}

platform_err_t service_chassis_clear_estop(void)
{
    if (s_chassis.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    /*
     * Clearing releases the latch but deliberately does NOT re-enable the
     * drivers: the host must send SET_ENABLE again after a fault.
     */
    s_chassis.estop = PLATFORM_FALSE;
    s_chassis.timed_out = PLATFORM_FALSE;
    s_chassis.target_vx_mm_s = 0;
    s_chassis.target_vy_mm_s = 0;
    s_chassis.target_wz_mrad_s = 0;

    return PLATFORM_ERR_OK;
}

platform_bool_t service_chassis_apply(uint32_t now_ms, uint32_t dt_ms)
{
    int16_t  wheel_target[SERVICE_CHASSIS_WHEEL_COUNT];
    int16_t  vx;
    int16_t  vy;
    int16_t  wz;
    uint32_t age;
    platform_bool_t timed_out_now = PLATFORM_FALSE;
    float_t  dt_s;
    uint8_t  i;

    if (s_chassis.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_FALSE;
    }

    /*
     * The control task owns the PID state. Reset it here on an enable/estop
     * transition instead of doing it inside the setters, which run in the
     * communication task and would race with a multi-byte float update.
     */
    if ((s_chassis.enabled != s_chassis.pid_was_enabled) ||
        (s_chassis.estop != s_chassis.pid_was_estopped) ||
        (s_chassis.test_active != s_chassis.pid_was_test_active))
    {
        s_chassis.pid_was_enabled = s_chassis.enabled;
        s_chassis.pid_was_estopped = s_chassis.estop;
        s_chassis.pid_was_test_active = s_chassis.test_active;
        service_chassis_reset_pid_state();
    }

    if ((s_chassis.enabled == PLATFORM_FALSE) ||
        (s_chassis.estop == PLATFORM_TRUE))
    {
        s_chassis.timed_out = PLATFORM_FALSE;
        s_chassis.pid_was_timed_out = PLATFORM_FALSE;
        return PLATFORM_FALSE;
    }

    if (dt_ms == 0u)
    {
        dt_ms = 1u;
    }

    /*
     * Bench open loop. The command watchdog does not apply here: the test
     * duty carries its own per-wheel expiry, so a crashed host coasts the
     * wheels within SERVICE_CHASSIS_TEST_DUTY_TIMEOUT_MS.
     */
    if (s_chassis.test_active == PLATFORM_TRUE)
    {
        platform_bool_t all_stale = PLATFORM_TRUE;

        s_chassis.timed_out = PLATFORM_FALSE;
        s_chassis.pid_was_timed_out = PLATFORM_FALSE;

        dt_s = (float_t)dt_ms / 1000.0f;

        for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
        {
            int16_t duty = 0;

            /* Speed feedback is what the dead-zone sweep keys on. */
            if (s_chassis.closed_loop == PLATFORM_TRUE)
            {
                (void)service_chassis_speed_mm_s(
                          i, dt_s, service_chassis_wheel_counts_to_mm(i));
            }

            if ((now_ms - s_chassis.test_duty_ms[i]) <=
                SERVICE_CHASSIS_TEST_DUTY_TIMEOUT_MS)
            {
                duty = s_chassis.test_duty[i];
                all_stale = PLATFORM_FALSE;
            }

            s_chassis.applied_duty[i] = duty;
            (void)platform_motor_set_duty(s_chassis.cfg.motor[i], duty);
        }

        if (all_stale == PLATFORM_TRUE)
        {
            /* Every wheel expired: hand the motors back to the velocity loop. */
            s_chassis.test_active = PLATFORM_FALSE;
        }

        return PLATFORM_FALSE;
    }

    /*
     * The loops were frozen during a timeout. A valid frame has cleared the
     * flag since the last tick, so restart from a fresh encoder baseline;
     * otherwise the counts accumulated while waiting would look like a speed
     * spike on the first update.
     */
    if (s_chassis.timed_out == PLATFORM_TRUE)
    {
        service_chassis_zero_duties();

        return PLATFORM_FALSE;
    }

    if (s_chassis.pid_was_timed_out == PLATFORM_TRUE)
    {
        service_chassis_reset_pid_state();
        s_chassis.pid_was_timed_out = PLATFORM_FALSE;
    }

    vx = s_chassis.target_vx_mm_s;
    vy = s_chassis.target_vy_mm_s;
    wz = s_chassis.target_wz_mrad_s;

    age = now_ms - s_chassis.last_cmd_ms;

    if (age > s_chassis.cfg.cmd_timeout_ms)
    {
        vx = 0;
        vy = 0;
        wz = 0;

        s_chassis.target_vx_mm_s = 0;
        s_chassis.target_vy_mm_s = 0;
        s_chassis.target_wz_mrad_s = 0;

        if (s_chassis.timed_out == PLATFORM_FALSE)
        {
            s_chassis.timed_out = PLATFORM_TRUE;
            timed_out_now = PLATFORM_TRUE;

            (void)platform_motor_stop_all();
            service_chassis_reset_pid_state();
        }

        s_chassis.pid_was_timed_out = PLATFORM_TRUE;

        service_chassis_zero_duties();

        return timed_out_now;
    }

    s_chassis.timed_out = PLATFORM_FALSE;

    service_chassis_inverse_kinematics(vx, vy, wz,
                                       (int16_t)(s_chassis.cfg.half_length_mm +
                                                 s_chassis.cfg.half_width_mm),
                                       wheel_target);

    if (s_chassis.closed_loop == PLATFORM_TRUE)
    {
        dt_s = (float_t)dt_ms / 1000.0f;

        for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
        {
            float_t measured_mm_s;
            float_t duty;
            int16_t target_mm_s;

            measured_mm_s = service_chassis_speed_mm_s(
                                i, dt_s,
                                service_chassis_wheel_counts_to_mm(i));

            target_mm_s = service_chassis_clamp_speed(
                              (int32_t)wheel_target[i],
                              s_chassis.cfg.max_wheel_mm_s);

            if ((target_mm_s == 0) &&
                (measured_mm_s <= SERVICE_CHASSIS_ZERO_SPEED_EPS_MM_S) &&
                (measured_mm_s >= -SERVICE_CHASSIS_ZERO_SPEED_EPS_MM_S))
            {
                /*
                 * Stopped at a zero target: do not let a negative integral
                 * keep a reverse duty on the motor and make the wheel creep.
                 */
                platform_pid_reset(&s_chassis.pid[i]);
                duty = 0.0f;
            }
            else
            {
                duty = platform_pid_update(&s_chassis.pid[i],
                                           (float_t)target_mm_s,
                                           measured_mm_s,
                                           dt_s);

                /*
                 * Reliable-start compensation. Only while the wheel is still
                 * at rest: once it turns, the loop must be free to brake
                 * below this duty without the floor holding the motor on.
                 * A calibrated value of 0 disables the whole thing.
                 */
                if ((target_mm_s != 0) &&
                    (measured_mm_s <= SERVICE_CHASSIS_ZERO_SPEED_EPS_MM_S) &&
                    (measured_mm_s >= -SERVICE_CHASSIS_ZERO_SPEED_EPS_MM_S))
                {
                    if ((target_mm_s > 0) &&
                        (duty < (float_t)s_chassis.start_duty_fwd_permille[i]))
                    {
                        duty = (float_t)
                                   s_chassis.start_duty_fwd_permille[i];
                    }
                    else if ((target_mm_s < 0) &&
                             (duty > -(float_t)
                                          s_chassis.start_duty_rev_permille[i]))
                    {
                        duty = -(float_t)
                                   s_chassis.start_duty_rev_permille[i];
                    }
                    else
                    {
                        /* The loop already commands more than the start duty. */
                    }
                }
            }

            s_chassis.applied_duty[i] = service_chassis_pid_duty(duty);
            (void)platform_motor_set_duty(s_chassis.cfg.motor[i],
                                          s_chassis.applied_duty[i]);
        }
    }
    else
    {
        /* No usable encoder scale: keep the old open-loop mapping. */
        for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
        {
            int16_t duty;

            s_chassis.measured_wheel_mm_s[i] = 0;
            duty = service_chassis_duty(wheel_target[i],
                                        s_chassis.cfg.max_wheel_mm_s);
            s_chassis.applied_duty[i] = duty;
            (void)platform_motor_set_duty(s_chassis.cfg.motor[i], duty);
        }
    }

    return timed_out_now;
}

void service_chassis_update_odometry(uint32_t dt_ms)
{
    int32_t counts[SERVICE_CHASSIS_WHEEL_COUNT];
    float   d_mm[SERVICE_CHASSIS_WHEEL_COUNT];
    float   dx_body;
    float   dy_body;
    float   d_theta;
    float   h_mid;
    float   dt_s;
    uint8_t i;

    if ((s_chassis.initialized == PLATFORM_FALSE) || (dt_ms == 0u) ||
        (s_chassis.closed_loop == PLATFORM_FALSE))
    {
        return;
    }

    for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
    {
        int32_t now_counts = s_chassis.prev_counts[i];

        /* A failed read contributes no motion rather than a fresh baseline. */
        (void)platform_motor_get_encoder(s_chassis.cfg.motor[i], &now_counts);

        /*
         * Subtract in unsigned arithmetic so the wrap at 2^32 stays defined;
         * the result is the signed step either way.
         */
        counts[i] = (int32_t)((uint32_t)now_counts -
                              (uint32_t)s_chassis.prev_counts[i]);
        s_chassis.prev_counts[i] = now_counts;

        d_mm[i] = (float)counts[i] *
                  service_chassis_wheel_counts_to_mm(i) *
                  (float)service_chassis_wheel_dir(i);
    }

    dx_body = (d_mm[SERVICE_CHASSIS_WHEEL_FL] + d_mm[SERVICE_CHASSIS_WHEEL_FR] +
               d_mm[SERVICE_CHASSIS_WHEEL_RL] + d_mm[SERVICE_CHASSIS_WHEEL_RR])
              * 0.25f;

    dy_body = (-d_mm[SERVICE_CHASSIS_WHEEL_FL] + d_mm[SERVICE_CHASSIS_WHEEL_FR] +
               d_mm[SERVICE_CHASSIS_WHEEL_RL] - d_mm[SERVICE_CHASSIS_WHEEL_RR])
              * 0.25f;

    d_theta = (-d_mm[SERVICE_CHASSIS_WHEEL_FL] + d_mm[SERVICE_CHASSIS_WHEEL_FR] -
               d_mm[SERVICE_CHASSIS_WHEEL_RL] + d_mm[SERVICE_CHASSIS_WHEEL_RR]) /
              (4.0f * (float)(s_chassis.cfg.half_length_mm +
                              s_chassis.cfg.half_width_mm));

    /* Midpoint heading: second-order accurate and cheap. */
    h_mid = s_chassis.heading_rad + (d_theta * 0.5f);

    s_chassis.x_mm += service_chassis_round((dx_body * cosf(h_mid)) -
                                            (dy_body * sinf(h_mid)));
    s_chassis.y_mm += service_chassis_round((dx_body * sinf(h_mid)) +
                                            (dy_body * cosf(h_mid)));

    s_chassis.heading_rad += d_theta;

    while (s_chassis.heading_rad > SERVICE_CHASSIS_PI)
    {
        s_chassis.heading_rad -= SERVICE_CHASSIS_TWO_PI;
    }

    while (s_chassis.heading_rad <= -SERVICE_CHASSIS_PI)
    {
        s_chassis.heading_rad += SERVICE_CHASSIS_TWO_PI;
    }

    dt_s = (float)dt_ms / 1000.0f;

    s_chassis.vl_mm_s = service_chassis_clamp_i16(
        service_chassis_round(
            ((d_mm[SERVICE_CHASSIS_WHEEL_FL] +
              d_mm[SERVICE_CHASSIS_WHEEL_RL]) * 0.5f) / dt_s));

    s_chassis.vr_mm_s = service_chassis_clamp_i16(
        service_chassis_round(
            ((d_mm[SERVICE_CHASSIS_WHEEL_FR] +
              d_mm[SERVICE_CHASSIS_WHEEL_RR]) * 0.5f) / dt_s));
}

void service_chassis_get_odometry(service_chassis_odometry_t *out)
{
    int32_t centi_deg;

    if (out == NULL)
    {
        return;
    }

    centi_deg = (int32_t)(s_chassis.heading_rad * SERVICE_CHASSIS_RAD_TO_CD);

    while (centi_deg >= 18000)
    {
        centi_deg -= 36000;
    }

    while (centi_deg < -18000)
    {
        centi_deg += 36000;
    }

    out->x_mm = s_chassis.x_mm;
    out->y_mm = s_chassis.y_mm;
    out->heading_centi_deg = (int16_t)centi_deg;
    out->vl_mm_s = s_chassis.vl_mm_s;
    out->vr_mm_s = s_chassis.vr_mm_s;
}

void service_chassis_get_wheel_speeds(
                    int16_t out_mm_s[SERVICE_CHASSIS_WHEEL_COUNT])
{
    uint8_t i;

    if (out_mm_s == NULL)
    {
        return;
    }

    for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
    {
        out_mm_s[i] = s_chassis.measured_wheel_mm_s[i];
    }
}

void service_chassis_get_wheel_duty(
                    int16_t out_permille[SERVICE_CHASSIS_WHEEL_COUNT])
{
    uint8_t i;

    if (out_permille == NULL)
    {
        return;
    }

    for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
    {
        out_permille[i] = s_chassis.applied_duty[i];
    }
}

platform_err_t service_chassis_set_test_duty(uint8_t wheel,
                                             int16_t permille,
                                             uint32_t now_ms)
{
    if ((s_chassis.initialized == PLATFORM_FALSE) ||
        (wheel >= SERVICE_CHASSIS_WHEEL_COUNT))
    {
        return PLATFORM_ERR_PARAM;
    }

    if ((permille > SERVICE_CHASSIS_TEST_DUTY_MAX) ||
        (permille < -SERVICE_CHASSIS_TEST_DUTY_MAX))
    {
        return PLATFORM_ERR_PARAM;
    }

    /*
     * The bridges must be enabled or the duty has nowhere to go. Refusing
     * here keeps "test mode is active but nothing moves" from looking like an
     * encoder fault on the bench.
     */
    if ((s_chassis.enabled == PLATFORM_FALSE) ||
        (s_chassis.estop == PLATFORM_TRUE))
    {
        return PLATFORM_ERR_FAIL;
    }

    if (s_chassis.test_active == PLATFORM_FALSE)
    {
        uint8_t i;

        /* Entering test mode: drop any pending velocity target. */
        s_chassis.target_vx_mm_s = 0;
        s_chassis.target_vy_mm_s = 0;
        s_chassis.target_wz_mrad_s = 0;

        /*
         * A duty left over from an earlier test session is still inside its
         * expiry window for up to 300 ms. Clear all four so only the wheel the
         * host just wrote can move.
         */
        for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
        {
            s_chassis.test_duty[i] = 0;
            s_chassis.test_duty_ms[i] = now_ms;
        }

        s_chassis.test_active = PLATFORM_TRUE;
    }

    s_chassis.test_duty[wheel] = permille;
    s_chassis.test_duty_ms[wheel] = now_ms;

    return PLATFORM_ERR_OK;
}

platform_bool_t service_chassis_is_test_mode(void)
{
    return s_chassis.test_active;
}

platform_bool_t service_chassis_cmd_timed_out(void)
{
    return s_chassis.timed_out;
}
