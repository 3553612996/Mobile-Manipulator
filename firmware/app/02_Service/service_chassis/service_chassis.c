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
    platform_bool_t             closed_loop;
    platform_bool_t             pid_was_enabled;
    platform_bool_t             pid_was_estopped;
    platform_bool_t             pid_was_timed_out;

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
    s_chassis.closed_loop = (cfg->counts_per_mm_x1000 > 0)
                                ? PLATFORM_TRUE : PLATFORM_FALSE;
    s_chassis.pid_was_enabled = PLATFORM_FALSE;
    s_chassis.pid_was_estopped = PLATFORM_FALSE;
    s_chassis.pid_was_timed_out = PLATFORM_FALSE;

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
        s_chassis.target_vx_mm_s = 0;
        s_chassis.target_vy_mm_s = 0;
        s_chassis.target_wz_mrad_s = 0;

        /* Zero the compare first, then let the bridge coast. */
        (void)platform_motor_stop_all();
        service_chassis_set_bridges(PLATFORM_FALSE);
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
    s_chassis.target_vx_mm_s = 0;
    s_chassis.target_vy_mm_s = 0;
    s_chassis.target_wz_mrad_s = 0;
}

void service_chassis_emergency_stop(void)
{
    s_chassis.estop = PLATFORM_TRUE;
    s_chassis.enabled = PLATFORM_FALSE;
    s_chassis.timed_out = PLATFORM_FALSE;
    s_chassis.target_vx_mm_s = 0;
    s_chassis.target_vy_mm_s = 0;
    s_chassis.target_wz_mrad_s = 0;

    (void)platform_motor_stop_all();
    service_chassis_set_bridges(PLATFORM_FALSE);
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
    float_t  counts_to_mm;
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
        (s_chassis.estop != s_chassis.pid_was_estopped))
    {
        s_chassis.pid_was_enabled = s_chassis.enabled;
        s_chassis.pid_was_estopped = s_chassis.estop;
        service_chassis_reset_pid_state();
    }

    if ((s_chassis.enabled == PLATFORM_FALSE) ||
        (s_chassis.estop == PLATFORM_TRUE))
    {
        s_chassis.timed_out = PLATFORM_FALSE;
        s_chassis.pid_was_timed_out = PLATFORM_FALSE;
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
        for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
        {
            (void)platform_motor_set_duty(s_chassis.cfg.motor[i], 0);
        }

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

        for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
        {
            (void)platform_motor_set_duty(s_chassis.cfg.motor[i], 0);
        }

        return timed_out_now;
    }

    s_chassis.timed_out = PLATFORM_FALSE;

    service_chassis_inverse_kinematics(vx, vy, wz,
                                       (int16_t)(s_chassis.cfg.half_length_mm +
                                                 s_chassis.cfg.half_width_mm),
                                       wheel_target);

    if (dt_ms == 0u)
    {
        dt_ms = 1u;
    }

    if (s_chassis.closed_loop == PLATFORM_TRUE)
    {
        dt_s = (float_t)dt_ms / 1000.0f;
        counts_to_mm = 1000.0f / (float_t)s_chassis.cfg.counts_per_mm_x1000;

        for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
        {
            int32_t now_counts = s_chassis.pid_prev_counts[i];
            int32_t delta_counts;
            float_t measured_mm_s;
            float_t duty;
            int16_t target_mm_s;

            /*
             * A failed read contributes no motion rather than a fresh
             * baseline, matching the odometry path.
             */
            (void)platform_motor_get_encoder(s_chassis.cfg.motor[i],
                                             &now_counts);

            delta_counts = (int32_t)((uint32_t)now_counts -
                                     (uint32_t)s_chassis.pid_prev_counts[i]);
            s_chassis.pid_prev_counts[i] = now_counts;

            measured_mm_s = ((float_t)delta_counts * counts_to_mm) / dt_s;
            s_chassis.measured_wheel_mm_s[i] =
                service_chassis_clamp_i16(service_chassis_round(measured_mm_s));

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
            }

            (void)platform_motor_set_duty(
                        s_chassis.cfg.motor[i],
                        service_chassis_pid_duty(duty));
        }
    }
    else
    {
        /* No usable encoder scale: keep the old open-loop mapping. */
        for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
        {
            s_chassis.measured_wheel_mm_s[i] = 0;
            (void)platform_motor_set_duty(
                        s_chassis.cfg.motor[i],
                        service_chassis_duty(wheel_target[i],
                                             s_chassis.cfg.max_wheel_mm_s));
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
    float   scale;
    uint8_t i;

    if ((s_chassis.initialized == PLATFORM_FALSE) || (dt_ms == 0u) ||
        (s_chassis.cfg.counts_per_mm_x1000 <= 0))
    {
        return;
    }

    scale = 1000.0f / (float)s_chassis.cfg.counts_per_mm_x1000;

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

        d_mm[i] = (float)counts[i] * scale;
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

platform_bool_t service_chassis_cmd_timed_out(void)
{
    return s_chassis.timed_out;
}
