/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file service_chassis.h
 *
 * @par dependencies
 * - platform_bsp_motor.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Mecanum chassis service: motion primitives, kinematics, safety.
 *
 * The application expresses intent in vehicle coordinates:
 *
 *     service_chassis_forward(200, now);
 *     service_chassis_strafe(-100, now);
 *     service_chassis_rotate(500, now);
 *
 * and never computes wheel speeds, duty or per-wheel signs. This service owns:
 *   - the mecanum inverse kinematics,
 *   - target validation and the 200 ms command timeout,
 *   - enable/disable and the latched emergency stop,
 *   - four independent wheel-speed loops (PID) on the encoder feedback,
 *   - dead-reckoning odometry from the four encoders.
 *
 * Coordinate frame (REP-103 style, flat floor):
 *   +x forward, +y left, +w counter-clockwise seen from above.
 *   Wheel order is FL, FR, RL, RR.
 *
 * Time is injected (now_ms from the caller's millisecond clock) so the timeout
 * logic is testable on a PC and has no hidden dependency on the scheduler.
 *
 * @version V1.1 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __SERVICE_CHASSIS_H__
#define __SERVICE_CHASSIS_H__

//******************************** Includes *********************************//

#include "platform_bsp_motor.h"

//******************************** Defines **********************************//

#define SERVICE_CHASSIS_WHEEL_COUNT     (4u)

/* Wheel indices, in the order every array in this service uses. */
#define SERVICE_CHASSIS_WHEEL_FL        (0u)
#define SERVICE_CHASSIS_WHEEL_FR        (1u)
#define SERVICE_CHASSIS_WHEEL_RL        (2u)
#define SERVICE_CHASSIS_WHEEL_RR        (3u)

//******************************** Declaring ********************************//

typedef struct
{
    /* Logical wheel -> physical motor. FL, FR, RL, RR. */
    platform_motor_id_t motor[SERVICE_CHASSIS_WHEEL_COUNT];

    int16_t             half_length_mm;         /* Wheelbase / 2. */
    int16_t             half_width_mm;          /* Track / 2. */

    int16_t             max_vx_mm_s;
    int16_t             max_vy_mm_s;
    int16_t             max_wz_mrad_s;

    /*
     * Wheel-speed limit. It caps each wheel target and is also the full scale
     * of the open-loop fallback used when counts_per_mm_x1000 is 0.
     */
    int16_t             max_wheel_mm_s;

    /*
     * Per-wheel speed-loop gains. They are arrays so one wheel can be tuned
     * without changing the others; the application normally starts with the
     * same values for the left and right pairs. Gains must be >= 0.
     */
    float_t             pid_kp[SERVICE_CHASSIS_WHEEL_COUNT];
    float_t             pid_ki[SERVICE_CHASSIS_WHEEL_COUNT];
    float_t             pid_kd[SERVICE_CHASSIS_WHEEL_COUNT];

    /*
     * Encoder counts per millimetre, multiplied by 1000. 0 disables odometry
     * and makes the chassis fall back to open-loop duty.
     */
    int32_t             counts_per_mm_x1000;

    uint32_t            cmd_timeout_ms;         /* 200 ms per protocol. */
} service_chassis_cfg_t;

typedef struct
{
    int32_t     x_mm;
    int32_t     y_mm;
    int16_t     heading_centi_deg;  /* Wrapped to [-18000, 18000). */
    int16_t     vl_mm_s;            /* Measured left side speed. */
    int16_t     vr_mm_s;            /* Measured right side speed. */
} service_chassis_odometry_t;

platform_err_t service_chassis_init(const service_chassis_cfg_t *cfg);

/* Enabling is refused while the emergency stop is latched. */
platform_err_t service_chassis_enable(platform_bool_t enable, uint32_t now_ms);
platform_bool_t service_chassis_is_enabled(void);
platform_bool_t service_chassis_is_estopped(void);

/*
 * Target velocity in vehicle coordinates. Returns PLATFORM_ERR_PARAM when a
 * component exceeds the configured limit; the previous target is kept so a
 * single bad frame cannot jerk the robot.
 */
platform_err_t service_chassis_set_velocity(int16_t vx_mm_s,
                                            int16_t vy_mm_s,
                                            int16_t wz_mrad_s,
                                            uint32_t now_ms);

/* Convenience primitives for the application and the on-board key. */
platform_err_t service_chassis_forward(uint16_t speed_mm_s, uint32_t now_ms);
platform_err_t service_chassis_backward(uint16_t speed_mm_s, uint32_t now_ms);
platform_err_t service_chassis_strafe(int16_t speed_mm_s, uint32_t now_ms);
platform_err_t service_chassis_rotate(int16_t wz_mrad_s, uint32_t now_ms);

/* Zero the target but stay enabled. */
void service_chassis_stop(void);

/* Latched stop: zero target, disable the drivers, refuse enable until clear. */
void service_chassis_emergency_stop(void);
platform_err_t service_chassis_clear_estop(void);

/*
 * One control tick: run the safety watchdog, update the four speed loops from
 * the encoders and write the motor duty. Call at the control period (10 ms).
 * dt_ms is the interval since the previous call and must be non-zero.
 * Returns PLATFORM_TRUE on the tick the timeout becomes active.
 *
 * While the command timeout is active the loops are held at zero and the
 * integral is kept reset; a fresh valid frame restarts them from a new
 * encoder baseline.
 */
platform_bool_t service_chassis_apply(uint32_t now_ms, uint32_t dt_ms);

/* Integrate one encoder interval. Call at the odometry period (10 ms). */
void service_chassis_update_odometry(uint32_t dt_ms);

void service_chassis_get_odometry(service_chassis_odometry_t *out);

/*
 * Last measured per-wheel speed in FL, FR, RL, RR order. Valid after apply();
 * all zeros when speed feedback is unavailable (open-loop fallback).
 */
void service_chassis_get_wheel_speeds(
                    int16_t out_mm_s[SERVICE_CHASSIS_WHEEL_COUNT]);

/* TRUE while the command timeout is active (since the last valid frame). */
platform_bool_t service_chassis_cmd_timed_out(void);

/*
 * Pure mecanum inverse kinematics: k_mm = half_length + half_width.
 * Exposed so the mapping can be verified without hardware.
 */
void service_chassis_inverse_kinematics(int16_t vx_mm_s,
                                        int16_t vy_mm_s,
                                        int16_t wz_mrad_s,
                                        int16_t k_mm,
                                        int16_t wheel_mm_s
                                            [SERVICE_CHASSIS_WHEEL_COUNT]);

#endif /* __SERVICE_CHASSIS_H__ */
