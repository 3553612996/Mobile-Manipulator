/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_params.h
 *
 * @par dependencies
 * - app_robot_config.h
 * - service_chassis.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Own the runtime parameter and calibration values and their byte layout.
 *
 * Why the application owns the bytes
 * ----------------------------------
 * service_storage only promises that the bytes it returns are the bytes that
 * were saved. What those bytes mean, how they are versioned, which of them may
 * be reset without losing a calibration, and which ones require the chassis to
 * be disabled all belong to the application. This module is that knowledge:
 * the values in RAM, the compile-time defaults, and the on-flash encoding.
 *
 * What lives here
 * ---------------
 *   - per-wheel PID gains (as x1000 integers, the same unit the protocol uses),
 *   - the log level, the command timeout and the velocity limits,
 *   - the chassis calibration: wheelbase, track, wheel diameter, and per wheel
 *     the encoder scale, direction and reliable-start duty,
 *   - the arm protection thresholds,
 *   - the arm calibration: each joint's bus servo ID and calibrated zero.
 *
 * What does not
 * -------------
 * The values themselves are not duplicated: the chassis service remains the
 * owner of the running PID state. This module holds the values that will be
 * saved and restored, and re-applies them through the service API.
 *
 * Dirtiness
 * ---------
 * SET_PARAM / PARAM_RESET only touch RAM and set a dirty bit; PARAM_SAVE is the
 * only path to flash. The dirty mask is what REPORT_STORAGE bit 0 reports.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __APP_PARAMS_H__
#define __APP_PARAMS_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "protocol_def.h"

//******************************** Defines **********************************//

/*
 * Layout version. 3 added the arm protection thresholds (parameter blob) and
 * the per-joint servo ID / calibrated zero (calibration blob). The parser
 * still accepts versions 1 and 2 so a board flashed with older firmware keeps
 * its saved gains, counts and geometry instead of silently reverting to the
 * compile-time defaults.
 */
#define APP_PARAMS_VERSION          ((uint16_t)3u)

#define APP_PARAMS_WHEEL_COUNT      (4u)
#define APP_PARAMS_GAIN_COUNT       (3u)
#define APP_PARAMS_JOINT_COUNT      (6u)

/* Encoded sizes, fixed by this file and checked on load. */
#define APP_PARAMS_BLOB_PARAM_SIZE  ((uint16_t)72u)
#define APP_PARAMS_BLOB_CALIB_SIZE  ((uint16_t)128u)

/* Largest blob, for the boot-time load buffer. */
#define APP_PARAMS_BLOB_MAX_SIZE    APP_PARAMS_BLOB_CALIB_SIZE

/* Earlier sizes, accepted on load only. */
#define APP_PARAMS_BLOB_PARAM_SIZE_V1 ((uint16_t)56u)
#define APP_PARAMS_BLOB_CALIB_SIZE_V1 ((uint16_t)28u)
#define APP_PARAMS_BLOB_PARAM_SIZE_V2 ((uint16_t)60u)
#define APP_PARAMS_BLOB_CALIB_SIZE_V2 ((uint16_t)80u)

/* Scope values are the protocol's: 0 = parameter area, 1 = calibration. */
#define APP_PARAMS_SCOPE_PARAM      PROTOCOL_STORAGE_SCOPE_PARAM
#define APP_PARAMS_SCOPE_CALIB      PROTOCOL_STORAGE_SCOPE_CALIB
#define APP_PARAMS_SCOPE_ALL        PROTOCOL_STORAGE_SCOPE_ALL

//******************************** Declaring ********************************//

/* Load the compile-time defaults from app_robot_config.h into RAM. */
void app_params_init(void);

/* Restore defaults for one scope (or both) and mark them dirty. */
void app_params_reset(uint8_t scope);

/* ------------------------------ Readers -------------------------------- */

/* Gain in x1000 units. out-of-range wheel/gain returns 0. */
int32_t  app_params_pid_gain_x1000(uint8_t wheel, uint8_t gain);

/* Geometry, as the calibration page and the protocol express it. */
int32_t  app_params_wheelbase_mm(void);
int32_t  app_params_track_mm(void);
int32_t  app_params_wheel_diameter_x10_mm(void);

/* Half wheelbase / half track, derived from the two values above. */
int32_t  app_params_half_length_mm(void);
int32_t  app_params_half_width_mm(void);

/* First wheel's scale, kept for callers that only need one number. */
int32_t  app_params_counts_per_mm_x1000(void);
int32_t  app_params_wheel_counts_per_mm_x1000(uint8_t wheel);
int8_t   app_params_wheel_dir(uint8_t wheel);
int16_t  app_params_deadzone_fwd_permille(uint8_t wheel);
int16_t  app_params_deadzone_rev_permille(uint8_t wheel);

/* Parameter area. */
uint32_t app_params_cmd_timeout_ms(void);
int16_t  app_params_max_linear_mm_s(void);
int16_t  app_params_max_angular_mrad_s(void);
uint8_t  app_params_log_level(void);

/* Arm protection thresholds (parameter area). */
int16_t  app_params_arm_overtemp_c(void);
int16_t  app_params_arm_undervolt_mv(void);
int16_t  app_params_arm_overvolt_mv(void);
int16_t  app_params_arm_stall_error_centi_deg(void);
int16_t  app_params_arm_stall_time_ms(void);
int16_t  app_params_arm_stall_margin_ms(void);

/* Joint identity and calibrated zero (calibration area). */
uint8_t  app_params_servo_id(uint8_t joint);
int16_t  app_params_servo_offset_centi_deg(uint8_t joint);

/* TRUE when a line at this level should reach the log ring. Level 0 is
 * DEBUG, 3 is ERROR: a higher configured level filters more. */
platform_bool_t app_params_log_enabled(uint8_t level);

/* ------------------------------ Writers -------------------------------- */

/* Validate against the same limits SET_PARAM enforces, then store in RAM. */
platform_err_t app_params_set_pid_gain(uint8_t wheel, uint8_t gain,
                                        int32_t value_x1000);
platform_err_t app_params_set_log_level(uint8_t level);

/* Calibration area. Every one of these marks the calibration dirty. */
platform_err_t app_params_set_track_mm(int32_t value);
platform_err_t app_params_set_wheel_diameter_mm(int32_t value);
platform_err_t app_params_set_wheel_counts(uint8_t wheel, int32_t value);
platform_err_t app_params_set_wheel_dir(uint8_t wheel, int32_t value);
platform_err_t app_params_set_wheel_deadzone(uint8_t wheel, int32_t fwd,
                                             int32_t rev);

/* Parameter area limits. */
platform_err_t app_params_set_cmd_timeout_ms(int32_t value);
platform_err_t app_params_set_max_linear(int32_t value);
platform_err_t app_params_set_max_angular(int32_t value);

/* Arm protection thresholds; each marks the parameter area dirty. */
platform_err_t app_params_set_arm_overtemp(int32_t value);
platform_err_t app_params_set_arm_undervolt(int32_t value);
platform_err_t app_params_set_arm_overvolt(int32_t value);
platform_err_t app_params_set_arm_stall_error(int32_t value);
platform_err_t app_params_set_arm_stall_time(int32_t value);

/* Servo identity; marks the calibration area dirty. */
platform_err_t app_params_set_servo_id(uint8_t joint, int32_t value);
platform_err_t app_params_set_servo_offset(uint8_t joint, int32_t value);

/* Push every stored value into the chassis service: gains, geometry, per-wheel
 * scale/direction/start duty and the velocity limits. Control task only: the
 * service re-initialises each wheel's PID, which must not race apply(). Boot
 * calls it once after service_chassis_init(), before any task is running. */
void app_params_apply_to_chassis(void);

/* Push only the command timeout and velocity limits. Control task only. This
 * is the reload used by SET_PARAM 0x10~0x12: those ids are writable while the
 * chassis is enabled, and resetting the PID state mid-motion for a limit change
 * would show up as a torque glitch. */
void app_params_apply_limits_to_chassis(void);

/* ------------------------------ Dirtiness ------------------------------ */

platform_bool_t app_params_dirty(void);

/* Mark one scope changed (RAM only). */
void app_params_mark_dirty(uint8_t scope);

/* A save for that scope finished; clears its dirty bit. */
void app_params_mark_saved(uint8_t scope);

/* ---------------------------- Serialization ---------------------------- */

/*
 * Encode one scope into buffer. Returns the encoded length, or 0 when the
 * scope is invalid or the buffer is too small. The encoding is little-endian
 * and versioned; bump APP_PARAMS_VERSION whenever a field moves.
 */
uint16_t app_params_serialize(uint8_t scope,
                              uint8_t *buffer,
                              uint16_t capacity);

/*
 * Decode one scope from flash bytes. Returns PLATFORM_TRUE when the version and
 * length match and every field is in range; the RAM copy is replaced only in
 * that case, so a corrupt blob leaves the defaults untouched.
 */
platform_bool_t app_params_parse(uint8_t scope,
                                 const uint8_t *data,
                                 uint16_t length);

#endif /* __APP_PARAMS_H__ */
