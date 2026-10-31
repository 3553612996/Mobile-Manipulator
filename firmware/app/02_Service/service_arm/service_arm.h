/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file service_arm.h
 *
 * @par dependencies
 * - platform_bsp_servo.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Six-joint arm service: limits, protection, target tracking.
 *
 * The application sends six joint angles in 0.01 degrees; this service checks
 * them against the configured limits, remembers the last accepted target and
 * forwards the move to the servo bus. The gripper is not part of this service:
 * there is no gripper hardware on the current board, so the protocol handler
 * reports SET_GRIPPER as failed instead of pretending it worked.
 *
 * Two threads touch the state:
 *   - the servo task runs service_arm_poll() every few milliseconds, which
 *     advances the bus state machine;
 *   - the control task runs service_arm_update(), which evaluates the cached
 *     readings against the protection thresholds and latches faults.
 *
 * Protection faults latch: once the arm has stalled or overheated it stays
 * faulted (and disabled) until the host clears it. A persisting condition
 * re-latches on the next update, so clearing is not a way around the limit.
 *
 * @version V2.0 2026-10-31
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __SERVICE_ARM_H__
#define __SERVICE_ARM_H__

//******************************** Includes *********************************//

#include "platform_bsp_servo.h"

//******************************** Defines **********************************//

#define SERVICE_ARM_JOINT_COUNT     PLATFORM_SERVO_JOINT_COUNT
#define SERVICE_ARM_MOVE_TIME_MS    ((uint16_t)800u)

/* Mirror of the platform bits, for the handlers that merge them. */
#define SERVICE_ARM_FAULT_OFFLINE   PLATFORM_SERVO_FAULT_OFFLINE
#define SERVICE_ARM_FAULT_OVERTEMP  PLATFORM_SERVO_FAULT_OVERTEMP
#define SERVICE_ARM_FAULT_UNDERVOLT PLATFORM_SERVO_FAULT_UNDERVOLT
#define SERVICE_ARM_FAULT_OVERVOLT  PLATFORM_SERVO_FAULT_OVERVOLT
#define SERVICE_ARM_FAULT_STALL     PLATFORM_SERVO_FAULT_STALL
#define SERVICE_ARM_FAULT_REPLY     PLATFORM_SERVO_FAULT_REPLY

//******************************** Declaring ********************************//

/*
 * Protection thresholds. The defaults come from app_robot_config.h and can be
 * replaced by SET_PARAM 0x60..0x64; they are persisted with the other arm
 * parameters.
 */
typedef struct
{
    int16_t     overtemp_c;             /* >= this temperature faults. */
    uint16_t    undervolt_mv;           /* < this voltage faults. */
    uint16_t    overvolt_mv;            /* > this voltage faults. */
    int16_t     stall_error_centi_deg;  /* |target - measured| considered stuck. */
    uint16_t    stall_time_ms;          /* ... for at least this long. */
    uint16_t    stall_margin_ms;        /* Grace after a move before checking. */
} service_arm_protection_t;

typedef struct
{
    int16_t     min_centi_deg[SERVICE_ARM_JOINT_COUNT];
    int16_t     max_centi_deg[SERVICE_ARM_JOINT_COUNT];
    int16_t     home_centi_deg[SERVICE_ARM_JOINT_COUNT];
    uint16_t    default_move_time_ms;   /* 0 selects SERVICE_ARM_MOVE_TIME_MS. */
    service_arm_protection_t protection;
} service_arm_cfg_t;

typedef struct
{
    platform_bool_t enabled;
    uint8_t         fault_bits[SERVICE_ARM_JOINT_COUNT];   /* Latched, merged. */
    platform_bool_t faulted;
} service_arm_status_t;

platform_err_t service_arm_init(const service_arm_cfg_t *cfg);

/* Replace the protection thresholds; see service_arm_protection_t. */
platform_err_t service_arm_get_protection(service_arm_protection_t *out);
platform_err_t service_arm_set_protection(
                                const service_arm_protection_t *protection);

platform_err_t service_arm_enable(platform_bool_t enable);
platform_bool_t service_arm_is_enabled(void);

/*
 * Validate and command all six joints. move_time_ms == 0 uses the configured
 * default. Returns PLATFORM_ERR_PARAM if any joint is outside its limit or
 * PLATFORM_ERR_FAIL when the arm is disabled or faulted; the previous target is
 * kept in both cases.
 */
platform_err_t service_arm_set_joints(
                                const int16_t joint_centi_deg
                                    [SERVICE_ARM_JOINT_COUNT],
                                uint16_t move_time_ms);

/* Advance the bus state machine. Servo task only. */
platform_err_t service_arm_poll(uint32_t now_ms);

/* Sample the cache and evaluate the protection rules. Control task only. */
platform_err_t service_arm_update(uint32_t now_ms);

platform_err_t service_arm_get_status(service_arm_status_t *out);
platform_bool_t service_arm_has_fault(void);

/* Clear every latched arm fault. A live condition latches again on update. */
platform_err_t service_arm_clear_faults(void);

/* Last accepted target, not the measured position. */
platform_err_t service_arm_get_targets(
                                platform_servo_angles_t *out);

/* Measured positions, load and fault flags from the bus. */
platform_err_t service_arm_get_states(platform_servo_states_t *out);

/*
 * Servo-side configuration (ID, bias, zero calibration, torque), queued for
 * the servo task. The comm task sleeps between ready/take, never here.
 */
platform_err_t service_arm_config(const platform_servo_cfg_req_t *req);
platform_bool_t service_arm_config_ready(void);
platform_err_t service_arm_config_take(int32_t *out_value);
void service_arm_config_cancel(void);

#endif /* __SERVICE_ARM_H__ */
