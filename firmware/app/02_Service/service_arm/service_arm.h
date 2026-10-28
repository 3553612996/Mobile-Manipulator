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
 * @brief Six-joint arm service: limits, enable state, target tracking.
 *
 * The application sends six joint angles in 0.01 degrees; this service checks
 * them against the configured limits, remembers the last accepted target and
 * forwards the move to the servo bus. The gripper is not part of this service:
 * there is no gripper hardware on the current board, so the protocol handler
 * reports SET_GRIPPER as failed instead of pretending it worked.
 *
 * @version V1.0 2026-10-28
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

//******************************** Declaring ********************************//

typedef struct
{
    int16_t     min_centi_deg[SERVICE_ARM_JOINT_COUNT];
    int16_t     max_centi_deg[SERVICE_ARM_JOINT_COUNT];
    int16_t     home_centi_deg[SERVICE_ARM_JOINT_COUNT];
    uint16_t    default_move_time_ms;   /* 0 selects SERVICE_ARM_MOVE_TIME_MS. */
} service_arm_cfg_t;

platform_err_t service_arm_init(const service_arm_cfg_t *cfg);

platform_err_t service_arm_enable(platform_bool_t enable);
platform_bool_t service_arm_is_enabled(void);

/*
 * Validate and command all six joints. move_time_ms == 0 uses the configured
 * default. Returns PLATFORM_ERR_PARAM if any joint is outside its limit or
 * PLATFORM_ERR_FAIL when the arm is disabled; the previous target is kept.
 */
platform_err_t service_arm_set_joints(
                                const int16_t joint_centi_deg
                                    [SERVICE_ARM_JOINT_COUNT],
                                uint16_t move_time_ms);

/* Last accepted target, not the measured position. */
platform_err_t service_arm_get_targets(
                                platform_servo_angles_t *out);

/* Measured positions, load and fault flags from the bus. */
platform_err_t service_arm_get_states(platform_servo_states_t *out);

#endif /* __SERVICE_ARM_H__ */
