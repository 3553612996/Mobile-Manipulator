/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_bsp_servo.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform six-axis servo-bus device interface.
 *
 * The concrete bus (half-duplex UART, RS-485, CAN or a dedicated servo
 * controller) is hidden behind platform_servo_ops_t. Angles are always
 * 0.01 degree, matching the host protocol and avoiding float on the wire.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_BSP_SERVO_H__
#define __PLATFORM_BSP_SERVO_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"

//******************************** Defines **********************************//

#define PLATFORM_SERVO_JOINT_COUNT  6u

/* Marker for an unmeasured load/temperature value. */
#define PLATFORM_SERVO_VALUE_INVALID    ((int16_t)-32768)

//******************************** Declaring ********************************//

typedef struct
{
    int16_t joint_centi_deg[PLATFORM_SERVO_JOINT_COUNT];
} platform_servo_angles_t;

typedef struct
{
    int16_t         angle_centi_deg;
    int16_t         load_permille;      /* PLATFORM_SERVO_VALUE_INVALID if N/A. */
    int16_t         temperature_c;      /* PLATFORM_SERVO_VALUE_INVALID if N/A. */
    platform_bool_t fault;
} platform_servo_state_t;

typedef struct
{
    platform_servo_state_t joint[PLATFORM_SERVO_JOINT_COUNT];
} platform_servo_states_t;

typedef struct
{
    platform_err_t (*init)(void *context);
    platform_err_t (*deinit)(void *context);
    platform_err_t (*set_enable)(void *context, platform_bool_t enable);
    platform_err_t (*set_angles)(void *context,
                                 const platform_servo_angles_t *angles,
                                 uint16_t move_time_ms);
    platform_err_t (*get_states)(void *context,
                                 platform_servo_states_t *states);
} platform_servo_ops_t;

/*
 * Register the servo-bus implementation. Must be called before
 * platform_servo_init(). Only one servo bus is assumed for this robot.
 */
platform_err_t platform_servo_register(const platform_servo_ops_t *ops,
                                       void *context);

platform_err_t platform_servo_init(void);
platform_err_t platform_servo_deinit(void);

platform_err_t platform_servo_set_enable(platform_bool_t enable);

/*
 * Send all six joint targets. move_time_ms == 0 means "as fast as the bus and
 * the servo allow"; a non-zero value asks the servos to interpolate.
 */
platform_err_t platform_servo_set_angles(
                                        const platform_servo_angles_t *angles,
                                        uint16_t move_time_ms);

platform_err_t platform_servo_get_states(platform_servo_states_t *states);

#endif /* __PLATFORM_BSP_SERVO_H__ */
