/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_bsp_motor.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform motor + encoder device interface.
 *
 * This is a device contract, not a chip driver. 04_Impl adapts a concrete
 * motor driver (PWM + direction GPIO + timer encoder + ADC current sense) to
 * platform_motor_ops_t and registers it. Upper layers only see this header.
 *
 * Design note: this interface exposes signed duty and encoder counts. The
 * speed PID lives in the control module, not here. Keeping the BSP free of
 * control logic makes the control loop testable and lets the same motor
 * interface drive different hardware.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_BSP_MOTOR_H__
#define __PLATFORM_BSP_MOTOR_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"

//******************************** Defines **********************************//

#define PLATFORM_MOTOR_ID_0         ((platform_motor_id_t)0u)
#define PLATFORM_MOTOR_ID_1         ((platform_motor_id_t)1u)
#define PLATFORM_MOTOR_ID_2         ((platform_motor_id_t)2u)
#define PLATFORM_MOTOR_ID_3         ((platform_motor_id_t)3u)
#define PLATFORM_MOTOR_ID_MAX       4u

/*
 * Signed permille. The sign selects direction; the magnitude selects PWM.
 * +1000 means full forward, -1000 full reverse, 0 coast/stop according to the
 * driver's brake policy.
 */
#define PLATFORM_MOTOR_DUTY_PERMILLE_MAX    ((int16_t)1000)
#define PLATFORM_MOTOR_DUTY_PERMILLE_MIN    ((int16_t)-1000)

/* Markers for fields the concrete driver cannot measure. */
#define PLATFORM_MOTOR_VALUE_INVALID        ((int32_t)0x7FFFFFFF)
#define PLATFORM_MOTOR_TEMPERATURE_INVALID  ((int16_t)-32768)

//******************************** Declaring ********************************//

typedef uint8_t platform_motor_id_t;

typedef struct
{
    int32_t         encoder_count;      /* Signed cumulative count. */
    int32_t         speed_mm_s;         /* Measured speed, if supported. */
    int32_t         current_ma;         /* PLATFORM_MOTOR_VALUE_INVALID if N/A. */
    int16_t         temperature_c;      /* PLATFORM_MOTOR_TEMPERATURE_INVALID if N/A. */
    platform_bool_t enabled;
    platform_bool_t fault;
} platform_motor_state_t;

/*
 * context is the driver instance owned by 04_Impl. The ops table itself is
 * usually a const static object; the mutable state stays in context.
 */
typedef struct
{
    platform_err_t (*init)(void *context);
    platform_err_t (*deinit)(void *context);
    platform_err_t (*set_enable)(void *context, platform_bool_t enable);
    platform_err_t (*set_duty)(void *context, int16_t duty_permille);
    platform_err_t (*get_encoder)(void *context, int32_t *encoder_count);
    platform_err_t (*get_state)(void *context, platform_motor_state_t *state);
} platform_motor_ops_t;

/*
 * Register a motor implementation. Must be called before platform_motor_init().
 * Registering the same id twice returns PLATFORM_ERR_ALREADY_INIT.
 */
platform_err_t platform_motor_register(platform_motor_id_t id,
                                       const platform_motor_ops_t *ops,
                                       void *context);

platform_err_t platform_motor_init(platform_motor_id_t id);
platform_err_t platform_motor_deinit(platform_motor_id_t id);

platform_err_t platform_motor_set_enable(platform_motor_id_t id,
                                         platform_bool_t enable);
platform_err_t platform_motor_set_duty(platform_motor_id_t id,
                                       int16_t duty_permille);

/* Fast path for the 1 ms control loop: no state struct copy. */
platform_err_t platform_motor_get_encoder(platform_motor_id_t id,
                                          int32_t *encoder_count);
platform_err_t platform_motor_get_state(platform_motor_id_t id,
                                        platform_motor_state_t *state);

/*
 * Safety helper: set duty to zero for every initialized motor. This is safe
 * from task context; the hard emergency stop still belongs in the ESTOP ISR
 * path and must not depend on the scheduler.
 */
platform_err_t platform_motor_stop_all(void);

#endif /* __PLATFORM_BSP_MOTOR_H__ */
