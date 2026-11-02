/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_pid.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide a small positional PID controller.
 *
 * The controller is hardware-independent: it takes a setpoint, a measurement
 * and an elapsed time, and returns an output in the caller's units. The state
 * is caller-owned and nothing is allocated. One instance belongs to one task;
 * it is not thread-safe.
 *
 * Implementation choices:
 *   - derivative acts on the measurement, not on the error, so a setpoint
 *     step does not produce a derivative kick;
 *   - integration is conditional: while the output is saturated and the error
 *     still drives it further into the same rail, the integral is not updated
 *     (anti-windup);
 *   - the first update after init/reset skips the derivative, because there is
 *     no previous sample to difference against.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_PID_H__
#define __PLATFORM_PID_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"

//******************************** Declaring ********************************//

typedef struct
{
    float_t     kp;
    float_t     ki;
    float_t     kd;

    /* Output saturation, in the caller's units. Must satisfy min < max. */
    float_t     output_min;
    float_t     output_max;
} platform_pid_cfg_t;

typedef struct
{
    platform_pid_cfg_t  cfg;
    float_t             integral;           /* Error integrated over time. */
    float_t             prev_measurement;
    float_t             output;
    platform_bool_t     initialized;
    platform_bool_t     started;            /* At least one update has run. */
} platform_pid_t;

/*
 * Configure the controller and clear its state. Gains must be >= 0.
 * Re-initialising is allowed; it discards the integral.
 */
platform_err_t platform_pid_init(platform_pid_t *pid,
                                 const platform_pid_cfg_t *cfg);

/* Clear integral, output and the derivative history. Keeps the configuration. */
void platform_pid_reset(platform_pid_t *pid);

/*
 * Run one control step. dt_s is the time since the previous update, in
 * seconds, and must be > 0. Returns the saturated output. An uninitialised
 * controller or an invalid dt returns 0 without changing the state.
 */
float_t platform_pid_update(platform_pid_t *pid,
                            float_t setpoint,
                            float_t measurement,
                            float_t dt_s);

float_t platform_pid_get_output(const platform_pid_t *pid);

#endif /* __PLATFORM_PID_H__ */
