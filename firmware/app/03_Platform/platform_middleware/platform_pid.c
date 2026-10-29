/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_pid.c
 *
 * @par dependencies
 * - platform_pid.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement the positional PID controller.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "platform_pid.h"

//******************************** Functions ********************************//

platform_err_t platform_pid_init(platform_pid_t *pid,
                                 const platform_pid_cfg_t *cfg)
{
    if ((pid == NULL) || (cfg == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    if ((cfg->kp < 0.0f) || (cfg->ki < 0.0f) || (cfg->kd < 0.0f) ||
        (cfg->output_min >= cfg->output_max))
    {
        return PLATFORM_ERR_PARAM;
    }

    pid->cfg = *cfg;
    platform_pid_reset(pid);
    pid->initialized = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

void platform_pid_reset(platform_pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->output = 0.0f;
    pid->started = PLATFORM_FALSE;
}

float_t platform_pid_update(platform_pid_t *pid,
                            float_t setpoint,
                            float_t measurement,
                            float_t dt_s)
{
    float_t error;
    float_t p_term;
    float_t i_term;
    float_t d_term;
    float_t candidate_integral;
    float_t output;

    if ((pid == NULL) || (pid->initialized == PLATFORM_FALSE) ||
        (dt_s <= 0.0f))
    {
        return 0.0f;
    }

    error = setpoint - measurement;
    p_term = pid->cfg.kp * error;

    if (pid->cfg.ki > 0.0f)
    {
        candidate_integral = pid->integral + (error * dt_s);
        i_term = pid->cfg.ki * candidate_integral;
    }
    else
    {
        /* No integral action: keep the state at zero instead of letting it
         * grow forever with no effect on the output. */
        candidate_integral = 0.0f;
        i_term = 0.0f;
    }

    if (pid->started == PLATFORM_FALSE)
    {
        /* No previous measurement: a derivative here would be a startup
         * kick, not real motion. */
        d_term = 0.0f;
    }
    else
    {
        d_term = -pid->cfg.kd *
                 (measurement - pid->prev_measurement) / dt_s;
    }

    output = p_term + i_term + d_term;

    if (output > pid->cfg.output_max)
    {
        output = pid->cfg.output_max;

        if (error > 0.0f)
        {
            candidate_integral = pid->integral;
        }
    }
    else if (output < pid->cfg.output_min)
    {
        output = pid->cfg.output_min;

        if (error < 0.0f)
        {
            candidate_integral = pid->integral;
        }
    }
    else
    {
        /* Within range: keep the new integral. */
    }

    pid->integral = candidate_integral;
    pid->prev_measurement = measurement;
    pid->output = output;
    pid->started = PLATFORM_TRUE;

    return output;
}

float_t platform_pid_get_output(const platform_pid_t *pid)
{
    if ((pid == NULL) || (pid->initialized == PLATFORM_FALSE))
    {
        return 0.0f;
    }

    return pid->output;
}
