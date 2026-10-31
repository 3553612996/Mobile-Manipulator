/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file service_arm.c
 *
 * @par dependencies
 * - service_arm.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement the six-joint arm service.
 *
 * @version V2.0 2026-10-31
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "service_arm.h"

#include <string.h>

//******************************** Variables ********************************//

static struct
{
    platform_bool_t         initialized;
    platform_bool_t         enabled;
    platform_bool_t         move_pending;
    platform_bool_t         has_target;     /* Host has commanded a pose. */
    service_arm_cfg_t       cfg;
    platform_servo_angles_t target;

    uint32_t                settle_until_ms;
    uint16_t                last_move_time_ms;

    /* Latched by service_arm_update(); cleared by CLEAR_FAULT. */
    uint8_t                 fault_bits[SERVICE_ARM_JOINT_COUNT];
    uint32_t                stall_since_ms[SERVICE_ARM_JOINT_COUNT];
    platform_bool_t         stall_timer[SERVICE_ARM_JOINT_COUNT];
} s_arm;

//******************************** Functions ********************************//

static platform_bool_t arm_is_settled(uint32_t now_ms)
{
    return ((uint32_t)(now_ms - s_arm.settle_until_ms) < 0x80000000u)
               ? PLATFORM_TRUE
               : PLATFORM_FALSE;
}

static platform_bool_t arm_protection_ok(
                                const service_arm_protection_t *protection)
{
    if ((protection->overtemp_c < -40) || (protection->overtemp_c > 150))
    {
        return PLATFORM_FALSE;
    }

    if ((protection->undervolt_mv < 4000u) ||
        (protection->undervolt_mv >= protection->overvolt_mv) ||
        (protection->overvolt_mv > 9000u))
    {
        return PLATFORM_FALSE;
    }

    if ((protection->stall_error_centi_deg <= 0) ||
        (protection->stall_error_centi_deg > 4500))
    {
        return PLATFORM_FALSE;
    }

    if ((protection->stall_time_ms < 100u) ||
        (protection->stall_time_ms > 5000u))
    {
        return PLATFORM_FALSE;
    }

    if (protection->stall_margin_ms > 2000u)
    {
        return PLATFORM_FALSE;
    }

    return PLATFORM_TRUE;
}

platform_err_t service_arm_init(const service_arm_cfg_t *cfg)
{
    uint8_t i;

    if (cfg == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (arm_protection_ok(&cfg->protection) == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_PARAM;
    }

    for (i = 0u; i < SERVICE_ARM_JOINT_COUNT; i++)
    {
        if (cfg->min_centi_deg[i] > cfg->max_centi_deg[i])
        {
            return PLATFORM_ERR_PARAM;
        }

        if ((cfg->home_centi_deg[i] < cfg->min_centi_deg[i]) ||
            (cfg->home_centi_deg[i] > cfg->max_centi_deg[i]))
        {
            return PLATFORM_ERR_PARAM;
        }
    }

    (void)memset(&s_arm, 0, sizeof(s_arm));

    s_arm.cfg = *cfg;

    if (s_arm.cfg.default_move_time_ms == 0u)
    {
        s_arm.cfg.default_move_time_ms = SERVICE_ARM_MOVE_TIME_MS;
    }

    for (i = 0u; i < SERVICE_ARM_JOINT_COUNT; i++)
    {
        s_arm.target.joint_centi_deg[i] = cfg->home_centi_deg[i];
    }

    /*
     * Start disabled: the bus servos need a restore-torque frame, and the app
     * sends it only after the UART and the persisted joint map are in place.
     */
    s_arm.enabled = PLATFORM_FALSE;

    /* Do not let the first control tick judge a move nobody requested. */
    s_arm.last_move_time_ms = s_arm.cfg.default_move_time_ms;
    s_arm.move_pending = PLATFORM_TRUE;

    s_arm.initialized = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t service_arm_get_protection(service_arm_protection_t *out)
{
    if (out == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (s_arm.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    *out = s_arm.cfg.protection;

    return PLATFORM_ERR_OK;
}

platform_err_t service_arm_set_protection(
                                const service_arm_protection_t *protection)
{
    if (protection == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (s_arm.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (arm_protection_ok(protection) == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_PARAM;
    }

    s_arm.cfg.protection = *protection;

    return PLATFORM_ERR_OK;
}

platform_err_t service_arm_enable(platform_bool_t enable)
{
    platform_err_t err;

    if (s_arm.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if ((enable == PLATFORM_TRUE) &&
        (service_arm_has_fault() == PLATFORM_TRUE))
    {
        /* Clearing the fault is the only way back; see the header. */
        return PLATFORM_ERR_FAIL;
    }

    err = platform_servo_set_enable(enable);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    s_arm.enabled = enable;

    if (enable == PLATFORM_TRUE)
    {
        s_arm.last_move_time_ms = s_arm.cfg.default_move_time_ms;
        s_arm.move_pending = PLATFORM_TRUE;
    }

    return PLATFORM_ERR_OK;
}

platform_bool_t service_arm_is_enabled(void)
{
    return s_arm.enabled;
}

platform_err_t service_arm_set_joints(const int16_t joint_centi_deg
                                          [SERVICE_ARM_JOINT_COUNT],
                                      uint16_t move_time_ms)
{
    platform_servo_angles_t target;
    uint8_t i;

    if (s_arm.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (joint_centi_deg == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (s_arm.enabled == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_FAIL;
    }

    /* Validate the whole frame before touching any joint. */
    for (i = 0u; i < SERVICE_ARM_JOINT_COUNT; i++)
    {
        if ((joint_centi_deg[i] < s_arm.cfg.min_centi_deg[i]) ||
            (joint_centi_deg[i] > s_arm.cfg.max_centi_deg[i]))
        {
            return PLATFORM_ERR_PARAM;
        }

        target.joint_centi_deg[i] = joint_centi_deg[i];
    }

    if (move_time_ms == 0u)
    {
        move_time_ms = s_arm.cfg.default_move_time_ms;
    }

    if (PLATFORM_IS_ERR(platform_servo_set_angles(&target, move_time_ms)))
    {
        return PLATFORM_ERR_FAIL;
    }

    s_arm.target = target;
    s_arm.last_move_time_ms = move_time_ms;
    s_arm.move_pending = PLATFORM_TRUE;
    s_arm.has_target = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t service_arm_poll(uint32_t now_ms)
{
    if (s_arm.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return platform_servo_poll(now_ms);
}

platform_err_t service_arm_update(uint32_t now_ms)
{
    platform_servo_states_t states;
    platform_bool_t stop = PLATFORM_FALSE;
    int16_t error;
    uint8_t bits;
    uint8_t i;

    if (s_arm.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (s_arm.move_pending == PLATFORM_TRUE)
    {
        s_arm.move_pending = PLATFORM_FALSE;
        s_arm.settle_until_ms = now_ms + (uint32_t)s_arm.last_move_time_ms +
                                (uint32_t)s_arm.cfg.protection.stall_margin_ms;
    }

    if (PLATFORM_IS_ERR(platform_servo_get_states(&states)))
    {
        return PLATFORM_ERR_FAIL;
    }

    for (i = 0u; i < SERVICE_ARM_JOINT_COUNT; i++)
    {
        bits = states.joint[i].fault_bits;

        if ((s_arm.enabled == PLATFORM_TRUE) &&
            (states.joint[i].online == PLATFORM_TRUE))
        {
            if ((states.joint[i].temperature_c !=
                 PLATFORM_SERVO_TEMP_INVALID) &&
                (states.joint[i].temperature_c >=
                 s_arm.cfg.protection.overtemp_c))
            {
                bits |= SERVICE_ARM_FAULT_OVERTEMP;
            }

            if (states.joint[i].voltage_mv !=
                PLATFORM_SERVO_VOLTAGE_INVALID)
            {
                if (states.joint[i].voltage_mv <
                    s_arm.cfg.protection.undervolt_mv)
                {
                    bits |= SERVICE_ARM_FAULT_UNDERVOLT;
                }
                else if (states.joint[i].voltage_mv >
                         s_arm.cfg.protection.overvolt_mv)
                {
                    bits |= SERVICE_ARM_FAULT_OVERVOLT;
                }
            }

            /*
             * A stall is only meaningful against a commanded pose. Between
             * boot and the first SET_ARM_JOINTS the servos hold where they
             * are, which need not be anywhere near the home default.
             */
            if ((s_arm.has_target == PLATFORM_TRUE) &&
                (arm_is_settled(now_ms) == PLATFORM_TRUE))
            {
                error = (int16_t)(s_arm.target.joint_centi_deg[i] -
                                  states.joint[i].angle_centi_deg);

                if (error < 0)
                {
                    error = (int16_t)-error;
                }

                if (error > s_arm.cfg.protection.stall_error_centi_deg)
                {
                    if (s_arm.stall_timer[i] == PLATFORM_FALSE)
                    {
                        s_arm.stall_timer[i] = PLATFORM_TRUE;
                        s_arm.stall_since_ms[i] = now_ms;
                    }
                    else if ((uint32_t)(now_ms - s_arm.stall_since_ms[i]) >=
                             (uint32_t)s_arm.cfg.protection.stall_time_ms)
                    {
                        bits |= SERVICE_ARM_FAULT_STALL;
                    }
                }
                else
                {
                    s_arm.stall_timer[i] = PLATFORM_FALSE;
                }
            }
            else
            {
                s_arm.stall_timer[i] = PLATFORM_FALSE;
            }

            /* Latch while powered; a disabled arm has nothing to protect. */
            s_arm.fault_bits[i] |= bits;
        }
        else
        {
            s_arm.stall_timer[i] = PLATFORM_FALSE;
        }

        if ((bits & (SERVICE_ARM_FAULT_OVERTEMP |
                     SERVICE_ARM_FAULT_UNDERVOLT |
                     SERVICE_ARM_FAULT_OVERVOLT |
                     SERVICE_ARM_FAULT_STALL)) != 0u)
        {
            stop = PLATFORM_TRUE;
        }
    }

    if ((stop == PLATFORM_TRUE) && (s_arm.enabled == PLATFORM_TRUE))
    {
        /* Drop torque first; the servo task sends the stop frame. */
        (void)platform_servo_set_enable(PLATFORM_FALSE);
        s_arm.enabled = PLATFORM_FALSE;
    }

    return PLATFORM_ERR_OK;
}

platform_err_t service_arm_get_status(service_arm_status_t *out)
{
    uint8_t i;

    if (out == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (s_arm.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    out->enabled = s_arm.enabled;
    out->faulted = service_arm_has_fault();

    for (i = 0u; i < SERVICE_ARM_JOINT_COUNT; i++)
    {
        out->fault_bits[i] = s_arm.fault_bits[i];
    }

    return PLATFORM_ERR_OK;
}

platform_bool_t service_arm_has_fault(void)
{
    uint8_t i;

    for (i = 0u; i < SERVICE_ARM_JOINT_COUNT; i++)
    {
        if (s_arm.fault_bits[i] != 0u)
        {
            return PLATFORM_TRUE;
        }
    }

    return PLATFORM_FALSE;
}

platform_err_t service_arm_clear_faults(void)
{
    uint8_t i;

    if (s_arm.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    for (i = 0u; i < SERVICE_ARM_JOINT_COUNT; i++)
    {
        s_arm.fault_bits[i] = 0u;
        s_arm.stall_timer[i] = PLATFORM_FALSE;
    }

    /*
     * The servo driver keeps its own OFFLINE/REPLY bits until the next poll
     * succeeds; clearing here only releases the application-side latches.
     */

    return PLATFORM_ERR_OK;
}

platform_err_t service_arm_get_targets(platform_servo_angles_t *out)
{
    if (out == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    *out = s_arm.target;

    return PLATFORM_ERR_OK;
}

platform_err_t service_arm_get_states(platform_servo_states_t *out)
{
    if (out == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (s_arm.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return platform_servo_get_states(out);
}

platform_err_t service_arm_config(const platform_servo_cfg_req_t *req)
{
    if (s_arm.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    /*
     * Changing an ID or a zero while the arm is powered and holding would move
     * the joints under the operator's hand; require it disabled.
     */
    if (s_arm.enabled == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_BUSY;
    }

    return platform_servo_config(req);
}

platform_bool_t service_arm_config_ready(void)
{
    if (s_arm.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_FALSE;
    }

    return platform_servo_config_ready();
}

platform_err_t service_arm_config_take(int32_t *out_value)
{
    if (s_arm.initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return platform_servo_config_take(out_value);
}

void service_arm_config_cancel(void)
{
    if (s_arm.initialized == PLATFORM_TRUE)
    {
        platform_servo_config_cancel();
    }
}
