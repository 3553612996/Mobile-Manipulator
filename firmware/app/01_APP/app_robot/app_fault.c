/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_fault.c
 *
 * @par dependencies
 * - app_fault.h
 * - app_state.h
 * - app_battery.h
 * - service_chassis.h, service_arm.h, service_proto.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement fault arbitration and the degradation policy.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "app_robot/app_fault.h"
#include "app_robot/app_state.h"
#include "app_robot/app_battery.h"

#include "service_chassis/service_chassis.h"
#include "service_arm/service_arm.h"
#include "service_proto/service_proto.h"

//******************************** Variables ********************************//

static uint16_t           s_fault_code;
static platform_bool_t    s_prev_timed_out;
static platform_bool_t    s_crc_alarm_seen;

//******************************** Functions ********************************//

static void app_fault_stop_scope(app_fault_scope_t scope, uint32_t now_ms)
{
    if ((scope == APP_FAULT_SCOPE_CHASSIS) || (scope == APP_FAULT_SCOPE_ALL))
    {
        (void)service_chassis_enable(PLATFORM_FALSE, now_ms);
    }
    else if (scope == APP_FAULT_SCOPE_ARM)
    {
        /*
         * An arm fault must not leave the chassis driving on its last
         * velocity target. It is not the failed subsystem, so keep the
         * bridges enabled and let the speed loops brake to zero.
         */
        service_chassis_stop();
    }
    else
    {
        /* No scope (WARN) or unknown: nothing to stop. */
    }

    if ((scope == APP_FAULT_SCOPE_ARM) || (scope == APP_FAULT_SCOPE_ALL))
    {
        (void)service_arm_enable(PLATFORM_FALSE);
    }
}

void app_fault_init(void)
{
    s_fault_code = 0u;
    s_prev_timed_out = PLATFORM_FALSE;
    s_crc_alarm_seen = PLATFORM_FALSE;
}

uint16_t app_fault_code(void)
{
    return s_fault_code;
}

app_fault_scope_t app_fault_scope_get(uint16_t code)
{
    switch (code)
    {
        case PROTOCOL_FAULT_OVERCURRENT:
        case PROTOCOL_FAULT_ENCODER:
            return APP_FAULT_SCOPE_CHASSIS;

        case PROTOCOL_FAULT_JOINT_LIMIT:
        case PROTOCOL_FAULT_JOINT_STALL:
        case PROTOCOL_FAULT_TRAJECTORY:
            return APP_FAULT_SCOPE_ARM;

        case PROTOCOL_FAULT_CRC_LIMIT:
        case PROTOCOL_FAULT_BATTERY_LOW:
        case PROTOCOL_FAULT_OVERTEMP:
            return APP_FAULT_SCOPE_NONE;

        case PROTOCOL_FAULT_COMM_TIMEOUT:
        case PROTOCOL_FAULT_ESTOP:
            return APP_FAULT_SCOPE_ALL;

        default:
            /* An unknown code is treated as a full stop, not a warning. */
            return APP_FAULT_SCOPE_ALL;
    }
}

platform_bool_t app_fault_raise(uint16_t code, uint32_t now_ms,
                                platform_bool_t force)
{
    protocol_severity_t severity;
    app_fault_scope_t scope;

    if (code == 0u)
    {
        return PLATFORM_FALSE;
    }

    severity = protocol_fault_severity(code);

    if ((force == PLATFORM_FALSE) && (s_fault_code != 0u) &&
        (s_fault_code != code) &&
        (severity <= protocol_fault_severity(s_fault_code)))
    {
        /*
         * Only one code is visible in REPORT_STATUS. A higher severity
         * replaces the latch; an equal one keeps the first arrival so a
         * repeating condition does not spam EVENT frames.
         */
        return PLATFORM_FALSE;
    }

    s_fault_code = code;

    if (severity == PROTOCOL_SEVERITY_FATAL)
    {
        service_chassis_emergency_stop();
        (void)service_arm_enable(PLATFORM_FALSE);
        app_state_on_estop();
    }
    else if (severity == PROTOCOL_SEVERITY_ERROR)
    {
        scope = app_fault_scope_get(code);
        app_fault_stop_scope(scope, now_ms);
        app_state_on_fault();
    }
    else
    {
        /* WARN: report only. The robot keeps doing whatever it was doing. */
    }

    (void)service_proto_send_event(code, now_ms);

    return PLATFORM_TRUE;
}

protocol_result_t app_fault_clear(uint16_t code, uint32_t now_ms)
{
    uint16_t cleared;

    if ((code != PROTOCOL_CLEAR_ALL_FAULTS) &&
        (code != s_fault_code) &&
        (code != PROTOCOL_FAULT_ESTOP))
    {
        return PROTOCOL_RESULT_STATE;
    }

    /*
     * docs/architecture/protocol.md section 8: refuse to clear while the
     * condition is still present. The pack voltage is the one condition this
     * firmware can measure; the others are latches by nature.
     */
    if (((code == PROTOCOL_FAULT_BATTERY_LOW) ||
         ((code == PROTOCOL_CLEAR_ALL_FAULTS) &&
          (s_fault_code == PROTOCOL_FAULT_BATTERY_LOW))) &&
        (app_battery_low() == PLATFORM_TRUE))
    {
        return PROTOCOL_RESULT_STATE;
    }

    /* Report what was actually latched, not what the host asked to clear. */
    cleared = s_fault_code;

    /*
     * CLEAR_FAULT releases the software latch. It never re-enables: the host
     * must send SET_ENABLE again before the chassis or arm can move.
     */
    (void)service_chassis_clear_estop();
    s_fault_code = 0u;
    app_state_on_cleared();

    if (cleared != 0u)
    {
        (void)service_proto_send_event(cleared, now_ms);
    }

    return PROTOCOL_RESULT_OK;
}

void app_fault_poll(uint32_t now_ms)
{
    platform_bool_t timed_out;

    timed_out = service_chassis_cmd_timed_out();

    if ((timed_out == PLATFORM_TRUE) &&
        (s_prev_timed_out == PLATFORM_FALSE) &&
        (s_fault_code != PROTOCOL_FAULT_ESTOP))
    {
        (void)app_fault_raise(PROTOCOL_FAULT_COMM_TIMEOUT, now_ms,
                              PLATFORM_FALSE);
    }

    s_prev_timed_out = timed_out;

    if ((service_proto_crc_alarm() == PLATFORM_TRUE) &&
        (s_crc_alarm_seen == PLATFORM_FALSE))
    {
        s_crc_alarm_seen = PLATFORM_TRUE;

        if (s_fault_code == 0u)
        {
            (void)app_fault_raise(PROTOCOL_FAULT_CRC_LIMIT, now_ms,
                                  PLATFORM_FALSE);
        }
    }
}
