/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_report.c
 *
 * @par dependencies
 * - app_report.h
 * - app_robot_config.h
 * - app_state.h, app_fault.h, app_battery.h
 * - service_chassis.h, service_arm.h, service_proto.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement the periodic reports and the status LED.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "app_robot/app_report.h"
#include "app_robot/app_robot_config.h"
#include "app_robot/app_state.h"
#include "app_robot/app_fault.h"
#include "app_robot/app_battery.h"
#include "app_robot/app_params.h"

#include "service_chassis/service_chassis.h"
#include "service_arm/service_arm.h"
#include "service_proto/service_proto.h"

#include "platform_bsp_led.h"

//******************************** Variables ********************************//

static uint32_t        s_odom_ms;
static uint32_t        s_wheel_ms;
static uint32_t        s_arm_ms;
static uint32_t        s_arm_status_ms;
static uint32_t        s_status_ms;
static uint32_t        s_heartbeat_ms;
static uint16_t        s_heartbeat;
static uint16_t        s_jitter_max_us;
static platform_bool_t s_led_on;

/*
 * GET_PARAM dump cursor. One chunk leaves per comm tick, so a full dump takes
 * five ticks (48 records / 10 per chunk) without ever blocking the handler.
 */
static platform_bool_t s_param_active;
static uint16_t        s_param_next;
static uint8_t         s_param_chunk;

//******************************** Functions ********************************//

void app_report_init(void)
{
    s_odom_ms = 0u;
    s_wheel_ms = 0u;
    s_arm_ms = 0u;
    s_arm_status_ms = 0u;
    s_status_ms = 0u;
    s_heartbeat_ms = 0u;
    s_heartbeat = 0u;
    s_jitter_max_us = 0u;
    s_led_on = PLATFORM_FALSE;
    s_param_active = PLATFORM_FALSE;
    s_param_next = 0u;
    s_param_chunk = 0u;
    (void)platform_led_set(s_led_on);
}

static void app_report_odom(void)
{
    service_chassis_odometry_t odom;

    service_chassis_get_odometry(&odom);
    (void)service_proto_send_odom(odom.x_mm, odom.y_mm,
                                  odom.heading_centi_deg,
                                  odom.vl_mm_s, odom.vr_mm_s);
}

static void app_report_wheel_speed(void)
{
    int16_t wheel_mm_s[SERVICE_CHASSIS_WHEEL_COUNT];

    service_chassis_get_wheel_speeds(wheel_mm_s);
    (void)service_proto_send_wheel_speed(wheel_mm_s);
}

static void app_report_wheel_duty(void)
{
    int16_t wheel_permille[SERVICE_CHASSIS_WHEEL_COUNT];

    service_chassis_get_wheel_duty(wheel_permille);
    (void)service_proto_send_wheel_duty(wheel_permille);
}

static void app_report_arm(void)
{
    platform_servo_states_t servo_states;
    platform_servo_angles_t servo_targets;
    int16_t angles[PROTOCOL_ARM_JOINT_COUNT];
    uint8_t i;

    if (PLATFORM_IS_OK(service_arm_get_states(&servo_states)))
    {
        for (i = 0u; i < PROTOCOL_ARM_JOINT_COUNT; i++)
        {
            angles[i] = servo_states.joint[i].angle_centi_deg;
        }
    }
    else if (PLATFORM_IS_OK(service_arm_get_targets(&servo_targets)))
    {
        for (i = 0u; i < PROTOCOL_ARM_JOINT_COUNT; i++)
        {
            angles[i] = servo_targets.joint_centi_deg[i];
        }
    }
    else
    {
        for (i = 0u; i < PROTOCOL_ARM_JOINT_COUNT; i++)
        {
            angles[i] = 0;
        }
    }

    (void)service_proto_send_arm(angles, 0, 0u);
}

/*
 * REPORT_ARM_STATUS: the two REPORT_ARM frames in between carry only the
 * angles, so the per-joint temperature, fault bits and the rail voltage go
 * out at the slower rate. A temperature of -128 means "not measured yet",
 * which is what the host draws as a dash instead of 0 degrees.
 */
static void app_report_arm_status(void)
{
    service_arm_status_t arm;
    platform_servo_states_t states;
    int16_t angles[PROTOCOL_ARM_JOINT_COUNT];
    int8_t temperatures[PROTOCOL_ARM_JOINT_COUNT];
    uint8_t fault_bits[PROTOCOL_ARM_JOINT_COUNT];
    uint16_t rail_mv = 0u;
    uint8_t flags = 0u;
    uint8_t i;

    if (PLATFORM_IS_ERR(service_arm_get_status(&arm)))
    {
        return;
    }

    if (arm.enabled == PLATFORM_TRUE)
    {
        flags |= PROTOCOL_ARM_FLAG_ENABLED;
    }

    if (arm.faulted == PLATFORM_TRUE)
    {
        flags |= PROTOCOL_ARM_FLAG_FAULTED;
    }

    for (i = 0u; i < PROTOCOL_ARM_JOINT_COUNT; i++)
    {
        angles[i] = 0;
        temperatures[i] = -128;
        fault_bits[i] = arm.fault_bits[i];
    }

    if (PLATFORM_IS_OK(service_arm_get_states(&states)))
    {
        for (i = 0u; i < PROTOCOL_ARM_JOINT_COUNT; i++)
        {
            angles[i] = states.joint[i].angle_centi_deg;
            fault_bits[i] |= states.joint[i].fault_bits;

            if (states.joint[i].temperature_c != PLATFORM_SERVO_TEMP_INVALID)
            {
                temperatures[i] = (int8_t)states.joint[i].temperature_c;
            }

            if (states.joint[i].online == PLATFORM_FALSE)
            {
                flags |= PROTOCOL_ARM_FLAG_OFFLINE;
            }
            else if ((rail_mv == 0u) &&
                     (states.joint[i].voltage_mv !=
                      PLATFORM_SERVO_VOLTAGE_INVALID))
            {
                /* One rail feeds the chain: the first online reading is it. */
                rail_mv = states.joint[i].voltage_mv;
            }
        }
    }
    else
    {
        flags |= PROTOCOL_ARM_FLAG_OFFLINE;
    }

    (void)service_proto_send_arm_status(flags, rail_mv, angles, temperatures,
                                        fault_bits);
}

static void app_report_status(uint32_t now_ms)
{
    uint32_t jitter_us;
    uint16_t battery_mv;
    platform_bool_t enabled;

    /*
     * Measure jitter on this 100 ms deadline, not on the 10 ms loop: the loop
     * also wakes on every RX event, so its period is not a deadline.
     */
    jitter_us = ((now_ms - s_status_ms) > APP_STATUS_PERIOD_MS)
                    ? ((now_ms - s_status_ms - APP_STATUS_PERIOD_MS) * 1000u)
                    : 0u;

    if (jitter_us > s_jitter_max_us)
    {
        s_jitter_max_us = (uint16_t)((jitter_us > 65535u) ? 65535u
                                                          : jitter_us);
    }

    s_status_ms = now_ms;
    battery_mv = app_battery_mv();

    if ((app_battery_low() == PLATFORM_TRUE) &&
        (app_fault_code() == 0u))
    {
        /* WARN: this raises the latch and sends EVENT, but does not stop. */
        (void)app_fault_raise(PROTOCOL_FAULT_BATTERY_LOW, now_ms,
                              PLATFORM_FALSE);
    }

    /* No board temperature sensor yet: report 0, not a fake reading. */
    (void)service_proto_send_status(app_state_protocol(), app_fault_code(),
                                    battery_mv, (int8_t)0,
                                    s_jitter_max_us);

    s_jitter_max_us = 0u;

    enabled = service_chassis_is_enabled();

    if (enabled != s_led_on)
    {
        s_led_on = enabled;
        (void)platform_led_set(s_led_on);
    }
}

/*
 * One record of the GET_PARAM dump, in a fixed order grouped by kind.
 * Returns PLATFORM_FALSE past the end.
 *
 * The values are read live from app_params (RAM), not from the flash blob:
 * after a SET_PARAM that has not been saved yet, the host must see what the
 * board is actually running. Whether that value is on flash is REPORT_STORAGE
 * bit0's job, not this report's.
 */
static platform_bool_t app_report_param_record(uint16_t index,
                                               protocol_param_record_t *record)
{
    uint16_t local;
    uint8_t wheel;

    if (record == NULL)
    {
        return PLATFORM_FALSE;
    }

    record->value = 0;

    /* 0..15: parameter area, ids are the SET_PARAM ids. */
    if (index < 3u)
    {
        record->kind = PROTOCOL_PARAM_KIND_PARAM;
        record->id = (uint8_t)(PROTOCOL_PARAM_CMD_TIMEOUT + index);

        switch (index)
        {
            case 0u:
                record->value = (int32_t)app_params_cmd_timeout_ms();
                break;

            case 1u:
                record->value = (int32_t)app_params_max_linear_mm_s();
                break;

            default:
                record->value = (int32_t)app_params_max_angular_mrad_s();
                break;
        }

        return PLATFORM_TRUE;
    }

    if (index < 15u)
    {
        local = (uint16_t)(index - 3u);
        record->kind = PROTOCOL_PARAM_KIND_PARAM;
        record->id = (uint8_t)(PROTOCOL_PARAM_PID_A_KP + local);
        wheel = (uint8_t)(local / 3u);
        record->value = app_params_pid_gain_x1000(
                            wheel, (uint8_t)(local % 3u));
        return PLATFORM_TRUE;
    }

    if (index == 15u)
    {
        record->kind = PROTOCOL_PARAM_KIND_PARAM;
        record->id = PROTOCOL_PARAM_LOG_LEVEL;
        record->value = (int32_t)app_params_log_level();
        return PLATFORM_TRUE;
    }

    /* 16..33: calibration area. 0x02 is reported in mm like SET_PARAM. */
    if (index < 34u)
    {
        local = (uint16_t)(index - 16u);
        record->kind = PROTOCOL_PARAM_KIND_CALIB;

        if (local < 2u)
        {
            if (local == 0u)
            {
                record->id = PROTOCOL_PARAM_WHEEL_BASE;
                record->value = app_params_track_mm();
            }
            else
            {
                record->id = PROTOCOL_PARAM_WHEEL_DIAMETER;
                record->value = app_params_wheel_diameter_x10_mm() / 10;
            }
        }
        else if (local < 6u)
        {
            wheel = (uint8_t)(local - 2u);
            record->id = (uint8_t)(PROTOCOL_PARAM_COUNTS_PER_MM_A + wheel);
            record->value = app_params_wheel_counts_per_mm_x1000(wheel);
        }
        else if (local < 10u)
        {
            wheel = (uint8_t)(local - 6u);
            record->id = (uint8_t)(PROTOCOL_PARAM_DUTY_FWD_A + wheel);
            record->value = (int32_t)app_params_deadzone_fwd_permille(wheel);
        }
        else if (local < 14u)
        {
            wheel = (uint8_t)(local - 10u);
            record->id = (uint8_t)(PROTOCOL_PARAM_DUTY_REV_A + wheel);
            record->value = (int32_t)app_params_deadzone_rev_permille(wheel);
        }
        else
        {
            wheel = (uint8_t)(local - 14u);
            record->id = (uint8_t)(PROTOCOL_PARAM_DIRECTION_A + wheel);
            record->value = (int32_t)app_params_wheel_dir(wheel);
        }

        return PLATFORM_TRUE;
    }

    /* 34..39 servo address, 40..45 calibrated zero (0.01 degree). */
    if (index < 40u)
    {
        record->kind = PROTOCOL_PARAM_KIND_SERVO_ID;
        record->id = (uint8_t)(index - 34u);
        record->value = (int32_t)app_params_servo_id(record->id);
        return PLATFORM_TRUE;
    }

    if (index < 46u)
    {
        record->kind = PROTOCOL_PARAM_KIND_SERVO_OFFSET;
        record->id = (uint8_t)(index - 40u);
        record->value =
            (int32_t)app_params_servo_offset_centi_deg(record->id);
        return PLATFORM_TRUE;
    }

    if (index == 46u)
    {
        record->kind = PROTOCOL_PARAM_KIND_INFO;
        record->id = PROTOCOL_PARAM_INFO_WHEELBASE_MM;
        record->value = app_params_wheelbase_mm();
        return PLATFORM_TRUE;
    }

    if (index == 47u)
    {
        record->kind = PROTOCOL_PARAM_KIND_INFO;
        record->id = PROTOCOL_PARAM_INFO_STALL_MARGIN_MS;
        record->value = (int32_t)app_params_arm_stall_margin_ms();
        return PLATFORM_TRUE;
    }

    return PLATFORM_FALSE;
}

void app_report_request_param_dump(void)
{
    s_param_next = 0u;
    s_param_chunk = 0u;
    s_param_active = PLATFORM_TRUE;
}

static void app_report_param_dump(void)
{
    protocol_param_record_t records[PROTOCOL_PARAM_RECORD_MAX];
    protocol_param_record_t probe;
    uint8_t count = 0u;
    uint8_t flags = 0u;

    if (s_param_active == PLATFORM_FALSE)
    {
        return;
    }

    while ((count < PROTOCOL_PARAM_RECORD_MAX) &&
           (app_report_param_record(s_param_next, &records[count]) ==
            PLATFORM_TRUE))
    {
        count++;
        s_param_next++;
    }

    if (app_report_param_record(s_param_next, &probe) == PLATFORM_FALSE)
    {
        flags = PROTOCOL_PARAM_FLAG_LAST;
        s_param_active = PLATFORM_FALSE;
    }

    if (PLATFORM_IS_ERR(
            service_proto_send_param(s_param_chunk, flags, records, count)))
    {
        /* A partial frame is already on the wire; let the host time out. */
        s_param_active = PLATFORM_FALSE;
        return;
    }

    s_param_chunk++;
}

void app_report_tick(uint32_t now_ms)
{
    if ((now_ms - s_odom_ms) >= APP_ODOM_PERIOD_MS)
    {
        s_odom_ms = now_ms;
        app_report_odom();
    }

    if ((now_ms - s_wheel_ms) >= APP_WHEEL_SPEED_PERIOD_MS)
    {
        s_wheel_ms = now_ms;

        /*
         * Speed and duty ride the same 20 ms deadline and are always sent as a
         * pair: the bench tool pairs them by arrival order, and a duty that
         * belonged to a different tick would blur the dead-zone measurement.
         */
        app_report_wheel_speed();
        app_report_wheel_duty();
    }

    if ((now_ms - s_arm_ms) >= APP_ARM_PERIOD_MS)
    {
        s_arm_ms = now_ms;
        app_report_arm();
    }

    if ((now_ms - s_arm_status_ms) >= APP_ARM_STATUS_PERIOD_MS)
    {
        s_arm_status_ms = now_ms;
        app_report_arm_status();
    }

    if ((now_ms - s_status_ms) >= APP_STATUS_PERIOD_MS)
    {
        app_report_status(now_ms);
    }

    if ((now_ms - s_heartbeat_ms) >= APP_HEARTBEAT_PERIOD_MS)
    {
        s_heartbeat_ms = now_ms;
        s_heartbeat++;
        (void)service_proto_send_heartbeat(app_state_protocol(), s_heartbeat);
    }

    /* GET_PARAM is event-driven, not periodic: the cursor decides. */
    app_report_param_dump();
}
