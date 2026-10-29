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

#include "service_chassis/service_chassis.h"
#include "service_arm/service_arm.h"
#include "service_proto/service_proto.h"

#include "platform_bsp_led.h"

//******************************** Variables ********************************//

static uint32_t        s_odom_ms;
static uint32_t        s_wheel_ms;
static uint32_t        s_arm_ms;
static uint32_t        s_status_ms;
static uint32_t        s_heartbeat_ms;
static uint16_t        s_heartbeat;
static uint16_t        s_jitter_max_us;
static platform_bool_t s_led_on;

//******************************** Functions ********************************//

void app_report_init(void)
{
    s_odom_ms = 0u;
    s_wheel_ms = 0u;
    s_arm_ms = 0u;
    s_status_ms = 0u;
    s_heartbeat_ms = 0u;
    s_heartbeat = 0u;
    s_jitter_max_us = 0u;
    s_led_on = PLATFORM_FALSE;
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
        app_report_wheel_speed();
    }

    if ((now_ms - s_arm_ms) >= APP_ARM_PERIOD_MS)
    {
        s_arm_ms = now_ms;
        app_report_arm();
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
}
