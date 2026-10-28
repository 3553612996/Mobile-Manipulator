/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_robot.c
 *
 * @par dependencies
 * - app_robot/app_robot.h
 * - app_robot/app_robot_config.h
 * - service_chassis.h, service_arm.h, service_key.h, service_proto.h
 * - platform_os_task.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement the robot application: task wiring, handlers, reports.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "app_robot/app_robot.h"
#include "app_robot/app_robot_config.h"

#include "service_chassis/service_chassis.h"
#include "service_arm/service_arm.h"
#include "service_key/service_key.h"
#include "service_proto/service_proto.h"

#include "platform_os_task.h"
#include "platform_os_time.h"
#include "platform_bsp_battery.h"
#include "platform_bsp_led.h"
#include "platform_bsp_buzzer.h"

//******************************** Variables ********************************//

/*
 * Written by the default task before the UART event is hooked, then read from
 * that ISR. The hook registration is a call barrier between the two, so the
 * handle is visible before any notification can reach the callback.
 */
static platform_os_task_t s_comm_task;
static platform_os_task_t s_control_task;

static uint16_t           s_fault_code;
static volatile platform_bool_t s_pending_estop;
static volatile uint32_t  s_beep_until_ms;
static platform_bool_t    s_manual_forward;

static platform_bool_t    s_prev_timed_out;
static platform_bool_t    s_crc_alarm_seen;

static uint16_t           s_jitter_max_us;
static platform_bool_t    s_led_on;
static platform_bool_t    s_buzzer_on;

//******************************** Functions ********************************//

static uint32_t app_now_ms(void)
{
    return platform_os_ticks_to_ms(platform_os_tick_get());
}

static void app_raise_fault(uint16_t code, uint32_t now_ms,
                            platform_bool_t force)
{
    if ((force == PLATFORM_FALSE) && (s_fault_code != 0u) &&
        (s_fault_code != code) &&
        (protocol_fault_severity(code) <=
         protocol_fault_severity(s_fault_code)))
    {
        /*
         * Only one code is visible in REPORT_STATUS, so the worst one wins: a
         * FATAL escalates over a latched WARN, but a WARN that arrives after a
         * FATAL does not hide it. Equal severity keeps the first latch.
         */
        return;
    }

    s_fault_code = code;
    (void)service_proto_send_event(code, now_ms);
}

/* Pack voltage in mV, or 0 while the monitor has no valid sample yet. */
static uint16_t app_battery_mv(void)
{
    platform_battery_state_t battery;

    if ((platform_battery_read(&battery) != PLATFORM_ERR_OK) ||
        (battery.valid != PLATFORM_TRUE))
    {
        return 0u;
    }

    return (battery.voltage_mv > 0xFFFFu) ? (uint16_t)0xFFFFu
                                          : (uint16_t)battery.voltage_mv;
}

static platform_bool_t app_battery_low(void)
{
    uint16_t mv = app_battery_mv();

    return ((mv != 0u) && (mv < APP_BATTERY_LOW_MV)) ? PLATFORM_TRUE
                                                     : PLATFORM_FALSE;
}

static uint8_t app_state(void)
{
    if (service_chassis_is_estopped() == PLATFORM_TRUE)
    {
        return (uint8_t)PROTOCOL_STATE_ESTOP;
    }

    if (s_fault_code != 0u)
    {
        return (uint8_t)PROTOCOL_STATE_FAULT;
    }

    if (service_chassis_is_enabled() == PLATFORM_TRUE)
    {
        return (uint8_t)PROTOCOL_STATE_MANUAL;
    }

    return (uint8_t)PROTOCOL_STATE_IDLE;
}

/* ------------------------------ Handlers ------------------------------- */

static protocol_result_t app_handle_set_velocity(
                                        const protocol_frame_t *frame,
                                        void *user_data)
{
    int16_t vx_mm_s;
    int16_t wz_mrad_s;
    platform_err_t err;

    (void)user_data;

    vx_mm_s = protocol_get_i16(&frame->payload[0]);
    wz_mrad_s = protocol_get_i16(&frame->payload[2]);

    /*
     * Protocol v1 has no lateral component: SET_VELOCITY is (v, w). The
     * mecanum strafe primitives stay available to the app and the key.
     */
    err = service_chassis_set_velocity(vx_mm_s, 0, wz_mrad_s, app_now_ms());

    if (err == PLATFORM_ERR_PARAM)
    {
        return PROTOCOL_RESULT_RANGE;
    }

    return PLATFORM_IS_OK(err) ? PROTOCOL_RESULT_OK : PROTOCOL_RESULT_FAILED;
}

static protocol_result_t app_handle_set_arm(const protocol_frame_t *frame,
                                            void *user_data)
{
    int16_t angles[PROTOCOL_ARM_JOINT_COUNT];
    uint8_t i;
    platform_err_t err;

    (void)user_data;

    if (service_chassis_is_estopped() == PLATFORM_TRUE)
    {
        return PROTOCOL_RESULT_STATE;
    }

    for (i = 0u; i < PROTOCOL_ARM_JOINT_COUNT; i++)
    {
        angles[i] = protocol_get_i16(&frame->payload[(uint32_t)i * 2u]);
    }

    err = service_arm_set_joints(angles, 0u);

    if (err == PLATFORM_ERR_PARAM)
    {
        return PROTOCOL_RESULT_RANGE;
    }

    return PLATFORM_IS_OK(err) ? PROTOCOL_RESULT_OK : PROTOCOL_RESULT_FAILED;
}

static protocol_result_t app_handle_set_enable(const protocol_frame_t *frame,
                                               void *user_data)
{
    uint8_t enable;
    platform_err_t err;
    uint32_t now_ms;

    (void)user_data;

    enable = frame->payload[0];

    if (enable > 1u)
    {
        return PROTOCOL_RESULT_RANGE;
    }

    if ((enable != 0u) &&
        (service_chassis_is_estopped() == PLATFORM_TRUE))
    {
        return PROTOCOL_RESULT_STATE;
    }

    now_ms = app_now_ms();
    err = service_chassis_enable((enable != 0u) ? PLATFORM_TRUE
                                                : PLATFORM_FALSE,
                                 now_ms);

    if (PLATFORM_IS_ERR(err))
    {
        return PROTOCOL_RESULT_FAILED;
    }

    if (enable == 0u)
    {
        s_manual_forward = PLATFORM_FALSE;
    }

    return PROTOCOL_RESULT_OK;
}

static protocol_result_t app_handle_estop(const protocol_frame_t *frame,
                                          void *user_data)
{
    (void)frame;
    (void)user_data;

    service_chassis_emergency_stop();
    s_manual_forward = PLATFORM_FALSE;
    app_raise_fault(PROTOCOL_FAULT_ESTOP, app_now_ms(), PLATFORM_TRUE);

    return PROTOCOL_RESULT_OK;
}

static protocol_result_t app_handle_clear_fault(
                                        const protocol_frame_t *frame,
                                        void *user_data)
{
    uint16_t code;
    uint16_t cleared;
    uint32_t now_ms;

    (void)user_data;

    code = protocol_get_u16(&frame->payload[0]);
    now_ms = app_now_ms();

    if ((code != PROTOCOL_CLEAR_ALL_FAULTS) &&
        (code != s_fault_code) &&
        (code != PROTOCOL_FAULT_ESTOP))
    {
        return PROTOCOL_RESULT_STATE;
    }

    /*
     * docs/protocol.md section 8: refuse to clear while the condition is
     * still present, otherwise the host sees "cleared, then raised again".
     * The pack being under voltage is measurable, so it is the one fault
     * this firmware can check; the others are latches by nature.
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
     * CLEAR_FAULT is the release for the software latch. The chassis stays
     * disabled; the host must send SET_ENABLE again before it can move.
     */
    (void)service_chassis_clear_estop();
    s_fault_code = 0u;

    /*
     * s_prev_timed_out is left alone: clear_estop() has already reset the
     * service's timeout flag, so there is no stale edge to re-fire. The next
     * loop copies that FALSE, and a fresh timeout needs a fresh target first.
     */
    if (cleared != 0u)
    {
        (void)service_proto_send_event(cleared, now_ms);
    }

    return PROTOCOL_RESULT_OK;
}

static protocol_result_t app_handle_set_gripper(const protocol_frame_t *frame,
                                                void *user_data)
{
    (void)frame;
    (void)user_data;

    /* No gripper hardware on this board: tell the host instead of lying. */
    return PROTOCOL_RESULT_FAILED;
}

static protocol_result_t app_handle_set_param(const protocol_frame_t *frame,
                                              void *user_data)
{
    (void)frame;
    (void)user_data;

    /* Parameter storage is not implemented yet; ACK with "failed". */
    return PROTOCOL_RESULT_FAILED;
}

/* -------------------------------- Key ---------------------------------- */

static void app_on_key_event(platform_key_id_t id,
                             service_key_event_t event,
                             void *user_data)
{
    uint32_t now_ms = app_now_ms();

    (void)id;
    (void)user_data;

    if (event == SERVICE_KEY_EVENT_SHORT_PRESS)
    {
        if ((service_chassis_is_enabled() == PLATFORM_FALSE) ||
            (service_chassis_is_estopped() == PLATFORM_TRUE))
        {
            return;
        }

        if (s_manual_forward == PLATFORM_TRUE)
        {
            service_chassis_stop();
            s_manual_forward = PLATFORM_FALSE;
        }
        else
        {
            (void)service_chassis_forward(APP_KEY_FORWARD_MM_S, now_ms);
            s_manual_forward = PLATFORM_TRUE;
        }

        return;
    }

    /* LONG_PRESS: stop first, report from the comm task (single TX owner). */
    service_chassis_emergency_stop();
    s_manual_forward = PLATFORM_FALSE;
    s_pending_estop = PLATFORM_TRUE;
    s_beep_until_ms = now_ms + APP_KEY_BEEP_MS;
}

/* ------------------------------- Tasks --------------------------------- */

static void app_comm_task_entry(void *argument)
{
    uint32_t now_tick;
    uint32_t now_ms;
    uint32_t odom_ms;
    uint32_t arm_ms;
    uint32_t status_ms;
    uint32_t heartbeat_ms;
    uint32_t jitter_us;
    uint16_t heartbeat;
    uint32_t notify;
    platform_bool_t timed_out;
    platform_bool_t enabled;
    service_chassis_odometry_t odom;
    platform_servo_states_t servo_states;
    platform_servo_angles_t servo_targets;
    int16_t angles[PROTOCOL_ARM_JOINT_COUNT];
    uint8_t i;
    uint16_t battery_mv;

    (void)argument;

    odom_ms = 0u;
    arm_ms = 0u;
    status_ms = 0u;
    heartbeat_ms = 0u;
    heartbeat = 0u;

    for (;;)
    {
        (void)platform_os_task_wait_notify(APP_COMM_PERIOD_MS, &notify);

        now_tick = platform_os_tick_get();
        now_ms = platform_os_ticks_to_ms(now_tick);

        (void)service_proto_process(now_ms);

        /* ------------------------------ Safety ------------------------- */

        if (s_pending_estop == PLATFORM_TRUE)
        {
            s_pending_estop = PLATFORM_FALSE;
            app_raise_fault(PROTOCOL_FAULT_ESTOP, now_ms, PLATFORM_TRUE);
        }

        timed_out = service_chassis_cmd_timed_out();

        if ((timed_out == PLATFORM_TRUE) &&
            (s_prev_timed_out == PLATFORM_FALSE) &&
            (s_fault_code != PROTOCOL_FAULT_ESTOP))
        {
            app_raise_fault(PROTOCOL_FAULT_COMM_TIMEOUT, now_ms,
                            PLATFORM_FALSE);
        }

        s_prev_timed_out = timed_out;

        if ((service_proto_crc_alarm() == PLATFORM_TRUE) &&
            (s_crc_alarm_seen == PLATFORM_FALSE))
        {
            s_crc_alarm_seen = PLATFORM_TRUE;

            if (s_fault_code == 0u)
            {
                app_raise_fault(PROTOCOL_FAULT_CRC_LIMIT, now_ms,
                                PLATFORM_FALSE);
            }
        }

        enabled = service_chassis_is_enabled();

        if (enabled != s_led_on)
        {
            s_led_on = enabled;
            (void)platform_led_set(s_led_on);
        }

        /* ------------------------------ Reports ------------------------ */

        if ((now_ms - odom_ms) >= APP_ODOM_PERIOD_MS)
        {
            odom_ms = now_ms;
            service_chassis_get_odometry(&odom);
            (void)service_proto_send_odom(odom.x_mm, odom.y_mm,
                                          odom.heading_centi_deg,
                                          odom.vl_mm_s, odom.vr_mm_s);
        }

        if ((now_ms - arm_ms) >= APP_ARM_PERIOD_MS)
        {
            arm_ms = now_ms;

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

        if ((now_ms - status_ms) >= APP_STATUS_PERIOD_MS)
        {
            /*
             * Measure jitter on this 100 ms deadline, not on the 10 ms loop:
             * the loop also wakes on every RX event, so its period is not a
             * deadline and would report the RX traffic as jitter.
             */
            jitter_us = ((now_ms - status_ms) > APP_STATUS_PERIOD_MS)
                            ? ((now_ms - status_ms - APP_STATUS_PERIOD_MS)
                                   * 1000u)
                            : 0u;

            if (jitter_us > s_jitter_max_us)
            {
                s_jitter_max_us = (uint16_t)((jitter_us > 65535u)
                                                 ? 65535u
                                                 : jitter_us);
            }

            status_ms = now_ms;
            battery_mv = app_battery_mv();

            if ((app_battery_low() == PLATFORM_TRUE) &&
                (s_fault_code == 0u))
            {
                app_raise_fault(PROTOCOL_FAULT_BATTERY_LOW, now_ms,
                                PLATFORM_FALSE);
            }

            /* No board temperature sensor yet: report 0, not a fake reading. */
            (void)service_proto_send_status(app_state(), s_fault_code,
                                            battery_mv, (int8_t)0,
                                            s_jitter_max_us);

            s_jitter_max_us = 0u;
        }

        if ((now_ms - heartbeat_ms) >= APP_HEARTBEAT_PERIOD_MS)
        {
            heartbeat_ms = now_ms;
            heartbeat++;
            (void)service_proto_send_heartbeat(app_state(), heartbeat);
        }
    }
}

static void app_control_task_entry(void *argument)
{
    uint32_t last_tick;
    uint32_t elapsed_ms;
    uint32_t now_ms;

    (void)argument;

    last_tick = platform_os_tick_get();

    for (;;)
    {
        uint32_t now_tick = platform_os_tick_get();

        elapsed_ms = platform_os_ticks_to_ms(now_tick - last_tick);
        last_tick = now_tick;

        if (elapsed_ms == 0u)
        {
            elapsed_ms = 1u;
        }

        now_ms = platform_os_ticks_to_ms(now_tick);

        (void)service_key_update(elapsed_ms);

        (void)service_chassis_apply(now_ms);
        service_chassis_update_odometry(elapsed_ms);

        {
            platform_bool_t beep =
                (now_ms < s_beep_until_ms) ? PLATFORM_TRUE : PLATFORM_FALSE;

            if (beep != s_buzzer_on)
            {
                s_buzzer_on = beep;
                (void)platform_buzzer_set(s_buzzer_on);
            }
        }

        (void)platform_os_task_delay_ms(APP_CONTROL_PERIOD_MS);
    }
}

static void app_uart_event(platform_uart_bus_t bus,
                           platform_uart_event_t event,
                           void *user_data)
{
    (void)bus;
    (void)user_data;

    if ((event == PLATFORM_UART_EVENT_RX_DATA) ||
        (event == PLATFORM_UART_EVENT_RX_IDLE))
    {
        if (s_comm_task != NULL)
        {
            (void)platform_os_task_notify_from_isr(s_comm_task);
        }
    }
}

//******************************** Functions ********************************//

platform_err_t app_robot_init(void)
{
    platform_err_t err;
    platform_uart_cfg_t uart_cfg;
    service_chassis_cfg_t chassis_cfg;
    service_arm_cfg_t arm_cfg;
    service_key_cfg_t key_cfg;
    service_proto_cfg_t proto_cfg;
    uint8_t i;

    /* ---------------------------- Transport ---------------------------- */

    uart_cfg.bus = APP_ROBOT_UART_BUS;
    uart_cfg.baudrate = APP_ROBOT_UART_BAUDRATE;
    uart_cfg.data_bits = 8u;
    uart_cfg.parity = PLATFORM_UART_PARITY_NONE;
    uart_cfg.stop_bits = PLATFORM_UART_STOP_BITS_1;
    uart_cfg.duplex = PLATFORM_UART_DUPLEX_FULL;
    uart_cfg.use_dma = PLATFORM_TRUE;

    err = platform_uart_init(&uart_cfg);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    /* ----------------------------- Chassis ----------------------------- */

    chassis_cfg.motor[SERVICE_CHASSIS_WHEEL_FL] = PLATFORM_MOTOR_ID_0;
    chassis_cfg.motor[SERVICE_CHASSIS_WHEEL_FR] = PLATFORM_MOTOR_ID_1;
    chassis_cfg.motor[SERVICE_CHASSIS_WHEEL_RL] = PLATFORM_MOTOR_ID_2;
    chassis_cfg.motor[SERVICE_CHASSIS_WHEEL_RR] = PLATFORM_MOTOR_ID_3;
    chassis_cfg.half_length_mm = APP_CHASSIS_HALF_LENGTH_MM;
    chassis_cfg.half_width_mm = APP_CHASSIS_HALF_WIDTH_MM;
    chassis_cfg.max_vx_mm_s = APP_CHASSIS_MAX_VX_MM_S;
    chassis_cfg.max_vy_mm_s = APP_CHASSIS_MAX_VY_MM_S;
    chassis_cfg.max_wz_mrad_s = APP_CHASSIS_MAX_WZ_MRAD_S;
    chassis_cfg.max_wheel_mm_s = APP_CHASSIS_MAX_WHEEL_MM_S;
    chassis_cfg.counts_per_mm_x1000 = APP_CHASSIS_COUNTS_PER_MM_X1000;
    chassis_cfg.cmd_timeout_ms = APP_CHASSIS_CMD_TIMEOUT_MS;

    err = service_chassis_init(&chassis_cfg);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    /* ------------------------------- Arm ------------------------------- */

    for (i = 0u; i < SERVICE_ARM_JOINT_COUNT; i++)
    {
        arm_cfg.min_centi_deg[i] = APP_ARM_MIN_CENTI_DEG;
        arm_cfg.max_centi_deg[i] = APP_ARM_MAX_CENTI_DEG;
        arm_cfg.home_centi_deg[i] = 0;
    }

    arm_cfg.default_move_time_ms = APP_ARM_MOVE_TIME_MS;

    err = service_arm_init(&arm_cfg);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    /* ------------------------------- Key ------------------------------- */

    key_cfg.id = PLATFORM_KEY_ID_0;
    key_cfg.long_press_ms = APP_KEY_LONG_PRESS_MS;

    err = service_key_init(&key_cfg, app_on_key_event, NULL);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    /* ----------------------------- Protocol ---------------------------- */

    proto_cfg.bus = APP_ROBOT_UART_BUS;

    err = service_proto_init(&proto_cfg);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = service_proto_register(PROTOCOL_CMD_SET_VELOCITY,
                                 app_handle_set_velocity, NULL);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = service_proto_register(PROTOCOL_CMD_SET_ARM_JOINTS,
                                 app_handle_set_arm, NULL);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = service_proto_register(PROTOCOL_CMD_SET_GRIPPER,
                                 app_handle_set_gripper, NULL);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = service_proto_register(PROTOCOL_CMD_SET_ENABLE,
                                 app_handle_set_enable, NULL);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = service_proto_register(PROTOCOL_CMD_SET_PARAM,
                                 app_handle_set_param, NULL);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = service_proto_register(PROTOCOL_CMD_ESTOP,
                                 app_handle_estop, NULL);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = service_proto_register(PROTOCOL_CMD_CLEAR_FAULT,
                                 app_handle_clear_fault, NULL);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    s_fault_code = 0u;
    s_pending_estop = PLATFORM_FALSE;
    s_beep_until_ms = 0u;
    s_manual_forward = PLATFORM_FALSE;
    s_prev_timed_out = PLATFORM_FALSE;
    s_crc_alarm_seen = PLATFORM_FALSE;
    s_jitter_max_us = 0u;
    s_led_on = PLATFORM_FALSE;
    s_buzzer_on = PLATFORM_FALSE;
    (void)platform_led_set(s_led_on);
    (void)platform_buzzer_set(s_buzzer_on);

    return PLATFORM_ERR_OK;
}

platform_err_t app_robot_start(void)
{
    platform_os_task_cfg_t task_cfg;
    platform_err_t err;

    task_cfg.name = "appComm";
    task_cfg.entry = app_comm_task_entry;
    task_cfg.argument = NULL;
    task_cfg.stack_size_bytes = APP_COMM_TASK_STACK_BYTES;
    task_cfg.priority = PLATFORM_OS_PRIORITY_HIGH;

    err = platform_os_task_create(&task_cfg, &s_comm_task);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    task_cfg.name = "appCtrl";
    task_cfg.entry = app_control_task_entry;
    task_cfg.argument = NULL;
    task_cfg.stack_size_bytes = APP_CONTROL_TASK_STACK_BYTES;
    task_cfg.priority = PLATFORM_OS_PRIORITY_ABOVE_NORMAL;

    err = platform_os_task_create(&task_cfg, &s_control_task);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    return platform_uart_register_event_cb(APP_ROBOT_UART_BUS,
                                           app_uart_event, NULL);
}
