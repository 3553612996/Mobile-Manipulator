/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_tasks.c
 *
 * @par dependencies
 * - app_tasks.h
 * - app_user_init.h
 * - app_handlers.h, app_fault.h, app_report.h
 * - service_key.h, service_chassis.h, service_proto.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement the comm and control task bodies.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "app_robot/app_tasks.h"
#include "app_robot/app_user_init.h"
#include "app_robot/app_handlers.h"
#include "app_robot/app_fault.h"
#include "app_robot/app_report.h"
#include "app_robot/app_params.h"
#include "app_robot/app_storage.h"
#include "app_robot/app_robot_config.h"

#include "service_chassis/service_chassis.h"
#include "service_arm/service_arm.h"
#include "service_key/service_key.h"
#include "service_proto/service_proto.h"

#include "platform_os_task.h"
#include "platform_os_time.h"
#include "platform_bsp_buzzer.h"

//******************************** Variables ********************************//

static volatile platform_bool_t s_pending_estop;
static volatile uint32_t         s_beep_until_ms;
static platform_bool_t           s_buzzer_on;

/* Control -> comm: one code per newly latched arm condition. */
static volatile uint16_t         s_pending_arm_fault;
static platform_bool_t           s_arm_fault_seen;

//******************************** Functions ********************************//

platform_bool_t app_tasks_post_pid_param(uint8_t wheel, uint8_t gain,
                                         float_t value)
{
    app_pid_param_msg_t msg;
    platform_os_queue_t queue = app_user_init_pid_queue();

    if (queue == NULL)
    {
        return PLATFORM_FALSE;
    }

    msg.kind = (uint8_t)APP_CTRL_MSG_PID_GAIN;
    msg.wheel = wheel;
    msg.gain = gain;
    msg.duty = 0;
    msg.value = value;

    return PLATFORM_IS_OK(platform_os_queue_send(queue, &msg, 0u))
               ? PLATFORM_TRUE
               : PLATFORM_FALSE;
}

platform_bool_t app_tasks_post_test_duty(uint8_t wheel, int16_t duty)
{
    app_pid_param_msg_t msg;
    platform_os_queue_t queue = app_user_init_pid_queue();

    if (queue == NULL)
    {
        return PLATFORM_FALSE;
    }

    msg.kind = (uint8_t)APP_CTRL_MSG_TEST_DUTY;
    msg.wheel = wheel;
    msg.gain = 0u;
    msg.duty = duty;
    msg.value = 0.0f;

    return PLATFORM_IS_OK(platform_os_queue_send(queue, &msg, 0u))
               ? PLATFORM_TRUE
               : PLATFORM_FALSE;
}

platform_bool_t app_tasks_reload_params(void)
{
    app_pid_param_msg_t msg;
    platform_os_queue_t queue = app_user_init_pid_queue();

    if (queue == NULL)
    {
        return PLATFORM_FALSE;
    }

    msg.kind = (uint8_t)APP_CTRL_MSG_PID_RELOAD;
    msg.wheel = 0u;
    msg.gain = 0u;
    msg.duty = 0;
    msg.value = 0.0f;

    return PLATFORM_IS_OK(platform_os_queue_send(queue, &msg, 0u))
               ? PLATFORM_TRUE
               : PLATFORM_FALSE;
}

platform_bool_t app_tasks_reload_limits(void)
{
    app_pid_param_msg_t msg;
    platform_os_queue_t queue = app_user_init_pid_queue();

    if (queue == NULL)
    {
        return PLATFORM_FALSE;
    }

    msg.kind = (uint8_t)APP_CTRL_MSG_LIMITS_RELOAD;
    msg.wheel = 0u;
    msg.gain = 0u;
    msg.duty = 0;
    msg.value = 0.0f;

    return PLATFORM_IS_OK(platform_os_queue_send(queue, &msg, 0u))
               ? PLATFORM_TRUE
               : PLATFORM_FALSE;
}

void app_tasks_request_estop(uint32_t beep_until_ms)
{
    s_pending_estop = PLATFORM_TRUE;
    s_beep_until_ms = beep_until_ms;
}

platform_bool_t app_tasks_take_pending_estop(void)
{
    if (s_pending_estop == PLATFORM_FALSE)
    {
        return PLATFORM_FALSE;
    }

    s_pending_estop = PLATFORM_FALSE;
    return PLATFORM_TRUE;
}

platform_bool_t app_tasks_take_pending_arm_fault(uint16_t *code)
{
    if ((s_pending_arm_fault == 0u) || (code == NULL))
    {
        return PLATFORM_FALSE;
    }

    *code = s_pending_arm_fault;
    s_pending_arm_fault = 0u;

    return PLATFORM_TRUE;
}

void app_tasks_clear_arm_fault_pending(void)
{
    s_pending_arm_fault = 0u;
    s_arm_fault_seen = PLATFORM_FALSE;
}

static void app_tasks_drain_pid_queue(uint32_t now_ms)
{
    app_pid_param_msg_t msg;
    platform_os_queue_t queue = app_user_init_pid_queue();

    if (queue == NULL)
    {
        return;
    }

    while (PLATFORM_IS_OK(platform_os_queue_receive(queue, &msg, 0u)))
    {
        if (msg.kind == (uint8_t)APP_CTRL_MSG_TEST_DUTY)
        {
            (void)service_chassis_set_test_duty(msg.wheel, msg.duty, now_ms);
        }
        else if (msg.kind == (uint8_t)APP_CTRL_MSG_PID_RELOAD)
        {
            app_params_apply_to_chassis();
        }
        else if (msg.kind == (uint8_t)APP_CTRL_MSG_LIMITS_RELOAD)
        {
            app_params_apply_limits_to_chassis();
        }
        else
        {
            (void)service_chassis_set_pid_gain(
                        msg.wheel,
                        (service_chassis_pid_gain_t)msg.gain,
                        msg.value);
        }
    }
}

/*
 * One arm condition maps to one protocol code. Temperature and rail voltage
 * come first because they threaten every joint at once; OFFLINE covers both a
 * missing reply and a malformed one (REPLY), and stall is the last resort.
 */
static uint16_t app_tasks_arm_fault_code(const service_arm_status_t *status)
{
    uint8_t i;

    for (i = 0u; i < SERVICE_ARM_JOINT_COUNT; i++)
    {
        if ((status->fault_bits[i] & SERVICE_ARM_FAULT_OVERTEMP) != 0u)
        {
            return PROTOCOL_FAULT_OVERTEMP;
        }
    }

    for (i = 0u; i < SERVICE_ARM_JOINT_COUNT; i++)
    {
        if ((status->fault_bits[i] &
             (SERVICE_ARM_FAULT_UNDERVOLT | SERVICE_ARM_FAULT_OVERVOLT)) != 0u)
        {
            return PROTOCOL_FAULT_ARM_VOLTAGE;
        }
    }

    for (i = 0u; i < SERVICE_ARM_JOINT_COUNT; i++)
    {
        if ((status->fault_bits[i] &
             (SERVICE_ARM_FAULT_OFFLINE | SERVICE_ARM_FAULT_REPLY)) != 0u)
        {
            return PROTOCOL_FAULT_ARM_OFFLINE;
        }
    }

    for (i = 0u; i < SERVICE_ARM_JOINT_COUNT; i++)
    {
        if ((status->fault_bits[i] & SERVICE_ARM_FAULT_STALL) != 0u)
        {
            return PROTOCOL_FAULT_JOINT_STALL;
        }
    }

    return 0u;
}

/*
 * Raise one report per latched condition, not one per control tick: the latch
 * stays set until the host clears it, and app_fault_raise would otherwise send
 * an EVENT frame every 10 ms.
 */
static void app_tasks_watch_arm_fault(void)
{
    service_arm_status_t status;
    uint16_t code;

    if (PLATFORM_IS_ERR(service_arm_get_status(&status)))
    {
        return;
    }

    if (status.faulted == PLATFORM_FALSE)
    {
        s_arm_fault_seen = PLATFORM_FALSE;
        return;
    }

    if ((s_arm_fault_seen == PLATFORM_FALSE) && (s_pending_arm_fault == 0u))
    {
        code = app_tasks_arm_fault_code(&status);

        if (code != 0u)
        {
            s_pending_arm_fault = code;
            s_arm_fault_seen = PLATFORM_TRUE;
        }
    }
}

void app_tasks_servo_entry(void *argument)
{
    uint32_t now_ms;

    (void)argument;

    for (;;)
    {
        now_ms = platform_os_ticks_to_ms(platform_os_tick_get());
        (void)service_arm_poll(now_ms);
        (void)platform_os_task_delay_ms(APP_SERVO_PERIOD_MS);
    }
}

void app_tasks_comm_entry(void *argument)
{
    uint32_t now_tick;
    uint32_t now_ms;
    uint32_t notify;

    (void)argument;

    app_report_init();

    for (;;)
    {
        (void)platform_os_task_wait_notify(APP_COMM_PERIOD_MS, &notify);

        now_tick = platform_os_tick_get();
        now_ms = platform_os_ticks_to_ms(now_tick);

        /* Handlers may raise a fault; poll edge conditions afterwards. */
        (void)service_proto_process(now_ms);

        /* Right after dispatch: ACKs first, then the reports they announced. */
        app_storage_pump(now_ms,
                         (service_proto_rx_age_ms(now_ms) <
                          APP_STORAGE_EXPORT_IDLE_MS) ? PLATFORM_TRUE
                                                      : PLATFORM_FALSE);

        app_handlers_process_pending(now_ms);
        app_fault_poll(now_ms);
        app_report_tick(now_ms);
    }
}

void app_tasks_control_entry(void *argument)
{
    uint32_t last_tick;
    uint32_t elapsed_ms;
    uint32_t now_ms;
    platform_bool_t beep;

    (void)argument;

    s_pending_estop = PLATFORM_FALSE;
    s_beep_until_ms = 0u;
    s_buzzer_on = PLATFORM_FALSE;
    s_pending_arm_fault = 0u;
    s_arm_fault_seen = PLATFORM_FALSE;
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

        /* Gains are applied before the next PID tick, never concurrently. */
        app_tasks_drain_pid_queue(now_ms);

        (void)service_chassis_apply(now_ms, elapsed_ms);
        service_chassis_update_odometry(elapsed_ms);

        /* Judged here, reported by the comm task: one UART owner stays. */
        (void)service_arm_update(now_ms);
        app_tasks_watch_arm_fault();

        beep = (now_ms < s_beep_until_ms) ? PLATFORM_TRUE : PLATFORM_FALSE;

        if (beep != s_buzzer_on)
        {
            s_buzzer_on = beep;
            (void)platform_buzzer_set(s_buzzer_on);
        }

        (void)platform_os_task_delay_ms(APP_CONTROL_PERIOD_MS);
    }
}
