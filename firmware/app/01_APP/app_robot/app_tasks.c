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
#include "app_robot/app_robot_config.h"

#include "service_chassis/service_chassis.h"
#include "service_key/service_key.h"
#include "service_proto/service_proto.h"

#include "platform_os_task.h"
#include "platform_os_time.h"
#include "platform_bsp_buzzer.h"

//******************************** Variables ********************************//

static volatile platform_bool_t s_pending_estop;
static volatile uint32_t         s_beep_until_ms;
static platform_bool_t           s_buzzer_on;

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

    msg.wheel = wheel;
    msg.gain = gain;
    msg.value = value;

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

static void app_tasks_drain_pid_queue(void)
{
    app_pid_param_msg_t msg;
    platform_os_queue_t queue = app_user_init_pid_queue();

    if (queue == NULL)
    {
        return;
    }

    while (PLATFORM_IS_OK(platform_os_queue_receive(queue, &msg, 0u)))
    {
        (void)service_chassis_set_pid_gain(
                    msg.wheel,
                    (service_chassis_pid_gain_t)msg.gain,
                    msg.value);
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
        app_tasks_drain_pid_queue();

        (void)service_chassis_apply(now_ms, elapsed_ms);
        service_chassis_update_odometry(elapsed_ms);

        beep = (now_ms < s_beep_until_ms) ? PLATFORM_TRUE : PLATFORM_FALSE;

        if (beep != s_buzzer_on)
        {
            s_buzzer_on = beep;
            (void)platform_buzzer_set(s_buzzer_on);
        }

        (void)platform_os_task_delay_ms(APP_CONTROL_PERIOD_MS);
    }
}
