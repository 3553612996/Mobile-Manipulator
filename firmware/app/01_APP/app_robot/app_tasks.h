/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_tasks.h
 *
 * @par dependencies
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Application task bodies and the comm -> control parameter queue.
 *
 * Threading contract:
 *   - comm task: protocol parsing/handlers, fault poll, reports, single UART
 *     transmitter;
 *   - control task: key scan, PID parameter queue drain, chassis apply,
 *     odometry, buzzer.
 * The only cross-task messages are the PID queue and the emergency-stop flag.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __APP_TASKS_H__
#define __APP_TASKS_H__

//******************************** Includes *********************************//

#include "platform_def.h"

//******************************** Declaring ********************************//

typedef struct
{
    uint8_t wheel;      /* SERVICE_CHASSIS_WHEEL_* */
    uint8_t gain;       /* SERVICE_CHASSIS_PID_* */
    float_t value;      /* Gain, not scaled. */
} app_pid_param_msg_t;

void app_tasks_comm_entry(void *argument);
void app_tasks_control_entry(void *argument);

/*
 * Queue one gain change for the control task. Returns PLATFORM_FALSE if the
 * queue is full; the caller turns that into an ACK failure.
 */
platform_bool_t app_tasks_post_pid_param(uint8_t wheel, uint8_t gain,
                                         float_t value);

/*
 * Ask the comm task to latch an emergency stop. Called from the control task
 * (key long press) so that only the comm task touches the UART.
 */
void app_tasks_request_estop(uint32_t beep_until_ms);

/* Consume the pending emergency-stop request. Comm task only. */
platform_bool_t app_tasks_take_pending_estop(void);

#endif /* __APP_TASKS_H__ */
