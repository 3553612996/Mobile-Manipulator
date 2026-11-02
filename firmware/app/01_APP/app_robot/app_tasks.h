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

/* Messages on the comm -> control queue. */
typedef enum
{
    APP_CTRL_MSG_PID_GAIN = 0,
    APP_CTRL_MSG_TEST_DUTY,
    APP_CTRL_MSG_PID_RELOAD,    /* Re-apply every stored gain (PARAM_RESET). */
    APP_CTRL_MSG_LIMITS_RELOAD  /* Timeout/limits only; keeps the PID state. */
} app_ctrl_msg_kind_t;

typedef struct
{
    uint8_t kind;       /* app_ctrl_msg_kind_t. */
    uint8_t wheel;      /* SERVICE_CHASSIS_WHEEL_* */
    uint8_t gain;       /* SERVICE_CHASSIS_PID_*, gain messages only. */
    int16_t duty;       /* Permille, test-duty messages only. */
    float_t value;      /* Gain, not scaled, gain messages only. */
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
 * Queue one open-loop test duty (permille) for the control task. Same queue
 * and same failure contract as the gain setter.
 */
platform_bool_t app_tasks_post_test_duty(uint8_t wheel, int16_t duty);

/*
 * Ask the control task to push every parameter in app_params into the chassis
 * service. Used after PARAM_RESET; the control task owns apply(), so the
 * reload must not run in the caller.
 */
platform_bool_t app_tasks_reload_params(void);

/*
 * Ask the control task to push the command timeout and velocity limits into
 * the chassis service. Used by SET_PARAM 0x10~0x12, which the host may write
 * while the chassis is enabled, so this reload must not touch the PID state.
 */
platform_bool_t app_tasks_reload_limits(void);

/*
 * Ask the comm task to latch an emergency stop. Called from the control task
 * (key long press) so that only the comm task touches the UART.
 */
void app_tasks_request_estop(uint32_t beep_until_ms);

/* Consume the pending emergency-stop request. Comm task only. */
platform_bool_t app_tasks_take_pending_estop(void);

#endif /* __APP_TASKS_H__ */
