/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_state.h
 *
 * @par dependencies
 * - platform_def.h
 * - protocol_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Application state machine.
 *
 * The state byte in REPORT_STATUS is no longer derived from whichever service
 * flag happens to be set. It is an explicit state that only this module owns;
 * handlers and the fault policy ask it for permission and report transitions.
 *
 * All transitions are non-blocking and touch one enum, so the comm task is the
 * normal writer. INIT -> IDLE happens in app_user_init_start().
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __APP_STATE_H__
#define __APP_STATE_H__

//******************************** Includes *********************************//

#include "platform_def.h"
#include "protocol_def.h"

//******************************** Declaring ********************************//

typedef enum
{
    APP_STATE_INIT  = PROTOCOL_STATE_INIT,
    APP_STATE_IDLE  = PROTOCOL_STATE_IDLE,
    APP_STATE_MANUAL = PROTOCOL_STATE_MANUAL,
    APP_STATE_AUTO  = PROTOCOL_STATE_AUTO,
    APP_STATE_ESTOP = PROTOCOL_STATE_ESTOP,
    APP_STATE_FAULT = PROTOCOL_STATE_FAULT
} app_state_t;

void app_state_init(void);

/* INIT -> IDLE once the tasks exist and the robot can accept commands. */
void app_state_ready(void);

app_state_t app_state_get(void);

/* Value for REPORT_STATUS / REPORT_HEARTBEAT (same numeric values). */
uint8_t app_state_protocol(void);

/* TRUE only in MANUAL: velocity and arm commands are accepted. */
platform_bool_t app_state_motion_allowed(void);

/* TRUE in IDLE or MANUAL: SET_ENABLE may turn the drivers on. */
platform_bool_t app_state_enable_allowed(void);

void app_state_on_enabled(void);
void app_state_on_disabled(void);
void app_state_on_estop(void);
void app_state_on_fault(void);
void app_state_on_cleared(void);

#endif /* __APP_STATE_H__ */
