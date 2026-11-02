/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_state.c
 *
 * @par dependencies
 * - app_state.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement the application state machine.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "app_robot/app_state.h"

//******************************** Variables ********************************//

static app_state_t s_state;

//******************************** Functions ********************************//

void app_state_init(void)
{
    s_state = APP_STATE_INIT;
}

void app_state_ready(void)
{
    if (s_state == APP_STATE_INIT)
    {
        s_state = APP_STATE_IDLE;
    }
}

app_state_t app_state_get(void)
{
    return s_state;
}

uint8_t app_state_protocol(void)
{
    return (uint8_t)s_state;
}

platform_bool_t app_state_motion_allowed(void)
{
    return (s_state == APP_STATE_MANUAL) ? PLATFORM_TRUE : PLATFORM_FALSE;
}

platform_bool_t app_state_enable_allowed(void)
{
    return ((s_state == APP_STATE_IDLE) || (s_state == APP_STATE_MANUAL))
               ? PLATFORM_TRUE
               : PLATFORM_FALSE;
}

void app_state_on_enabled(void)
{
    if ((s_state == APP_STATE_IDLE) || (s_state == APP_STATE_MANUAL))
    {
        s_state = APP_STATE_MANUAL;
    }
}

void app_state_on_disabled(void)
{
    if (s_state == APP_STATE_MANUAL)
    {
        s_state = APP_STATE_IDLE;
    }
}

void app_state_on_estop(void)
{
    s_state = APP_STATE_ESTOP;
}

void app_state_on_fault(void)
{
    if (s_state != APP_STATE_ESTOP)
    {
        s_state = APP_STATE_FAULT;
    }
}

void app_state_on_cleared(void)
{
    /*
     * Clearing a WARN must not disturb a manual drive that was running when
     * the warning latched. ERROR and FATAL did disable the hardware, so those
     * states fall back to IDLE and wait for a fresh SET_ENABLE.
     */
    if ((s_state == APP_STATE_FAULT) || (s_state == APP_STATE_ESTOP))
    {
        s_state = APP_STATE_IDLE;
    }
}
