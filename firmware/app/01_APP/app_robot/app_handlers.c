/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_handlers.c
 *
 * @par dependencies
 * - app_handlers.h
 * - app_state.h, app_fault.h, app_tasks.h
 * - service_chassis.h, service_arm.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement the protocol command handlers.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "app_robot/app_handlers.h"
#include "app_robot/app_state.h"
#include "app_robot/app_fault.h"
#include "app_robot/app_tasks.h"
#include "app_robot/app_robot_config.h"

#include "service_chassis/service_chassis.h"
#include "service_arm/service_arm.h"

#include "platform_os_time.h"

//******************************** Variables ********************************//

static platform_bool_t s_manual_forward;

//******************************** Functions ********************************//

static uint32_t app_handlers_now_ms(void)
{
    return platform_os_ticks_to_ms(platform_os_tick_get());
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
    err = service_chassis_set_velocity(vx_mm_s, 0, wz_mrad_s,
                                       app_handlers_now_ms());

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

    if ((app_state_get() == APP_STATE_ESTOP) ||
        (app_state_get() == APP_STATE_FAULT))
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

    now_ms = app_handlers_now_ms();

    if (enable != 0u)
    {
        if ((app_state_enable_allowed() == PLATFORM_FALSE) ||
            (service_chassis_is_estopped() == PLATFORM_TRUE))
        {
            return PROTOCOL_RESULT_STATE;
        }

        /*
         * The fault policy may have disabled both subsystems. SET_ENABLE is
         * the single "resume" command, so it brings the arm back as well.
         */
        if (PLATFORM_IS_ERR(service_arm_enable(PLATFORM_TRUE)))
        {
            return PROTOCOL_RESULT_FAILED;
        }

        err = service_chassis_enable(PLATFORM_TRUE, now_ms);

        if (PLATFORM_IS_ERR(err))
        {
            (void)service_arm_enable(PLATFORM_FALSE);
            return PROTOCOL_RESULT_FAILED;
        }

        app_state_on_enabled();
    }
    else
    {
        err = service_chassis_enable(PLATFORM_FALSE, now_ms);
        (void)service_arm_enable(PLATFORM_FALSE);
        s_manual_forward = PLATFORM_FALSE;

        if (PLATFORM_IS_ERR(err))
        {
            return PROTOCOL_RESULT_FAILED;
        }

        app_state_on_disabled();
    }

    return PROTOCOL_RESULT_OK;
}

static protocol_result_t app_handle_estop(const protocol_frame_t *frame,
                                          void *user_data)
{
    (void)frame;
    (void)user_data;

    service_chassis_emergency_stop();
    (void)service_arm_enable(PLATFORM_FALSE);
    s_manual_forward = PLATFORM_FALSE;
    (void)app_fault_raise(PROTOCOL_FAULT_ESTOP, app_handlers_now_ms(),
                          PLATFORM_TRUE);

    return PROTOCOL_RESULT_OK;
}

static protocol_result_t app_handle_clear_fault(
                                        const protocol_frame_t *frame,
                                        void *user_data)
{
    uint16_t code;
    uint32_t now_ms;

    (void)user_data;

    code = protocol_get_u16(&frame->payload[0]);
    now_ms = app_handlers_now_ms();

    return app_fault_clear(code, now_ms);
}

static protocol_result_t app_handle_set_gripper(const protocol_frame_t *frame,
                                                void *user_data)
{
    (void)frame;
    (void)user_data;

    /* No gripper hardware on this board: tell the host instead of lying. */
    return PROTOCOL_RESULT_FAILED;
}

/*
 * Map a SET_PARAM id to up to two wheels and one gain index. The legacy
 * left/right ids stay valid and write both wheels on that side.
 */
static uint8_t app_handlers_pid_targets(uint8_t param_id,
                                        uint8_t wheels[2],
                                        uint8_t *gain)
{
    if ((param_id >= PROTOCOL_PARAM_PID_A_KP) &&
        (param_id <= PROTOCOL_PARAM_PID_D_KD))
    {
        wheels[0] = (uint8_t)((param_id - PROTOCOL_PARAM_PID_A_KP) / 3u);
        *gain = (uint8_t)((param_id - PROTOCOL_PARAM_PID_A_KP) % 3u);
        return 1u;
    }

    if ((param_id >= 0x03u) && (param_id <= 0x05u))
    {
        wheels[0] = SERVICE_CHASSIS_WHEEL_FL;
        wheels[1] = SERVICE_CHASSIS_WHEEL_RL;
        *gain = (uint8_t)(param_id - 0x03u);
        return 2u;
    }

    if ((param_id >= 0x06u) && (param_id <= 0x08u))
    {
        wheels[0] = SERVICE_CHASSIS_WHEEL_FR;
        wheels[1] = SERVICE_CHASSIS_WHEEL_RR;
        *gain = (uint8_t)(param_id - 0x06u);
        return 2u;
    }

    return 0u;
}

static protocol_result_t app_handle_set_param(const protocol_frame_t *frame,
                                              void *user_data)
{
    uint8_t param_id;
    uint8_t gain;
    uint8_t wheels[2];
    uint8_t wheel_count;
    uint8_t i;
    int32_t value;
    int32_t limit;
    float_t gain_value;

    (void)user_data;

    param_id = frame->payload[0];
    value = protocol_get_i32(&frame->payload[1]);

    /*
     * Bench open-loop duty (0x30..0x33). It must be refused while the chassis
     * is off, because the state is read here and the value is applied later in
     * the control task: an ACK 0x00 means "the control task will see a legal
     * request", and the service re-checks enable/estop when it does.
     */
    if ((param_id >= PROTOCOL_PARAM_TEST_DUTY_A) &&
        (param_id <= PROTOCOL_PARAM_TEST_DUTY_D))
    {
        if ((value < -APP_TEST_DUTY_MAX_PERMILLE) ||
            (value > APP_TEST_DUTY_MAX_PERMILLE))
        {
            return PROTOCOL_RESULT_RANGE;
        }

        if ((service_chassis_is_enabled() == PLATFORM_FALSE) ||
            (service_chassis_is_estopped() == PLATFORM_TRUE))
        {
            return PROTOCOL_RESULT_STATE;
        }

        if (app_tasks_post_test_duty(
                (uint8_t)(param_id - PROTOCOL_PARAM_TEST_DUTY_A),
                (int16_t)value) == PLATFORM_FALSE)
        {
            return PROTOCOL_RESULT_FAILED;
        }

        return PROTOCOL_RESULT_OK;
    }

    wheel_count = app_handlers_pid_targets(param_id, wheels, &gain);

    if (wheel_count == 0u)
    {
        /*
         * Geometry and limit parameters are recognized but not runtime
         * writable yet; an unknown id is a range error, not a silent success.
         */
        switch (param_id)
        {
            case PROTOCOL_PARAM_WHEEL_BASE:
            case PROTOCOL_PARAM_WHEEL_DIAMETER:
            case PROTOCOL_PARAM_CMD_TIMEOUT:
            case PROTOCOL_PARAM_MAX_LINEAR:
            case PROTOCOL_PARAM_MAX_ANGULAR:
                return PROTOCOL_RESULT_FAILED;

            default:
                return PROTOCOL_RESULT_RANGE;
        }
    }

    limit = (gain == (uint8_t)SERVICE_CHASSIS_PID_KD)
                ? APP_PID_KD_MAX_X1000
                : APP_PID_GAIN_MAX_X1000;

    if ((value < 0) || (value > limit))
    {
        return PROTOCOL_RESULT_RANGE;
    }

    gain_value = (float_t)value / 1000.0f;

    for (i = 0u; i < wheel_count; i++)
    {
        if (app_tasks_post_pid_param(wheels[i], gain, gain_value) ==
            PLATFORM_FALSE)
        {
            return PROTOCOL_RESULT_FAILED;
        }
    }

    return PROTOCOL_RESULT_OK;
}

/* -------------------------------- Key ---------------------------------- */

void app_handlers_on_key(platform_key_id_t id, service_key_event_t event,
                         void *user_data)
{
    uint32_t now_ms;
    (void)id;
    (void)user_data;

    now_ms = app_handlers_now_ms();

    if (event == SERVICE_KEY_EVENT_SHORT_PRESS)
    {
        if ((app_state_motion_allowed() == PLATFORM_FALSE) ||
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
    app_tasks_request_estop(now_ms + APP_KEY_BEEP_MS);
}

void app_handlers_process_pending(uint32_t now_ms)
{
    if (app_tasks_take_pending_estop() == PLATFORM_TRUE)
    {
        (void)app_fault_raise(PROTOCOL_FAULT_ESTOP, now_ms, PLATFORM_TRUE);
    }
}

/* ----------------------------- Registration ---------------------------- */

static const struct
{
    uint8_t cmd;
    service_proto_handler_t handler;
} s_handler_table[] =
{
    { PROTOCOL_CMD_SET_VELOCITY,   app_handle_set_velocity },
    { PROTOCOL_CMD_SET_ARM_JOINTS, app_handle_set_arm },
    { PROTOCOL_CMD_SET_GRIPPER,    app_handle_set_gripper },
    { PROTOCOL_CMD_SET_ENABLE,     app_handle_set_enable },
    { PROTOCOL_CMD_SET_PARAM,      app_handle_set_param },
    { PROTOCOL_CMD_ESTOP,          app_handle_estop },
    { PROTOCOL_CMD_CLEAR_FAULT,    app_handle_clear_fault }
};

#define APP_HANDLER_COUNT \
    ((uint8_t)(sizeof(s_handler_table) / sizeof(s_handler_table[0])))

platform_err_t app_handlers_register(void)
{
    platform_err_t err;
    uint8_t i;

    s_manual_forward = PLATFORM_FALSE;

    for (i = 0u; i < APP_HANDLER_COUNT; i++)
    {
        err = service_proto_register(s_handler_table[i].cmd,
                                     s_handler_table[i].handler, NULL);

        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }
    }

    return PLATFORM_ERR_OK;
}
