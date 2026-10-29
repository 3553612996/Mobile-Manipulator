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
#include "app_robot/app_params.h"
#include "app_robot/app_storage.h"
#include "app_robot/app_robot_config.h"

#include "service_chassis/service_chassis.h"
#include "service_arm/service_arm.h"
#include "service_storage/service_storage.h"

#include "platform_os_time.h"

//******************************** Variables ********************************//

static platform_bool_t s_manual_forward;

/*
 * Serialization buffers for PARAM_SAVE. Only the comm task runs handlers, and
 * app_storage copies short records into its staging pool at enqueue time, so
 * one buffer per scope is enough.
 */
static uint8_t s_param_buffer[APP_PARAMS_BLOB_PARAM_SIZE];
static uint8_t s_calib_buffer[APP_PARAMS_BLOB_CALIB_SIZE];

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

/*
 * Calibration ids: 0x01, 0x02 and the per-wheel block 0x50~0x5F. All of them
 * change the ruler the odometry and the inverse kinematics use, so protocol.md
 * requires the chassis to be off before they may be written.
 */
static platform_bool_t app_handlers_is_calib_param(uint8_t param_id)
{
    if ((param_id == PROTOCOL_PARAM_WHEEL_BASE) ||
        (param_id == PROTOCOL_PARAM_WHEEL_DIAMETER))
    {
        return PLATFORM_TRUE;
    }

    return ((param_id >= PROTOCOL_PARAM_COUNTS_PER_MM_A) &&
            (param_id <= PROTOCOL_PARAM_DIRECTION_D))
               ? PLATFORM_TRUE
               : PLATFORM_FALSE;
}

/* Store one calibration id in RAM. Returns the setter's result. */
static platform_err_t app_handlers_store_calib(uint8_t param_id,
                                               int32_t value)
{
    uint8_t wheel;

    if (param_id == PROTOCOL_PARAM_WHEEL_BASE)
    {
        /* 0x01 is the track width; the wheelbase stays at its saved value. */
        return app_params_set_track_mm(value);
    }

    if (param_id == PROTOCOL_PARAM_WHEEL_DIAMETER)
    {
        return app_params_set_wheel_diameter_mm(value);
    }

    if (param_id <= PROTOCOL_PARAM_COUNTS_PER_MM_D)
    {
        wheel = (uint8_t)(param_id - PROTOCOL_PARAM_COUNTS_PER_MM_A);
        return app_params_set_wheel_counts(wheel, value);
    }

    if (param_id <= PROTOCOL_PARAM_DUTY_FWD_D)
    {
        wheel = (uint8_t)(param_id - PROTOCOL_PARAM_DUTY_FWD_A);
        return app_params_set_wheel_deadzone(
                   wheel, value, app_params_deadzone_rev_permille(wheel));
    }

    if (param_id <= PROTOCOL_PARAM_DUTY_REV_D)
    {
        wheel = (uint8_t)(param_id - PROTOCOL_PARAM_DUTY_REV_A);
        return app_params_set_wheel_deadzone(
                   wheel, app_params_deadzone_fwd_permille(wheel), value);
    }

    wheel = (uint8_t)(param_id - PROTOCOL_PARAM_DIRECTION_A);
    return app_params_set_wheel_dir(wheel, value);
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
    platform_err_t err;

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

    /*
     * Log level (0x40). It is a reporting filter, not a motion parameter, so
     * it needs no enabled chassis; it lands in the parameter blob and changes
     * what REPORT_STORAGE echoes.
     */
    if (param_id == PROTOCOL_PARAM_LOG_LEVEL)
    {
        if ((value < 0) || (value > 3))
        {
            return PROTOCOL_RESULT_RANGE;
        }

        if (PLATFORM_IS_ERR(app_params_set_log_level((uint8_t)value)))
        {
            return PROTOCOL_RESULT_RANGE;
        }

        service_storage_set_log_level((uint8_t)value);
        service_storage_set_clean(PLATFORM_FALSE);

        return PROTOCOL_RESULT_OK;
    }

    /*
     * Calibration ids (0x01, 0x02, 0x50~0x5F). They are stored in RAM and the
     * control task pushes them into the service on its next tick, so the state
     * is checked here: the host must not believe a write landed while the
     * wheels were turning.
     */
    if (app_handlers_is_calib_param(param_id) == PLATFORM_TRUE)
    {
        if ((service_chassis_is_enabled() == PLATFORM_TRUE) ||
            (service_chassis_is_estopped() == PLATFORM_TRUE))
        {
            return PROTOCOL_RESULT_STATE;
        }

        err = app_handlers_store_calib(param_id, value);

        if (err == PLATFORM_ERR_PARAM)
        {
            return PROTOCOL_RESULT_RANGE;
        }

        if (PLATFORM_IS_ERR(err))
        {
            return PROTOCOL_RESULT_FAILED;
        }

        service_storage_set_clean(PLATFORM_FALSE);

        if (app_tasks_reload_params() == PLATFORM_FALSE)
        {
            /* RAM holds the new value; the control task will get it on reset. */
            return PROTOCOL_RESULT_FAILED;
        }

        return PROTOCOL_RESULT_OK;
    }

    /*
     * Limits (0x10~0x12). Unlike the calibration ids these are legal while the
     * chassis is enabled, so the control task applies them without touching the
     * PID state.
     */
    if ((param_id == PROTOCOL_PARAM_CMD_TIMEOUT) ||
        (param_id == PROTOCOL_PARAM_MAX_LINEAR) ||
        (param_id == PROTOCOL_PARAM_MAX_ANGULAR))
    {
        switch (param_id)
        {
            case PROTOCOL_PARAM_CMD_TIMEOUT:
                err = app_params_set_cmd_timeout_ms(value);
                break;

            case PROTOCOL_PARAM_MAX_LINEAR:
                err = app_params_set_max_linear(value);
                break;

            default:
                err = app_params_set_max_angular(value);
                break;
        }

        if (err == PLATFORM_ERR_PARAM)
        {
            return PROTOCOL_RESULT_RANGE;
        }

        if (PLATFORM_IS_ERR(err))
        {
            return PROTOCOL_RESULT_FAILED;
        }

        service_storage_set_clean(PLATFORM_FALSE);

        if (app_tasks_reload_limits() == PLATFORM_FALSE)
        {
            return PROTOCOL_RESULT_FAILED;
        }

        return PROTOCOL_RESULT_OK;
    }

    wheel_count = app_handlers_pid_targets(param_id, wheels, &gain);

    if (wheel_count == 0u)
    {
        /* Unknown id: a range error, not a silent success. */
        return PROTOCOL_RESULT_RANGE;
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

        /* The RAM copy is what PARAM_SAVE writes; the control task applies it. */
        (void)app_params_set_pid_gain(wheels[i], gain, value);
    }

    service_storage_set_clean(PLATFORM_FALSE);

    return PROTOCOL_RESULT_OK;
}

/* ----------------------------- Storage --------------------------------- */

/*
 * Runs in the storage task when a PARAM_SAVE blob has been written (or failed).
 * tag carries the scope that was saved, so REPORT_STORAGE bit 0 keeps telling
 * the truth: "clean" only once every dirty scope is on flash.
 */
static void app_handlers_on_saved(app_storage_result_t result,
                                  uint16_t tag,
                                  void *user_data)
{
    (void)user_data;

    if (result != APP_STORAGE_OK)
    {
        /* Dirty stays set; REPORT_STORAGE bit 3 reports the failure. */
        return;
    }

    app_params_mark_saved((uint8_t)tag);
    service_storage_set_clean((app_params_dirty() == PLATFORM_TRUE)
                                  ? PLATFORM_FALSE
                                  : PLATFORM_TRUE);
}

static protocol_result_t app_handle_param_save(const protocol_frame_t *frame,
                                               void *user_data)
{
    uint8_t scope;
    uint16_t length;

    (void)user_data;

    scope = frame->payload[0];

    if (scope > PROTOCOL_STORAGE_SCOPE_ALL)
    {
        return PROTOCOL_RESULT_RANGE;
    }

    if (app_storage_available() == PLATFORM_FALSE)
    {
        return PROTOCOL_RESULT_STORAGE_UNAVAILABLE;
    }

    /*
     * Calibration changes the ruler the odometry is measured with, so it is
     * only writable with the wheels off. The parameter area (gains, log level)
     * needs no such guard: bench tuning happens with the robot standing still
     * but the host is free to ask anyway.
     */
    if ((scope != PROTOCOL_STORAGE_SCOPE_PARAM) &&
        (service_chassis_is_enabled() == PLATFORM_TRUE))
    {
        return PROTOCOL_RESULT_STATE;
    }

    /* protocol.md 5.5.9: one write at a time. */
    if (app_storage_saves_pending() == PLATFORM_TRUE)
    {
        return PROTOCOL_RESULT_BUSY;
    }

    if ((scope == PROTOCOL_STORAGE_SCOPE_PARAM) ||
        (scope == PROTOCOL_STORAGE_SCOPE_ALL))
    {
        length = app_params_serialize(APP_PARAMS_SCOPE_PARAM,
                                      s_param_buffer,
                                      (uint16_t)sizeof(s_param_buffer));

        if ((length == 0u) ||
            (app_storage_save_blob(SERVICE_STORAGE_BLOB_PARAM,
                                   s_param_buffer, length,
                                   APP_PARAMS_SCOPE_PARAM,
                                   app_handlers_on_saved, NULL) ==
             PLATFORM_FALSE))
        {
            return PROTOCOL_RESULT_BUSY;
        }
    }

    if ((scope == PROTOCOL_STORAGE_SCOPE_CALIB) ||
        (scope == PROTOCOL_STORAGE_SCOPE_ALL))
    {
        length = app_params_serialize(APP_PARAMS_SCOPE_CALIB,
                                      s_calib_buffer,
                                      (uint16_t)sizeof(s_calib_buffer));

        if ((length == 0u) ||
            (app_storage_save_blob(SERVICE_STORAGE_BLOB_CALIB,
                                   s_calib_buffer, length,
                                   APP_PARAMS_SCOPE_CALIB,
                                   app_handlers_on_saved, NULL) ==
             PLATFORM_FALSE))
        {
            /*
             * scope == ALL can fail on the second blob after the first was
             * already accepted; that half still lands and its callback clears
             * its own dirty bit. BUSY is the honest answer for the other half.
             */
            return PROTOCOL_RESULT_BUSY;
        }
    }

    return PROTOCOL_RESULT_OK;
}

static protocol_result_t app_handle_param_reset(const protocol_frame_t *frame,
                                                void *user_data)
{
    uint8_t scope;
    uint8_t confirm;

    (void)user_data;

    scope = frame->payload[0];
    confirm = frame->payload[1];

    if ((confirm != PROTOCOL_STORAGE_CONFIRM) ||
        (scope > PROTOCOL_STORAGE_SCOPE_ALL))
    {
        return PROTOCOL_RESULT_RANGE;
    }

    if ((scope != PROTOCOL_STORAGE_SCOPE_PARAM) &&
        (service_chassis_is_enabled() == PLATFORM_TRUE))
    {
        return PROTOCOL_RESULT_STATE;
    }

    /*
     * A save already on its way would mark the *old* values as saved after the
     * reset changed RAM, and the clean bit would lie. Let it finish first.
     */
    if (app_storage_saves_pending() == PLATFORM_TRUE)
    {
        return PROTOCOL_RESULT_BUSY;
    }

    app_params_reset(scope);

    /* RAM only: the restore is not on flash until a PARAM_SAVE follows. */
    service_storage_set_log_level(app_params_log_level());
    service_storage_set_clean(PLATFORM_FALSE);

    /*
     * The control task re-applies every stored value, so a reset takes effect
     * at once: gains, geometry, per-wheel scale and the limits.
     */
    if (app_tasks_reload_params() == PLATFORM_FALSE)
    {
        return PROTOCOL_RESULT_BUSY;
    }

    return PROTOCOL_RESULT_OK;
}

static protocol_result_t app_handle_storage_query(const protocol_frame_t *frame,
                                                  void *user_data)
{
    (void)frame;
    (void)user_data;

    if (app_storage_available() == PLATFORM_FALSE)
    {
        return PROTOCOL_RESULT_STORAGE_UNAVAILABLE;
    }

    /* The comm loop sends REPORT_STORAGE right after the ACK, same iteration. */
    app_storage_request_status();

    return PROTOCOL_RESULT_OK;
}

static protocol_result_t app_handle_log_export(const protocol_frame_t *frame,
                                               void *user_data)
{
    uint8_t source;
    uint8_t action;
    uint32_t from_seq;
    platform_bool_t start;

    (void)user_data;

    source = frame->payload[0];
    action = frame->payload[1];
    from_seq = (uint32_t)protocol_get_u16(&frame->payload[2]) |
               ((uint32_t)protocol_get_u16(&frame->payload[4]) << 16);

    if ((source > PROTOCOL_LOG_SOURCE_FAULT) || (action > 1u))
    {
        return PROTOCOL_RESULT_RANGE;
    }

    if (app_storage_available() == PLATFORM_FALSE)
    {
        return PROTOCOL_RESULT_STORAGE_UNAVAILABLE;
    }

    start = (action != 0u) ? PLATFORM_TRUE : PLATFORM_FALSE;

    if ((start == PLATFORM_TRUE) && (app_storage_exporting() == PLATFORM_TRUE))
    {
        return PROTOCOL_RESULT_BUSY;
    }

    if (app_storage_export(source, start, from_seq, NULL, NULL) ==
        PLATFORM_FALSE)
    {
        /* Only a full request queue is left; the host may retry. */
        return (start == PLATFORM_TRUE) ? PROTOCOL_RESULT_BUSY
                                        : PROTOCOL_RESULT_FAILED;
    }

    return PROTOCOL_RESULT_OK;
}

static protocol_result_t app_handle_crash_read(const protocol_frame_t *frame,
                                               void *user_data)
{
    uint8_t slot;
    uint8_t flags;
    uint8_t newest;
    platform_bool_t mark_read;

    (void)user_data;

    slot = frame->payload[0];
    flags = frame->payload[1];

    if ((slot != PROTOCOL_CRASH_SLOT_LATEST) &&
        (slot >= SERVICE_STORAGE_CRASH_SLOTS))
    {
        return PROTOCOL_RESULT_RANGE;
    }

    if (app_storage_available() == PLATFORM_FALSE)
    {
        return PROTOCOL_RESULT_STORAGE_UNAVAILABLE;
    }

    /*
     * protocol.md 5.5.7: "no valid record" is ACK 0x04. Both checks read the
     * mount scan's cache, so answering it never touches the flash.
     */
    if (slot == PROTOCOL_CRASH_SLOT_LATEST)
    {
        if (service_storage_crash_latest(&newest) == PLATFORM_FALSE)
        {
            return PROTOCOL_RESULT_FAILED;
        }
    }
    else if (service_storage_crash_slot_valid(slot) == PLATFORM_FALSE)
    {
        return PROTOCOL_RESULT_FAILED;
    }

    /* CRASH_READ flags bit 0 = mark the record read (REPORT_CRASH bit 0 is
     * "more fragments follow"; the two must not be confused). */
    mark_read = ((flags & 0x01u) != 0u) ? PLATFORM_TRUE : PLATFORM_FALSE;

    if (app_storage_crash_read(slot, mark_read, NULL, NULL) == PLATFORM_FALSE)
    {
        return PROTOCOL_RESULT_BUSY;
    }

    return PROTOCOL_RESULT_OK;
}

static protocol_result_t app_handle_log_clear(const protocol_frame_t *frame,
                                              void *user_data)
{
    uint8_t source;
    uint8_t confirm;

    (void)user_data;

    source = frame->payload[0];
    confirm = frame->payload[1];

    if ((confirm != PROTOCOL_STORAGE_CONFIRM) ||
        (source > APP_STORAGE_CLEAR_ALL))
    {
        return PROTOCOL_RESULT_RANGE;
    }

    if (app_storage_available() == PLATFORM_FALSE)
    {
        return PROTOCOL_RESULT_STORAGE_UNAVAILABLE;
    }

    if (app_storage_clear(source, NULL, NULL) == PLATFORM_FALSE)
    {
        return PROTOCOL_RESULT_BUSY;
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
    { PROTOCOL_CMD_CLEAR_FAULT,    app_handle_clear_fault },
    { PROTOCOL_CMD_PARAM_SAVE,     app_handle_param_save },
    { PROTOCOL_CMD_PARAM_RESET,    app_handle_param_reset },
    { PROTOCOL_CMD_STORAGE_QUERY,  app_handle_storage_query },
    { PROTOCOL_CMD_LOG_EXPORT,     app_handle_log_export },
    { PROTOCOL_CMD_CRASH_READ,     app_handle_crash_read },
    { PROTOCOL_CMD_LOG_CLEAR,      app_handle_log_clear }
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
