/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_user_init.c
 *
 * @par dependencies
 * - app_user_init.h
 * - app_robot_config.h
 * - app_state.h, app_fault.h, app_handlers.h, app_tasks.h
 * - service_chassis.h, service_arm.h, service_key.h, service_proto.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement the user initialization and the task/queue tables.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "app_robot/app_user_init.h"
#include "app_robot/app_robot_config.h"
#include "app_robot/app_state.h"
#include "app_robot/app_fault.h"
#include "app_robot/app_handlers.h"
#include "app_robot/app_tasks.h"
#include "app_robot/app_storage.h"
#include "app_robot/app_params.h"

#include "service_chassis/service_chassis.h"
#include "service_arm/service_arm.h"
#include "service_key/service_key.h"
#include "service_proto/service_proto.h"
#include "service_storage/service_storage.h"

#include "platform_os_task.h"
#include "platform_os_time.h"
#include "platform_mcu_uart.h"
#include "platform_bsp_led.h"
#include "platform_bsp_buzzer.h"

//******************************** Variables ********************************//

static platform_os_task_t s_comm_task;
static platform_os_task_t s_control_task;
static platform_os_task_t s_storage_task;
static platform_os_queue_t s_pid_queue;
static platform_os_queue_t s_storage_queue;
static platform_os_queue_t s_export_queue;

typedef struct
{
    const char *                name;
    platform_os_task_entry_t    entry;
    uint32_t                    stack_size_bytes;
    platform_os_priority_t      priority;
    platform_os_task_t *        handle;
} app_task_def_t;

/*
 * One row per task. Adding a task means adding a row, not another block of
 * task_cfg assignments in app_robot_start().
 */
static const app_task_def_t s_task_table[] =
{
    {
        "appComm",
        app_tasks_comm_entry,
        APP_COMM_TASK_STACK_BYTES,
        PLATFORM_OS_PRIORITY_HIGH,
        &s_comm_task
    },
    {
        "appCtrl",
        app_tasks_control_entry,
        APP_CONTROL_TASK_STACK_BYTES,
        PLATFORM_OS_PRIORITY_ABOVE_NORMAL,
        &s_control_task
    },
    {
        /*
         * Below both: flash work is never urgent, and the task must not run
         * while the control loop or the protocol dispatch has something to do.
         */
        "appStorage",
        app_storage_entry,
        APP_STORAGE_TASK_STACK_BYTES,
        PLATFORM_OS_PRIORITY_LOW,
        &s_storage_task
    }
};

typedef struct
{
    const char *        name;
    uint32_t            item_size_bytes;
    uint32_t            item_count;
    platform_os_queue_t *handle;
} app_queue_def_t;

static const app_queue_def_t s_queue_table[] =
{
    {
        "appPid",
        (uint32_t)sizeof(app_pid_param_msg_t),
        APP_PID_QUEUE_LEN,
        &s_pid_queue
    },
    {
        "appStore",
        (uint32_t)sizeof(app_storage_req_t),
        APP_STORAGE_QUEUE_LEN,
        &s_storage_queue
    },
    {
        "appExport",
        (uint32_t)sizeof(app_storage_export_t),
        APP_STORAGE_EXPORT_QUEUE_LEN,
        &s_export_queue
    }
};

#define APP_TASK_COUNT \
    ((uint8_t)(sizeof(s_task_table) / sizeof(s_task_table[0])))
#define APP_QUEUE_COUNT \
    ((uint8_t)(sizeof(s_queue_table) / sizeof(s_queue_table[0])))

//******************************** Functions ********************************//

platform_os_queue_t app_user_init_pid_queue(void)
{
    return s_pid_queue;
}

/*
 * Mount the flash and restore the saved parameters. A missing chip must not
 * stop the robot from booting (board_init.c makes the same promise), so the
 * result is a capability flag, not an error: everything storage-related then
 * answers "unavailable" and the host sees ACK 0x06.
 */
static platform_bool_t app_user_init_storage(void)
{
    /* Sized for the larger blob: the calibration area is 80 B. */
    uint8_t buffer[APP_PARAMS_BLOB_MAX_SIZE];
    uint16_t length = 0u;
    uint32_t seq = 0u;
    platform_err_t err;

    app_params_init();

    err = service_storage_init(
              platform_os_ticks_to_ms(platform_os_tick_get()));
    if (PLATFORM_IS_ERR(err))
    {
        return PLATFORM_FALSE;
    }

    if (PLATFORM_IS_OK(service_storage_load_blob(SERVICE_STORAGE_BLOB_PARAM,
                                                 buffer,
                                                 (uint16_t)sizeof(buffer),
                                                 &length, &seq)))
    {
        /* A blob that fails to parse leaves the defaults in place. */
        (void)app_params_parse(APP_PARAMS_SCOPE_PARAM, buffer, length);
    }

    if (PLATFORM_IS_OK(service_storage_load_blob(SERVICE_STORAGE_BLOB_CALIB,
                                                 buffer,
                                                 (uint16_t)sizeof(buffer),
                                                 &length, &seq)))
    {
        (void)app_params_parse(APP_PARAMS_SCOPE_CALIB, buffer, length);
    }

    service_storage_set_log_level(app_params_log_level());
    service_storage_set_clean((app_params_dirty() == PLATFORM_TRUE)
                                  ? PLATFORM_FALSE
                                  : PLATFORM_TRUE);

    return PLATFORM_TRUE;
}

static platform_err_t app_user_init_chassis(void)
{
    service_chassis_cfg_t chassis_cfg;
    platform_err_t err;
    uint8_t i;

    chassis_cfg.motor[SERVICE_CHASSIS_WHEEL_FL] = PLATFORM_MOTOR_ID_0;
    chassis_cfg.motor[SERVICE_CHASSIS_WHEEL_FR] = PLATFORM_MOTOR_ID_1;
    chassis_cfg.motor[SERVICE_CHASSIS_WHEEL_RL] = PLATFORM_MOTOR_ID_2;
    chassis_cfg.motor[SERVICE_CHASSIS_WHEEL_RR] = PLATFORM_MOTOR_ID_3;
    /*
     * Geometry and gains come from app_params, which already holds either the
     * saved blob or the compile-time defaults restored in app_user_init_storage.
     */
    chassis_cfg.half_length_mm = (int16_t)app_params_half_length_mm();
    chassis_cfg.half_width_mm = (int16_t)app_params_half_width_mm();
    /* Protocol v1 has no lateral limit: max_linear bounds both vx and vy. */
    chassis_cfg.max_vx_mm_s = app_params_max_linear_mm_s();
    chassis_cfg.max_vy_mm_s = app_params_max_linear_mm_s();
    chassis_cfg.max_wz_mrad_s = app_params_max_angular_mrad_s();
    chassis_cfg.max_wheel_mm_s = APP_CHASSIS_MAX_WHEEL_MM_S;
    chassis_cfg.counts_per_mm_x1000 = app_params_counts_per_mm_x1000();
    chassis_cfg.cmd_timeout_ms = app_params_cmd_timeout_ms();

    for (i = 0u; i < SERVICE_CHASSIS_WHEEL_COUNT; i++)
    {
        chassis_cfg.pid_kp[i] =
            (float_t)app_params_pid_gain_x1000(i, SERVICE_CHASSIS_PID_KP) /
            1000.0f;
        chassis_cfg.pid_ki[i] =
            (float_t)app_params_pid_gain_x1000(i, SERVICE_CHASSIS_PID_KI) /
            1000.0f;
        chassis_cfg.pid_kd[i] =
            (float_t)app_params_pid_gain_x1000(i, SERVICE_CHASSIS_PID_KD) /
            1000.0f;
    }

    err = service_chassis_init(&chassis_cfg);

    if (PLATFORM_IS_OK(err))
    {
        /*
         * The cfg carries one scale for all four wheels; the per-wheel scale,
         * direction, start duty and the limits come from app_params. No task
         * is running yet, so this is the one place outside the control task
         * where the service setters are allowed.
         */
        app_params_apply_to_chassis();
    }

    return err;
}

static platform_err_t app_user_init_arm(void)
{
    service_arm_cfg_t arm_cfg;
    uint8_t i;

    for (i = 0u; i < SERVICE_ARM_JOINT_COUNT; i++)
    {
        arm_cfg.min_centi_deg[i] = APP_ARM_MIN_CENTI_DEG;
        arm_cfg.max_centi_deg[i] = APP_ARM_MAX_CENTI_DEG;
        arm_cfg.home_centi_deg[i] = 0;
    }

    arm_cfg.default_move_time_ms = APP_ARM_MOVE_TIME_MS;

    return service_arm_init(&arm_cfg);
}

static platform_err_t app_user_init_key(void)
{
    service_key_cfg_t key_cfg;

    key_cfg.id = PLATFORM_KEY_ID_0;
    key_cfg.long_press_ms = APP_KEY_LONG_PRESS_MS;

    return service_key_init(&key_cfg, app_handlers_on_key, NULL);
}

platform_err_t app_user_init_init(void)
{
    platform_uart_cfg_t uart_cfg;
    service_proto_cfg_t proto_cfg;
    platform_bool_t storage_ok;
    platform_err_t err;
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

    /* ------------------------------ Queues ----------------------------- */

    /*
     * Before every service: the storage task's queues exist before anything
     * can enqueue, and FreeRTOS allows queue creation before the scheduler.
     */
    for (i = 0u; i < APP_QUEUE_COUNT; i++)
    {
        err = platform_os_queue_create(s_queue_table[i].name,
                                       s_queue_table[i].item_size_bytes,
                                       s_queue_table[i].item_count,
                                       s_queue_table[i].handle);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }
    }

    /* ---------------------------- Storage ------------------------------ */

    /*
     * Mount and restore before the chassis service: the saved gains and
     * geometry have to be in RAM when the chassis is configured.
     */
    storage_ok = app_user_init_storage();

    /* ----------------------------- Services ---------------------------- */

    err = app_user_init_chassis();
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = app_user_init_arm();
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = app_user_init_key();
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    proto_cfg.bus = APP_ROBOT_UART_BUS;

    err = service_proto_init(&proto_cfg);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    /* ------------------------- Application state ----------------------- */

    app_state_init();
    app_fault_init();

    err = app_handlers_register();
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = app_storage_init(s_storage_queue, s_export_queue, storage_ok);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    (void)platform_led_set(PLATFORM_FALSE);
    (void)platform_buzzer_set(PLATFORM_FALSE);

    return PLATFORM_ERR_OK;
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

platform_err_t app_user_init_start(void)
{
    platform_os_task_cfg_t task_cfg;
    platform_err_t err;
    uint8_t i;

    for (i = 0u; i < APP_TASK_COUNT; i++)
    {
        task_cfg.name = s_task_table[i].name;
        task_cfg.entry = s_task_table[i].entry;
        task_cfg.argument = NULL;
        task_cfg.stack_size_bytes = s_task_table[i].stack_size_bytes;
        task_cfg.priority = s_task_table[i].priority;

        err = platform_os_task_create(&task_cfg, s_task_table[i].handle);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }
    }

    err = platform_uart_register_event_cb(APP_ROBOT_UART_BUS,
                                          app_uart_event, NULL);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    app_state_ready();

    return PLATFORM_ERR_OK;
}
