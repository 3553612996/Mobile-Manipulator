/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_bsp_servo.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform six-axis servo-bus device interface.
 *
 * The concrete bus (half-duplex UART, RS-485, CAN or a dedicated servo
 * controller) is hidden behind platform_servo_ops_t. Angles are always
 * 0.01 degree, matching the host protocol and avoiding float on the wire.
 *
 * The bus is polled: get_states() returns the cache filled by the last
 * transaction, never a blocking read. The owner of the bus calls
 * platform_servo_poll() from a dedicated low-priority task; the control loop
 * only ever reads the cache.
 *
 * @version V2.0 2026-10-31
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_BSP_SERVO_H__
#define __PLATFORM_BSP_SERVO_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"

//******************************** Defines **********************************//

#define PLATFORM_SERVO_JOINT_COUNT  6u

/* Marker for an unmeasured load/temperature value. */
#define PLATFORM_SERVO_VALUE_INVALID    ((int16_t)-32768)
#define PLATFORM_SERVO_TEMP_INVALID     PLATFORM_SERVO_VALUE_INVALID
#define PLATFORM_SERVO_VOLTAGE_INVALID  ((uint16_t)0xFFFFu)

/*
 * Per-joint condition bits. The numeric values are mirrored by
 * PROTOCOL_SERVO_FAULT_* in protocol_def.h (and by the host tools); keep the
 * two tables in sync.
 */
#define PLATFORM_SERVO_FAULT_OFFLINE    ((uint8_t)0x01u)  /* No reply. */
#define PLATFORM_SERVO_FAULT_OVERTEMP   ((uint8_t)0x02u)
#define PLATFORM_SERVO_FAULT_UNDERVOLT  ((uint8_t)0x04u)
#define PLATFORM_SERVO_FAULT_OVERVOLT   ((uint8_t)0x08u)
#define PLATFORM_SERVO_FAULT_STALL      ((uint8_t)0x10u)
#define PLATFORM_SERVO_FAULT_REPLY      ((uint8_t)0x20u)  /* Malformed reply. */

/* Configuration operations, used by platform_servo_config(). */
#define PLATFORM_SERVO_CFG_SET_ID       ((uint8_t)0x01u)
#define PLATFORM_SERVO_CFG_SET_BIAS     ((uint8_t)0x02u)  /* Servo-side PSCK. */
#define PLATFORM_SERVO_CFG_CALIBRATE    ((uint8_t)0x03u)  /* Current = zero. */
#define PLATFORM_SERVO_CFG_CLEAR_CALIB  ((uint8_t)0x04u)
#define PLATFORM_SERVO_CFG_TORQUE       ((uint8_t)0x05u)  /* 0 release, 1 hold. */

//******************************** Declaring ********************************//

typedef struct
{
    int16_t joint_centi_deg[PLATFORM_SERVO_JOINT_COUNT];
} platform_servo_angles_t;

typedef struct
{
    int16_t         angle_centi_deg;
    int16_t         load_permille;      /* PLATFORM_SERVO_VALUE_INVALID if N/A. */
    int16_t         temperature_c;      /* PLATFORM_SERVO_VALUE_INVALID if N/A. */
    uint16_t        voltage_mv;         /* PLATFORM_SERVO_VOLTAGE_INVALID if N/A. */
    uint8_t         fault_bits;         /* PLATFORM_SERVO_FAULT_*. */
    platform_bool_t online;             /* TRUE when the last poll answered. */
} platform_servo_state_t;

typedef struct
{
    platform_servo_state_t joint[PLATFORM_SERVO_JOINT_COUNT];
} platform_servo_states_t;

/* One joint's identity and zero reference, as stored by the application. */
typedef struct
{
    uint8_t  joint;
    uint8_t  servo_id;                  /* Bus address, 0..254. */
    int16_t  offset_centi_deg;          /* Calibrated mechanical zero. */
} platform_servo_joint_cfg_t;

typedef struct
{
    uint8_t  joint;
    uint8_t  op;                        /* PLATFORM_SERVO_CFG_*. */
    int32_t  value;
} platform_servo_cfg_req_t;

typedef struct
{
    platform_err_t (*init)(void *context);
    platform_err_t (*deinit)(void *context);
    platform_err_t (*set_enable)(void *context, platform_bool_t enable);
    platform_err_t (*set_angles)(void *context,
                                 const platform_servo_angles_t *angles,
                                 uint16_t move_time_ms);
    platform_err_t (*get_states)(void *context,
                                 platform_servo_states_t *states);

    /* Advance the bus state machine. Called from the servo-bus task only. */
    platform_err_t (*poll)(void *context, uint32_t now_ms);

    /* Apply the persisted ID/calibration map. Before the servo task starts. */
    platform_err_t (*set_joint_map)(
                        void *context,
                        const platform_servo_joint_cfg_t
                            map[PLATFORM_SERVO_JOINT_COUNT]);

    /*
     * Configuration is split so that no layer below 01_APP ever blocks: the
     * comm task queues a request, sleeps while the servo-bus task executes it,
     * then takes the result (new ID, new zero offset or applied bias).
     */
    platform_err_t (*config)(void *context,
                             const platform_servo_cfg_req_t *req);
    platform_bool_t (*config_ready)(void *context);
    platform_err_t (*config_take)(void *context, int32_t *out_value);
    void (*config_cancel)(void *context);
} platform_servo_ops_t;

/*
 * Register the servo-bus implementation. Must be called before
 * platform_servo_init(). Only one servo bus is assumed for this robot.
 */
platform_err_t platform_servo_register(const platform_servo_ops_t *ops,
                                       void *context);

platform_err_t platform_servo_init(void);
platform_err_t platform_servo_deinit(void);

platform_err_t platform_servo_set_enable(platform_bool_t enable);

/*
 * Send all six joint targets. move_time_ms == 0 means "as fast as the bus and
 * the servo allow"; a non-zero value asks the servos to interpolate. The move
 * is queued and sent by platform_servo_poll(), so this never blocks.
 */
platform_err_t platform_servo_set_angles(
                                        const platform_servo_angles_t *angles,
                                        uint16_t move_time_ms);

/* Cached states from the last completed bus transactions. Never blocks. */
platform_err_t platform_servo_get_states(platform_servo_states_t *states);

/* Advance the bus state machine. Servo-bus task only. */
platform_err_t platform_servo_poll(uint32_t now_ms);

/* Apply the persisted joint map. Must run before platform_servo_init(). */
platform_err_t platform_servo_set_joint_map(
                    const platform_servo_joint_cfg_t
                        map[PLATFORM_SERVO_JOINT_COUNT]);

/*
 * Queue one configuration operation. Returns PLATFORM_ERR_BUSY while another
 * one is still on the bus. The servo-bus task executes it on a later poll;
 * the caller polls config_ready()/config_take().
 */
platform_err_t platform_servo_config(const platform_servo_cfg_req_t *req);

platform_bool_t platform_servo_config_ready(void);

/* Return the completed result and free the slot. */
platform_err_t platform_servo_config_take(int32_t *out_value);

/* Give up on a request the caller stopped waiting for. */
void platform_servo_config_cancel(void);

#endif /* __PLATFORM_BSP_SERVO_H__ */
