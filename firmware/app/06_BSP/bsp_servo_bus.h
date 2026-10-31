/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file bsp_servo_bus.h
 *
 * @par dependencies
 * - platform_mcu_uart.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Drive a daisy chain of YH-protocol bus servos on one half-duplex UART.
 *
 * Wire format (Yahoo/Yeahbot "YH" ASCII protocol v2.0, 115200 8N1):
 *
 *   move      #IDPpwmTtime!           pwm 0500..2500, time 0000..9999 ms
 *   multi     {#000P1500T0800!#001P...!}
 *   angle     #IDPRAD!      -> #IDPpppp!
 *   temp/volt #IDPRTV!      -> #IDT25V07!   (firmware dependent, also seen
 *                              as #IDP1063T34V5.1!)
 *   id        #IDPID!       -> #IDppp!  ; write #IDPIDnew! -> #OK!
 *   bias      #IDPSCK+010! / #IDPSCK-010!
 *   torque    #IDPULK! (release) / #IDPULR! (restore)
 *   stop      #255PDST!
 *
 * The vendor requires at least 100 ms between two commands, so the driver is a
 * state machine polled by a dedicated task: one transaction per poll slot,
 * moves take priority over the round-robin PRAD/PRTV scan.
 *
 * Threading:
 *   - request_move(), set_enable(), request_config() may be called from the
 *     comm/control task;
 *   - poll() and everything it touches belongs to the servo-bus task;
 *   - get_states() returns the cache and never touches the UART.
 *
 * @version V1.0 2026-10-31
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __BSP_SERVO_BUS_H__
#define __BSP_SERVO_BUS_H__

//******************************** Includes *********************************//

#include "platform_mcu_uart.h"

//******************************** Defines **********************************//

#define BSP_SERVO_BUS_JOINT_MAX         ((uint8_t)6u)
#define BSP_SERVO_BUS_ID_BROADCAST      ((uint8_t)255u)
#define BSP_SERVO_BUS_ID_AUTO           ((uint8_t)0xFFu)

#define BSP_SERVO_BUS_PULSE_CENTER_US   ((uint16_t)1500u)
#define BSP_SERVO_BUS_PULSE_MIN_US      ((uint16_t)500u)
#define BSP_SERVO_BUS_PULSE_MAX_US      ((uint16_t)2500u)

#define BSP_SERVO_BUS_TEMP_INVALID      ((int16_t)-128)
#define BSP_SERVO_BUS_VOLTAGE_INVALID   ((uint16_t)0xFFFFu)

#define BSP_SERVO_BUS_REPLY_MAX         ((uint8_t)48u)
#define BSP_SERVO_BUS_TX_MAX            ((uint8_t)96u)
#define BSP_SERVO_BUS_TX_TIMEOUT_MS     ((uint32_t)20u)
#define BSP_SERVO_BUS_ID_TIMEOUT_MS     ((uint32_t)150u)

/* The vendor manual: "two commands must be at least 100 ms apart". */
#define BSP_SERVO_BUS_DEFAULT_GAP_MS    ((uint16_t)100u)
#define BSP_SERVO_BUS_DEFAULT_REPLY_MS  ((uint16_t)30u)
#define BSP_SERVO_BUS_DEFAULT_LIMIT_CENTI_DEG ((int16_t)9000)
#define BSP_SERVO_BUS_DEFAULT_OFFLINE_MISSES  ((uint8_t)3u)

/* Fault bits, numerically equal to PLATFORM_SERVO_FAULT_*. */
#define BSP_SERVO_BUS_FAULT_OFFLINE     ((uint8_t)0x01u)
#define BSP_SERVO_BUS_FAULT_REPLY       ((uint8_t)0x20u)

/* Configuration operations, numerically equal to PLATFORM_SERVO_CFG_*. */
#define BSP_SERVO_BUS_CFG_SET_ID        ((uint8_t)0x01u)
#define BSP_SERVO_BUS_CFG_SET_BIAS      ((uint8_t)0x02u)
#define BSP_SERVO_BUS_CFG_CALIBRATE     ((uint8_t)0x03u)
#define BSP_SERVO_BUS_CFG_CLEAR_CALIB   ((uint8_t)0x04u)
#define BSP_SERVO_BUS_CFG_TORQUE        ((uint8_t)0x05u)

#define BSP_SERVO_BUS_BIAS_LIMIT        ((int32_t)100)

//******************************** Declaring ********************************//

typedef struct
{
    platform_uart_bus_t bus;
    uint8_t             joint_count;

    /* Identity and calibrated zero, one row per logical joint. */
    uint8_t             servo_id[BSP_SERVO_BUS_JOINT_MAX];
    int16_t             offset_centi_deg[BSP_SERVO_BUS_JOINT_MAX];

    /* Pulse range and the angle that maps to its two ends. */
    uint16_t            pulse_min_us;
    uint16_t            pulse_max_us;
    int16_t             angle_limit_centi_deg;

    uint16_t            bus_gap_ms;
    uint16_t            reply_timeout_ms;
    uint8_t             offline_misses;
} bsp_servo_bus_cfg_t;

typedef struct
{
    bsp_servo_bus_cfg_t cfg;

    /* Measurements, written by poll() only. */
    int16_t             angle_centi_deg[BSP_SERVO_BUS_JOINT_MAX];
    int16_t             temperature_c[BSP_SERVO_BUS_JOINT_MAX];
    uint16_t            voltage_mv[BSP_SERVO_BUS_JOINT_MAX];
    uint8_t             fault_bits[BSP_SERVO_BUS_JOINT_MAX];
    platform_bool_t     online[BSP_SERVO_BUS_JOINT_MAX];
    uint32_t            angle_ms[BSP_SERVO_BUS_JOINT_MAX];

    /* Move request: sequence-locked, one writer, one consumer. */
    volatile uint32_t   req_seq;
    uint32_t            req_consumed;
    int16_t             req_target[BSP_SERVO_BUS_JOINT_MAX];
    uint16_t            req_move_ms;
    platform_bool_t     has_last;
    int16_t             last_target[BSP_SERVO_BUS_JOINT_MAX];
    uint16_t            last_move_ms;

    /* Enable request: 0 none, 1 disable, 2 enable. */
    volatile uint8_t    enable_req;
    volatile uint8_t    enabled;

    /* Single-depth configuration request. */
    volatile uint8_t    cfg_pending;
    uint8_t             cfg_joint;
    uint8_t             cfg_op;
    int32_t             cfg_value;
    volatile uint8_t    cfg_done;
    volatile platform_err_t cfg_result;
    int32_t             cfg_out;

    /* Reply assembly and the one outstanding transaction. */
    char                rx_line[BSP_SERVO_BUS_REPLY_MAX];
    uint8_t             rx_len;
    uint8_t             await_kind;     /* 0 none, 1 angle, 2 temp, 3 id. */
    uint8_t             await_joint;
    uint32_t            sent_ms;
    uint32_t            last_tx_ms;
    platform_bool_t     tx_seen;

    /*
     * Scan schedule. A full PRAD round (one query per joint, 100 ms apart)
     * feeds the angle cache, then one PRTV fills that joint's temperature and
     * rail voltage. Moves only go out between two queries, so telemetry can
     * never be starved by a host streaming targets.
     */
    uint8_t             poll_joint;
    uint8_t             poll_temp_joint;
    uint8_t             poll_query;     /* 1 PRAD, 2 PRTV. */
    uint8_t             queries_since_move;
    uint8_t             misses[BSP_SERVO_BUS_JOINT_MAX];
    platform_bool_t     initialized;
} bsp_servo_bus_t;

//******************************** Functions ********************************//

/* Initialize the state machine. The UART must already be initialized. */
platform_err_t bsp_servo_bus_init(bsp_servo_bus_t *bus);

platform_err_t bsp_servo_bus_deinit(bsp_servo_bus_t *bus);

/* Queue an enable/disable. The stop/restore frame is sent by the next poll. */
platform_err_t bsp_servo_bus_set_enable(bsp_servo_bus_t *bus,
                                        platform_bool_t enable);

/* Queue a move for all joints. Never blocks. */
platform_err_t bsp_servo_bus_request_move(
                    bsp_servo_bus_t *bus,
                    const int16_t joint_centi_deg[BSP_SERVO_BUS_JOINT_MAX],
                    uint16_t move_time_ms);

/* Run one bus slot. Called from the servo-bus task at ~5 ms. */
void bsp_servo_bus_poll(bsp_servo_bus_t *bus, uint32_t now_ms);

/* Copy the measurement cache. Scalars are written atomically by poll(). */
void bsp_servo_bus_get_states(
                    const bsp_servo_bus_t *bus,
                    int16_t angle_centi_deg[BSP_SERVO_BUS_JOINT_MAX],
                    int16_t temperature_c[BSP_SERVO_BUS_JOINT_MAX],
                    uint16_t voltage_mv[BSP_SERVO_BUS_JOINT_MAX],
                    uint8_t fault_bits[BSP_SERVO_BUS_JOINT_MAX],
                    platform_bool_t online[BSP_SERVO_BUS_JOINT_MAX]);

/* Replace the persisted ID/zero map. Must run before the servo task starts. */
void bsp_servo_bus_set_map(
                    bsp_servo_bus_t *bus,
                    const uint8_t servo_id[BSP_SERVO_BUS_JOINT_MAX],
                    const int16_t offset_centi_deg[BSP_SERVO_BUS_JOINT_MAX]);

/*
 * Queue one configuration operation and return immediately with
 * PLATFORM_ERR_BUSY when another one is still on the bus. The caller polls
 * bsp_servo_bus_config_ready()/take() (04_Impl sleeps between checks).
 */
platform_err_t bsp_servo_bus_request_config(bsp_servo_bus_t *bus,
                                            uint8_t joint,
                                            uint8_t op,
                                            int32_t value);

platform_bool_t bsp_servo_bus_config_ready(const bsp_servo_bus_t *bus);

/* Return the completed result and clear the slot. */
platform_err_t bsp_servo_bus_config_take(bsp_servo_bus_t *bus,
                                         int32_t *out_value);

/* Abandon a request that the caller stopped waiting for. */
void bsp_servo_bus_config_cancel(bsp_servo_bus_t *bus);

#endif /* __BSP_SERVO_BUS_H__ */
