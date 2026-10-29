/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file service_proto.h
 *
 * @par dependencies
 * - platform_mcu_uart.h
 * - protocol_codec.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Serial protocol service: framing, dispatch, ACK, reports.
 *
 * Why this is a service and not part of the application
 * -----------------------------------------------------
 * The application expresses intent ("forward", "register a handler for
 * SET_ENABLE"). Everything between that intent and the wire belongs here:
 * partial-frame reassembly, CRC, length checks, SEQ de-duplication, ACK
 * generation, report serialization and link statistics. Keeping it in a
 * service means the same protocol can drive a different app, and the app can
 * be tested with handler calls instead of byte streams.
 *
 * The protocol table itself (command bytes, payload lengths, result and fault
 * codes) lives one layer down in 05_Middlewares/protocol; this service does
 * not define any wire constant of its own.
 *
 * Threading: register, process and every send function are called from one
 * task. The UART RX interrupt only notifies that task, and the TX path is
 * serialized by that single caller.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __SERVICE_PROTO_H__
#define __SERVICE_PROTO_H__

//******************************** Includes *********************************//

#include "platform_mcu_uart.h"
#include "protocol_codec.h"

//******************************** Defines **********************************//

#define SERVICE_PROTO_MAX_HANDLERS     (8u)
#define SERVICE_PROTO_RX_CHUNK         (32u)
#define SERVICE_PROTO_TX_TIMEOUT_MS    (20u)
#define SERVICE_PROTO_DEDUPE_MS        (1000u)
#define SERVICE_PROTO_CRC_ALARM_FRAMES (10u)

//******************************** Declaring ********************************//

/*
 * Return the result code that goes into the ACK. For commands that do not
 * require an ACK the value is ignored, but returning the right code keeps the
 * handler honest and lets a future version add ACKs without touching it.
 */
typedef protocol_result_t (*service_proto_handler_t)(
                                        const protocol_frame_t *frame,
                                        void *user_data);

typedef struct
{
    platform_uart_bus_t bus;
} service_proto_cfg_t;

typedef struct
{
    uint32_t    frames_ok;
    uint32_t    crc_errors;
    uint32_t    len_errors;
    uint32_t    resyncs;
    uint32_t    unknown_cmds;
    uint32_t    acks_sent;
    uint32_t    tx_failures;
} service_proto_stats_t;

platform_err_t service_proto_init(const service_proto_cfg_t *cfg);

/*
 * Register the handler for one downstream command. The command must exist in
 * the protocol table; registering it twice is an error.
 */
platform_err_t service_proto_register(uint8_t cmd,
                                      service_proto_handler_t handler,
                                      void *user_data);

/*
 * Drain the UART ring buffer and dispatch every complete frame. now_ms is the
 * caller's millisecond clock, used for the 50 ms parser reset and SEQ dedupe.
 * Call at 10 ms or whenever the RX event wakes the task.
 */
platform_err_t service_proto_process(uint32_t now_ms);

/* ------------------------------- Reports ------------------------------- */

platform_err_t service_proto_send_odom(int32_t x_mm,
                                       int32_t y_mm,
                                       int16_t heading_centi_deg,
                                       int16_t vl_mm_s,
                                       int16_t vr_mm_s);

platform_err_t service_proto_send_arm(
                            const int16_t joint_centi_deg
                                [PROTOCOL_ARM_JOINT_COUNT],
                            int16_t gripper_position,
                            uint8_t gripper_state);

/*
 * Measured wheel speeds in FL, FR, RL, RR order, mm/s. Sent at 50 Hz while
 * the host is tuning so the effect of a gain change is visible.
 */
platform_err_t service_proto_send_wheel_speed(
                    const int16_t wheel_mm_s[PROTOCOL_WHEEL_COUNT]);

/*
 * Applied motor duty in FL, FR, RL, RR order, permille of full scale. Signed:
 * the sign is the direction the bridge was told to drive. Sent next to the
 * wheel speed so a tuning host can see saturation and integral wind-up.
 */
platform_err_t service_proto_send_wheel_duty(
                    const int16_t wheel_permille[PROTOCOL_WHEEL_COUNT]);

platform_err_t service_proto_send_status(uint8_t state,
                                         uint16_t fault_code,
                                         uint16_t battery_mv,
                                         int8_t temperature_c,
                                         uint16_t max_jitter_us);

platform_err_t service_proto_send_event(uint16_t event_code,
                                        uint32_t timestamp_ms);

platform_err_t service_proto_send_heartbeat(uint8_t state,
                                            uint16_t run_counter);

/* -------------------------- Link diagnostics --------------------------- */

/* Milliseconds since the last valid downstream frame, capped at UINT32_MAX. */
uint32_t service_proto_rx_age_ms(uint32_t now_ms);

platform_bool_t service_proto_has_received(void);

/* PLATFORM_TRUE after SERVICE_PROTO_CRC_ALARM_FRAMES consecutive CRC errors. */
platform_bool_t service_proto_crc_alarm(void);

void service_proto_get_stats(service_proto_stats_t *out);

#endif /* __SERVICE_PROTO_H__ */
