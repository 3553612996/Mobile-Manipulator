/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file protocol_def.h
 *
 * @par dependencies
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Define the wire protocol table (docs/architecture/protocol.md v1).
 *
 * This file is the single source of truth for frame constants, command and
 * report identifiers, payload lengths, result codes, fault codes and the
 * state byte of REPORT_STATUS. The codec, the service layer and the PC tests
 * all include this table; no caller may hard-code a command byte.
 *
 * The table is deliberately pure data + pure functions: no HAL, no FreeRTOS,
 * no UART. It can be compiled unchanged for the PC unit tests.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PROTOCOL_DEF_H__
#define __PROTOCOL_DEF_H__

//******************************** Includes *********************************//

#include "platform_def.h"

//******************************** Defines **********************************//

/* Frame: AA 55 LEN CMD SEQ PAYLOAD CRC_L CRC_H (little endian). */
#define PROTOCOL_MAGIC_0                ((uint8_t)0xAAu)
#define PROTOCOL_MAGIC_1                ((uint8_t)0x55u)
#define PROTOCOL_LEN_MIN                ((uint8_t)2u)   /* CMD + SEQ. */
#define PROTOCOL_LEN_MAX                ((uint8_t)66u)
#define PROTOCOL_PAYLOAD_MAX            ((uint8_t)(PROTOCOL_LEN_MAX - 2u))
#define PROTOCOL_FRAME_OVERHEAD         ((uint8_t)5u)   /* AA 55 LEN + CRC16. */
#define PROTOCOL_FRAME_MAX              ((uint8_t)(PROTOCOL_LEN_MAX + \
                                                   PROTOCOL_FRAME_OVERHEAD))

/* ACK command = command | 0x80. Reports live in 0xC0..0xFF. */
#define PROTOCOL_ACK_FLAG               ((uint8_t)0x80u)
#define PROTOCOL_CMD_MAX                ((uint8_t)0x3Fu)
#define PROTOCOL_REPORT_MIN             ((uint8_t)0xC0u)

/* Parser resets if two bytes are further apart than this
 * (docs/architecture/protocol.md 9). */
#define PROTOCOL_RX_GAP_RESET_MS        ((uint32_t)50u)

/* Status byte of REPORT_STATUS / REPORT_HEARTBEAT. */
typedef enum
{
    PROTOCOL_STATE_INIT = 0,
    PROTOCOL_STATE_IDLE,
    PROTOCOL_STATE_MANUAL,
    PROTOCOL_STATE_AUTO,
    PROTOCOL_STATE_ESTOP,
    PROTOCOL_STATE_FAULT
} protocol_state_t;

/* Host -> MCU commands. */
typedef enum
{
    PROTOCOL_CMD_SET_VELOCITY = 0x01,
    PROTOCOL_CMD_SET_ARM_JOINTS = 0x02,
    PROTOCOL_CMD_SET_GRIPPER = 0x03,
    PROTOCOL_CMD_SET_ENABLE = 0x04,
    PROTOCOL_CMD_SET_PARAM = 0x05,
    PROTOCOL_CMD_ESTOP = 0x06,
    PROTOCOL_CMD_CLEAR_FAULT = 0x07
} protocol_cmd_t;

/* MCU -> Host periodic/event reports. */
typedef enum
{
    PROTOCOL_REPORT_ODOM = 0xC1,
    PROTOCOL_REPORT_ARM = 0xC2,
    PROTOCOL_REPORT_STATUS = 0xC3,
    PROTOCOL_REPORT_EVENT = 0xC4,
    PROTOCOL_REPORT_HEARTBEAT = 0xC5,
    PROTOCOL_REPORT_WHEEL_SPEED = 0xC6
} protocol_report_t;

/* Second byte of an ACK payload. */
typedef enum
{
    PROTOCOL_RESULT_OK = 0,
    PROTOCOL_RESULT_RANGE = 1,      /* Value outside the allowed range. */
    PROTOCOL_RESULT_STATE = 2,      /* Command not allowed in this state. */
    PROTOCOL_RESULT_UNKNOWN_CMD = 3,
    PROTOCOL_RESULT_FAILED = 4      /* Execution failed (e.g. joint limit). */
} protocol_result_t;

/* Fixed payload sizes, in bytes. Keep in sync with the table below. */
#define PROTOCOL_LEN_SET_VELOCITY       ((uint8_t)4u)
#define PROTOCOL_LEN_SET_ARM_JOINTS     ((uint8_t)12u)
#define PROTOCOL_LEN_SET_GRIPPER        ((uint8_t)2u)
#define PROTOCOL_LEN_SET_ENABLE         ((uint8_t)1u)
#define PROTOCOL_LEN_SET_PARAM          ((uint8_t)5u)
#define PROTOCOL_LEN_ESTOP              ((uint8_t)0u)
#define PROTOCOL_LEN_CLEAR_FAULT        ((uint8_t)2u)

#define PROTOCOL_ARM_JOINT_COUNT        ((uint8_t)6u)

#define PROTOCOL_LEN_REPORT_ODOM        ((uint8_t)14u)
#define PROTOCOL_LEN_REPORT_ARM         ((uint8_t)15u)
#define PROTOCOL_LEN_REPORT_STATUS      ((uint8_t)8u)
#define PROTOCOL_LEN_REPORT_EVENT       ((uint8_t)6u)
#define PROTOCOL_LEN_REPORT_HEARTBEAT   ((uint8_t)3u)
#define PROTOCOL_LEN_REPORT_WHEEL_SPEED ((uint8_t)8u)

/* Wheel order in REPORT_WHEEL_SPEED and in the per-wheel PID ids. */
#define PROTOCOL_WHEEL_FL               ((uint8_t)0u)
#define PROTOCOL_WHEEL_FR               ((uint8_t)1u)
#define PROTOCOL_WHEEL_RL               ((uint8_t)2u)
#define PROTOCOL_WHEEL_RR               ((uint8_t)3u)
#define PROTOCOL_WHEEL_COUNT            ((uint8_t)4u)

/* Fault / event codes (docs/architecture/protocol.md 8). */
#define PROTOCOL_FAULT_OVERCURRENT      ((uint16_t)0x0101u)
#define PROTOCOL_FAULT_ENCODER          ((uint16_t)0x0102u)
#define PROTOCOL_FAULT_CRC_LIMIT        ((uint16_t)0x0201u)
#define PROTOCOL_FAULT_COMM_TIMEOUT     ((uint16_t)0x0202u)
#define PROTOCOL_FAULT_JOINT_LIMIT      ((uint16_t)0x0301u)
#define PROTOCOL_FAULT_JOINT_STALL      ((uint16_t)0x0302u)
#define PROTOCOL_FAULT_TRAJECTORY       ((uint16_t)0x0303u)
#define PROTOCOL_FAULT_BATTERY_LOW      ((uint16_t)0x0401u)
#define PROTOCOL_FAULT_OVERTEMP         ((uint16_t)0x0402u)
#define PROTOCOL_FAULT_ESTOP            ((uint16_t)0x0501u)

#define PROTOCOL_CLEAR_ALL_FAULTS       ((uint16_t)0xFFFFu)

/* Fault severity (docs/architecture/protocol.md 8).
 * A larger value outranks a smaller one. */
typedef enum
{
    PROTOCOL_SEVERITY_NONE = 0,
    PROTOCOL_SEVERITY_WARN,
    PROTOCOL_SEVERITY_ERROR,
    PROTOCOL_SEVERITY_FATAL
} protocol_severity_t;

/* SET_PARAM ids (docs/architecture/protocol.md 5.4). */
#define PROTOCOL_PARAM_WHEEL_BASE       ((uint8_t)0x01u)
#define PROTOCOL_PARAM_WHEEL_DIAMETER   ((uint8_t)0x02u)
#define PROTOCOL_PARAM_CMD_TIMEOUT      ((uint8_t)0x10u)
#define PROTOCOL_PARAM_MAX_LINEAR       ((uint8_t)0x11u)
#define PROTOCOL_PARAM_MAX_ANGULAR      ((uint8_t)0x12u)

/*
 * Per-wheel PID gains, value = gain * 1000. 0x20..0x2B in A/B/C/D order.
 * 0x03..0x08 remain as left/right aliases for existing host builds:
 * 0x03..0x05 write FL and RL, 0x06..0x08 write FR and RR.
 */
#define PROTOCOL_PARAM_PID_A_KP         ((uint8_t)0x20u)
#define PROTOCOL_PARAM_PID_A_KI         ((uint8_t)0x21u)
#define PROTOCOL_PARAM_PID_A_KD         ((uint8_t)0x22u)
#define PROTOCOL_PARAM_PID_B_KP         ((uint8_t)0x23u)
#define PROTOCOL_PARAM_PID_B_KI         ((uint8_t)0x24u)
#define PROTOCOL_PARAM_PID_B_KD         ((uint8_t)0x25u)
#define PROTOCOL_PARAM_PID_C_KP         ((uint8_t)0x26u)
#define PROTOCOL_PARAM_PID_C_KI         ((uint8_t)0x27u)
#define PROTOCOL_PARAM_PID_C_KD         ((uint8_t)0x28u)
#define PROTOCOL_PARAM_PID_D_KP         ((uint8_t)0x29u)
#define PROTOCOL_PARAM_PID_D_KI         ((uint8_t)0x2Au)
#define PROTOCOL_PARAM_PID_D_KD         ((uint8_t)0x2Bu)

//******************************** Declaring ********************************//

typedef struct
{
    uint8_t         cmd;
    uint8_t         payload_len;    /* Exact payload size, not a maximum. */
    platform_bool_t requires_ack;   /* State-changing commands. */
    const char *    name;           /* For traces only; may be NULL. */
} protocol_cmd_info_t;

/*
 * Look up a downstream command. Returns NULL for unknown, ACK and report
 * identifiers; callers must handle NULL as "not a host command".
 */
const protocol_cmd_info_t *protocol_cmd_lookup(uint8_t cmd);

/* TRUE when payload_len is exactly the size the table promises. */
platform_bool_t protocol_cmd_payload_ok(uint8_t cmd, uint8_t payload_len);

/*
 * Severity of a fault code. Unknown codes come back as ERROR: a code this
 * table has not learned yet must not lose to a WARN that is already latched.
 */
protocol_severity_t protocol_fault_severity(uint16_t code);

#endif /* __PROTOCOL_DEF_H__ */
