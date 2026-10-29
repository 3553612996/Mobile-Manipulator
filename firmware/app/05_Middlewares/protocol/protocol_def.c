/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file protocol_def.c
 *
 * @par dependencies
 * - protocol_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement the protocol command table.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "protocol_def.h"

//******************************** Variables ********************************//

/*
 * One row per downstream command. Keeping the name here means a trace can
 * print "SET_ENABLE" without a second lookup table in the service layer.
 * SET_ARM_JOINTS and SET_VELOCITY deliberately do not require an ACK: a lost
 * joint frame is corrected by the next one and velocity has the 200 ms stop.
 *
 * The storage commands 0x08..0x0D are listed here from the moment the app
 * layer registers handlers for them; until then they were absent so the
 * firmware answered 0x03 (unknown), as protocol.md promised. Their ACK means
 * "accepted", not "written": the erase happens in the storage task and the
 * outcome arrives in REPORT_STORAGE.
 */
static const protocol_cmd_info_t s_cmd_table[] =
{
    { PROTOCOL_CMD_SET_VELOCITY,   PROTOCOL_LEN_SET_VELOCITY,
      PLATFORM_FALSE, "SET_VELOCITY" },
    { PROTOCOL_CMD_SET_ARM_JOINTS, PROTOCOL_LEN_SET_ARM_JOINTS,
      PLATFORM_FALSE, "SET_ARM_JOINTS" },
    { PROTOCOL_CMD_SET_GRIPPER,    PROTOCOL_LEN_SET_GRIPPER,
      PLATFORM_TRUE,  "SET_GRIPPER" },
    { PROTOCOL_CMD_SET_ENABLE,     PROTOCOL_LEN_SET_ENABLE,
      PLATFORM_TRUE,  "SET_ENABLE" },
    { PROTOCOL_CMD_SET_PARAM,      PROTOCOL_LEN_SET_PARAM,
      PLATFORM_TRUE,  "SET_PARAM" },
    { PROTOCOL_CMD_ESTOP,          PROTOCOL_LEN_ESTOP,
      PLATFORM_TRUE,  "ESTOP" },
    { PROTOCOL_CMD_CLEAR_FAULT,    PROTOCOL_LEN_CLEAR_FAULT,
      PLATFORM_TRUE,  "CLEAR_FAULT" },
    { PROTOCOL_CMD_PARAM_SAVE,     PROTOCOL_LEN_PARAM_SAVE,
      PLATFORM_TRUE,  "PARAM_SAVE" },
    { PROTOCOL_CMD_PARAM_RESET,    PROTOCOL_LEN_PARAM_RESET,
      PLATFORM_TRUE,  "PARAM_RESET" },
    { PROTOCOL_CMD_STORAGE_QUERY,  PROTOCOL_LEN_STORAGE_QUERY,
      PLATFORM_TRUE,  "STORAGE_QUERY" },
    { PROTOCOL_CMD_LOG_EXPORT,     PROTOCOL_LEN_LOG_EXPORT,
      PLATFORM_TRUE,  "LOG_EXPORT" },
    { PROTOCOL_CMD_CRASH_READ,     PROTOCOL_LEN_CRASH_READ,
      PLATFORM_TRUE,  "CRASH_READ" },
    { PROTOCOL_CMD_LOG_CLEAR,      PROTOCOL_LEN_LOG_CLEAR,
      PLATFORM_TRUE,  "LOG_CLEAR" }
};

#define PROTOCOL_CMD_TABLE_SIZE \
    ((uint8_t)(sizeof(s_cmd_table) / sizeof(s_cmd_table[0])))

//******************************** Functions ********************************//

const protocol_cmd_info_t *protocol_cmd_lookup(uint8_t cmd)
{
    uint8_t i;

    for (i = 0u; i < PROTOCOL_CMD_TABLE_SIZE; i++)
    {
        if (s_cmd_table[i].cmd == cmd)
        {
            return &s_cmd_table[i];
        }
    }

    return NULL;
}

platform_bool_t protocol_cmd_payload_ok(uint8_t cmd, uint8_t payload_len)
{
    const protocol_cmd_info_t *info = protocol_cmd_lookup(cmd);

    if (info == NULL)
    {
        return PLATFORM_FALSE;
    }

    return (info->payload_len == payload_len) ? PLATFORM_TRUE
                                              : PLATFORM_FALSE;
}

protocol_severity_t protocol_fault_severity(uint16_t code)
{
    /*
     * REPORT_STATUS carries a single fault code, so when two conditions exist
     * at once only one is visible. Severity decides which: a comm timeout must
     * not disappear behind a low battery that latched 200 ms earlier.
     */
    switch (code)
    {
        case PROTOCOL_FAULT_CRC_LIMIT:
        case PROTOCOL_FAULT_BATTERY_LOW:
        case PROTOCOL_FAULT_OVERTEMP:
        case PROTOCOL_FAULT_PARAM_INVALID:
        case PROTOCOL_FAULT_CALIB_INVALID:
            return PROTOCOL_SEVERITY_WARN;

        case PROTOCOL_FAULT_OVERCURRENT:
        case PROTOCOL_FAULT_ENCODER:
        case PROTOCOL_FAULT_JOINT_LIMIT:
        case PROTOCOL_FAULT_JOINT_STALL:
        case PROTOCOL_FAULT_TRAJECTORY:
        case PROTOCOL_FAULT_STORAGE_WRITE:
        case PROTOCOL_FAULT_PARTITION:
            return PROTOCOL_SEVERITY_ERROR;

        case PROTOCOL_FAULT_COMM_TIMEOUT:
        case PROTOCOL_FAULT_ESTOP:
            return PROTOCOL_SEVERITY_FATAL;

        default:
            return PROTOCOL_SEVERITY_ERROR;
    }
}
