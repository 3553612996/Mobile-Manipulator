/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_fault.h
 *
 * @par dependencies
 * - platform_def.h
 * - protocol_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Fault latch, severity arbitration and degradation policy.
 *
 * REPORT_STATUS carries one fault code. The latch keeps the most severe one:
 * FATAL > ERROR > WARN, and an equal severity keeps the first arrival. The
 * degradation action follows the severity:
 *
 *   WARN   report only; the robot keeps running.
 *   ERROR  stop the affected subsystem (chassis or arm) and enter FAULT.
 *   FATAL  stop everything and enter ESTOP.
 *
 * app_fault_raise() sends REPORT_EVENT, so it must run in the comm task, the
 * only owner of the UART transmit path.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __APP_FAULT_H__
#define __APP_FAULT_H__

//******************************** Includes *********************************//

#include "platform_def.h"
#include "protocol_def.h"

//******************************** Declaring ********************************//

typedef enum
{
    APP_FAULT_SCOPE_NONE = 0,
    APP_FAULT_SCOPE_CHASSIS,
    APP_FAULT_SCOPE_ARM,
    APP_FAULT_SCOPE_ALL
} app_fault_scope_t;

void app_fault_init(void);

uint16_t app_fault_code(void);

/* Degradation scope for a code, independent of the current latch. */
app_fault_scope_t app_fault_scope_get(uint16_t code);

/*
 * Latch a fault and apply its degradation action.
 * force overrides the severity arbitration (the emergency stop is explicit).
 * Returns TRUE when the code was accepted and an event was sent.
 */
platform_bool_t app_fault_raise(uint16_t code, uint32_t now_ms,
                                platform_bool_t force);

/* Clear one code, CLEAR_ALL, or the emergency stop. */
protocol_result_t app_fault_clear(uint16_t code, uint32_t now_ms);

/*
 * Edge detection for conditions that are polled rather than pushed:
 * 200 ms command timeout and the consecutive-CRC alarm. Call once per comm
 * task iteration, after service_proto_process().
 */
void app_fault_poll(uint32_t now_ms);

#endif /* __APP_FAULT_H__ */
