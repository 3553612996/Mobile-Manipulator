/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_handlers.h
 *
 * @par dependencies
 * - service_proto.h
 * - service_key.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Protocol command handlers and the on-board key gesture.
 *
 * Handlers translate wire fields into service calls and results; they do not
 * touch frame bytes (service_proto already did that) or report scheduling.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __APP_HANDLERS_H__
#define __APP_HANDLERS_H__

//******************************** Includes *********************************//

#include "service_proto/service_proto.h"
#include "service_key/service_key.h"

//******************************** Declaring ********************************//

/* Register one handler per downstream command. Call before the scheduler. */
platform_err_t app_handlers_register(void);

/* service_key callback. Runs in the control task. */
void app_handlers_on_key(platform_key_id_t id, service_key_event_t event,
                         void *user_data);

/*
 * Turn a control-task emergency-stop request into a latched fault and an
 * EVENT frame. Comm task only, after service_proto_process().
 */
void app_handlers_process_pending(uint32_t now_ms);

#endif /* __APP_HANDLERS_H__ */
