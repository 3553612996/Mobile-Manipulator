/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_report.h
 *
 * @par dependencies
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Periodic REPORT_* scheduling and the status LED.
 *
 * Called from the comm task only: it is the single UART transmitter. The
 * periods live in app_robot_config.h so a host-side observer can be matched
 * without reading this file.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __APP_REPORT_H__
#define __APP_REPORT_H__

//******************************** Includes *********************************//

#include "platform_def.h"

//******************************** Declaring ********************************//

void app_report_init(void);

/* One scheduling tick. Call every comm-task iteration with the same clock. */
void app_report_tick(uint32_t now_ms);

#endif /* __APP_REPORT_H__ */
