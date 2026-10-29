/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_robot.h
 *
 * @par dependencies
 * - platform_error.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Wire the robot services together and own the application tasks.
 *
 * The app contains no kinematics, no protocol bytes and no device calls. It is
 * split into modules:
 *   app_user_init  services, queues and the task/queue tables;
 *   app_handlers   protocol handlers and key gestures;
 *   app_state      explicit INIT/IDLE/MANUAL/AUTO/ESTOP/FAULT state machine;
 *   app_fault      fault latch, severity arbitration and degradation policy;
 *   app_report     periodic reports and the status LED;
 *   app_battery    pack voltage helpers.
 *
 * Tasks
 * -----
 *   comm     wakes on the UART RX event or every 10 ms: parse + ACK, fault
 *            poll, then reports (ODOM/WHEEL 50 Hz, ARM 20 Hz, STATUS and
 *            HEARTBEAT 10 Hz). It is the only task that transmits.
 *   control  10 ms: key sampling, PID parameter queue, chassis watchdog/apply,
 *            odometry, buzzer.
 *
 * @version V1.1 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __APP_ROBOT_H__
#define __APP_ROBOT_H__

//******************************** Includes *********************************//

#include "platform_error.h"

//******************************** Declaring ********************************//

/*
 * Called from main() after board_init() and before the scheduler starts.
 * Initializes the UART DMA transport and the services; no tasks yet.
 */
platform_err_t app_robot_init(void);

/*
 * Called from the first FreeRTOS task. Creates the app tasks and hooks the
 * UART RX event to the comm task; from here on the robot runs.
 */
platform_err_t app_robot_start(void);

#endif /* __APP_ROBOT_H__ */
