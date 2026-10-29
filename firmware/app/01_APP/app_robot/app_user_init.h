/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_user_init.h
 *
 * @par dependencies
 * - platform_os_queue.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief User-side initialization: services, queues, tasks and their tables.
 *
 * Kept separate from app_robot.c so the composition root stays three lines.
 * Queue creation runs before the scheduler (FreeRTOS allows it); task creation
 * runs from the first task, because the current port creates tasks only after
 * osKernelStart(). Both loops walk a constant table.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __APP_USER_INIT_H__
#define __APP_USER_INIT_H__

//******************************** Includes *********************************//

#include "platform_os_queue.h"

//******************************** Declaring ********************************//

/* Init transport and services, register handlers, create queues. No tasks. */
platform_err_t app_user_init_init(void);

/* Create the tasks from the table and hook the UART event. */
platform_err_t app_user_init_start(void);

/* Comm -> control PID parameter queue. NULL until init has run. */
platform_os_queue_t app_user_init_pid_queue(void);

#endif /* __APP_USER_INIT_H__ */
