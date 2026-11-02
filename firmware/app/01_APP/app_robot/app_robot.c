/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_robot.c
 *
 * @par dependencies
 * - app_robot/app_robot.h
 * - app_robot/app_user_init.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Composition root of the application.
 *
 * The robot is split into small modules; this file only preserves the public
 * init/start pair that main() already calls:
 *
 *   app_robot_init()  -> app_user_init_init()
 *   app_robot_start() -> app_user_init_start()
 *
 * @version V1.1 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "app_robot/app_robot.h"
#include "app_robot/app_user_init.h"

//******************************** Functions ********************************//

platform_err_t app_robot_init(void)
{
    return app_user_init_init();
}

platform_err_t app_robot_start(void)
{
    return app_user_init_start();
}
