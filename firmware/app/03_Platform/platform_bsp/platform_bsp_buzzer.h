/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_bsp_buzzer.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform buzzer device interface.
 *
 * The device contract only switches the buzzer; beep patterns (count, on/off
 * duration) belong to the application so that the BSP never blocks a task with
 * its own delays.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_BSP_BUZZER_H__
#define __PLATFORM_BSP_BUZZER_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"

//******************************** Declaring ********************************//

typedef struct
{
    platform_err_t (*init)(void *context);
    platform_err_t (*deinit)(void *context);
    platform_err_t (*set)(void *context, platform_bool_t on);
} platform_buzzer_ops_t;

/* Only one buzzer is assumed for this robot. */
platform_err_t platform_buzzer_register(const platform_buzzer_ops_t *ops,
                                        void *context);

platform_err_t platform_buzzer_init(void);
platform_err_t platform_buzzer_deinit(void);
platform_err_t platform_buzzer_set(platform_bool_t on);

#endif /* __PLATFORM_BSP_BUZZER_H__ */
