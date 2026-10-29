/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_battery.c
 *
 * @par dependencies
 * - app_battery.h
 * - platform_bsp_battery.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Read the pack voltage monitor.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "app_robot/app_battery.h"
#include "app_robot/app_robot_config.h"

#include "platform_bsp_battery.h"

//******************************** Functions ********************************//

uint16_t app_battery_mv(void)
{
    platform_battery_state_t battery;

    if ((platform_battery_read(&battery) != PLATFORM_ERR_OK) ||
        (battery.valid != PLATFORM_TRUE))
    {
        return 0u;
    }

    return (battery.voltage_mv > 0xFFFFu) ? (uint16_t)0xFFFFu
                                          : (uint16_t)battery.voltage_mv;
}

platform_bool_t app_battery_low(void)
{
    uint16_t mv = app_battery_mv();

    return ((mv != 0u) && (mv < APP_BATTERY_LOW_MV)) ? PLATFORM_TRUE
                                                     : PLATFORM_FALSE;
}
