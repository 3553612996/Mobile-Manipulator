/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_bsp_battery.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform battery-monitor device interface.
 *
 * The concrete driver owns the divider ratio and the ADC channel; upper layers
 * consume millivolts only. raw is kept for diagnostics (cell quality, divider
 * calibration) and is expressed in ADC counts.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_BSP_BATTERY_H__
#define __PLATFORM_BSP_BATTERY_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"

//******************************** Declaring ********************************//

typedef struct
{
    uint32_t        voltage_mv;     /* Battery voltage after the divider. */
    uint16_t        raw;            /* Last ADC sample, counts. */
    platform_bool_t valid;          /* False until the first good sample. */
} platform_battery_state_t;

typedef struct
{
    platform_err_t (*init)(void *context);
    platform_err_t (*deinit)(void *context);
    platform_err_t (*read)(void *context, platform_battery_state_t *state);
} platform_battery_ops_t;

/* Only one battery monitor is assumed for this robot. */
platform_err_t platform_battery_register(const platform_battery_ops_t *ops,
                                         void *context);

platform_err_t platform_battery_init(void);
platform_err_t platform_battery_deinit(void);
platform_err_t platform_battery_read(platform_battery_state_t *state);

#endif /* __PLATFORM_BSP_BATTERY_H__ */
