/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file bsp_battery_adc.c
 *
 * @par dependencies
 * - 05_BSP/bsp_battery_adc.h
 * - platform_mcu_adc.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Battery-divider driver on top of the platform ADC.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "bsp_battery_adc.h"

//******************************** Defines **********************************//

#define BSP_BATTERY_DIVIDER_RATIO_DEFAULT   ((uint16_t)11u)

//******************************** Functions ********************************//

platform_err_t bsp_battery_adc_init(bsp_battery_adc_t *battery)
{
    platform_adc_cfg_t adc_cfg;
    platform_err_t     err;

    if (battery == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (battery->initialized == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    if (battery->cfg.divider_ratio == 0u)
    {
        battery->cfg.divider_ratio = BSP_BATTERY_DIVIDER_RATIO_DEFAULT;
    }

    adc_cfg.channel      = battery->cfg.channel;
    adc_cfg.reference_mv = battery->cfg.reference_mv;

    err = platform_adc_init(&adc_cfg);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    battery->voltage_mv  = 0u;
    battery->raw         = 0u;
    battery->valid       = PLATFORM_FALSE;
    battery->initialized = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t bsp_battery_adc_deinit(bsp_battery_adc_t *battery)
{
    if (battery == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (battery->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    (void)platform_adc_deinit(battery->cfg.channel);

    battery->valid       = PLATFORM_FALSE;
    battery->initialized = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

platform_err_t bsp_battery_adc_read(bsp_battery_adc_t *battery)
{
    uint16_t       raw;
    uint32_t       adc_mv;
    platform_err_t err;

    if (battery == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (battery->initialized != PLATFORM_TRUE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = platform_adc_read_raw(battery->cfg.channel, &raw);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = platform_adc_read_mv(battery->cfg.channel, &adc_mv);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    battery->raw        = raw;
    battery->voltage_mv = adc_mv * (uint32_t)battery->cfg.divider_ratio;
    battery->valid      = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

uint32_t bsp_battery_adc_get_mv(const bsp_battery_adc_t *battery)
{
    return (battery == NULL) ? 0u : battery->voltage_mv;
}
