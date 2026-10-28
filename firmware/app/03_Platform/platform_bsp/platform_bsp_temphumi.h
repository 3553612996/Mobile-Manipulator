/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_bsp_temphumi.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform temperature/humidity sensor interface.
 *
 * Values are fixed-point centi-units so the platform interface stays free of
 * float and matches the host protocol style. Convert to float only at the
 * display/log boundary if needed.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_BSP_TEMPHUMI_H__
#define __PLATFORM_BSP_TEMPHUMI_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"

//******************************** Declaring ********************************//

typedef struct
{
    int16_t         temperature_centi_c;    /* 0.01 degC. */
    uint16_t        humidity_centi_rh;      /* 0.01 %RH. */
    platform_bool_t valid;                  /* FALSE when the sample is stale. */
} platform_temphumi_data_t;

typedef struct
{
    platform_err_t (*init)(void *context);
    platform_err_t (*deinit)(void *context);
    platform_err_t (*read)(void *context,
                           platform_temphumi_data_t *data,
                           uint32_t timeout_ms);
} platform_temphumi_ops_t;

/*
 * Register the sensor implementation. Must be called before
 * platform_temphumi_init(). Only one on-board temperature/humidity sensor is
 * assumed.
 */
platform_err_t platform_temphumi_register(
                                        const platform_temphumi_ops_t *ops,
                                        void *context);

platform_err_t platform_temphumi_init(void);
platform_err_t platform_temphumi_deinit(void);

platform_err_t platform_temphumi_read(platform_temphumi_data_t *data,
                                      uint32_t timeout_ms);

#endif /* __PLATFORM_BSP_TEMPHUMI_H__ */
