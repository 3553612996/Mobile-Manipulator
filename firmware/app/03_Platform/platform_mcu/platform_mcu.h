/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_mcu.h
 *
 * @par dependencies
 * - platform_mcu_gpio.h
 * - platform_mcu_uart.h
 * - platform_mcu_spi.h
 * - platform_mcu_i2c.h
 * - platform_mcu_pwm.h
 * - platform_mcu_adc.h
 * - platform_mcu_wdg.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Aggregate header for the platform MCU interfaces.
 *
 * Upper layers include this file only. They must not include stm32f4xx_hal.h
 * or any vendor header.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_MCU_H__
#define __PLATFORM_MCU_H__

//******************************** Includes *********************************//

#include "platform_mcu_gpio.h"
#include "platform_mcu_uart.h"
#include "platform_mcu_spi.h"
#include "platform_mcu_i2c.h"
#include "platform_mcu_pwm.h"
#include "platform_mcu_adc.h"
#include "platform_mcu_wdg.h"

#endif /* __PLATFORM_MCU_H__ */
