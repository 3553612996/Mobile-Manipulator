/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_mcu_gpio.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform GPIO interface.
 *
 * This header is part of the frozen Platform contract. Upper layers call it;
 * the concrete HAL/LL/register implementation belongs to 04_Impl.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_MCU_GPIO_H__
#define __PLATFORM_MCU_GPIO_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"

//******************************** Defines **********************************//

#define PLATFORM_GPIO_PORT_MAX      16u
#define PLATFORM_GPIO_PIN_MAX       16u

//******************************** Declaring ********************************//

/*
 * Logical pin descriptor. The port number is logical (0..15) and is mapped to
 * GPIOA/GPIOB/... by 04_Impl/impl_board. Do not pass GPIOA directly.
 */
typedef struct
{
    uint8_t port;
    uint8_t pin;
} platform_gpio_t;

typedef enum
{
    PLATFORM_GPIO_MODE_INPUT = 0,
    PLATFORM_GPIO_MODE_OUTPUT_PP,
    PLATFORM_GPIO_MODE_OUTPUT_OD,
    PLATFORM_GPIO_MODE_ANALOG
} platform_gpio_mode_t;

typedef enum
{
    PLATFORM_GPIO_PULL_NONE = 0,
    PLATFORM_GPIO_PULL_UP,
    PLATFORM_GPIO_PULL_DOWN
} platform_gpio_pull_t;

typedef enum
{
    PLATFORM_GPIO_LEVEL_LOW = 0,
    PLATFORM_GPIO_LEVEL_HIGH = 1
} platform_gpio_level_t;

typedef enum
{
    PLATFORM_GPIO_IRQ_EDGE_RISING = 0,
    PLATFORM_GPIO_IRQ_EDGE_FALLING,
    PLATFORM_GPIO_IRQ_EDGE_BOTH
} platform_gpio_irq_edge_t;

typedef struct
{
    platform_gpio_t         pin;
    platform_gpio_mode_t    mode;
    platform_gpio_pull_t    pull;
    platform_gpio_level_t   initial_level;  /* Only used by output modes. */
} platform_gpio_cfg_t;

/*
 * Interrupt callback. It runs in ISR context: it must not block, allocate or
 * print. Keep it to a flag/notification only.
 */
typedef void (*platform_gpio_irq_cb_t)(platform_gpio_t pin, void *user_data);

platform_err_t platform_gpio_init(const platform_gpio_cfg_t *cfg);
platform_err_t platform_gpio_deinit(platform_gpio_t pin);

platform_err_t platform_gpio_write(platform_gpio_t pin,
                                   platform_gpio_level_t level);
platform_err_t platform_gpio_read(platform_gpio_t pin,
                                  platform_gpio_level_t *level);
platform_err_t platform_gpio_toggle(platform_gpio_t pin);

platform_err_t platform_gpio_irq_attach(platform_gpio_t pin,
                                        platform_gpio_irq_edge_t edge,
                                        platform_gpio_irq_cb_t callback,
                                        void *user_data);
platform_err_t platform_gpio_irq_detach(platform_gpio_t pin);

#endif /* __PLATFORM_MCU_GPIO_H__ */
