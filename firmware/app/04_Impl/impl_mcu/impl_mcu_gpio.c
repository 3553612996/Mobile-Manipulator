/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_mcu_gpio.c
 *
 * @par dependencies
 * - impl_mcu/impl_mcu_gpio.h
 * - stm32f1xx_hal.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Adapt the platform GPIO contract to the STM32F1 HAL.
 *
 * Pins that the board also uses as a peripheral alternate function are
 * configured by CubeMX at start-up. This adapter only serves the plain
 * GPIO pins the device drivers own at run time (LED, buzzer, software-PWM
 * servo outputs, key input), plus the EXTI fan-out used by
 * platform_gpio_irq_attach().
 *
 * Logical port numbers: 0 -> GPIOA, 1 -> GPIOB, 2 -> GPIOC, 3 -> GPIOD,
 * 4 -> GPIOE. A logical port is not a HAL GPIO_TypeDef pointer on purpose, so
 * that upper layers stay vendor-free.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_mcu/impl_mcu_gpio.h"
#include "stm32f1xx_hal.h"

//******************************** Defines **********************************//

#define IMPL_GPIO_PORT_COUNT        5u      /* GPIOA .. GPIOE */
#define IMPL_GPIO_EXTI_LINE_COUNT   16u

#define IMPL_GPIO_IRQ_PRIORITY      6u      /* Below the servo tick (2). */

//******************************** Declaring ********************************//

typedef struct
{
    platform_gpio_irq_cb_t  callback;
    void                   *user_data;
    platform_gpio_t         pin;        /* Reported back to the callback. */
} impl_gpio_irq_ctx_t;

//******************************** Variables ********************************//

static impl_gpio_irq_ctx_t s_gpio_irq[IMPL_GPIO_EXTI_LINE_COUNT];

static GPIO_TypeDef *const s_gpio_port[IMPL_GPIO_PORT_COUNT] =
{
    GPIOA, GPIOB, GPIOC, GPIOD, GPIOE
};

//******************************** Functions ********************************//

static GPIO_TypeDef *impl_gpio_port_get(uint8_t port)
{
    if (port >= IMPL_GPIO_PORT_COUNT)
    {
        return NULL;
    }

    return s_gpio_port[port];
}

static void impl_gpio_clock_enable(uint8_t port)
{
    switch (port)
    {
    case 0u:
        __HAL_RCC_GPIOA_CLK_ENABLE();
        break;
    case 1u:
        __HAL_RCC_GPIOB_CLK_ENABLE();
        break;
    case 2u:
        __HAL_RCC_GPIOC_CLK_ENABLE();
        break;
    case 3u:
        __HAL_RCC_GPIOD_CLK_ENABLE();
        break;
    case 4u:
        __HAL_RCC_GPIOE_CLK_ENABLE();
        break;
    default:
        break;
    }
}

static uint32_t impl_gpio_hal_pin(uint8_t pin)
{
    return (uint32_t)1u << pin;
}

platform_err_t impl_gpio_init(const platform_gpio_cfg_t *cfg)
{
    GPIO_InitTypeDef  init = { 0 };
    GPIO_TypeDef     *port;

    if (cfg == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    port = impl_gpio_port_get(cfg->pin.port);
    if ((port == NULL) || (cfg->pin.pin >= PLATFORM_GPIO_PIN_MAX))
    {
        return PLATFORM_ERR_PARAM;
    }

    impl_gpio_clock_enable(cfg->pin.port);

    init.Pin   = impl_gpio_hal_pin(cfg->pin.pin);
    init.Speed = GPIO_SPEED_FREQ_HIGH;
    init.Pull  = GPIO_NOPULL;

    switch (cfg->mode)
    {
    case PLATFORM_GPIO_MODE_INPUT:
        init.Mode = GPIO_MODE_INPUT;
        if (cfg->pull == PLATFORM_GPIO_PULL_UP)
        {
            init.Pull = GPIO_PULLUP;
        }
        else if (cfg->pull == PLATFORM_GPIO_PULL_DOWN)
        {
            init.Pull = GPIO_PULLDOWN;
        }
        break;

    case PLATFORM_GPIO_MODE_OUTPUT_PP:
        init.Mode = GPIO_MODE_OUTPUT_PP;
        break;

    case PLATFORM_GPIO_MODE_OUTPUT_OD:
        init.Mode = GPIO_MODE_OUTPUT_OD;
        break;

    case PLATFORM_GPIO_MODE_ANALOG:
        init.Mode = GPIO_MODE_ANALOG;
        break;

    default:
        return PLATFORM_ERR_PARAM;
    }

    HAL_GPIO_Init(port, &init);

    if ((cfg->mode == PLATFORM_GPIO_MODE_OUTPUT_PP) ||
        (cfg->mode == PLATFORM_GPIO_MODE_OUTPUT_OD))
    {
        HAL_GPIO_WritePin(port, init.Pin,
                          (cfg->initial_level == PLATFORM_GPIO_LEVEL_HIGH)
                          ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    return PLATFORM_ERR_OK;
}

platform_err_t impl_gpio_deinit(platform_gpio_t pin)
{
    GPIO_TypeDef *port = impl_gpio_port_get(pin.port);

    if ((port == NULL) || (pin.pin >= PLATFORM_GPIO_PIN_MAX))
    {
        return PLATFORM_ERR_PARAM;
    }

    HAL_GPIO_DeInit(port, impl_gpio_hal_pin(pin.pin));

    return PLATFORM_ERR_OK;
}

platform_err_t impl_gpio_write(platform_gpio_t pin,
                               platform_gpio_level_t level)
{
    GPIO_TypeDef *port = impl_gpio_port_get(pin.port);

    if ((port == NULL) || (pin.pin >= PLATFORM_GPIO_PIN_MAX))
    {
        return PLATFORM_ERR_PARAM;
    }

    HAL_GPIO_WritePin(port, impl_gpio_hal_pin(pin.pin),
                      (level == PLATFORM_GPIO_LEVEL_HIGH)
                      ? GPIO_PIN_SET : GPIO_PIN_RESET);

    return PLATFORM_ERR_OK;
}

platform_err_t impl_gpio_read(platform_gpio_t pin,
                              platform_gpio_level_t *level)
{
    GPIO_TypeDef *port = impl_gpio_port_get(pin.port);

    if ((port == NULL) || (pin.pin >= PLATFORM_GPIO_PIN_MAX) ||
        (level == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    *level = (HAL_GPIO_ReadPin(port, impl_gpio_hal_pin(pin.pin)) == GPIO_PIN_SET)
             ? PLATFORM_GPIO_LEVEL_HIGH : PLATFORM_GPIO_LEVEL_LOW;

    return PLATFORM_ERR_OK;
}

platform_err_t impl_gpio_toggle(platform_gpio_t pin)
{
    GPIO_TypeDef *port = impl_gpio_port_get(pin.port);

    if ((port == NULL) || (pin.pin >= PLATFORM_GPIO_PIN_MAX))
    {
        return PLATFORM_ERR_PARAM;
    }

    HAL_GPIO_TogglePin(port, impl_gpio_hal_pin(pin.pin));

    return PLATFORM_ERR_OK;
}

platform_err_t impl_gpio_irq_attach(platform_gpio_t pin,
                                    platform_gpio_irq_edge_t edge,
                                    platform_gpio_irq_cb_t callback,
                                    void *user_data)
{
    GPIO_InitTypeDef  init = { 0 };
    GPIO_TypeDef     *port = impl_gpio_port_get(pin.port);
    IRQn_Type         irq;

    if ((port == NULL) || (pin.pin >= PLATFORM_GPIO_PIN_MAX) ||
        (callback == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    impl_gpio_clock_enable(pin.port);

    init.Pin  = impl_gpio_hal_pin(pin.pin);
    init.Pull = GPIO_NOPULL;

    switch (edge)
    {
    case PLATFORM_GPIO_IRQ_EDGE_RISING:
        init.Mode = GPIO_MODE_IT_RISING;
        break;
    case PLATFORM_GPIO_IRQ_EDGE_FALLING:
        init.Mode = GPIO_MODE_IT_FALLING;
        break;
    case PLATFORM_GPIO_IRQ_EDGE_BOTH:
        init.Mode = GPIO_MODE_IT_RISING_FALLING;
        break;
    default:
        return PLATFORM_ERR_PARAM;
    }

    HAL_GPIO_Init(port, &init);

    s_gpio_irq[pin.pin].callback  = callback;
    s_gpio_irq[pin.pin].user_data = user_data;
    s_gpio_irq[pin.pin].pin       = pin;

    if (pin.pin <= 4u)
    {
        irq = (IRQn_Type)(EXTI0_IRQn + pin.pin);
    }
    else if (pin.pin <= 9u)
    {
        irq = EXTI9_5_IRQn;
    }
    else
    {
        irq = EXTI15_10_IRQn;
    }

    HAL_NVIC_SetPriority(irq, IMPL_GPIO_IRQ_PRIORITY, 0u);
    HAL_NVIC_EnableIRQ(irq);

    return PLATFORM_ERR_OK;
}

platform_err_t impl_gpio_irq_detach(platform_gpio_t pin)
{
    GPIO_TypeDef *port = impl_gpio_port_get(pin.port);
    IRQn_Type     irq;

    if ((port == NULL) || (pin.pin >= PLATFORM_GPIO_PIN_MAX))
    {
        return PLATFORM_ERR_PARAM;
    }

    HAL_GPIO_DeInit(port, impl_gpio_hal_pin(pin.pin));

    s_gpio_irq[pin.pin].callback  = NULL;
    s_gpio_irq[pin.pin].user_data = NULL;
    s_gpio_irq[pin.pin].pin.port  = 0u;
    s_gpio_irq[pin.pin].pin.pin   = 0u;

    if (pin.pin <= 4u)
    {
        irq = (IRQn_Type)(EXTI0_IRQn + pin.pin);
    }
    else if (pin.pin <= 9u)
    {
        irq = EXTI9_5_IRQn;
    }
    else
    {
        irq = EXTI15_10_IRQn;
    }

    HAL_NVIC_DisableIRQ(irq);

    return PLATFORM_ERR_OK;
}

/*
 * HAL calls this from every EXTI line handler. The EXTI line number is the pin
 * number, so the port stored at registration time is the one to report. Only
 * the callback for the line that fired is invoked; a line can serve one port at
 * a time (an F1 hardware limitation).
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint8_t  line = 0u;
    uint16_t mask = GPIO_Pin;

    while ((mask > 1u) && (line < IMPL_GPIO_EXTI_LINE_COUNT))
    {
        mask >>= 1;
        line++;
    }

    if (line >= IMPL_GPIO_EXTI_LINE_COUNT)
    {
        return;
    }

    if (s_gpio_irq[line].callback != NULL)
    {
        s_gpio_irq[line].callback(s_gpio_irq[line].pin,
                                  s_gpio_irq[line].user_data);
    }
}
