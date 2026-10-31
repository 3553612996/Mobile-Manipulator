/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_mcu_uart.c
 *
 * @par dependencies
 * - impl_mcu/impl_mcu_uart.h
 * - platform_ringbuf.h
 * - usart.h (CubeMX handle)
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Adapt the platform UART contract to USART3 + DMA and UART5 half-duplex.
 *
 * Mapping
 * -------
 * - Logical bus 0: USART3 (PB10/PB11) to the host, full duplex, RX by circular
 *   DMA with full/half/idle interrupts.
 * - Logical bus 1: UART5 (PC12) to the bus servos, half duplex single wire,
 *   open drain, RX by RXNE interrupt. This is the wiring of the vendor board:
 *   the daisy-chain data line is pulled up and the MCU only drives it low.
 *
 * RX path (bus 0)
 * ---------------
 * A circular DMA channel writes into a 256-byte buffer. The transfer-complete
 * (full), half-transfer and USART idle interrupts all call the same capture
 * routine: it computes the producer position from the DMA counter, copies the
 * new bytes into a platform_ringbuf_t, and notifies the registered event
 * callback (which must only wake a task). The ring buffer is single-producer
 * (ISR) / single-consumer (task), so no lock is needed.
 *
 * RX path (bus 1)
 * ---------------
 * UART5 has no DMA request on this part in the vendor wiring, so the RXNE
 * interrupt pushes bytes straight into the ring. While transmitting, the
 * half-duplex line echoes our own frame; RXNE is masked for the duration and
 * the ring is cleared once TC is reached, because a servo can only answer
 * after the last byte is on the wire.
 *
 * TX path
 * -------
 * Blocking HAL_UART_Transmit. Protocol frames are at most 71 bytes (6.2 ms at
 * 115200) and bus frames at most 92 bytes, and only one task transmits per
 * bus, so a DMA TX channel would add state without buying anything yet.
 *
 * @version V2.0 2026-10-31
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_mcu/impl_mcu_uart.h"
#include "platform_ringbuf.h"

#include "stm32f1xx_hal.h"
#include "usart.h"

//******************************** Defines **********************************//

#define IMPL_UART_BUS_HOST          ((platform_uart_bus_t)0u)
#define IMPL_UART_BUS_SERVO         ((platform_uart_bus_t)1u)
#define IMPL_UART_BUS_COUNT         (2u)

#define IMPL_UART_DMA_BUFFER_SIZE   (256u)
#define IMPL_UART_RING_SIZE         (256u)
#define IMPL_UART_DMA_MASK          (IMPL_UART_DMA_BUFFER_SIZE - 1u)

/* Must stay numerically >= configMAX_SYSCALL_INTERRUPT_PRIORITY (5). */
#define IMPL_UART_DMA_IRQ_PRIORITY  (6u)
#define IMPL_UART5_IRQ_PRIORITY     (6u)

#define IMPL_UART_TX_TIMEOUT_MS     (100u)

//******************************** Declaring ********************************//

typedef struct
{
    platform_bool_t             initialized;
    platform_bool_t             use_dma;
    uint8_t                     dma_buffer[IMPL_UART_DMA_BUFFER_SIZE];
    uint32_t                    dma_read;       /* ISR-owned read index. */
    platform_ringbuf_t          ring;
    uint8_t                     ring_storage[IMPL_UART_RING_SIZE];
    platform_uart_event_cb_t    callback;
    void *                      user_data;
    uint32_t                    dropped_bytes;
} impl_uart_ctx_t;

//******************************** Variables ********************************//

static impl_uart_ctx_t      s_uart[IMPL_UART_BUS_COUNT];
static DMA_HandleTypeDef    s_hdma_usart3_rx;
static UART_HandleTypeDef   s_huart5;

//******************************** Functions ********************************//

static platform_bool_t impl_uart_bus_ok(platform_uart_bus_t bus)
{
    return (bus < IMPL_UART_BUS_COUNT) ? PLATFORM_TRUE : PLATFORM_FALSE;
}

static void impl_uart_notify(platform_uart_bus_t bus,
                             platform_uart_event_t event)
{
    if (s_uart[bus].callback != NULL)
    {
        s_uart[bus].callback(bus, event, s_uart[bus].user_data);
    }
}

static void impl_uart_ring_push(platform_uart_bus_t bus, uint8_t byte)
{
    if (platform_ringbuf_write_byte(&s_uart[bus].ring, byte) != 1u)
    {
        s_uart[bus].dropped_bytes++;
    }
    else
    {
        impl_uart_notify(bus, PLATFORM_UART_EVENT_RX_DATA);
    }
}

/*
 * Move every byte the DMA has written since the last call into the ring
 * buffer. Runs from USART3_IRQHandler and from the DMA HT/TC interrupts.
 * Producer position = size - remaining counter; both sides are masked because
 * the counter wraps at the end of the circular buffer.
 */
static void impl_uart_capture(void)
{
    impl_uart_ctx_t *ctx = &s_uart[IMPL_UART_BUS_HOST];
    uint32_t write_pos;
    uint32_t moved = 0u;

    write_pos = (IMPL_UART_DMA_BUFFER_SIZE -
                 __HAL_DMA_GET_COUNTER(&s_hdma_usart3_rx)) &
                IMPL_UART_DMA_MASK;

    while (ctx->dma_read != write_pos)
    {
        uint8_t byte = ctx->dma_buffer[ctx->dma_read];

        ctx->dma_read = (ctx->dma_read + 1u) & IMPL_UART_DMA_MASK;

        if (platform_ringbuf_write_byte(&ctx->ring, byte) != 1u)
        {
            ctx->dropped_bytes++;
        }
        else
        {
            moved++;
        }
    }

    if (moved > 0u)
    {
        impl_uart_notify(IMPL_UART_BUS_HOST, PLATFORM_UART_EVENT_RX_DATA);
    }
}

static HAL_StatusTypeDef impl_uart_start_rx_dma(void)
{
    HAL_StatusTypeDef status;

    status = HAL_UART_Receive_DMA(&huart3,
                                  s_uart[IMPL_UART_BUS_HOST].dma_buffer,
                                  (uint16_t)IMPL_UART_DMA_BUFFER_SIZE);
    if (status != HAL_OK)
    {
        return status;
    }

    /* HAL does not enable the idle interrupt for plain DMA reception. */
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    return HAL_OK;
}

static platform_err_t impl_uart_apply_cfg_usart3(
                                        const platform_uart_cfg_t *cfg)
{
    if (cfg->duplex != PLATFORM_UART_DUPLEX_FULL)
    {
        return PLATFORM_ERR_NOT_SUPPORTED;
    }

    huart3.Init.BaudRate = cfg->baudrate;
    huart3.Init.WordLength = (cfg->data_bits == 9u)
                                 ? UART_WORDLENGTH_9B
                                 : UART_WORDLENGTH_8B;
    huart3.Init.StopBits = (cfg->stop_bits == PLATFORM_UART_STOP_BITS_2)
                               ? UART_STOPBITS_2
                               : UART_STOPBITS_1;

    switch (cfg->parity)
    {
        case PLATFORM_UART_PARITY_EVEN:
            huart3.Init.Parity = UART_PARITY_EVEN;
            break;
        case PLATFORM_UART_PARITY_ODD:
            huart3.Init.Parity = UART_PARITY_ODD;
            break;
        case PLATFORM_UART_PARITY_NONE:
        default:
            huart3.Init.Parity = UART_PARITY_NONE;
            break;
    }

    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        return PLATFORM_ERR_FAIL;
    }

    return PLATFORM_ERR_OK;
}

/*
 * UART5 is not in the CubeMX project, so its clock, pin and NVIC line are
 * configured here. PC12 is the single bidirectional data line: alternate
 * function open drain, exactly as the vendor's board drives it.
 */
static platform_err_t impl_uart_apply_cfg_uart5(
                                        const platform_uart_cfg_t *cfg)
{
    GPIO_InitTypeDef gpio;

    if (cfg->duplex != PLATFORM_UART_DUPLEX_HALF)
    {
        return PLATFORM_ERR_NOT_SUPPORTED;
    }

    if (cfg->parity != PLATFORM_UART_PARITY_NONE)
    {
        return PLATFORM_ERR_NOT_SUPPORTED;
    }

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_UART5_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &gpio);

    s_huart5.Instance = UART5;
    s_huart5.Init.BaudRate = cfg->baudrate;
    s_huart5.Init.WordLength = (cfg->data_bits == 9u)
                                   ? UART_WORDLENGTH_9B
                                   : UART_WORDLENGTH_8B;
    s_huart5.Init.StopBits = (cfg->stop_bits == PLATFORM_UART_STOP_BITS_2)
                                 ? UART_STOPBITS_2
                                 : UART_STOPBITS_1;
    s_huart5.Init.Parity = UART_PARITY_NONE;
    s_huart5.Init.Mode = UART_MODE_TX_RX;
    s_huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    s_huart5.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_HalfDuplex_Init(&s_huart5) != HAL_OK)
    {
        return PLATFORM_ERR_FAIL;
    }

    HAL_NVIC_SetPriority(UART5_IRQn, IMPL_UART5_IRQ_PRIORITY, 0u);
    HAL_NVIC_EnableIRQ(UART5_IRQn);

    __HAL_UART_ENABLE_IT(&s_huart5, UART_IT_RXNE);

    return PLATFORM_ERR_OK;
}

static platform_err_t impl_uart_apply_cfg(const platform_uart_cfg_t *cfg)
{
    if ((cfg->data_bits != 8u) && (cfg->data_bits != 9u))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (cfg->bus == IMPL_UART_BUS_HOST)
    {
        return impl_uart_apply_cfg_usart3(cfg);
    }

    return impl_uart_apply_cfg_uart5(cfg);
}

static platform_err_t impl_uart_setup_rx_dma(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    s_hdma_usart3_rx.Instance = DMA1_Channel3;
    s_hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    s_hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    s_hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    s_hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    s_hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    s_hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
    s_hdma_usart3_rx.Init.Priority = DMA_PRIORITY_HIGH;

    if (HAL_DMA_Init(&s_hdma_usart3_rx) != HAL_OK)
    {
        return PLATFORM_ERR_FAIL;
    }

    __HAL_LINKDMA(&huart3, hdmarx, s_hdma_usart3_rx);

    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, IMPL_UART_DMA_IRQ_PRIORITY, 0u);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

    if (impl_uart_start_rx_dma() != HAL_OK)
    {
        return PLATFORM_ERR_FAIL;
    }

    return PLATFORM_ERR_OK;
}

//******************************** Functions ********************************//

platform_err_t impl_uart_init(const platform_uart_cfg_t *cfg)
{
    platform_err_t err;
    impl_uart_ctx_t *ctx;

    if (cfg == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (impl_uart_bus_ok(cfg->bus) == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_SUPPORTED;
    }

    ctx = &s_uart[cfg->bus];

    if (ctx->initialized == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }

    err = platform_ringbuf_init(&ctx->ring,
                                ctx->ring_storage,
                                IMPL_UART_RING_SIZE);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    ctx->dma_read = 0u;
    ctx->callback = NULL;
    ctx->user_data = NULL;
    ctx->dropped_bytes = 0u;
    ctx->use_dma = cfg->use_dma;

    err = impl_uart_apply_cfg(cfg);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    if ((cfg->use_dma == PLATFORM_TRUE) && (cfg->bus == IMPL_UART_BUS_HOST))
    {
        err = impl_uart_setup_rx_dma();
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }
    }

    ctx->initialized = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

static platform_err_t impl_uart_deinit_usart3(void)
{
    __HAL_UART_DISABLE_IT(&huart3, UART_IT_IDLE);
    (void)HAL_UART_DMAStop(&huart3);
    HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn);
    (void)HAL_DMA_DeInit(&s_hdma_usart3_rx);

    return PLATFORM_ERR_OK;
}

static platform_err_t impl_uart_deinit_uart5(void)
{
    __HAL_UART_DISABLE_IT(&s_huart5, UART_IT_RXNE);
    HAL_NVIC_DisableIRQ(UART5_IRQn);
    (void)HAL_UART_DeInit(&s_huart5);

    /* HAL_UART_MspDeInit does not know UART5; release it here. */
    __HAL_RCC_UART5_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

    return PLATFORM_ERR_OK;
}

platform_err_t impl_uart_deinit(platform_uart_bus_t bus)
{
    if (impl_uart_bus_ok(bus) == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_SUPPORTED;
    }

    if (s_uart[bus].initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (bus == IMPL_UART_BUS_HOST)
    {
        (void)impl_uart_deinit_usart3();
    }
    else
    {
        (void)impl_uart_deinit_uart5();
    }

    s_uart[bus].initialized = PLATFORM_FALSE;
    s_uart[bus].callback = NULL;
    s_uart[bus].user_data = NULL;

    return PLATFORM_ERR_OK;
}

platform_err_t impl_uart_write(platform_uart_bus_t bus,
                               const uint8_t *data,
                               uint32_t len,
                               uint32_t timeout_ms)
{
    HAL_StatusTypeDef status;

    if (impl_uart_bus_ok(bus) == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_SUPPORTED;
    }

    if (s_uart[bus].initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if ((data == NULL) || (len == 0u))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (len > 0xFFFFu)
    {
        return PLATFORM_ERR_PARAM;
    }

    if (bus == IMPL_UART_BUS_SERVO)
    {
        /*
         * Half duplex: mask the echo while our frame is on the wire, then
         * throw it away. A reply cannot start before our last stop bit, and
         * HAL_UART_Transmit already waited for TC before returning.
         */
        __HAL_UART_DISABLE_IT(&s_huart5, UART_IT_RXNE);

        status = HAL_UART_Transmit(&s_huart5,
                                   (uint8_t *)data,
                                   (uint16_t)len,
                                   timeout_ms);

        platform_ringbuf_reset(&s_uart[bus].ring);
        __HAL_UART_CLEAR_OREFLAG(&s_huart5);

        __HAL_UART_ENABLE_IT(&s_huart5, UART_IT_RXNE);
    }
    else
    {
        status = HAL_UART_Transmit(&huart3,
                                   (uint8_t *)data,
                                   (uint16_t)len,
                                   timeout_ms);
    }

    if (status == HAL_OK)
    {
        return PLATFORM_ERR_OK;
    }

    return (status == HAL_BUSY) ? PLATFORM_ERR_BUSY : PLATFORM_ERR_TIMEOUT;
}

platform_err_t impl_uart_read(platform_uart_bus_t bus,
                              uint8_t *data,
                              uint32_t len,
                              uint32_t timeout_ms)
{
    uint32_t start_ms;
    uint32_t got = 0u;

    if (impl_uart_bus_ok(bus) == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_SUPPORTED;
    }

    if (s_uart[bus].initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if ((data == NULL) || (len == 0u))
    {
        return PLATFORM_ERR_PARAM;
    }

    start_ms = HAL_GetTick();

    while (got < len)
    {
        uint32_t chunk = platform_ringbuf_read(&s_uart[bus].ring,
                                               &data[got],
                                               len - got);
        got += chunk;

        if (got == len)
        {
            break;
        }

        if ((HAL_GetTick() - start_ms) >= timeout_ms)
        {
            return PLATFORM_ERR_TIMEOUT;
        }

        HAL_Delay(1u);
    }

    return PLATFORM_ERR_OK;
}

platform_err_t impl_uart_write_async(platform_uart_bus_t bus,
                                     const uint8_t *data,
                                     uint32_t len)
{
    (void)bus;
    (void)data;
    (void)len;

    /* Blocking TX is enough for protocol-sized frames; see the file header. */
    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_uart_read_async(platform_uart_bus_t bus,
                                    uint8_t *data,
                                    uint32_t max_len)
{
    (void)bus;
    (void)data;
    (void)max_len;

    /* Use platform_uart_rx_read(): RX is owned by the ring buffer. */
    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t impl_uart_get_rx_count(platform_uart_bus_t bus,
                                      uint32_t *rx_len)
{
    return impl_uart_rx_available(bus, rx_len);
}

platform_err_t impl_uart_rx_read(platform_uart_bus_t bus,
                                 uint8_t *data,
                                 uint32_t max_len,
                                 uint32_t *out_len)
{
    uint32_t copied;

    if (out_len == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    *out_len = 0u;

    if (impl_uart_bus_ok(bus) == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_SUPPORTED;
    }

    if (s_uart[bus].initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if ((data == NULL) && (max_len > 0u))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (max_len == 0u)
    {
        return PLATFORM_ERR_OK;
    }

    copied = platform_ringbuf_read(&s_uart[bus].ring, data, max_len);
    *out_len = copied;

    return PLATFORM_ERR_OK;
}

platform_err_t impl_uart_rx_available(platform_uart_bus_t bus,
                                      uint32_t *count)
{
    if (count == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    *count = 0u;

    if (impl_uart_bus_ok(bus) == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_SUPPORTED;
    }

    if (s_uart[bus].initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    *count = platform_ringbuf_count(&s_uart[bus].ring);

    return PLATFORM_ERR_OK;
}

platform_err_t impl_uart_register_event_cb(
                                        platform_uart_bus_t bus,
                                        platform_uart_event_cb_t callback,
                                        void *user_data)
{
    if (impl_uart_bus_ok(bus) == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_SUPPORTED;
    }

    s_uart[bus].callback = callback;
    s_uart[bus].user_data = user_data;

    return PLATFORM_ERR_OK;
}

platform_err_t impl_uart_flush_rx(platform_uart_bus_t bus)
{
    uint32_t primask;

    if (impl_uart_bus_ok(bus) == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_SUPPORTED;
    }

    if (s_uart[bus].initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    platform_ringbuf_reset(&s_uart[bus].ring);

    if (bus == IMPL_UART_BUS_HOST)
    {
        /* Discard bytes the DMA has already written but not yet captured. */
        s_uart[bus].dma_read = (IMPL_UART_DMA_BUFFER_SIZE -
                                __HAL_DMA_GET_COUNTER(&s_hdma_usart3_rx)) &
                               IMPL_UART_DMA_MASK;

        __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    }
    else
    {
        __HAL_UART_CLEAR_OREFLAG(&s_huart5);
    }

    __set_PRIMASK(primask);

    return PLATFORM_ERR_OK;
}

void impl_uart_rx_poll_isr(void)
{
    if (s_uart[IMPL_UART_BUS_HOST].initialized == PLATFORM_FALSE)
    {
        return;
    }

    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET)
    {
        /* Reading SR then DR clears IDLE on STM32F1. */
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);

        impl_uart_capture();
        impl_uart_notify(IMPL_UART_BUS_HOST, PLATFORM_UART_EVENT_RX_IDLE);
    }
}

//****************************** HAL callbacks ******************************//

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        impl_uart_capture();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        impl_uart_capture();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (s_uart[IMPL_UART_BUS_HOST].initialized == PLATFORM_FALSE)
        {
            return;
        }

        /* Keep whatever arrived before the error. */
        impl_uart_capture();

        __HAL_UART_CLEAR_OREFLAG(huart);

        /*
         * HAL_UART_Receive_DMA() refuses to start while RxState is BUSY_RX.
         * The error path has already moved it back to READY, but a stop here
         * makes the restart deterministic on every HAL revision.
         */
        (void)HAL_UART_DMAStop(huart);

        if (impl_uart_start_rx_dma() != HAL_OK)
        {
            impl_uart_notify(IMPL_UART_BUS_HOST, PLATFORM_UART_EVENT_RX_ERROR);
        }
    }
}

/*
 * DMA1 Channel 3 is the USART3 RX channel (see the STM32F103 mapping).
 * The handler lives here, not in stm32f1xx_it.c, so the driver owns its
 * interrupt. Do not also enable USART3_RX DMA in CubeMX without removing
 * one of the two handlers.
 */
void DMA1_Channel3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&s_hdma_usart3_rx);
}

/*
 * UART5 RXNE. Half duplex with the transmitter on the same pin, so an error
 * is cleared by the SR-then-DR read and the byte is simply dropped.
 */
void UART5_IRQHandler(void)
{
    uint32_t status;
    uint8_t byte;

    if (s_uart[IMPL_UART_BUS_SERVO].initialized == PLATFORM_FALSE)
    {
        return;
    }

    status = s_huart5.Instance->SR;

    if ((status & USART_SR_RXNE) != 0u)
    {
        byte = (uint8_t)(s_huart5.Instance->DR & 0xFFu);
        impl_uart_ring_push(IMPL_UART_BUS_SERVO, byte);
    }
    else if ((status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE |
                        USART_SR_PE)) != 0u)
    {
        (void)s_huart5.Instance->DR;
        impl_uart_notify(IMPL_UART_BUS_SERVO, PLATFORM_UART_EVENT_RX_ERROR);
    }
    else
    {
        /* Nothing we service; leave the interrupt source alone. */
    }
}
