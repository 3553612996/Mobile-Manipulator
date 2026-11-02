/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_mcu_time.c
 *
 * @par dependencies
 * - platform_def.h
 * - stm32f1xx_hal.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Busy-wait delays declared by platform_def.h.
 *
 * platform_delay_ms() uses the HAL tick (TIM6 timebase), so it works before
 * the scheduler starts and from any context except an ISR handler whose
 * priority is above the timebase interrupt. The drivers use it to poll the
 * W25Q64 BUSY bit, where a few hundred microseconds of jitter do not matter.
 *
 * platform_delay_us() spins on the DWT cycle counter; it is exact to within a
 * cycle or two and safe in an ISR. The trace unit is enabled on first use.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "platform_def.h"
#include "stm32f1xx_hal.h"

//******************************** Functions ********************************//

void platform_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

void platform_delay_us(uint32_t us)
{
    uint32_t ticks;
    uint32_t start;

    if (us == 0u)
    {
        return;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL       |= DWT_CTRL_CYCCNTENA_Msk;

    ticks = us * (SystemCoreClock / 1000000u);
    start = DWT->CYCCNT;

    /* Unsigned subtraction also handles the counter wrapping at 2^32. */
    while ((DWT->CYCCNT - start) < ticks)
    {
    }
}
