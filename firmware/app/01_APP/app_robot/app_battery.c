/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_battery.c
 *
 * @par dependencies
 * - app_battery.h
 * - app_robot_config.h
 * - platform_bsp_battery.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Median-filter the pack voltage and debounce the low-battery decision.
 *
 * Why not a single sample: starting the drive motors pulls the pack down for
 * a few tens of milliseconds. Comparing that one conversion against
 * APP_BATTERY_LOW_MV latches a WARN fault on every start. A 5-sample median
 * removes a dip shorter than half the window, and requiring 5 consecutive
 * filtered lows removes the rest of the noise. At the 100 ms sample period
 * this reports a real sag after roughly 0.7 s.
 *
 * @version V1.1 2026-11-05
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "app_robot/app_battery.h"
#include "app_robot/app_robot_config.h"

#include "platform_bsp_battery.h"

//******************************** Variables ********************************//

static uint16_t        s_window[APP_BATTERY_FILTER_WINDOW];
static uint8_t         s_count;       /* Valid samples in the window. */
static uint8_t         s_head;        /* Next slot to overwrite. */
static uint16_t        s_latest_mv;   /* Last valid sample. */
static uint16_t        s_median_mv;   /* Median of the full window. */
static uint8_t         s_low_count;   /* Consecutive filtered lows. */
static uint32_t        s_last_sample_ms;
static platform_bool_t s_started;

//******************************** Declaring ********************************//

static uint16_t app_battery_median(void)
{
    uint16_t sorted[APP_BATTERY_FILTER_WINDOW];
    uint8_t  i;
    uint8_t  j;

    for (i = 0u; i < APP_BATTERY_FILTER_WINDOW; i++)
    {
        sorted[i] = s_window[i];
    }

    for (i = 1u; i < APP_BATTERY_FILTER_WINDOW; i++)
    {
        uint16_t value = sorted[i];

        j = i;
        while ((j > 0u) && (sorted[j - 1u] > value))
        {
            sorted[j] = sorted[j - 1u];
            j--;
        }
        sorted[j] = value;
    }

    return sorted[APP_BATTERY_FILTER_WINDOW / 2u];
}

static void app_battery_push(uint16_t mv)
{
    s_window[s_head] = mv;
    s_head = (uint8_t)((s_head + 1u) % APP_BATTERY_FILTER_WINDOW);

    if (s_count < APP_BATTERY_FILTER_WINDOW)
    {
        s_count++;
    }
}

//******************************** Functions ********************************//

void app_battery_init(void)
{
    uint8_t i;

    for (i = 0u; i < APP_BATTERY_FILTER_WINDOW; i++)
    {
        s_window[i] = 0u;
    }

    s_count = 0u;
    s_head = 0u;
    s_latest_mv = 0u;
    s_median_mv = 0u;
    s_low_count = 0u;
    s_last_sample_ms = 0u;
    s_started = PLATFORM_FALSE;
}

void app_battery_tick(uint32_t now_ms)
{
    platform_battery_state_t battery;
    uint16_t mv;
    uint16_t median;

    if ((s_started == PLATFORM_TRUE) &&
        ((now_ms - s_last_sample_ms) < APP_BATTERY_SAMPLE_PERIOD_MS))
    {
        return;
    }

    s_started = PLATFORM_TRUE;
    s_last_sample_ms = now_ms;

    if ((platform_battery_read(&battery) != PLATFORM_ERR_OK) ||
        (battery.valid != PLATFORM_TRUE) ||
        (battery.voltage_mv == 0u))
    {
        /* No new information: keep the window and the debounce counter. */
        return;
    }

    mv = (battery.voltage_mv > 0xFFFFu) ? (uint16_t)0xFFFFu
                                        : (uint16_t)battery.voltage_mv;

    s_latest_mv = mv;
    app_battery_push(mv);

    /* Do not judge before the window is full: partial medians are noise. */
    if (s_count < APP_BATTERY_FILTER_WINDOW)
    {
        return;
    }

    median = app_battery_median();
    s_median_mv = median;

    if (median < APP_BATTERY_LOW_MV)
    {
        if (s_low_count < APP_BATTERY_LOW_COUNT)
        {
            s_low_count++;
        }
    }
    else
    {
        s_low_count = 0u;
    }
}

uint16_t app_battery_mv(void)
{
    if (s_count >= APP_BATTERY_FILTER_WINDOW)
    {
        return s_median_mv;
    }

    return s_latest_mv;
}

platform_bool_t app_battery_low(void)
{
    return (s_low_count >= APP_BATTERY_LOW_COUNT) ? PLATFORM_TRUE
                                                  : PLATFORM_FALSE;
}
