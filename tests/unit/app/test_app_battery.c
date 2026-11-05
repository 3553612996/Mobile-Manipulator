/* L1 unit test for 01_APP/app_robot/app_battery.c.
 *
 * Covers the median window and the consecutive-low debounce that keep motor
 * inrush from latching the low-battery fault:
 *   - a dip shorter than half the window never reaches the median;
 *   - the decision starts only after the window fills;
 *   - APP_BATTERY_LOW_COUNT consecutive filtered lows are required;
 *   - a recovery clears the counter;
 *   - invalid conversions keep the last good state;
 *   - sampling honours APP_BATTERY_SAMPLE_PERIOD_MS.
 *
 * Build:
 *   gcc -std=c99 -Wall -Wextra -Werror \
 *       -I firmware/app/03_Platform/platform_common \
 *       -I firmware/app/03_Platform/platform_mcu \
 *       -I firmware/app/04_Impl \
 *       -I firmware/app/03_Platform/platform_bsp \
 *       -I firmware/app/01_APP \
 *       tests/unit/app/test_app_battery.c \
 *       firmware/app/01_APP/app_robot/app_battery.c \
 *       -o build/test_app_battery
 */

#include <stdio.h>

#include "app_robot/app_battery.h"
#include "app_robot/app_robot_config.h"

#include "platform_bsp_battery.h"

static int g_failed = 0;

#define CHECK(cond, msg)                                                  \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);        \
            g_failed++;                                                   \
        }                                                                 \
    } while (0)

/* Platform stub: the only hardware dependency of the module under test. */
static platform_battery_state_t g_state;
static platform_err_t           g_read_result;

platform_err_t platform_battery_read(platform_battery_state_t *state)
{
    if (state == 0)
    {
        return PLATFORM_ERR_PARAM;
    }

    *state = g_state;
    return g_read_result;
}

static uint32_t g_now;

static void set_voltage(uint32_t mv, platform_bool_t valid)
{
    g_state.voltage_mv = mv;
    g_state.raw = 0u;
    g_state.valid = valid;
    g_read_result = PLATFORM_ERR_OK;
}

static void tick(void)
{
    g_now += APP_BATTERY_SAMPLE_PERIOD_MS;
    app_battery_tick(g_now);
}

static void feed(uint32_t mv, uint8_t samples)
{
    uint8_t i;

    set_voltage(mv, PLATFORM_TRUE);
    for (i = 0u; i < samples; i++)
    {
        tick();
    }
}

static void test_startup_dip_is_filtered(void)
{
    app_battery_init();
    g_now = 0u;

    feed(7500u, 5u);
    CHECK(app_battery_mv() == 7500u, "median equals a healthy pack");
    CHECK(app_battery_low() == PLATFORM_FALSE, "healthy pack is not low");

    /* 200 ms inrush dip: 2 of 5 samples, below the median. */
    feed(6000u, 2u);
    CHECK(app_battery_mv() == 7500u, "median ignores a 2-sample dip");
    CHECK(app_battery_low() == PLATFORM_FALSE,
          "startup dip does not latch the fault");

    feed(7500u, 3u);
    CHECK(app_battery_low() == PLATFORM_FALSE,
          "pack back to normal stays normal");
}

static void test_sustained_low_is_debounced(void)
{
    app_battery_init();
    g_now = 0u;

    feed(7500u, 5u);

    /* Three lows move the median of five, so this is filtered low #1. */
    feed(6000u, 3u);
    CHECK(app_battery_low() == PLATFORM_FALSE,
          "first filtered low alone is not enough");

    feed(6000u, 3u);
    CHECK(app_battery_low() == PLATFORM_FALSE,
          "4 consecutive filtered lows are still not enough");

    feed(6000u, 1u);
    CHECK(app_battery_low() == PLATFORM_TRUE,
          "5 consecutive filtered lows latch the fault");
}

static void test_recovery_clears_counter(void)
{
    app_battery_init();
    g_now = 0u;

    feed(7500u, 5u);
    feed(6000u, 7u);
    CHECK(app_battery_low() == PLATFORM_TRUE, "sustained sag is low");

    /* Median returns to normal on the third high sample. */
    feed(7500u, 2u);
    CHECK(app_battery_low() == PLATFORM_TRUE,
          "two high samples do not clear a 3-of-5 window");

    feed(7500u, 1u);
    CHECK(app_battery_low() == PLATFORM_FALSE,
          "recovered median clears the low state");
}

static void test_partial_window_gives_no_verdict(void)
{
    app_battery_init();
    g_now = 0u;

    feed(6500u, 4u);
    CHECK(app_battery_mv() == 6500u, "latest good value while filling");
    CHECK(app_battery_low() == PLATFORM_FALSE,
          "no low verdict before the window fills");

    feed(6500u, 1u);
    CHECK(app_battery_low() == PLATFORM_FALSE,
          "window fill starts the debounce from zero");
}

static void test_invalid_samples_are_ignored(void)
{
    uint8_t i;

    app_battery_init();
    g_now = 0u;

    feed(7500u, 5u);

    set_voltage(0u, PLATFORM_FALSE);
    for (i = 0u; i < 10u; i++)
    {
        tick();
    }

    CHECK(app_battery_mv() == 7500u, "invalid samples keep the last median");
    CHECK(app_battery_low() == PLATFORM_FALSE,
          "invalid samples do not latch the fault");
}

static void test_read_error_is_ignored(void)
{
    uint8_t i;

    app_battery_init();
    g_now = 0u;

    feed(7500u, 5u);

    g_read_result = PLATFORM_ERR_TIMEOUT;
    for (i = 0u; i < 10u; i++)
    {
        tick();
    }

    CHECK(app_battery_mv() == 7500u, "read errors keep the last median");
    CHECK(app_battery_low() == PLATFORM_FALSE, "read errors do not latch");
}

static void test_sample_period_is_honoured(void)
{
    app_battery_init();
    g_now = 0u;

    set_voltage(7500u, PLATFORM_TRUE);
    app_battery_tick(g_now);
    CHECK(app_battery_mv() == 7500u, "first tick samples immediately");

    set_voltage(6000u, PLATFORM_TRUE);
    app_battery_tick(g_now + (APP_BATTERY_SAMPLE_PERIOD_MS - 1u));
    CHECK(app_battery_mv() == 7500u, "sample inside the period is skipped");

    app_battery_tick(g_now + APP_BATTERY_SAMPLE_PERIOD_MS);
    CHECK(app_battery_mv() == 6000u, "sample at the period boundary runs");
}

int main(void)
{
    test_startup_dip_is_filtered();
    test_sustained_low_is_debounced();
    test_recovery_clears_counter();
    test_partial_window_gives_no_verdict();
    test_invalid_samples_are_ignored();
    test_read_error_is_ignored();
    test_sample_period_is_honoured();

    if (g_failed != 0)
    {
        printf("FAIL: %d failure(s)\n", g_failed);
        return 1;
    }

    printf("PASS: app_battery tests\n");
    return 0;
}
