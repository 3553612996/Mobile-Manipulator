/* L1 unit test for 02_Service/service_key. Runs on the PC, no hardware.
 *
 * The key device only exposes a debounced level and held time, so the test
 * scripts those two values and checks the short/long policy on top of them:
 * one event per gesture, LONG while still held, SHORT only if LONG did not fire.
 *
 * Build (MinGW or any host gcc):
 *   gcc -std=c99 -Wall -Wextra \
 *       -I firmware/app/03_Platform/platform_common \
 *       -I firmware/app/04_Impl \
 *       -I firmware/app/03_Platform/platform_bsp \
 *       -I firmware/app/02_Service/service_key \
 *       tests/unit/service/test_service_key.c \
 *       firmware/app/02_Service/service_key/service_key.c \
 *       -o build/test_service_key
 */

#include <stdio.h>
#include <string.h>

#include "service_key.h"

static int g_failed = 0;

#define CHECK(cond, msg)                                                  \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);        \
            g_failed++;                                                   \
        }                                                                 \
    } while (0)

/* -------------------------- platform key stubs ------------------------- */

static platform_key_state_t g_state;

platform_err_t platform_key_poll(platform_key_id_t id, uint32_t elapsed_ms)
{
    (void)id;
    (void)elapsed_ms;
    return PLATFORM_ERR_OK;
}

platform_err_t platform_key_get_state(platform_key_id_t id,
                                      platform_key_state_t *state)
{
    if ((id != PLATFORM_KEY_ID_0) || (state == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    *state = g_state;
    return PLATFORM_ERR_OK;
}

/* --------------------------- event recording --------------------------- */

#define MAX_EVENTS  8

static service_key_event_t g_events[MAX_EVENTS];
static int                 g_event_count;

static void on_event(platform_key_id_t id,
                     service_key_event_t event,
                     void *user_data)
{
    (void)id;
    (void)user_data;

    if (g_event_count < MAX_EVENTS)
    {
        g_events[g_event_count] = event;
    }

    g_event_count++;
}

static void set_key(platform_bool_t pressed, uint32_t held_ms)
{
    g_state.pressed = pressed;
    g_state.raw_pressed = pressed;
    g_state.pressed_ms = held_ms;
}

static void reset(void)
{
    memset(&g_state, 0, sizeof(g_state));
    memset(g_events, 0, sizeof(g_events));
    g_event_count = 0;
}

/* -------------------------------- tests -------------------------------- */

#define TEST_LONG_MS    ((uint32_t)1500u)

static void init_service(void)
{
    service_key_cfg_t cfg;

    cfg.id = PLATFORM_KEY_ID_0;
    cfg.long_press_ms = TEST_LONG_MS;

    reset();
    CHECK(service_key_init(&cfg, on_event, NULL) == PLATFORM_ERR_OK, "init");
}

static void test_short_press(void)
{
    init_service();

    set_key(PLATFORM_TRUE, 10u);
    (void)service_key_update(10u);
    CHECK(g_event_count == 0, "nothing on press");

    set_key(PLATFORM_TRUE, 600u);
    (void)service_key_update(590u);
    CHECK(g_event_count == 0, "nothing below threshold");

    set_key(PLATFORM_FALSE, 0u);
    (void)service_key_update(10u);
    CHECK(g_event_count == 1, "one event on release");
    CHECK(g_events[0] == SERVICE_KEY_EVENT_SHORT_PRESS, "short press");

    /* A second release without a new press must not fire again. */
    (void)service_key_update(10u);
    CHECK(g_event_count == 1, "no repeat while released");
}

static void test_long_press(void)
{
    init_service();

    set_key(PLATFORM_TRUE, 10u);
    (void)service_key_update(10u);

    set_key(PLATFORM_TRUE, TEST_LONG_MS - 1u);
    (void)service_key_update(10u);
    CHECK(g_event_count == 0, "not yet");

    set_key(PLATFORM_TRUE, TEST_LONG_MS);
    (void)service_key_update(10u);
    CHECK(g_event_count == 1, "long fires while held");
    CHECK(g_events[0] == SERVICE_KEY_EVENT_LONG_PRESS, "long press");

    /* Still held: the long event must not repeat. */
    set_key(PLATFORM_TRUE, TEST_LONG_MS + 500u);
    (void)service_key_update(10u);
    CHECK(g_event_count == 1, "long fires once");

    /* Release after a long press is not also a short press. */
    set_key(PLATFORM_FALSE, 0u);
    (void)service_key_update(10u);
    CHECK(g_event_count == 1, "no short after long");
}

static void test_new_gesture_after_long(void)
{
    init_service();

    set_key(PLATFORM_TRUE, 10u);
    (void)service_key_update(10u);
    set_key(PLATFORM_TRUE, TEST_LONG_MS);
    (void)service_key_update(10u);
    set_key(PLATFORM_FALSE, 0u);
    (void)service_key_update(10u);
    CHECK(g_event_count == 1, "first gesture: long");

    set_key(PLATFORM_TRUE, 10u);
    (void)service_key_update(10u);
    set_key(PLATFORM_FALSE, 0u);
    (void)service_key_update(10u);
    CHECK(g_event_count == 2, "second gesture fires");
    CHECK(g_events[1] == SERVICE_KEY_EVENT_SHORT_PRESS, "second is short");
}

static void test_default_threshold(void)
{
    service_key_cfg_t cfg;

    cfg.id = PLATFORM_KEY_ID_0;
    cfg.long_press_ms = 0u;

    reset();
    CHECK(service_key_init(&cfg, on_event, NULL) == PLATFORM_ERR_OK,
          "init default");

    set_key(PLATFORM_TRUE, 10u);
    (void)service_key_update(10u);
    set_key(PLATFORM_TRUE, SERVICE_KEY_LONG_PRESS_MS_DEFAULT - 1u);
    (void)service_key_update(10u);
    CHECK(g_event_count == 0, "default threshold not reached");

    set_key(PLATFORM_TRUE, SERVICE_KEY_LONG_PRESS_MS_DEFAULT);
    (void)service_key_update(10u);
    CHECK(g_event_count == 1, "default threshold fires");
}

int main(void)
{
    test_short_press();
    test_long_press();
    test_new_gesture_after_long();
    test_default_threshold();

    if (g_failed != 0)
    {
        printf("FAIL: service_key tests (%d failure(s))\n", g_failed);
        return 1;
    }

    printf("PASS: service_key tests\n");
    return 0;
}
