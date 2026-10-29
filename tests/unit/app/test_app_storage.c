/* L1 unit test for 01_APP/app_robot/app_storage.c.
 *
 * The storage module is a task body, two queues and a small staging pool. The
 * test supplies an in-memory queue with the same by-value contract as the OS
 * one, a storage service that records what it was asked to write, and then
 * drives the task through app_storage_step() instead of a scheduler.
 *
 * What is checked here is exactly what the task owns:
 *   - short records are copied before the producer's buffer goes away;
 *   - one large blob borrows the caller's buffer and a second is refused;
 *   - the staging pool refuses a third short record instead of overwriting;
 *   - a callback runs once, in the task, with the tag it was given;
 *   - the level filter drops lines before they cost anything;
 *   - an export paces at APP_STORAGE_EXPORT_PERIOD_MS and ends with a frame.
 *
 * Build:
 *   gcc -std=c99 -Wall -Wextra -Werror \
 *       -I firmware/app/03_Platform/platform_common \
 *       -I firmware/app/03_Platform/platform_mcu \
 *       -I firmware/app/04_Impl \
 *       -I firmware/app/03_Platform/platform_os \
 *       -I firmware/app/03_Platform/platform_bsp \
 *       -I firmware/app/03_Platform/platform_middleware \
 *       -I firmware/app/05_Middlewares/protocol \
 *       -I firmware/app/02_Service \
 *       -I firmware/app/01_APP \
 *       tests/unit/app/test_app_storage.c \
 *       firmware/app/01_APP/app_robot/app_storage.c \
 *       firmware/app/05_Middlewares/protocol/protocol_def.c \
 *       -o build/test_app_storage
 */

#include <stdio.h>
#include <string.h>

#include "app_robot/app_storage.h"
#include "app_robot/app_params.h"
#include "app_robot/app_robot_config.h"

#include "service_storage/service_storage.h"
#include "service_proto/service_proto.h"

#include "platform_os_critical.h"
#include "platform_os_queue.h"
#include "platform_os_time.h"
#include "platform_logstore.h"

static int g_failed = 0;

#define CHECK(cond, msg)                                                  \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);        \
            g_failed++;                                                   \
        }                                                                 \
    } while (0)

/* ------------------------------ fake queue ----------------------------- */

#define FAKE_ITEMS ((uint32_t)8u)

struct platform_os_queue
{
    uint32_t item_size;
    uint32_t capacity;
    uint32_t used;
    uint32_t head;
    uint8_t  slot[FAKE_ITEMS][sizeof(app_storage_export_t)];
};

static struct platform_os_queue s_requests;
static struct platform_os_queue s_exports;

static void queue_reset(struct platform_os_queue *queue, uint32_t item_size)
{
    (void)memset(queue, 0, sizeof(*queue));
    queue->item_size = item_size;
    queue->capacity = FAKE_ITEMS;
}

static uint32_t queue_index(const struct platform_os_queue *queue,
                            uint32_t offset)
{
    return (queue->head + offset) % queue->capacity;
}

platform_err_t platform_os_queue_send(platform_os_queue_t queue,
                                      const void *item,
                                      uint32_t timeout_ms)
{
    (void)timeout_ms;

    if (queue->used >= queue->capacity)
    {
        return PLATFORM_ERR_BUSY;
    }

    (void)memcpy(queue->slot[queue_index(queue, queue->used)],
                 item, queue->item_size);
    queue->used++;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_os_queue_receive(platform_os_queue_t queue,
                                         void *item,
                                         uint32_t timeout_ms)
{
    (void)timeout_ms;

    if (queue->used == 0u)
    {
        return PLATFORM_ERR_BUSY;
    }

    (void)memcpy(item, queue->slot[queue->head], queue->item_size);
    queue->head = queue_index(queue, 1u);
    queue->used--;

    return PLATFORM_ERR_OK;
}

uint32_t platform_os_queue_free(platform_os_queue_t queue)
{
    return queue->capacity - queue->used;
}

/* --------------------------- platform stubs ---------------------------- */

static uint32_t g_now_ms;

uint32_t platform_os_tick_get(void)
{
    return g_now_ms;
}

uint32_t platform_os_ticks_to_ms(uint32_t ticks)
{
    return ticks;
}

platform_os_critical_state_t platform_os_critical_enter(void)
{
    return 0u;
}

void platform_os_critical_exit(platform_os_critical_state_t state)
{
    (void)state;
}

/* -------------------------- storage service stubs ---------------------- */

static uint8_t        g_saved[SERVICE_STORAGE_BLOB_COUNT][512];
static uint16_t       g_saved_len[SERVICE_STORAGE_BLOB_COUNT];
static int            g_save_calls;
static int            g_flush_wear_calls;
static int            g_flush_wear_at_done = -1;
static platform_err_t g_save_result;

static int            g_append_calls;
static service_storage_ring_t g_append_ring;
static uint8_t        g_append_level;
static uint8_t        g_append_tag;
static char           g_append_text[64];

static int            g_iter_records_left;
static int            g_iter_begins;

platform_err_t service_storage_save_blob(service_storage_blob_t which,
                                         const uint8_t *data,
                                         uint16_t length,
                                         uint32_t now_ms)
{
    (void)now_ms;

    g_save_calls++;

    if ((uint32_t)which < (uint32_t)SERVICE_STORAGE_BLOB_COUNT)
    {
        if (length > (uint16_t)sizeof(g_saved[which]))
        {
            length = (uint16_t)sizeof(g_saved[which]);
        }

        (void)memcpy(g_saved[which], data, length);
        g_saved_len[which] = length;
    }

    return g_save_result;
}

platform_err_t service_storage_flush_wear(uint32_t now_ms)
{
    (void)now_ms;

    g_flush_wear_calls++;

    return PLATFORM_ERR_OK;
}

platform_err_t service_storage_append(service_storage_ring_t ring,
                                      uint8_t level,
                                      uint8_t tag,
                                      const char *text,
                                      uint16_t length,
                                      uint32_t now_ms)
{
    (void)now_ms;

    g_append_calls++;
    g_append_ring = ring;
    g_append_level = level;
    g_append_tag = tag;

    if (length > (uint16_t)(sizeof(g_append_text) - 1u))
    {
        length = (uint16_t)(sizeof(g_append_text) - 1u);
    }

    (void)memcpy(g_append_text, text, length);
    g_append_text[length] = '\0';

    return PLATFORM_ERR_OK;
}

platform_err_t service_storage_iter_begin(service_storage_ring_t ring,
                                          platform_logstore_iter_t *iter,
                                          uint32_t from_seq)
{
    (void)ring;
    (void)from_seq;

    g_iter_begins++;
    (void)memset(iter, 0, sizeof(*iter));

    return PLATFORM_ERR_OK;
}

platform_err_t platform_logstore_iter_next(platform_logstore_iter_t *iter,
                                           platform_logstore_entry_t *entry)
{
    (void)iter;

    if (g_iter_records_left <= 0)
    {
        return PLATFORM_ERR_NO_RESOURCE;
    }

    g_iter_records_left--;

    (void)memset(entry, 0, sizeof(*entry));
    entry->level = 1u;
    entry->tag = APP_STORAGE_TAG_CHASSIS;
    entry->ts = 42u;
    entry->seq = 7u;
    entry->payload[0] = 'h';
    entry->payload[1] = 'i';
    entry->len = 2u;

    return PLATFORM_ERR_OK;
}

platform_err_t service_storage_ring_clear(service_storage_ring_t ring,
                                          uint32_t now_ms)
{
    (void)ring;
    (void)now_ms;
    return PLATFORM_ERR_OK;
}

platform_err_t service_storage_crash_clear(void)
{
    return PLATFORM_ERR_OK;
}

platform_err_t service_storage_crash_info(uint8_t slot,
                                          uint16_t *out_length,
                                          uint8_t *out_flags)
{
    (void)slot;
    (void)out_flags;

    if (out_length != NULL)
    {
        *out_length = 0u;
    }

    return PLATFORM_ERR_OK;
}

platform_err_t service_storage_crash_read(uint8_t slot,
                                          uint16_t offset,
                                          uint8_t *buffer,
                                          uint16_t capacity,
                                          uint16_t *out_length)
{
    (void)slot;
    (void)offset;
    (void)buffer;
    (void)capacity;

    if (out_length != NULL)
    {
        *out_length = 0u;
    }

    return PLATFORM_ERR_OK;
}

platform_err_t service_storage_crash_mark_read(uint8_t slot)
{
    (void)slot;
    return PLATFORM_ERR_OK;
}

platform_bool_t service_storage_crash_slot_valid(uint8_t slot)
{
    (void)slot;
    return PLATFORM_FALSE;
}

platform_bool_t service_storage_crash_latest(uint8_t *out_slot)
{
    (void)out_slot;
    return PLATFORM_FALSE;
}

void service_storage_get_status(service_storage_status_t *status)
{
    (void)memset(status, 0, sizeof(*status));
}

/* ----------------------------- proto stubs ----------------------------- */

static int g_status_frames;

platform_err_t service_proto_send_storage(uint8_t flags, uint8_t log_level,
                                          uint16_t param_seq,
                                          uint16_t calib_seq,
                                          uint16_t reserved,
                                          uint32_t boot_total,
                                          uint32_t log_used,
                                          uint32_t log_erase,
                                          uint32_t last_save_s)
{
    (void)flags;
    (void)log_level;
    (void)param_seq;
    (void)calib_seq;
    (void)reserved;
    (void)boot_total;
    (void)log_used;
    (void)log_erase;
    (void)last_save_s;

    g_status_frames++;
    return PLATFORM_ERR_OK;
}

platform_err_t service_proto_send_log(uint8_t source, uint8_t level,
                                      uint8_t tag, uint8_t flags,
                                      uint32_t ts_s, uint32_t seq,
                                      uint16_t code, const char *text,
                                      uint8_t text_len)
{
    (void)source;
    (void)level;
    (void)tag;
    (void)flags;
    (void)ts_s;
    (void)seq;
    (void)code;
    (void)text;
    (void)text_len;
    return PLATFORM_ERR_OK;
}

platform_err_t service_proto_send_crash(uint8_t slot, uint8_t flags,
                                        uint16_t offset, uint16_t total,
                                        const uint8_t *data,
                                        uint8_t data_len)
{
    (void)slot;
    (void)flags;
    (void)offset;
    (void)total;
    (void)data;
    (void)data_len;
    return PLATFORM_ERR_OK;
}

/* --------------------------- parameter stub ---------------------------- */

static uint8_t g_log_threshold;

platform_bool_t app_params_log_enabled(uint8_t level)
{
    return (level >= g_log_threshold) ? PLATFORM_TRUE : PLATFORM_FALSE;
}

/* ------------------------------ fixtures ------------------------------- */

static int                 g_done_calls;
static app_storage_result_t g_done_result;
static uint16_t            g_done_tag;
static int                 g_done_user_hits;

static void done_cb(app_storage_result_t result, uint16_t tag, void *user)
{
    g_done_calls++;
    g_done_result = result;
    g_done_tag = tag;
    g_flush_wear_at_done = g_flush_wear_calls;

    if (user != NULL)
    {
        g_done_user_hits += *(int *)user;
    }
}

static void init_module(platform_bool_t available)
{
    queue_reset(&s_requests, (uint32_t)sizeof(app_storage_req_t));
    queue_reset(&s_exports, (uint32_t)sizeof(app_storage_export_t));

    g_now_ms = 1000u;
    g_save_calls = 0;
    g_flush_wear_calls = 0;
    g_flush_wear_at_done = -1;
    g_save_result = PLATFORM_ERR_OK;
    g_saved_len[SERVICE_STORAGE_BLOB_PARAM] = 0u;
    g_saved_len[SERVICE_STORAGE_BLOB_CALIB] = 0u;
    g_append_calls = 0;
    g_append_ring = SERVICE_STORAGE_RING_COUNT;
    g_append_level = 0u;
    g_append_tag = 0u;
    g_append_text[0] = '\0';
    g_iter_begins = 0;
    g_iter_records_left = 0;
    g_status_frames = 0;
    g_log_threshold = 0u;
    g_done_calls = 0;
    g_done_result = APP_STORAGE_FAILED;
    g_done_tag = 0u;
    g_done_user_hits = 0;

    CHECK(PLATFORM_IS_OK(app_storage_init(&s_requests, &s_exports, available)),
          "init succeeds");
}

/* -------------------------------- tests -------------------------------- */

static void test_save_is_staged_and_completed(void)
{
    uint8_t buffer[APP_PARAMS_BLOB_PARAM_SIZE];
    uint8_t expected[APP_PARAMS_BLOB_PARAM_SIZE];
    uint32_t i;
    int user = 1;

    init_module(PLATFORM_TRUE);

    for (i = 0u; i < (uint32_t)sizeof(buffer); i++)
    {
        buffer[i] = (uint8_t)(i + 1u);
    }
    (void)memcpy(expected, buffer, sizeof(expected));

    CHECK(app_storage_save_blob(SERVICE_STORAGE_BLOB_PARAM, buffer,
                                (uint16_t)sizeof(buffer), 7u,
                                done_cb, &user) == PLATFORM_TRUE,
          "first save accepted");
    CHECK(app_storage_saves_pending() == PLATFORM_TRUE,
          "accepted save is visible as pending");

    /* The caller's buffer is gone before the task runs; staging kept a copy. */
    (void)memset(buffer, 0xAA, sizeof(buffer));

    app_storage_step(0u);

    CHECK(g_save_calls == 1, "save executed once");
    CHECK(g_saved_len[SERVICE_STORAGE_BLOB_PARAM] == (uint16_t)sizeof(expected),
          "full blob length reached the service");
    CHECK(memcmp(g_saved[SERVICE_STORAGE_BLOB_PARAM], expected,
                 sizeof(expected)) == 0,
          "the staged copy is what was written");
    CHECK(g_done_calls == 1, "callback ran once");
    CHECK(g_done_result == APP_STORAGE_OK && g_done_tag == 7u,
          "callback carries result and tag");
    CHECK(g_done_user_hits == 1, "callback carries user data");
    CHECK(app_storage_saves_pending() == PLATFORM_FALSE,
          "pending cleared after execution");
    CHECK(g_flush_wear_calls == 1,
          "wear record flushed once after the save");
    CHECK(g_flush_wear_at_done == 0,
          "callback runs before the wear flush");

    /* The task only queues the status frame; the comm pump serializes it. */
    app_storage_pump(1000u, PLATFORM_TRUE);
    CHECK(g_status_frames == 1, "save end posts REPORT_STORAGE");
}

static void test_staging_pool_bounds(void)
{
    uint8_t record[APP_STORAGE_STAGING_BYTES];
    uint32_t i;

    init_module(PLATFORM_TRUE);

    for (i = 0u; i < (uint32_t)sizeof(record); i++)
    {
        record[i] = (uint8_t)i;
    }

    CHECK(app_storage_save_blob(SERVICE_STORAGE_BLOB_PARAM, record,
                                (uint16_t)sizeof(record), 0u,
                                NULL, NULL) == PLATFORM_TRUE,
          "staging slot 0");
    CHECK(app_storage_save_blob(SERVICE_STORAGE_BLOB_PARAM, record,
                                (uint16_t)sizeof(record), 1u,
                                NULL, NULL) == PLATFORM_TRUE,
          "staging slot 1");

    CHECK(app_storage_save_blob(SERVICE_STORAGE_BLOB_PARAM, record,
                                (uint16_t)sizeof(record), 2u,
                                NULL, NULL) == PLATFORM_FALSE,
          "a third record is refused, not overwritten");

    app_storage_step(0u);
    app_storage_step(0u);

    CHECK(g_save_calls == 2, "both staged saves executed");

    CHECK(app_storage_save_blob(SERVICE_STORAGE_BLOB_PARAM, record,
                                (uint16_t)sizeof(record), 3u,
                                NULL, NULL) == PLATFORM_TRUE,
          "a released slot is reusable");
}

static void test_large_blob_borrows(void)
{
    uint8_t big[APP_STORAGE_STAGING_BYTES + 1u];
    uint8_t second[APP_STORAGE_STAGING_BYTES + 1u];
    uint32_t i;

    init_module(PLATFORM_TRUE);

    for (i = 0u; i < (uint32_t)sizeof(big); i++)
    {
        big[i] = (uint8_t)(0x5Au ^ (uint8_t)i);
        second[i] = 0x00u;
    }

    CHECK(app_storage_save_blob(SERVICE_STORAGE_BLOB_PARAM, big,
                                (uint16_t)sizeof(big), 0u,
                                NULL, NULL) == PLATFORM_TRUE,
          "large blob borrows the caller's buffer");
    CHECK(app_storage_save_blob(SERVICE_STORAGE_BLOB_PARAM, second,
                                (uint16_t)sizeof(second), 0u,
                                NULL, NULL) == PLATFORM_FALSE,
          "only one borrow at a time");

    app_storage_step(0u);

    CHECK(g_saved_len[SERVICE_STORAGE_BLOB_PARAM] == (uint16_t)sizeof(big),
          "borrowed length written");
    CHECK(memcmp(g_saved[SERVICE_STORAGE_BLOB_PARAM], big, sizeof(big)) == 0,
          "borrowed bytes written as-is");

    CHECK(app_storage_save_blob(SERVICE_STORAGE_BLOB_PARAM, second,
                                (uint16_t)sizeof(second), 0u,
                                NULL, NULL) == PLATFORM_TRUE,
          "the borrow is released after execution");
}

static void test_save_failure_is_reported(void)
{
    uint8_t blob[8] = { 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u };

    init_module(PLATFORM_TRUE);
    g_save_result = PLATFORM_ERR_PARAM;

    CHECK(app_storage_save_blob(SERVICE_STORAGE_BLOB_CALIB, blob,
                                (uint16_t)sizeof(blob), 3u,
                                done_cb, NULL) == PLATFORM_TRUE,
          "request accepted");

    app_storage_step(0u);

    CHECK(g_done_calls == 1 && g_done_result == APP_STORAGE_FAILED,
          "service failure maps to FAILED");
    CHECK(g_done_tag == 3u, "tag survives a failure");
    CHECK(app_storage_saves_pending() == PLATFORM_FALSE,
          "pending cleared after a failed save");
}

static void test_log_filter_and_append(void)
{
    app_storage_stats_t stats;

    init_module(PLATFORM_TRUE);

    g_log_threshold = 3u;

    CHECK(app_storage_append_log(1u, APP_STORAGE_TAG_COMM, "skip", 4u) ==
          PLATFORM_FALSE, "line below the level is dropped");

    app_storage_get_stats(&stats);
    CHECK(stats.drops == 1u, "the filtered line is counted as a drop");

    g_log_threshold = 0u;

    CHECK(app_storage_append_log(1u, APP_STORAGE_TAG_COMM, "hello", 5u) ==
          PLATFORM_TRUE, "line at/above the level is queued");

    app_storage_step(0u);

    CHECK(g_append_calls == 1, "one append reached the service");
    CHECK(g_append_ring == SERVICE_STORAGE_RING_LOG, "runtime ring");
    CHECK(g_append_level == 1u && g_append_tag == APP_STORAGE_TAG_COMM,
          "level and tag preserved");
    CHECK(strcmp(g_append_text, "hello") == 0, "text preserved");
}

static void test_unavailable_flash_refuses_everything(void)
{
    uint8_t blob[4] = { 0u, 1u, 2u, 3u };

    init_module(PLATFORM_FALSE);

    CHECK(app_storage_available() == PLATFORM_FALSE, "not available");
    CHECK(app_storage_save_blob(SERVICE_STORAGE_BLOB_PARAM, blob,
                                (uint16_t)sizeof(blob), 0u, NULL, NULL) ==
          PLATFORM_FALSE, "save refused");
    CHECK(app_storage_append_log(0u, APP_STORAGE_TAG_COMM, "x", 1u) ==
          PLATFORM_FALSE, "log refused");
    CHECK(app_storage_clear(APP_STORAGE_CLEAR_ALL, NULL, NULL) ==
          PLATFORM_FALSE, "clear refused");
    CHECK(app_storage_export(PROTOCOL_LOG_SOURCE_RUN, PLATFORM_TRUE, 0u,
                             NULL, NULL) == PLATFORM_FALSE, "export refused");
    CHECK(app_storage_crash_read(PROTOCOL_CRASH_SLOT_LATEST, PLATFORM_FALSE,
                                 NULL, NULL) == PLATFORM_FALSE,
          "crash read refused");
}

static void test_export_pacing_and_end_frame(void)
{
    app_storage_export_t item;

    init_module(PLATFORM_TRUE);
    g_iter_records_left = 1;

    CHECK(app_storage_export(PROTOCOL_LOG_SOURCE_RUN, PLATFORM_TRUE, 0u,
                             NULL, NULL) == PLATFORM_TRUE, "export accepted");
    CHECK(app_storage_exporting() == PLATFORM_TRUE, "channel is busy");
    CHECK(app_storage_export(PROTOCOL_LOG_SOURCE_FAULT, PLATFORM_TRUE, 0u,
                             NULL, NULL) == PLATFORM_FALSE,
          "second export refused");

    app_storage_step(0u);   /* executes the start request */
    CHECK(g_iter_begins == 1, "the ring iterator was opened once");

    app_storage_step(0u);   /* first record */
    CHECK(PLATFORM_IS_OK(platform_os_queue_receive(&s_exports, &item, 0u)),
          "a record was produced");
    CHECK(item.kind == (uint8_t)APP_STORAGE_EXPORT_LOG, "log record");
    CHECK(item.u.log.seq == 7u && item.u.log.text_len == 2u, "record fields");
    CHECK((item.u.log.flags & PROTOCOL_LOG_FLAG_MORE) != 0u,
          "every record carries MORE");

    app_storage_step(0u);   /* inside the pacing window */
    CHECK(PLATFORM_IS_ERR(platform_os_queue_receive(&s_exports, &item, 0u)),
          "no second record inside one pacing period");

    g_now_ms += APP_STORAGE_EXPORT_PERIOD_MS;
    app_storage_step(0u);   /* iterator exhausted: end frame */

    CHECK(PLATFORM_IS_OK(platform_os_queue_receive(&s_exports, &item, 0u)),
          "end frame produced");
    CHECK(item.u.log.flags == PROTOCOL_LOG_FLAG_END, "flags = END only");
    CHECK(item.u.log.text_len == 0u, "end frame carries no text");
    CHECK(app_storage_exporting() == PLATFORM_FALSE,
          "channel is free after the end frame");
}

int main(void)
{
    test_save_is_staged_and_completed();
    test_staging_pool_bounds();
    test_large_blob_borrows();
    test_save_failure_is_reported();
    test_log_filter_and_append();
    test_unavailable_flash_refuses_everything();
    test_export_pacing_and_end_frame();

    if (g_failed != 0)
    {
        printf("FAIL: app_storage tests (%d checks)\n", g_failed);
        return 1;
    }

    printf("PASS: app_storage tests\n");
    return 0;
}
