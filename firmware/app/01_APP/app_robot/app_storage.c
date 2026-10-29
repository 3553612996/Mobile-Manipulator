/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_storage.c
 *
 * @par dependencies
 * - app_storage.h
 * - app_robot_config.h
 * - service_storage.h, service_proto.h, platform_logstore.h
 * - platform_os_task.h, platform_os_time.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement the storage task, its queue protocol and the export pump.
 *
 * Two tasks, one direction each:
 *
 *   producer --(app_storage_req_t)--> storage task --(service_storage_*)--> flash
 *   storage task --(app_storage_export_t)--> comm task --(service_proto_*)--> UART
 *
 * Everything to the left of the flash is descriptor-in / callback-out; nothing
 * but this task ever touches the bus. Everything to the right of it is plain
 * bytes queued for the comm task, because that task is the only UART writer.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include <string.h>

#include "app_robot/app_storage.h"
#include "app_robot/app_params.h"
#include "app_robot/app_robot_config.h"

#include "platform_os_task.h"
#include "platform_os_time.h"
#include "platform_os_critical.h"
#include "platform_logstore.h"

//******************************** Defines **********************************//

#define APP_STORAGE_NO_STAGING     ((uint8_t)0xFFu)

//******************************** Variables ********************************//

/* Staging pool: small records are copied here at enqueue time. */
static uint8_t        s_staging[APP_STORAGE_STAGING_SLOTS]
                                [APP_STORAGE_STAGING_BYTES];
static platform_bool_t s_staging_busy[APP_STORAGE_STAGING_SLOTS];

static struct
{
    platform_bool_t         initialized;
    platform_bool_t         available;
    platform_os_queue_t     requests;
    platform_os_queue_t     exports;

    /* One borrowed blob at a time: see the header comment. */
    volatile platform_bool_t borrow_busy;

    /* Accepted but not yet executed saves; the protocol allows one at a time. */
    volatile uint8_t         saves_pending;

    /* Export channel state, readable from the comm task to answer BUSY. */
    volatile platform_bool_t export_busy;
    platform_bool_t          status_pending;

    app_storage_stats_t      stats;
} s_storage;

/* Storage-task-private export state. */
static struct
{
    platform_bool_t             active;
    uint8_t                     source;
    uint32_t                    last_emit_ms;
    platform_logstore_iter_t    iter;
    platform_logstore_entry_t   entry;
} s_export;

static struct
{
    platform_bool_t active;
    uint8_t         slot;
    uint16_t        offset;
    uint16_t        total;
    platform_bool_t mark_read;
    uint32_t        last_emit_ms;
} s_crash;

/* Comm-task-private: last time the host was heard. */
static uint32_t s_last_link_ms;

//******************************** Functions ********************************//

static uint32_t s_now_ms(void)
{
    return platform_os_ticks_to_ms(platform_os_tick_get());
}

static app_storage_result_t s_map_err(platform_err_t err)
{
    if (PLATFORM_IS_OK(err))
    {
        return APP_STORAGE_OK;
    }

    if (err == PLATFORM_ERR_NOT_INITIALIZED)
    {
        return APP_STORAGE_UNAVAILABLE;
    }

    if (err == PLATFORM_ERR_NO_RESOURCE)
    {
        return APP_STORAGE_EMPTY;
    }

    return APP_STORAGE_FAILED;
}

static uint8_t s_level_of_fault(uint16_t code)
{
    switch (protocol_fault_severity(code))
    {
        case PROTOCOL_SEVERITY_WARN:
            return 2u;
        case PROTOCOL_SEVERITY_FATAL:
            return 3u;
        case PROTOCOL_SEVERITY_ERROR:
            return 3u;
        default:
            return 1u;
    }
}

//***************************** Staging pool ********************************//

/*
 * The pool is shared by every producer (comm task now, fault policy and the
 * control task later), while only the storage task releases slots. A critical
 * section around the claim is cheaper than a mutex and is the only shared
 * state where a lost update would corrupt a record instead of losing one.
 */
static uint8_t s_staging_take(const uint8_t *data, uint16_t length)
{
    platform_os_critical_state_t state;
    uint32_t i;
    uint8_t found = APP_STORAGE_NO_STAGING;

    if ((data == NULL) || (length > APP_STORAGE_STAGING_BYTES))
    {
        return APP_STORAGE_NO_STAGING;
    }

    state = platform_os_critical_enter();

    for (i = 0u; i < APP_STORAGE_STAGING_SLOTS; i++)
    {
        if (s_staging_busy[i] == PLATFORM_FALSE)
        {
            (void)memcpy(s_staging[i], data, length);
            s_staging_busy[i] = PLATFORM_TRUE;
            found = (uint8_t)i;
            break;
        }
    }

    platform_os_critical_exit(state);

    return found;
}

static void s_staging_release(uint8_t index)
{
    platform_os_critical_state_t state;

    if (index >= APP_STORAGE_STAGING_SLOTS)
    {
        return;
    }

    state = platform_os_critical_enter();
    s_staging_busy[index] = PLATFORM_FALSE;
    platform_os_critical_exit(state);
}

//****************************** Export items *******************************//

static platform_bool_t s_export_post(const app_storage_export_t *item)
{
    if (s_storage.exports == NULL)
    {
        return PLATFORM_FALSE;
    }

    return PLATFORM_IS_OK(platform_os_queue_send(s_storage.exports, item, 0u))
               ? PLATFORM_TRUE
               : PLATFORM_FALSE;
}

/* Ask the comm task for one REPORT_STORAGE frame (save finished, log cleared). */
static void s_post_status(void)
{
    app_storage_export_t item;

    (void)memset(&item, 0, sizeof(item));
    item.kind = (uint8_t)APP_STORAGE_EXPORT_STATUS;

    (void)s_export_post(&item);
}

static void s_export_finish_log(uint32_t now_ms)
{
    app_storage_export_t item;

    (void)memset(&item, 0, sizeof(item));
    item.kind = (uint8_t)APP_STORAGE_EXPORT_LOG;
    item.u.log.source = s_export.source;
    item.u.log.flags = PROTOCOL_LOG_FLAG_END;

    (void)s_export_post(&item);

    s_export.active = PLATFORM_FALSE;
    s_storage.export_busy = PLATFORM_FALSE;
    s_export.last_emit_ms = now_ms;
}

static void s_produce_log(uint32_t now_ms)
{
    app_storage_export_t item;
    platform_err_t err;

    err = platform_logstore_iter_next(&s_export.iter, &s_export.entry);
    if (PLATFORM_IS_ERR(err))
    {
        /*
         * Both the ring end and an I/O error close the export with an end
         * frame: the host must be able to tell "finished" from "link dropped".
         */
        s_export_finish_log(now_ms);
        return;
    }

    (void)memset(&item, 0, sizeof(item));
    item.kind = (uint8_t)APP_STORAGE_EXPORT_LOG;
    item.u.log.source = s_export.source;
    item.u.log.level = s_export.entry.level;
    item.u.log.tag = s_export.entry.tag;
    item.u.log.ts_s = s_export.entry.ts;
    item.u.log.seq = s_export.entry.seq;

    /*
     * Every record carries MORE because an end frame always follows; a host
     * that sees MORE stay set has lost the end frame, which is exactly the
     * information it needs.
     */
    item.u.log.flags = PROTOCOL_LOG_FLAG_MORE;

    if (s_export.source == PROTOCOL_LOG_SOURCE_FAULT)
    {
        if (s_export.entry.len >= 2u)
        {
            item.u.log.code = (uint16_t)((uint16_t)s_export.entry.payload[0] |
                                         ((uint16_t)s_export.entry.payload[1]
                                          << 8));
        }
        item.u.log.text_len = 0u;
    }
    else
    {
        item.u.log.text_len =
            (s_export.entry.len > APP_STORAGE_LOG_TEXT_MAX)
                ? (uint8_t)APP_STORAGE_LOG_TEXT_MAX
                : (uint8_t)s_export.entry.len;
        item.u.log.code = 0u;

        if (item.u.log.text_len > 0u)
        {
            (void)memcpy(item.u.log.text, s_export.entry.payload,
                         item.u.log.text_len);
        }
    }

    if (s_export_post(&item) == PLATFORM_FALSE)
    {
        /* Queue full: retry this record after the next pacing period. */
        return;
    }

    s_export.last_emit_ms = now_ms;
}

static void s_produce_crash(uint32_t now_ms)
{
    app_storage_export_t item;
    uint16_t count;
    uint16_t got = 0u;
    platform_err_t err;

    count = (uint16_t)(s_crash.total - s_crash.offset);
    if (count > APP_STORAGE_CRASH_DATA_MAX)
    {
        count = APP_STORAGE_CRASH_DATA_MAX;
    }

    (void)memset(&item, 0, sizeof(item));
    item.kind = (uint8_t)APP_STORAGE_EXPORT_CRASH;

    err = service_storage_crash_read(s_crash.slot, s_crash.offset,
                                     item.u.crash.data, count, &got);
    if (PLATFORM_IS_ERR(err) || (got == 0u))
    {
        /*
         * The slot went away underneath us (LOG_CLEAR, failed re-validation).
         * A crash export has no end frame by design, so the host times out on
         * offset + length and starts over.
         */
        s_crash.active = PLATFORM_FALSE;
        s_storage.export_busy = PLATFORM_FALSE;
        s_post_status();
        return;
    }

    item.u.crash.slot = s_crash.slot;
    item.u.crash.offset = s_crash.offset;
    item.u.crash.total = s_crash.total;
    item.u.crash.data_len = (uint8_t)got;
    item.u.crash.flags = ((uint16_t)(s_crash.offset + got) < s_crash.total)
                             ? PROTOCOL_CRASH_FLAG_MORE
                             : 0u;

    if (s_export_post(&item) == PLATFORM_FALSE)
    {
        return;
    }

    s_crash.offset = (uint16_t)(s_crash.offset + got);
    s_crash.last_emit_ms = now_ms;

    if (s_crash.offset >= s_crash.total)
    {
        s_crash.active = PLATFORM_FALSE;
        s_storage.export_busy = PLATFORM_FALSE;

        if (s_crash.mark_read == PLATFORM_TRUE)
        {
            (void)service_storage_crash_mark_read(s_crash.slot);
        }

        s_post_status();
    }
}

/* Called once per loop iteration: at most one record per pacing period. */
static void s_produce(uint32_t now_ms)
{
    if (s_storage.exports == NULL)
    {
        return;
    }

    if (platform_os_queue_free(s_storage.exports) == 0u)
    {
        return;
    }

    if (s_export.active == PLATFORM_TRUE)
    {
        if ((now_ms - s_export.last_emit_ms) < APP_STORAGE_EXPORT_PERIOD_MS)
        {
            return;
        }

        s_produce_log(now_ms);
        return;
    }

    if (s_crash.active == PLATFORM_TRUE)
    {
        if ((now_ms - s_crash.last_emit_ms) < APP_STORAGE_EXPORT_PERIOD_MS)
        {
            return;
        }

        s_produce_crash(now_ms);
    }
}

//*************************** Request execution *****************************//

static app_storage_result_t s_start_log_export(uint8_t source, uint32_t from_seq,
                                               uint32_t now_ms)
{
    service_storage_ring_t ring;

    if (source > PROTOCOL_LOG_SOURCE_FAULT)
    {
        return APP_STORAGE_FAILED;
    }

    ring = (source == PROTOCOL_LOG_SOURCE_FAULT)
               ? SERVICE_STORAGE_RING_FAULT
               : SERVICE_STORAGE_RING_LOG;

    if (PLATFORM_IS_ERR(service_storage_iter_begin(ring, &s_export.iter,
                                                   from_seq)))
    {
        return APP_STORAGE_FAILED;
    }

    s_export.active = PLATFORM_TRUE;
    s_export.source = source;
    s_export.last_emit_ms = now_ms - APP_STORAGE_EXPORT_PERIOD_MS;

    return APP_STORAGE_OK;
}

static app_storage_result_t s_start_crash_read(uint8_t slot,
                                               platform_bool_t mark_read,
                                               uint32_t now_ms)
{
    uint16_t total = 0u;

    if (slot == PROTOCOL_CRASH_SLOT_LATEST)
    {
        if (service_storage_crash_latest(&slot) == PLATFORM_FALSE)
        {
            return APP_STORAGE_EMPTY;
        }
    }

    if (PLATFORM_IS_ERR(service_storage_crash_info(slot, &total, NULL)))
    {
        return APP_STORAGE_EMPTY;
    }

    s_crash.active = PLATFORM_TRUE;
    s_crash.slot = slot;
    s_crash.offset = 0u;
    s_crash.total = total;
    s_crash.mark_read = mark_read;
    s_crash.last_emit_ms = now_ms - APP_STORAGE_EXPORT_PERIOD_MS;

    return APP_STORAGE_OK;
}

static app_storage_result_t s_execute(const app_storage_req_t *req)
{
    uint32_t now_ms;
    platform_err_t err;
    app_storage_result_t result;

    now_ms = s_now_ms();

    if (s_storage.available == PLATFORM_FALSE)
    {
        return APP_STORAGE_UNAVAILABLE;
    }

    switch (req->kind)
    {
        case APP_STORAGE_REQ_SAVE:
            err = service_storage_save_blob((service_storage_blob_t)req->arg0,
                                            req->payload, req->length, now_ms);
            result = s_map_err(err);

            if (result == APP_STORAGE_OK)
            {
                s_storage.stats.saves++;
            }
            else
            {
                s_storage.stats.save_failures++;
            }

            /*
             * Clear the in-flight mark before the report goes out. The comm
             * task runs at a higher priority than this one, so a host that
             * answers REPORT_STORAGE with the next PARAM_SAVE while this task
             * is still suspended would otherwise see saves_pending == 1 and
             * get BUSY for a save that already finished.
             */
            if (s_storage.saves_pending > 0u)
            {
                s_storage.saves_pending--;
            }

            s_post_status();
            break;

        case APP_STORAGE_REQ_LOG:
            err = service_storage_append((service_storage_ring_t)req->arg0,
                                         req->arg1, req->arg2,
                                         (const char *)req->payload,
                                         req->length, now_ms);
            result = s_map_err(err);

            if (result == APP_STORAGE_OK)
            {
                s_storage.stats.logs++;
            }
            break;

        case APP_STORAGE_REQ_CLEAR:
            result = APP_STORAGE_OK;

            if ((req->arg0 == APP_STORAGE_CLEAR_RUN) ||
                (req->arg0 == APP_STORAGE_CLEAR_ALL))
            {
                err = service_storage_ring_clear(SERVICE_STORAGE_RING_LOG,
                                                 now_ms);
                result = s_map_err(err);
            }

            if ((result == APP_STORAGE_OK) &&
                ((req->arg0 == APP_STORAGE_CLEAR_FAULT) ||
                 (req->arg0 == APP_STORAGE_CLEAR_ALL)))
            {
                err = service_storage_ring_clear(SERVICE_STORAGE_RING_FAULT,
                                                 now_ms);
                result = s_map_err(err);
            }

            if ((result == APP_STORAGE_OK) &&
                ((req->arg0 == APP_STORAGE_CLEAR_CRASH) ||
                 (req->arg0 == APP_STORAGE_CLEAR_ALL)))
            {
                result = s_map_err(service_storage_crash_clear());
            }

            if (req->arg0 > APP_STORAGE_CLEAR_ALL)
            {
                result = APP_STORAGE_FAILED;
            }

            if (result == APP_STORAGE_OK)
            {
                s_post_status();
            }
            break;

        case APP_STORAGE_REQ_EXPORT:
            if (req->arg1 == 0u)
            {
                /* Stop: the export queue is left to drain, then silence. */
                if (s_export.active == PLATFORM_TRUE)
                {
                    s_export_finish_log(now_ms);
                }

                if (s_crash.active == PLATFORM_TRUE)
                {
                    /*
                     * A crash export has no end frame by design: the host
                     * times out on offset + total and knows it was cut short.
                     */
                    s_crash.active = PLATFORM_FALSE;
                    s_storage.export_busy = PLATFORM_FALSE;
                    s_post_status();
                }

                result = APP_STORAGE_OK;
            }
            else
            {
                result = s_start_log_export(req->arg0, req->value, now_ms);

                if (result != APP_STORAGE_OK)
                {
                    s_storage.export_busy = PLATFORM_FALSE;
                }
            }

            if (result == APP_STORAGE_OK)
            {
                s_storage.stats.exports++;
            }
            break;

        case APP_STORAGE_REQ_CRASH_READ:
            result = s_start_crash_read(req->arg0,
                                        (req->arg1 != 0u) ? PLATFORM_TRUE
                                                          : PLATFORM_FALSE,
                                        now_ms);
            if (result != APP_STORAGE_OK)
            {
                s_storage.export_busy = PLATFORM_FALSE;
            }
            break;

        default:
            result = APP_STORAGE_FAILED;
            break;
    }

    return result;
}

void app_storage_step(uint32_t timeout_ms)
{
    app_storage_req_t req;
    app_storage_result_t result;

    s_produce(s_now_ms());

    if (PLATFORM_IS_OK(platform_os_queue_receive(s_storage.requests,
                                                 &req,
                                                 timeout_ms)))
    {
        result = s_execute(&req);

        /* saves_pending is cleared inside s_execute, before its report. */

        s_staging_release(req.staging);

        if ((req.length > APP_STORAGE_STAGING_BYTES) &&
            (req.staging == APP_STORAGE_NO_STAGING))
        {
            /* The request borrowed the caller's buffer. */
            s_storage.borrow_busy = PLATFORM_FALSE;
        }

        if (req.done != NULL)
        {
            req.done(result, (uint16_t)req.arg2, req.user);
        }

        if (req.kind == (uint8_t)APP_STORAGE_REQ_SAVE)
        {
            /*
             * The wear record erases another sector. It runs after the
             * completion report and the caller's callback, so the host never
             * observes "save finished" while the storage task is still busy
             * (see service_storage_save_blob).
             */
            (void)service_storage_flush_wear(s_now_ms());
        }
    }
}

void app_storage_entry(void *argument)
{
    (void)argument;

    for (;;)
    {
        app_storage_step(APP_STORAGE_TICK_MS);
    }
}

//****************************** Enqueue path *******************************//

static platform_bool_t s_enqueue(const app_storage_req_t *req)
{
    if ((s_storage.initialized == PLATFORM_FALSE) ||
        (s_storage.requests == NULL))
    {
        return PLATFORM_FALSE;
    }

    if (PLATFORM_IS_ERR(platform_os_queue_send(s_storage.requests, req, 0u)))
    {
        return PLATFORM_FALSE;
    }

    s_storage.stats.requests++;

    return PLATFORM_TRUE;
}

platform_err_t app_storage_init(platform_os_queue_t requests,
                                platform_os_queue_t exports,
                                platform_bool_t service_ok)
{
    uint32_t i;

    if ((requests == NULL) || (exports == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    for (i = 0u; i < APP_STORAGE_STAGING_SLOTS; i++)
    {
        s_staging_busy[i] = PLATFORM_FALSE;
    }

    s_storage.requests = requests;
    s_storage.exports = exports;
    s_storage.available = (service_ok == PLATFORM_TRUE) ? PLATFORM_TRUE
                                                        : PLATFORM_FALSE;
    s_storage.borrow_busy = PLATFORM_FALSE;
    s_storage.saves_pending = 0u;
    s_storage.export_busy = PLATFORM_FALSE;
    s_storage.status_pending = PLATFORM_FALSE;
    (void)memset(&s_storage.stats, 0, sizeof(s_storage.stats));

    s_export.active = PLATFORM_FALSE;
    s_export.source = PROTOCOL_LOG_SOURCE_RUN;
    s_export.last_emit_ms = 0u;

    s_crash.active = PLATFORM_FALSE;
    s_crash.last_emit_ms = 0u;

    s_last_link_ms = s_now_ms();

    s_storage.initialized = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_bool_t app_storage_available(void)
{
    return s_storage.available;
}

platform_bool_t app_storage_save_blob(service_storage_blob_t which,
                                      const uint8_t *data,
                                      uint16_t length,
                                      uint8_t tag,
                                      app_storage_done_t done,
                                      void *user_data)
{
    app_storage_req_t req;
    uint8_t staging;

    if ((s_storage.available == PLATFORM_FALSE) || (data == NULL) ||
        (length == 0u) || (length > SERVICE_STORAGE_MAX_BLOB_LENGTH))
    {
        return PLATFORM_FALSE;
    }

    (void)memset(&req, 0, sizeof(req));
    req.kind = (uint8_t)APP_STORAGE_REQ_SAVE;
    req.arg0 = (uint8_t)which;
    req.arg2 = tag;
    req.length = length;
    req.done = done;
    req.user = user_data;

    if (length <= APP_STORAGE_STAGING_BYTES)
    {
        staging = s_staging_take(data, length);

        if (staging == APP_STORAGE_NO_STAGING)
        {
            return PLATFORM_FALSE;
        }

        req.staging = staging;
        req.payload = s_staging[staging];
    }
    else
    {
        if (s_storage.borrow_busy == PLATFORM_TRUE)
        {
            return PLATFORM_FALSE;
        }

        s_storage.borrow_busy = PLATFORM_TRUE;
        req.staging = APP_STORAGE_NO_STAGING;
        req.payload = data;
    }

    /*
     * Count before the item can be seen by the storage task, and take it back
     * if the queue refused it: PARAM_SAVE answers BUSY from this counter, so a
     * save that never reached the queue must not leave it stuck.
     */
    s_storage.saves_pending++;

    if (s_enqueue(&req) == PLATFORM_FALSE)
    {
        s_storage.saves_pending--;

        if (req.staging != APP_STORAGE_NO_STAGING)
        {
            s_staging_release(req.staging);
        }
        else
        {
            s_storage.borrow_busy = PLATFORM_FALSE;
        }

        return PLATFORM_FALSE;
    }

    return PLATFORM_TRUE;
}

platform_bool_t app_storage_append_log(uint8_t level,
                                       uint8_t tag,
                                       const char *text,
                                       uint16_t length)
{
    app_storage_req_t req;
    uint8_t staging;

    if ((s_storage.available == PLATFORM_FALSE) || (text == NULL) ||
        (length == 0u))
    {
        return PLATFORM_FALSE;
    }

    /* The configured level is a filter, not a stored attribute. */
    if (app_params_log_enabled(level) == PLATFORM_FALSE)
    {
        s_storage.stats.drops++;
        return PLATFORM_FALSE;
    }

    staging = s_staging_take((const uint8_t *)text, length);
    if (staging == APP_STORAGE_NO_STAGING)
    {
        s_storage.stats.drops++;
        return PLATFORM_FALSE;
    }

    (void)memset(&req, 0, sizeof(req));
    req.kind = (uint8_t)APP_STORAGE_REQ_LOG;
    req.arg0 = (uint8_t)SERVICE_STORAGE_RING_LOG;
    req.arg1 = level;
    req.arg2 = tag;
    req.length = length;
    req.staging = staging;
    req.payload = s_staging[staging];

    if (s_enqueue(&req) == PLATFORM_FALSE)
    {
        s_staging_release(staging);
        s_storage.stats.drops++;
        return PLATFORM_FALSE;
    }

    return PLATFORM_TRUE;
}

platform_bool_t app_storage_append_fault(uint16_t code)
{
    app_storage_req_t req;
    uint8_t record[2];
    uint8_t staging;

    if (s_storage.available == PLATFORM_FALSE)
    {
        return PLATFORM_FALSE;
    }

    /* The same filter as text lines: a level below the threshold is noise. */
    if (app_params_log_enabled(s_level_of_fault(code)) == PLATFORM_FALSE)
    {
        s_storage.stats.drops++;
        return PLATFORM_FALSE;
    }

    /* Little-endian, the same rule as every other on-flash field. */
    record[0] = (uint8_t)(code & 0xFFu);
    record[1] = (uint8_t)((code >> 8) & 0xFFu);

    staging = s_staging_take(record, 2u);
    if (staging == APP_STORAGE_NO_STAGING)
    {
        s_storage.stats.drops++;
        return PLATFORM_FALSE;
    }

    (void)memset(&req, 0, sizeof(req));
    req.kind = (uint8_t)APP_STORAGE_REQ_LOG;
    req.arg0 = (uint8_t)SERVICE_STORAGE_RING_FAULT;
    req.arg1 = s_level_of_fault(code);
    req.arg2 = (uint8_t)((code >> 8) & 0xFFu);
    req.length = 2u;
    req.staging = staging;
    req.payload = s_staging[staging];

    if (s_enqueue(&req) == PLATFORM_FALSE)
    {
        s_staging_release(staging);
        s_storage.stats.drops++;
        return PLATFORM_FALSE;
    }

    return PLATFORM_TRUE;
}

platform_bool_t app_storage_clear(uint8_t source,
                                  app_storage_done_t done,
                                  void *user_data)
{
    app_storage_req_t req;

    if ((s_storage.available == PLATFORM_FALSE) ||
        (source > APP_STORAGE_CLEAR_ALL))
    {
        return PLATFORM_FALSE;
    }

    (void)memset(&req, 0, sizeof(req));
    req.kind = (uint8_t)APP_STORAGE_REQ_CLEAR;
    req.arg0 = source;
    req.arg2 = source;
    req.staging = APP_STORAGE_NO_STAGING;
    req.done = done;
    req.user = user_data;

    return s_enqueue(&req);
}

platform_bool_t app_storage_export(uint8_t source,
                                   platform_bool_t start,
                                   uint32_t from_seq,
                                   app_storage_done_t done,
                                   void *user_data)
{
    app_storage_req_t req;

    if (s_storage.available == PLATFORM_FALSE)
    {
        return PLATFORM_FALSE;
    }

    if ((start == PLATFORM_TRUE) && (s_storage.export_busy == PLATFORM_TRUE))
    {
        return PLATFORM_FALSE;
    }

    (void)memset(&req, 0, sizeof(req));
    req.kind = (uint8_t)APP_STORAGE_REQ_EXPORT;
    req.arg0 = source;
    req.arg1 = (start == PLATFORM_TRUE) ? 1u : 0u;
    req.arg2 = source;
    req.value = from_seq;
    req.staging = APP_STORAGE_NO_STAGING;
    req.done = done;
    req.user = user_data;

    if (s_enqueue(&req) == PLATFORM_FALSE)
    {
        return PLATFORM_FALSE;
    }

    if (start == PLATFORM_TRUE)
    {
        /*
         * Claim the channel before the storage task runs, otherwise a second
         * LOG_EXPORT arriving inside the same 10 ms would also be accepted and
         * the host would see two interleaved exports.
         */
        s_storage.export_busy = PLATFORM_TRUE;
    }

    return PLATFORM_TRUE;
}

platform_bool_t app_storage_crash_read(uint8_t slot,
                                       platform_bool_t mark_read,
                                       app_storage_done_t done,
                                       void *user_data)
{
    app_storage_req_t req;

    if (s_storage.available == PLATFORM_FALSE)
    {
        return PLATFORM_FALSE;
    }

    if (s_storage.export_busy == PLATFORM_TRUE)
    {
        return PLATFORM_FALSE;
    }

    if ((slot != PROTOCOL_CRASH_SLOT_LATEST) &&
        (service_storage_crash_slot_valid(slot) == PLATFORM_FALSE))
    {
        return PLATFORM_FALSE;
    }

    (void)memset(&req, 0, sizeof(req));
    req.kind = (uint8_t)APP_STORAGE_REQ_CRASH_READ;
    req.arg0 = slot;
    req.arg1 = (mark_read == PLATFORM_TRUE) ? 1u : 0u;
    req.arg2 = slot;
    req.staging = APP_STORAGE_NO_STAGING;
    req.done = done;
    req.user = user_data;

    if (s_enqueue(&req) == PLATFORM_FALSE)
    {
        return PLATFORM_FALSE;
    }

    s_storage.export_busy = PLATFORM_TRUE;

    return PLATFORM_TRUE;
}

platform_bool_t app_storage_exporting(void)
{
    return s_storage.export_busy;
}

platform_bool_t app_storage_saves_pending(void)
{
    return (s_storage.saves_pending != 0u) ? PLATFORM_TRUE : PLATFORM_FALSE;
}

void app_storage_get_stats(app_storage_stats_t *out)
{
    if (out == NULL)
    {
        return;
    }

    *out = s_storage.stats;
}

//***************************** Comm-task pump ******************************//

void app_storage_request_status(void)
{
    s_storage.status_pending = PLATFORM_TRUE;
}

static void s_send_status(void)
{
    service_storage_status_t status;

    (void)memset(&status, 0, sizeof(status));
    service_storage_get_status(&status);

    (void)service_proto_send_storage(status.flags,
                                     status.log_level,
                                     status.param_seq,
                                     status.calib_seq,
                                     status.reserved,
                                     status.boot_total,
                                     status.log_used,
                                     status.log_erase,
                                     status.last_save_s);
}

void app_storage_pump(uint32_t now_ms, platform_bool_t link_alive)
{
    app_storage_export_t item;

    if (s_storage.initialized == PLATFORM_FALSE)
    {
        return;
    }

    if (s_storage.status_pending == PLATFORM_TRUE)
    {
        s_storage.status_pending = PLATFORM_FALSE;
        s_send_status();
    }

    if (link_alive == PLATFORM_TRUE)
    {
        s_last_link_ms = now_ms;
    }
    else if ((s_storage.export_busy == PLATFORM_TRUE) &&
             ((now_ms - s_last_link_ms) > APP_STORAGE_EXPORT_IDLE_MS))
    {
        /*
         * Host went quiet: stop the export instead of talking to a void. The
         * source only matters for a start request; a stop ends whichever
         * export is running, log ring or crash slot.
         */
        (void)app_storage_export(PROTOCOL_LOG_SOURCE_RUN, PLATFORM_FALSE, 0u,
                                 NULL, NULL);
        s_last_link_ms = now_ms;
    }

    while (PLATFORM_IS_OK(platform_os_queue_receive(s_storage.exports,
                                                    &item, 0u)))
    {
        switch (item.kind)
        {
            case APP_STORAGE_EXPORT_LOG:
                (void)service_proto_send_log(item.u.log.source,
                                             item.u.log.level,
                                             item.u.log.tag,
                                             item.u.log.flags,
                                             item.u.log.ts_s,
                                             item.u.log.seq,
                                             item.u.log.code,
                                             item.u.log.text,
                                             item.u.log.text_len);
                break;

            case APP_STORAGE_EXPORT_CRASH:
                (void)service_proto_send_crash(item.u.crash.slot,
                                               item.u.crash.flags,
                                               item.u.crash.offset,
                                               item.u.crash.total,
                                               item.u.crash.data,
                                               item.u.crash.data_len);
                break;

            case APP_STORAGE_EXPORT_STATUS:
            default:
                s_send_status();
                break;
        }
    }
}
