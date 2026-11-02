/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_storage.h
 *
 * @par dependencies
 * - service_storage.h
 * - platform_os_queue.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Storage task: the single owner of every external-flash operation.
 *
 * Why a task and not a call
 * -------------------------
 * A sector erase holds the SPI bus lock for up to a second. Any task that
 * called service_storage_* directly would stall that long — the comm task
 * would miss its 200 ms command timeout and the control task would stop
 * updating the motors. So the flash gets one owner, and everybody else talks
 * to it through a queue.
 *
 * The queue carries descriptors, not payloads
 * -------------------------------------------
 * One static queue slot is 512 bytes for the whole queue, so a 4 KiB blob can
 * never travel inside a message. Two mechanisms replace copying everything:
 *
 *   - records up to APP_STORAGE_STAGING_BYTES are copied into a small pool of
 *     staging slots when they are queued, because log lines are often built in
 *     a caller's stack frame;
 *   - blobs larger than that borrow the caller's buffer. Only one borrow may be
 *     outstanding at a time; a second request is refused with BUSY, so the
 *     caller always knows when its buffer is free again.
 *
 * Completion
 * ----------
 * When a request carries a callback it is called exactly once, in the storage
 * task, after the flash operation returned — and never at all when the request
 * was refused at enqueue time. A callback must not block, must not call back
 * into this module, and may be NULL. The protocol layer never uses one: its
 * ACK means "accepted" and the outcome arrives in REPORT_STORAGE.
 *
 * Threading
 * ---------
 *   producers (comm task, control task, fault policy) -> request queue;
 *   storage task -> requests, flash;
 *   comm task -> app_storage_pump(), which is the only place the export queue
 *   is drained and REPORT_LOG / REPORT_CRASH / REPORT_STORAGE are serialized.
 * The comm task is the single UART transmitter, so nothing here writes to it.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __APP_STORAGE_H__
#define __APP_STORAGE_H__

//******************************** Includes *********************************//

#include "platform_def.h"
#include "platform_os_queue.h"

#include "service_storage/service_storage.h"
#include "service_proto/service_proto.h"

//******************************** Defines **********************************//

/* Module numbers of REPORT_LOG tag, protocol.md 5.5.6. */
#define APP_STORAGE_TAG_CHASSIS     ((uint8_t)1u)
#define APP_STORAGE_TAG_COMM        ((uint8_t)2u)
#define APP_STORAGE_TAG_ARM         ((uint8_t)3u)
#define APP_STORAGE_TAG_POWER       ((uint8_t)4u)
#define APP_STORAGE_TAG_ESTOP       ((uint8_t)5u)
#define APP_STORAGE_TAG_STORAGE     ((uint8_t)6u)

#define APP_STORAGE_LOG_TEXT_MAX    PROTOCOL_REPORT_LOG_TEXT_MAX
#define APP_STORAGE_CRASH_DATA_MAX  PROTOCOL_REPORT_CRASH_DATA_MAX

/* LOG_CLEAR sources, protocol.md 5.5.8. */
#define APP_STORAGE_CLEAR_RUN       ((uint8_t)0u)
#define APP_STORAGE_CLEAR_FAULT     ((uint8_t)1u)
#define APP_STORAGE_CLEAR_CRASH     ((uint8_t)2u)
#define APP_STORAGE_CLEAR_ALL       ((uint8_t)3u)

//******************************** Declaring ********************************//

typedef enum
{
    APP_STORAGE_OK = 0,
    APP_STORAGE_FAILED,
    APP_STORAGE_BUSY,
    APP_STORAGE_UNAVAILABLE,    /* External flash missing or not mounted. */
    APP_STORAGE_EMPTY           /* Nothing to read: no crash slot, no record. */
} app_storage_result_t;

/*
 * Completion callback. tag and user_data are whatever the request carried, so
 * one callback can serve several requests. Runs in the storage task.
 */
typedef void (*app_storage_done_t)(app_storage_result_t result,
                                   uint16_t tag,
                                   void *user_data);

/* Request kinds, internal to the queue but public so its item size is honest. */
typedef enum
{
    APP_STORAGE_REQ_SAVE = 0,
    APP_STORAGE_REQ_LOG,
    APP_STORAGE_REQ_CLEAR,
    APP_STORAGE_REQ_EXPORT,
    APP_STORAGE_REQ_CRASH_READ
} app_storage_req_kind_t;

typedef struct
{
    uint8_t             kind;       /* app_storage_req_kind_t. */
    uint8_t             arg0;       /* blob / ring / source / slot. */
    uint8_t             arg1;       /* level / start flag. */
    uint8_t             arg2;       /* tag / scope. */
    uint16_t            length;
    uint8_t             staging;    /* staging slot index, or 0xFF. */
    uint8_t             reserved;
    uint32_t            value;      /* from_seq for exports. */
    const uint8_t *     payload;
    app_storage_done_t  done;
    void *              user;
} app_storage_req_t;

/* One record handed to the comm task for transmission. */
typedef struct
{
    uint8_t     source;
    uint8_t     level;
    uint8_t     tag;
    uint8_t     flags;
    uint32_t    ts_s;
    uint32_t    seq;
    uint16_t    code;
    uint8_t     text_len;
    char        text[APP_STORAGE_LOG_TEXT_MAX];
} app_storage_log_export_t;

typedef struct
{
    uint8_t     slot;
    uint8_t     flags;
    uint16_t    offset;
    uint16_t    total;
    uint8_t     data_len;
    uint8_t     data[APP_STORAGE_CRASH_DATA_MAX];
} app_storage_crash_export_t;

typedef enum
{
    APP_STORAGE_EXPORT_LOG = 0,
    APP_STORAGE_EXPORT_CRASH,
    APP_STORAGE_EXPORT_STATUS
} app_storage_export_kind_t;

typedef struct
{
    uint8_t                 kind;   /* app_storage_export_kind_t. */
    union
    {
        app_storage_log_export_t    log;
        app_storage_crash_export_t  crash;
    } u;
} app_storage_export_t;

typedef struct
{
    uint32_t    requests;
    uint32_t    drops;          /* Log appends refused: filtered or no slot. */
    uint32_t    saves;
    uint32_t    save_failures;
    uint32_t    logs;
    uint32_t    exports;
} app_storage_stats_t;

/*
 * Bind the two queues and record whether the service mounted. Called from
 * app_user_init_init() after the queues exist; service_storage_init() itself
 * runs earlier, from the same function.
 */
platform_err_t app_storage_init(platform_os_queue_t requests,
                                platform_os_queue_t exports,
                                platform_bool_t service_ok);

/* Task body; created by the task table in app_user_init.c. */
void app_storage_entry(void *argument);

/*
 * One iteration of that body: produce at most one export record, execute at
 * most one queued request, then return. app_storage_entry() is this in a loop;
 * the L1 test drives the task through it instead of thread scheduling.
 */
void app_storage_step(uint32_t timeout_ms);

platform_bool_t app_storage_available(void);

/* ------------------------- Producers (any task) ------------------------ */

/*
 * Save one blob. The buffer must stay valid until the callback fires (or
 * forever, for a static buffer). Returns FALSE when the flash is unavailable,
 * another save/borrow is outstanding, or the queue is full: in those cases the
 * callback is NOT called.
 */
platform_bool_t app_storage_save_blob(service_storage_blob_t which,
                                      const uint8_t *data,
                                      uint16_t length,
                                      uint8_t tag,
                                      app_storage_done_t done,
                                      void *user_data);

/*
 * Append one log line. Dropped (and counted) when the log level filter rejects
 * it, the queue is full or both staging slots are busy: logging must never
 * slow down whoever is reporting. Returns FALSE when dropped.
 */
platform_bool_t app_storage_append_log(uint8_t level,
                                       uint8_t tag,
                                       const char *text,
                                       uint16_t length);

/*
 * Append one fault event (code only) to the fault ring. The module number is
 * the high byte of the code and the level comes from protocol_fault_severity,
 * so a caller only has to pass the code. Same filter and drop policy as
 * append_log.
 */
platform_bool_t app_storage_append_fault(uint16_t code);

/*
 * LOG_CLEAR. source: APP_STORAGE_CLEAR_RUN / _FAULT / _CRASH / _ALL.
 * ACK semantics: TRUE means accepted, the erase happens in the storage task.
 */
platform_bool_t app_storage_clear(uint8_t source,
                                  app_storage_done_t done,
                                  void *user_data);

/*
 * LOG_EXPORT. start == TRUE begins an export from from_seq (0 = oldest);
 * start == FALSE stops the one in progress. Only one export runs at a time.
 */
platform_bool_t app_storage_export(uint8_t source,
                                   platform_bool_t start,
                                   uint32_t from_seq,
                                   app_storage_done_t done,
                                   void *user_data);

/*
 * CRASH_READ. slot 0xFF means the newest valid slot. mark_read clears the
 * UNREAD bit after the last fragment was produced.
 */
platform_bool_t app_storage_crash_read(uint8_t slot,
                                       platform_bool_t mark_read,
                                       app_storage_done_t done,
                                       void *user_data);

platform_bool_t app_storage_exporting(void);

/* TRUE while an accepted save has not finished. PARAM_SAVE answers BUSY. */
platform_bool_t app_storage_saves_pending(void);

/* ---------------------------- Comm task -------------------------------- */

/* Ask the pump to send one REPORT_STORAGE on its next call. */
void app_storage_request_status(void);

/*
 * Drain export items and serialize them. Call from the comm loop after
 * service_proto_process(), once per iteration. link_alive FALSE for
 * APP_STORAGE_EXPORT_IDLE_MS stops an export in progress.
 */
void app_storage_pump(uint32_t now_ms, platform_bool_t link_alive);

void app_storage_get_stats(app_storage_stats_t *out);

#endif /* __APP_STORAGE_H__ */
