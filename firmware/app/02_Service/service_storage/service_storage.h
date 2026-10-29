/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file service_storage.h
 *
 * @par dependencies
 * - platform_bsp_flash.h
 * - platform_logstore.h
 * - platform_crc.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Persistent storage service: partition map, saved blobs, rings, crash.
 *
 * What this service owns
 * ----------------------
 * Everything that turns the 8 MiB W25Q64 partition map in
 * docs/architecture/flash-layout.md into operations:
 *   - A/B blob records for parameters and calibration (CRC-32, increasing seq),
 *   - the fault ring and the run-log ring (platform_logstore instances),
 *   - the 16 crash slots with the commit-CRC-written-last rule,
 *   - the boot counter and the wear statistics record.
 *
 * What it does not own
 * --------------------
 * The contents of a parameter blob. The application serializes its parameter
 * struct and hands over bytes; this service only guarantees "the bytes that
 * come back are exactly the bytes that were saved, or nothing".
 *
 * Time is injected (now_ms) for the same reason service_chassis injects it:
 * the whole module runs on the PC against a RAM flash fake.
 *
 * Write access is single-threaded (appStorage). Reads of the status struct are
 * cheap and may be called from the reporting path.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __SERVICE_STORAGE_H__
#define __SERVICE_STORAGE_H__

//******************************** Includes *********************************//

#include "platform_bsp_flash.h"
#include "platform_logstore.h"

//******************************** Defines **********************************//

/* Partition map, block 0 is at the bottom of the device. */
#define SERVICE_STORAGE_PART_TABLE_A_ADDRESS    ((uint32_t)0x000000u)
#define SERVICE_STORAGE_PART_TABLE_B_ADDRESS    ((uint32_t)0x001000u)
#define SERVICE_STORAGE_BOOT_CTRL_A_ADDRESS     ((uint32_t)0x002000u)
#define SERVICE_STORAGE_BOOT_CTRL_B_ADDRESS     ((uint32_t)0x003000u)
#define SERVICE_STORAGE_BOOT_COUNT_ADDRESS      ((uint32_t)0x004000u)
#define SERVICE_STORAGE_IMAGE_A_ADDRESS         ((uint32_t)0x010000u)
#define SERVICE_STORAGE_IMAGE_B_ADDRESS         ((uint32_t)0x030000u)
#define SERVICE_STORAGE_PARAM_A_ADDRESS         ((uint32_t)0x050000u)
#define SERVICE_STORAGE_PARAM_B_ADDRESS         ((uint32_t)0x058000u)
#define SERVICE_STORAGE_CALIB_A_ADDRESS         ((uint32_t)0x060000u)
#define SERVICE_STORAGE_CALIB_B_ADDRESS         ((uint32_t)0x068000u)
#define SERVICE_STORAGE_CRASH_ADDRESS           ((uint32_t)0x070000u)
#define SERVICE_STORAGE_FAULT_RING_ADDRESS      ((uint32_t)0x080000u)
#define SERVICE_STORAGE_LOG_RING_ADDRESS        ((uint32_t)0x0A0000u)
#define SERVICE_STORAGE_WEAR_A_ADDRESS          ((uint32_t)0x180000u)
#define SERVICE_STORAGE_WEAR_B_ADDRESS          ((uint32_t)0x181000u)
#define SERVICE_STORAGE_FACTORY_ADDRESS         ((uint32_t)0x7F0000u)

#define SERVICE_STORAGE_FAULT_RING_SECTORS      ((uint32_t)32u)
#define SERVICE_STORAGE_LOG_RING_SECTORS        ((uint32_t)224u)
#define SERVICE_STORAGE_CRASH_SLOTS             ((uint32_t)16u)
#define SERVICE_STORAGE_BOOT_COUNT_SLOTS        ((uint32_t)1024u)

#define SERVICE_STORAGE_RECORD_HEADER_SIZE      ((uint32_t)32u)
#define SERVICE_STORAGE_SECTOR_SIZE             ((uint32_t)4096u)
#define SERVICE_STORAGE_MAX_BLOB_LENGTH                                       \
    (SERVICE_STORAGE_SECTOR_SIZE - SERVICE_STORAGE_RECORD_HEADER_SIZE)
#define SERVICE_STORAGE_CRASH_MAX_LENGTH                                      \
    (SERVICE_STORAGE_SECTOR_SIZE - SERVICE_STORAGE_RECORD_HEADER_SIZE - 4u)

#define SERVICE_STORAGE_LAYOUT_VERSION          ((uint16_t)1u)
#define SERVICE_STORAGE_WRITER_APP              ((uint16_t)1u)

/* Record magics, flash-layout.md section 4. */
#define SERVICE_STORAGE_MAGIC_PARAM             ((uint32_t)0x50415241u)
#define SERVICE_STORAGE_MAGIC_CALIB             ((uint32_t)0x43414C49u)
#define SERVICE_STORAGE_MAGIC_WEAR              ((uint32_t)0x57454152u)
#define SERVICE_STORAGE_MAGIC_CRASH             ((uint32_t)0x43524153u)

/* Record flags, common bit 0 plus region-specific bits. */
#define SERVICE_STORAGE_RECORD_FLAG_VALID       ((uint16_t)0x0001u)
#define SERVICE_STORAGE_RECORD_FLAG_UNREAD      ((uint16_t)0x0002u)

/* storage_flags bits of REPORT_STORAGE (protocol.md 5.5.5). */
#define SERVICE_STORAGE_FLAG_CLEAN              ((uint8_t)0x01u)
#define SERVICE_STORAGE_FLAG_PARAM_VALID        ((uint8_t)0x02u)
#define SERVICE_STORAGE_FLAG_CALIB_VALID        ((uint8_t)0x04u)
#define SERVICE_STORAGE_FLAG_SAVE_FAILED        ((uint8_t)0x08u)
#define SERVICE_STORAGE_FLAG_IMAGE_A_VALID      ((uint8_t)0x10u)
#define SERVICE_STORAGE_FLAG_IMAGE_B_VALID      ((uint8_t)0x20u)
#define SERVICE_STORAGE_FLAG_CRASH_UNREAD       ((uint8_t)0x40u)
#define SERVICE_STORAGE_FLAG_LOG_WRAPPED        ((uint8_t)0x80u)

/* Persist the wear record once this many ring erases are unaccounted for. */
#define SERVICE_STORAGE_WEAR_SAVE_ERASES        ((uint32_t)16u)

//******************************** Declaring ********************************//

typedef enum
{
    SERVICE_STORAGE_BLOB_PARAM = 0,
    SERVICE_STORAGE_BLOB_CALIB = 1,
    SERVICE_STORAGE_BLOB_COUNT = 2
} service_storage_blob_t;

typedef enum
{
    SERVICE_STORAGE_RING_FAULT = 0,
    SERVICE_STORAGE_RING_LOG   = 1,
    SERVICE_STORAGE_RING_COUNT = 2
} service_storage_ring_t;

typedef struct
{
    uint8_t     flags;
    uint8_t     log_level;
    uint16_t    param_seq;
    uint16_t    calib_seq;
    uint16_t    reserved;
    uint32_t    boot_total;
    uint32_t    log_used;
    uint32_t    log_erase;
    uint32_t    last_save_s;
} service_storage_status_t;

/*
 * Mount the flash, restore the wear statistics, count the boot and probe the
 * saved blobs. Does not load blob contents; the application calls
 * service_storage_load_blob() for that and decides its own defaults.
 */
platform_err_t service_storage_init(uint32_t now_ms);
platform_err_t service_storage_deinit(void);

/*
 * Save a blob. Returns only after erase, program and read-back verification;
 * the previous copy stays valid until the new one verifies.
 */
platform_err_t service_storage_save_blob(service_storage_blob_t which,
                                         const uint8_t *data,
                                         uint16_t length,
                                         uint32_t now_ms);

platform_err_t service_storage_load_blob(service_storage_blob_t which,
                                         uint8_t *buffer,
                                         uint16_t capacity,
                                         uint16_t *out_length,
                                         uint32_t *out_seq);

platform_bool_t service_storage_blob_valid(service_storage_blob_t which);
uint32_t service_storage_blob_seq(service_storage_blob_t which);

/* Dirty flag of REPORT_STORAGE bit 0, maintained by the application. */
void service_storage_set_clean(platform_bool_t clean);
void service_storage_set_log_level(uint8_t level);

platform_err_t service_storage_append(service_storage_ring_t ring,
                                      uint8_t level,
                                      uint8_t tag,
                                      const char *text,
                                      uint16_t length,
                                      uint32_t now_ms);

platform_err_t service_storage_iter_begin(service_storage_ring_t ring,
                                          platform_logstore_iter_t *iter,
                                          uint32_t from_seq);

/* Erase-logically clear a ring (header invalidation, see platform_logstore). */
platform_err_t service_storage_ring_clear(service_storage_ring_t ring,
                                          uint32_t now_ms);

uint32_t service_storage_ring_used(service_storage_ring_t ring);
uint32_t service_storage_ring_erase_total(service_storage_ring_t ring);

/*
 * Crash scene. write finds the oldest slot, erases it, writes header, payload
 * and finally the commit CRC in the last four bytes of the sector.
 */
platform_err_t service_storage_crash_write(const uint8_t *data,
                                           uint16_t length,
                                           uint32_t now_ms);
platform_err_t service_storage_crash_info(uint8_t slot,
                                          uint16_t *out_length,
                                          uint8_t *out_flags);
platform_err_t service_storage_crash_read(uint8_t slot,
                                          uint16_t offset,
                                          uint8_t *buffer,
                                          uint16_t capacity,
                                          uint16_t *out_length);
platform_err_t service_storage_crash_mark_read(uint8_t slot);

/* LOG_CLEAR for the crash region: drop the VALID bit of every valid slot. */
platform_err_t service_storage_crash_clear(void);

platform_bool_t service_storage_crash_unread(void);

/*
 * Cache-only queries, no bus access: the protocol task answers CRASH_READ
 * from them instead of paying for a flash read that can sit behind an erase.
 */
platform_bool_t service_storage_crash_slot_valid(uint8_t slot);
platform_bool_t service_storage_crash_latest(uint8_t *out_slot);

void service_storage_get_status(service_storage_status_t *status);

/* Write the wear record now (called after a save and on a normal shutdown). */
platform_err_t service_storage_flush_wear(uint32_t now_ms);

#endif /* __SERVICE_STORAGE_H__ */
