/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_logstore.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 * - platform_bsp_flash.h
 * - platform_crc.h
 * - platform_wearlevel.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide an append-only, CRC-checked record ring over raw NOR flash.
 *
 * What this module owns
 * ---------------------
 * The mechanics that every persistent ring shares: sector headers, record
 * headers, page-boundary-safe writes, the mount scan that finds where writing
 * stopped, and the oldest-first iterator. The fault ring and the run-log ring
 * are both an instance of this store, so the code exists once.
 *
 * What this module does not own
 * -----------------------------
 * Record semantics. A log line and a fault event have different payloads and
 * different consumers; both hand the middleware an opaque byte string.
 *
 * On-flash format (all multi-byte fields little-endian)
 * -----------------------------------------------------
 * Sector header (16 B, written once when the sector is opened):
 *   +0  u16 magic 0x5EC7
 *   +2  u16 state      0x0000 = open (a valid header); erased reads 0xFFFF
 *   +4  u32 sector_seq logical ring position, 1, 2, 3 ... never reset
 *   +8  u16 first_record_offset (16 today, room for a future preamble)
 *   +10 u16 crc16 over +0..+9
 *   +12 u32 0xFFFFFFFF reserved
 *
 * Record header (16 B) followed by payload padded to a 4-byte boundary:
 *   +0  u16 magic 0x5BF5
 *   +2  u8  level
 *   +3  u8  tag
 *   +4  u32 ts
 *   +8  u16 len       0..PLATFORM_LOGSTORE_MAX_PAYLOAD
 *   +10 u16 crc16 over +0..+9, +12..+15 and the payload
 *   +12 u32 seq       globally increasing record sequence
 *
 * A record never spans a sector. A torn write leaves the magic or the CRC
 * wrong; mounting stops at the first bad slot of the newest sector and seals
 * it, so nothing written later can be hidden behind the torn record.
 *
 * Wear levelling
 * --------------
 * Opening a sector is the only erase. The candidate set is the sectors that
 * may be discarded (free sectors; if there are none, the oldest by sector_seq)
 * and the helper picks the coldest candidate inside that set. In a healthy
 * full ring the set has exactly one member, so the policy costs nothing and
 * the ring behaves sequentially; after a clear, or on first use, the coldest
 * free sector is chosen and wear converges.
 *
 * Concurrency
 * -----------
 * Not thread safe. One owner task per store (appStorage).
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_LOGSTORE_H__
#define __PLATFORM_LOGSTORE_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"
#include "platform_bsp_flash.h"
#include "platform_crc.h"
#include "platform_wearlevel.h"

//******************************** Defines **********************************//

#define PLATFORM_LOGSTORE_MAX_SECTORS       ((uint32_t)256u)
#define PLATFORM_LOGSTORE_MAX_PAYLOAD       ((uint16_t)240u)
#define PLATFORM_LOGSTORE_SECTOR_HEADER_SIZE ((uint32_t)16u)
#define PLATFORM_LOGSTORE_RECORD_HEADER_SIZE ((uint32_t)16u)
#define PLATFORM_LOGSTORE_RECORD_ALIGN      ((uint32_t)4u)

#define PLATFORM_LOGSTORE_SECTOR_MAGIC      ((uint16_t)0x5EC7u)
#define PLATFORM_LOGSTORE_RECORD_MAGIC      ((uint16_t)0x5BF5u)
#define PLATFORM_LOGSTORE_SECTOR_OPEN       ((uint16_t)0x0000u)

#define PLATFORM_LOGSTORE_NONE              ((uint32_t)0xFFFFFFFFu)

//******************************** Declaring ********************************//

typedef struct
{
    uint32_t            base_address;
    uint32_t            sector_size;
    uint32_t            page_size;
    uint32_t            sector_count;
    uint16_t *          erase_counts;   /* sector_count entries, caller-owned */
    uint32_t *          sector_seq;     /* sector_count entries, caller-owned */
    platform_wearlevel_t wearlevel;

    uint32_t            active_sector;  /* NONE until the first append */
    uint32_t            active_offset;  /* next free byte, or sector_size */
    uint32_t            last_sector;    /* most recently opened sector */
    uint32_t            top_sector_seq; /* highest sector_seq in flash */
    uint32_t            next_seq;       /* record seq to hand out next */
    uint32_t            allocated_sectors;
    uint32_t            used_bytes;
    uint32_t            written_bytes;  /* cumulative, survives clear() */
    uint32_t            wrap_count;
    platform_bool_t     mounted;
} platform_logstore_t;

typedef struct
{
    uint8_t     level;
    uint8_t     tag;
    uint16_t    len;
    uint32_t    ts;
    uint32_t    seq;
    uint8_t     payload[PLATFORM_LOGSTORE_MAX_PAYLOAD];
} platform_logstore_entry_t;

typedef struct
{
    platform_logstore_t *   store;
    uint32_t                sector;         /* sector index being read */
    uint32_t                offset;         /* next byte to inspect */
    uint32_t                sector_seq;     /* seq of the sector being read */
    uint32_t                from_seq;       /* skip records older than this */
    platform_bool_t         finished;
} platform_logstore_iter_t;

/*
 * Bind the store to a flash range and scan the existing sector headers.
 * Erase counters and sector_seq are zeroed first; the caller restores the
 * counters from the persistent wear statistics afterwards.
 */
platform_err_t platform_logstore_init(platform_logstore_t *store,
                                      uint32_t base_address,
                                      uint32_t sector_count,
                                      uint16_t *erase_counts,
                                      uint32_t *sector_seq);

/*
 * Append one record. Returns PLATFORM_ERR_PARAM when len exceeds
 * PLATFORM_LOGSTORE_MAX_PAYLOAD. On success *out_seq (optional) receives the
 * sequence number that was stored.
 */
platform_err_t platform_logstore_append(platform_logstore_t *store,
                                        uint8_t level,
                                        uint8_t tag,
                                        uint32_t ts,
                                        const uint8_t *payload,
                                        uint16_t len,
                                        uint32_t *out_seq);

/*
 * Invalidate every sector header. This is a 2-byte program per sector rather
 * than 224 sector erases: the ring becomes logically empty in well under a
 * second, sector erases happen lazily when a sector is reused, the sequence
 * counters keep increasing, and a power cut halfway leaves a partially
 * cleared but still consistent ring.
 */
platform_err_t platform_logstore_clear(platform_logstore_t *store);

/* Iterate oldest record first, skipping records with seq < from_seq. */
platform_err_t platform_logstore_iter_begin(platform_logstore_t *store,
                                            platform_logstore_iter_t *iter,
                                            uint32_t from_seq);

/*
 * Copy the next record into entry. Returns PLATFORM_ERR_OK for a record and
 * PLATFORM_ERR_NO_RESOURCE when the iteration is over; any other value is an
 * I/O error.
 */
platform_err_t platform_logstore_iter_next(platform_logstore_iter_t *iter,
                                           platform_logstore_entry_t *entry);

uint32_t platform_logstore_used_bytes(const platform_logstore_t *store);
uint32_t platform_logstore_written_bytes(const platform_logstore_t *store);
uint32_t platform_logstore_wrap_count(const platform_logstore_t *store);
uint32_t platform_logstore_erase_total(const platform_logstore_t *store);
uint32_t platform_logstore_allocated_sectors(
                                        const platform_logstore_t *store);

#endif /* __PLATFORM_LOGSTORE_H__ */
