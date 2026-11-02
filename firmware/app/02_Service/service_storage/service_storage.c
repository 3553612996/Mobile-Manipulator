/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file service_storage.c
 *
 * @par dependencies
 * - service_storage.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement the persistent storage service.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "service_storage.h"
#include "platform_crc.h"

//******************************** Defines **********************************//

#define WEAR_LENGTH             ((uint16_t)488u)
#define WEAR_OFF_ERASE_COUNT    ((uint32_t)4u)
#define WEAR_OFF_LOG_BYTES      ((uint32_t)28u)
#define WEAR_OFF_LOG_WRAP       ((uint32_t)32u)
#define WEAR_OFF_LAST_TS        ((uint32_t)36u)
#define WEAR_OFF_SECTOR_ERASE   ((uint32_t)40u)

#define ERASE_PARAM             ((uint32_t)0u)
#define ERASE_CALIB             ((uint32_t)1u)
#define ERASE_IMAGE             ((uint32_t)2u)
#define ERASE_FAULT             ((uint32_t)3u)
#define ERASE_LOG               ((uint32_t)4u)
#define ERASE_CRASH             ((uint32_t)5u)
#define ERASE_KINDS             ((uint32_t)6u)

#define SECTOR_ERASE_SLOTS      ((uint32_t)224u)

#define SCAN_CHUNK              ((uint32_t)32u)
#define BOOT_COUNT_CHUNK        ((uint32_t)64u)

//******************************** Variables ********************************//

typedef struct
{
    uint32_t            address_a;
    uint32_t            address_b;
    uint32_t            magic;
    uint32_t            seq;
    int8_t              active;     /* -1 none, 0 copy A, 1 copy B */
    platform_bool_t     valid;
} s_ab_t;

static s_ab_t s_blobs[SERVICE_STORAGE_BLOB_COUNT] =
{
    { SERVICE_STORAGE_PARAM_A_ADDRESS, SERVICE_STORAGE_PARAM_B_ADDRESS,
      SERVICE_STORAGE_MAGIC_PARAM, 0u, -1, PLATFORM_FALSE },
    { SERVICE_STORAGE_CALIB_A_ADDRESS, SERVICE_STORAGE_CALIB_B_ADDRESS,
      SERVICE_STORAGE_MAGIC_CALIB, 0u, -1, PLATFORM_FALSE }
};

static s_ab_t s_wear =
{
    SERVICE_STORAGE_WEAR_A_ADDRESS, SERVICE_STORAGE_WEAR_B_ADDRESS,
    SERVICE_STORAGE_MAGIC_WEAR, 0u, -1, PLATFORM_FALSE
};

static platform_logstore_t s_rings[SERVICE_STORAGE_RING_COUNT];

static uint16_t s_fault_erase_counts[SERVICE_STORAGE_FAULT_RING_SECTORS];
static uint32_t s_fault_sector_seq[SERVICE_STORAGE_FAULT_RING_SECTORS];
static uint16_t s_log_erase_counts[SERVICE_STORAGE_LOG_RING_SECTORS];
static uint32_t s_log_sector_seq[SERVICE_STORAGE_LOG_RING_SECTORS];

static uint8_t  s_wear_payload[WEAR_LENGTH];

static uint32_t s_page_size;
static uint32_t s_boot_epoch;
static uint32_t s_boot_total;
static uint32_t s_param_erases;
static uint32_t s_calib_erases;
static uint32_t s_crash_erases;
static uint32_t s_fault_erases_base;
static uint32_t s_log_bytes_base;
static uint32_t s_last_save_s;
static uint32_t s_last_wear_log_erase;
static uint32_t s_last_wear_fault_erase;

static uint8_t  s_crash_state[SERVICE_STORAGE_CRASH_SLOTS];
static uint32_t s_crash_seq[SERVICE_STORAGE_CRASH_SLOTS];
static platform_bool_t s_crash_unread;

static uint8_t  s_log_level = 1u;
static platform_bool_t s_clean = PLATFORM_TRUE;
static platform_bool_t s_save_failed = PLATFORM_FALSE;
static platform_bool_t s_initialized = PLATFORM_FALSE;

//******************************** Functions ********************************//

static void s_put16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFFu);
    p[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static void s_put32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value & 0xFFu);
    p[1] = (uint8_t)((value >> 8) & 0xFFu);
    p[2] = (uint8_t)((value >> 16) & 0xFFu);
    p[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static uint16_t s_get16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t s_get32(const uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static uint32_t s_time_s(uint32_t now_ms)
{
    return now_ms / 1000u;
}

/* Split at page boundaries; the driver rejects crossing writes. */
static platform_err_t s_write_at(uint32_t address,
                                 const uint8_t *data,
                                 uint32_t length)
{
    uint32_t chunk;
    uint32_t page_offset;
    platform_err_t err;

    while (length > 0u)
    {
        page_offset = address % s_page_size;
        chunk = s_page_size - page_offset;
        if (chunk > length)
        {
            chunk = length;
        }

        err = platform_flash_write(address, data, chunk);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }

        address += chunk;
        data += chunk;
        length -= chunk;
    }

    return PLATFORM_ERR_OK;
}

static void s_build_header(uint8_t *header,
                           uint32_t magic,
                           uint16_t flags,
                           uint32_t seq,
                           uint32_t length,
                           uint32_t ts)
{
    s_put32(header + 0u, magic);
    s_put16(header + 4u, SERVICE_STORAGE_LAYOUT_VERSION);
    s_put16(header + 6u, flags);
    s_put32(header + 8u, seq);
    s_put32(header + 12u, length);
    s_put32(header + 16u, 0u);      /* CRC-32, filled after the payload hash */
    s_put32(header + 20u, ts);
    s_put16(header + 24u, SERVICE_STORAGE_WRITER_APP);
    s_put16(header + 26u, 0xFFFFu);
    s_put32(header + 28u, 0xFFFFFFFFu);
}

/*
 * CRC-32 covers the header except its own field, then the payload. Bytes
 * 16..19 are skipped so the value can be patched in after hashing; this is the
 * one place the implementation refines flash-layout.md, which only says
 * "covers magic..payload".
 */
static uint32_t s_crc_start(const uint8_t *header)
{
    uint32_t crc;

    crc = platform_crc32_update(PLATFORM_CRC32_INIT_VALUE, header, 16u);
    crc = platform_crc32_update(crc, header + 20u, 12u);

    return crc;
}

static uint32_t s_crc_finish(uint32_t crc)
{
    return crc ^ PLATFORM_CRC32_XOROUT_VALUE;
}

/*
 * Crash records leave the flags word (offset 6..7) out of the CRC: marking a
 * scene read clears one bit in place, and NOR flash can only clear bits, so
 * rewriting the CRC after the fact is impossible. Everything else, including
 * the length and the sequence number, stays covered.
 */
static uint32_t s_crash_crc_start(const uint8_t *header)
{
    uint32_t crc;

    crc = platform_crc32_update(PLATFORM_CRC32_INIT_VALUE, header, 6u);
    crc = platform_crc32_update(crc, header + 8u, 8u);
    crc = platform_crc32_update(crc, header + 20u, 12u);

    return crc;
}

/*
 * Read a record and verify it. buffer may be NULL, in which case the payload
 * is streamed through a scratch buffer. Returns PLATFORM_ERR_FAIL when the
 * magic/length/CRC do not agree, so callers can fall through to the other
 * copy.
 */
static platform_err_t s_verify_record(uint32_t address,
                                      uint32_t magic,
                                      uint32_t max_length,
                                      platform_bool_t exclude_flags,
                                      uint8_t *buffer,
                                      uint16_t capacity,
                                      uint32_t *out_seq,
                                      uint16_t *out_length,
                                      uint16_t *out_flags)
{
    uint8_t header[SERVICE_STORAGE_RECORD_HEADER_SIZE];
    uint8_t scratch[SCAN_CHUNK];
    uint8_t *target;
    uint32_t length;
    uint32_t done;
    uint32_t chunk;
    uint32_t crc;
    platform_err_t err;

    err = platform_flash_read(address, header,
                              SERVICE_STORAGE_RECORD_HEADER_SIZE);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    if (s_get32(header + 0u) != magic)
    {
        return PLATFORM_ERR_FAIL;
    }
    if (s_get16(header + 4u) != SERVICE_STORAGE_LAYOUT_VERSION)
    {
        return PLATFORM_ERR_FAIL;
    }
    if ((s_get16(header + 6u) & SERVICE_STORAGE_RECORD_FLAG_VALID) == 0u)
    {
        return PLATFORM_ERR_FAIL;
    }

    length = s_get32(header + 12u);
    if (length > max_length)
    {
        return PLATFORM_ERR_FAIL;
    }
    if ((buffer != NULL) && (length > (uint32_t)capacity))
    {
        return PLATFORM_ERR_PARAM;
    }

    crc = (exclude_flags == PLATFORM_TRUE) ? s_crash_crc_start(header)
                                           : s_crc_start(header);
    done = 0u;
    while (done < length)
    {
        chunk = length - done;
        if (chunk > SCAN_CHUNK)
        {
            chunk = SCAN_CHUNK;
        }
        target = (buffer != NULL) ? (buffer + done) : scratch;

        err = platform_flash_read(address +
                                      SERVICE_STORAGE_RECORD_HEADER_SIZE +
                                      done,
                                  target,
                                  chunk);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }
        crc = platform_crc32_update(crc, target, chunk);
        done += chunk;
    }

    if (s_crc_finish(crc) != s_get32(header + 16u))
    {
        return PLATFORM_ERR_FAIL;
    }

    if (out_seq != NULL)
    {
        *out_seq = s_get32(header + 8u);
    }
    if (out_length != NULL)
    {
        *out_length = (uint16_t)length;
    }
    if (out_flags != NULL)
    {
        *out_flags = s_get16(header + 6u);
    }

    return PLATFORM_ERR_OK;
}

/* Read only the header fields needed to order the two copies. */
static platform_bool_t s_header_plausible(uint32_t address,
                                          uint32_t magic,
                                          uint32_t max_length,
                                          uint32_t *out_seq,
                                          uint32_t *out_length)
{
    uint8_t header[SERVICE_STORAGE_RECORD_HEADER_SIZE];

    if (PLATFORM_IS_ERR(platform_flash_read(
            address, header, SERVICE_STORAGE_RECORD_HEADER_SIZE)))
    {
        return PLATFORM_FALSE;
    }

    if (s_get32(header + 0u) != magic)
    {
        return PLATFORM_FALSE;
    }
    if (s_get16(header + 4u) != SERVICE_STORAGE_LAYOUT_VERSION)
    {
        return PLATFORM_FALSE;
    }
    if ((s_get16(header + 6u) & SERVICE_STORAGE_RECORD_FLAG_VALID) == 0u)
    {
        return PLATFORM_FALSE;
    }
    if (s_get32(header + 12u) > max_length)
    {
        return PLATFORM_FALSE;
    }

    *out_seq = s_get32(header + 8u);
    *out_length = s_get32(header + 12u);

    return PLATFORM_TRUE;
}

/*
 * Try both copies of an A/B region, newest first, and stop at the first one
 * whose CRC verifies. *found remains FALSE when neither copy is usable.
 */
static platform_err_t s_ab_load(s_ab_t *region,
                                uint32_t max_length,
                                uint8_t *buffer,
                                uint16_t capacity,
                                uint32_t *out_seq,
                                uint16_t *out_length,
                                platform_bool_t *found)
{
    platform_bool_t ok_a;
    platform_bool_t ok_b;
    uint32_t seq_a;
    uint32_t seq_b;
    uint32_t len_a;
    uint32_t len_b;
    uint32_t targets[2];
    uint32_t count;
    uint32_t i;
    uint32_t address;
    uint32_t seq;
    uint16_t length;
    platform_err_t err;

    *found = PLATFORM_FALSE;

    ok_a = s_header_plausible(region->address_a, region->magic, max_length,
                              &seq_a, &len_a);
    ok_b = s_header_plausible(region->address_b, region->magic, max_length,
                              &seq_b, &len_b);
    (void)len_a;
    (void)len_b;

    if ((ok_a == PLATFORM_TRUE) && (ok_b == PLATFORM_TRUE))
    {
        /* Newest first; a seq tie is impossible in normal use, prefer A. */
        if (seq_b > seq_a)
        {
            targets[0] = 1u;
            targets[1] = 0u;
        }
        else
        {
            targets[0] = 0u;
            targets[1] = 1u;
        }
        count = 2u;
    }
    else if (ok_a == PLATFORM_TRUE)
    {
        targets[0] = 0u;
        count = 1u;
    }
    else if (ok_b == PLATFORM_TRUE)
    {
        targets[0] = 1u;
        count = 1u;
    }
    else
    {
        region->valid = PLATFORM_FALSE;
        return PLATFORM_ERR_OK;
    }

    for (i = 0u; i < count; i++)
    {
        address = (targets[i] == 0u) ? region->address_a : region->address_b;
        err = s_verify_record(address, region->magic, max_length,
                              PLATFORM_FALSE, buffer, capacity, &seq, &length,
                              NULL);
        if (PLATFORM_IS_ERR(err))
        {
            if (err == PLATFORM_ERR_PARAM)
            {
                return err;     /* caller's buffer is too small */
            }
            continue;
        }

        region->active = (int8_t)targets[i];
        region->seq = seq;
        region->valid = PLATFORM_TRUE;
        *found = PLATFORM_TRUE;
        if (out_seq != NULL)
        {
            *out_seq = seq;
        }
        if (out_length != NULL)
        {
            *out_length = length;
        }

        return PLATFORM_ERR_OK;
    }

    region->valid = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

static platform_err_t s_ab_store(s_ab_t *region,
                                 const uint8_t *payload,
                                 uint16_t length,
                                 uint32_t ts)
{
    uint8_t header[SERVICE_STORAGE_RECORD_HEADER_SIZE];
    uint32_t target;
    uint32_t address;
    uint32_t new_seq;
    uint32_t crc;
    platform_err_t err;

    target = (region->active == 0) ? 1u : 0u;
    address = (target == 0u) ? region->address_a : region->address_b;
    new_seq = region->seq + 1u;
    if (new_seq == 0u)
    {
        new_seq = 1u;
    }

    s_build_header(header, region->magic, SERVICE_STORAGE_RECORD_FLAG_VALID,
                   new_seq, length, ts);
    crc = s_crc_start(header);
    crc = platform_crc32_update(crc, payload, length);
    s_put32(header + 16u, s_crc_finish(crc));

    err = platform_flash_erase(address, SERVICE_STORAGE_SECTOR_SIZE);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = s_write_at(address, header, SERVICE_STORAGE_RECORD_HEADER_SIZE);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    if (length > 0u)
    {
        err = s_write_at(address + SERVICE_STORAGE_RECORD_HEADER_SIZE,
                         payload,
                         length);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }
    }

    /* Read back before the previous copy loses its "active" status. */
    err = s_verify_record(address, region->magic, length, PLATFORM_FALSE,
                          NULL, 0u, NULL, NULL, NULL);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    region->active = (int8_t)target;
    region->seq = new_seq;
    region->valid = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

//******************************** Wear statistics **************************//

static void s_apply_wear(const uint8_t *payload, uint16_t length)
{
    uint32_t i;
    uint32_t count;

    if (length < WEAR_OFF_SECTOR_ERASE)
    {
        return;
    }

    s_boot_epoch = s_get32(payload + 0u);
    s_param_erases = s_get32(payload + WEAR_OFF_ERASE_COUNT + 4u * ERASE_PARAM);
    s_calib_erases = s_get32(payload + WEAR_OFF_ERASE_COUNT + 4u * ERASE_CALIB);
    s_fault_erases_base = s_get32(payload + WEAR_OFF_ERASE_COUNT +
                                 4u * ERASE_FAULT);
    s_crash_erases = s_get32(payload + WEAR_OFF_ERASE_COUNT + 4u * ERASE_CRASH);
    s_log_bytes_base = s_get32(payload + WEAR_OFF_LOG_BYTES);
    /* The wear record is written right after a successful save, so its
     * timestamp doubles as "when did the last save happen". */
    s_last_save_s = s_get32(payload + WEAR_OFF_LAST_TS);

    count = (uint32_t)(length - WEAR_OFF_SECTOR_ERASE) / 2u;
    if (count > SECTOR_ERASE_SLOTS)
    {
        count = SECTOR_ERASE_SLOTS;
    }
    for (i = 0u; i < count; i++)
    {
        platform_wearlevel_set_count(&s_rings[SERVICE_STORAGE_RING_LOG].wearlevel,
                                     i,
                                     s_get16(payload + WEAR_OFF_SECTOR_ERASE +
                                             2u * i));
    }
}

static void s_build_wear(uint32_t now_ms)
{
    uint32_t i;
    uint32_t log_erase_total;

    s_put32(s_wear_payload + 0u, s_boot_epoch);
    s_put32(s_wear_payload + WEAR_OFF_ERASE_COUNT + 4u * ERASE_PARAM,
            s_param_erases);
    s_put32(s_wear_payload + WEAR_OFF_ERASE_COUNT + 4u * ERASE_CALIB,
            s_calib_erases);
    s_put32(s_wear_payload + WEAR_OFF_ERASE_COUNT + 4u * ERASE_IMAGE, 0u);

    log_erase_total = platform_logstore_erase_total(
                                &s_rings[SERVICE_STORAGE_RING_LOG]);
    s_put32(s_wear_payload + WEAR_OFF_ERASE_COUNT + 4u * ERASE_FAULT,
            s_fault_erases_base + platform_logstore_erase_total(
                                      &s_rings[SERVICE_STORAGE_RING_FAULT]));
    s_put32(s_wear_payload + WEAR_OFF_ERASE_COUNT + 4u * ERASE_LOG,
            log_erase_total);
    s_put32(s_wear_payload + WEAR_OFF_ERASE_COUNT + 4u * ERASE_CRASH,
            s_crash_erases);

    s_put32(s_wear_payload + WEAR_OFF_LOG_BYTES,
            s_log_bytes_base + platform_logstore_written_bytes(
                                   &s_rings[SERVICE_STORAGE_RING_LOG]));
    s_put32(s_wear_payload + WEAR_OFF_LOG_WRAP,
            platform_logstore_wrap_count(&s_rings[SERVICE_STORAGE_RING_LOG]));
    s_put32(s_wear_payload + WEAR_OFF_LAST_TS, s_time_s(now_ms));

    for (i = 0u; i < SECTOR_ERASE_SLOTS; i++)
    {
        s_put16(s_wear_payload + WEAR_OFF_SECTOR_ERASE + 2u * i,
                (uint16_t)platform_wearlevel_get_count(
                    &s_rings[SERVICE_STORAGE_RING_LOG].wearlevel, i));
    }
}

static platform_err_t s_wear_store(uint32_t now_ms)
{
    platform_err_t err;

    s_build_wear(now_ms);
    err = s_ab_store(&s_wear, s_wear_payload, WEAR_LENGTH, s_time_s(now_ms));
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    s_last_wear_log_erase = platform_logstore_erase_total(
                                &s_rings[SERVICE_STORAGE_RING_LOG]);
    s_last_wear_fault_erase = platform_logstore_erase_total(
                                &s_rings[SERVICE_STORAGE_RING_FAULT]);

    return PLATFORM_ERR_OK;
}

/*
 * The wear record must not become a wear hotspot itself: it is written after a
 * blob save, after a crash dump, and when at least
 * SERVICE_STORAGE_WEAR_SAVE_ERASES ring erases are unaccounted for.
 */
static void s_wear_maybe_store(uint32_t now_ms)
{
    uint32_t log_erase;
    uint32_t fault_erase;

    log_erase = platform_logstore_erase_total(&s_rings[SERVICE_STORAGE_RING_LOG]);
    fault_erase = platform_logstore_erase_total(
                                &s_rings[SERVICE_STORAGE_RING_FAULT]);

    if (((log_erase - s_last_wear_log_erase) >=
         SERVICE_STORAGE_WEAR_SAVE_ERASES) ||
        ((fault_erase - s_last_wear_fault_erase) >=
         SERVICE_STORAGE_WEAR_SAVE_ERASES))
    {
        (void)s_wear_store(now_ms);
    }
}

//******************************** Boot counter *****************************//

static platform_err_t s_boot_count_update(void)
{
    uint8_t chunk[BOOT_COUNT_CHUNK];
    /* flash-layout.md 5.1: a consumed slot holds 0x00000001, erased is 0xFF. */
    uint8_t used[4] = { 1u, 0u, 0u, 0u };
    uint32_t base;
    uint32_t offset;
    uint32_t slot;
    uint32_t i;
    uint32_t value;
    platform_err_t err;

    slot = SERVICE_STORAGE_BOOT_COUNT_SLOTS;

    for (offset = 0u;
         offset < SERVICE_STORAGE_SECTOR_SIZE;
         offset += BOOT_COUNT_CHUNK)
    {
        base = SERVICE_STORAGE_BOOT_COUNT_ADDRESS + offset;
        err = platform_flash_read(base, chunk, BOOT_COUNT_CHUNK);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }

        for (i = 0u; i < BOOT_COUNT_CHUNK; i += 4u)
        {
            value = s_get32(&chunk[i]);
            if (value == 0xFFFFFFFFu)
            {
                slot = (offset + i) / 4u;
                break;
            }
        }
        if (slot != SERVICE_STORAGE_BOOT_COUNT_SLOTS)
        {
            break;
        }
    }

    if (slot >= SERVICE_STORAGE_BOOT_COUNT_SLOTS)
    {
        err = platform_flash_erase(SERVICE_STORAGE_BOOT_COUNT_ADDRESS,
                                   SERVICE_STORAGE_SECTOR_SIZE);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }
        s_boot_epoch++;
        slot = 0u;
    }

    err = s_write_at(SERVICE_STORAGE_BOOT_COUNT_ADDRESS + slot * 4u,
                     used,
                     4u);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    s_boot_total = (s_boot_epoch * SERVICE_STORAGE_BOOT_COUNT_SLOTS) +
                   slot + 1u;

    return PLATFORM_ERR_OK;
}

//******************************** Crash slots ******************************//

static uint32_t s_crash_slot_address(uint32_t slot)
{
    return SERVICE_STORAGE_CRASH_ADDRESS + slot * SERVICE_STORAGE_SECTOR_SIZE;
}

/*
 * A crash record is valid only when the commit CRC in the last four bytes of
 * the sector equals the CRC stored in the header (written after the payload).
 */
static platform_err_t s_crash_validate(uint8_t slot,
                                       uint16_t *out_length,
                                       uint16_t *out_flags)
{
    uint8_t header[SERVICE_STORAGE_RECORD_HEADER_SIZE];
    uint8_t tail[4];
    uint32_t address;
    platform_err_t err;

    if (slot >= SERVICE_STORAGE_CRASH_SLOTS)
    {
        return PLATFORM_ERR_PARAM;
    }
    if (s_crash_state[slot] == 2u)
    {
        return PLATFORM_ERR_NO_RESOURCE;
    }

    address = s_crash_slot_address(slot);
    err = platform_flash_read(address, header,
                              SERVICE_STORAGE_RECORD_HEADER_SIZE);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    if ((s_get32(header + 0u) != SERVICE_STORAGE_MAGIC_CRASH) ||
        (s_get16(header + 4u) != SERVICE_STORAGE_LAYOUT_VERSION) ||
        ((s_get16(header + 6u) & SERVICE_STORAGE_RECORD_FLAG_VALID) == 0u) ||
        (s_get32(header + 12u) > SERVICE_STORAGE_CRASH_MAX_LENGTH))
    {
        s_crash_state[slot] = 2u;
        return PLATFORM_ERR_NO_RESOURCE;
    }

    err = platform_flash_read(address + SERVICE_STORAGE_SECTOR_SIZE - 4u,
                              tail, 4u);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }
    if (s_get32(tail) != s_get32(header + 16u))
    {
        s_crash_state[slot] = 2u;
        return PLATFORM_ERR_NO_RESOURCE;
    }

    if (s_crash_state[slot] != 1u)
    {
        err = s_verify_record(address, SERVICE_STORAGE_MAGIC_CRASH,
                              SERVICE_STORAGE_CRASH_MAX_LENGTH, PLATFORM_TRUE,
                              NULL, 0u, NULL, NULL, NULL);
        if (PLATFORM_IS_ERR(err))
        {
            s_crash_state[slot] = 2u;
            return PLATFORM_ERR_NO_RESOURCE;
        }
        s_crash_state[slot] = 1u;
        s_crash_seq[slot] = s_get32(header + 8u);
    }

    if (out_length != NULL)
    {
        *out_length = (uint16_t)s_get32(header + 12u);
    }
    if (out_flags != NULL)
    {
        *out_flags = (uint8_t)(s_get16(header + 6u) & 0xFFu);
    }

    return PLATFORM_ERR_OK;
}

static void s_crash_scan(void)
{
    uint32_t slot;
    uint16_t flags;

    s_crash_unread = PLATFORM_FALSE;

    for (slot = 0u; slot < SERVICE_STORAGE_CRASH_SLOTS; slot++)
    {
        s_crash_state[slot] = 0u;
    }

    for (slot = 0u; slot < SERVICE_STORAGE_CRASH_SLOTS; slot++)
    {
        if (PLATFORM_IS_OK(s_crash_validate((uint8_t)slot, NULL, &flags)))
        {
            if ((flags & SERVICE_STORAGE_RECORD_FLAG_UNREAD) != 0u)
            {
                s_crash_unread = PLATFORM_TRUE;
            }
        }
    }
}

//******************************** Init *************************************//

/*
 * Every RAM counter is rebuilt from flash at init. Resetting them first means a
 * board that was re-flashed, or a wear record that vanished, cannot inherit
 * numbers from the previous mount.
 */
static void s_reset_state(void)
{
    uint32_t i;

    s_boot_epoch = 0u;
    s_boot_total = 0u;
    s_param_erases = 0u;
    s_calib_erases = 0u;
    s_crash_erases = 0u;
    s_fault_erases_base = 0u;
    s_log_bytes_base = 0u;
    s_last_save_s = 0u;
    s_last_wear_log_erase = 0u;
    s_last_wear_fault_erase = 0u;
    s_crash_unread = PLATFORM_FALSE;
    s_log_level = 1u;
    s_clean = PLATFORM_TRUE;
    s_save_failed = PLATFORM_FALSE;

    for (i = 0u; i < SERVICE_STORAGE_CRASH_SLOTS; i++)
    {
        s_crash_state[i] = 0u;
        s_crash_seq[i] = 0u;
    }

    for (i = 0u; i < SERVICE_STORAGE_BLOB_COUNT; i++)
    {
        s_blobs[i].seq = 0u;
        s_blobs[i].active = -1;
        s_blobs[i].valid = PLATFORM_FALSE;
    }

    s_wear.seq = 0u;
    s_wear.active = -1;
    s_wear.valid = PLATFORM_FALSE;
}

platform_err_t service_storage_init(uint32_t now_ms)
{
    platform_flash_info_t info;
    platform_bool_t found;
    uint16_t length;
    uint32_t seq;
    platform_err_t err;

    if (s_initialized == PLATFORM_TRUE)
    {
        return PLATFORM_ERR_ALREADY_INIT;
    }
    (void)now_ms;

    s_reset_state();

    err = platform_flash_get_info(&info);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }
    if (info.capacity_bytes < (SERVICE_STORAGE_FACTORY_ADDRESS +
                               SERVICE_STORAGE_SECTOR_SIZE))
    {
        return PLATFORM_ERR_NOT_SUPPORTED;
    }
    if (info.page_size_bytes == 0u)
    {
        return PLATFORM_ERR_PARAM;
    }
    s_page_size = info.page_size_bytes;

    err = platform_logstore_init(&s_rings[SERVICE_STORAGE_RING_FAULT],
                                 SERVICE_STORAGE_FAULT_RING_ADDRESS,
                                 SERVICE_STORAGE_FAULT_RING_SECTORS,
                                 s_fault_erase_counts,
                                 s_fault_sector_seq);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = platform_logstore_init(&s_rings[SERVICE_STORAGE_RING_LOG],
                                 SERVICE_STORAGE_LOG_RING_ADDRESS,
                                 SERVICE_STORAGE_LOG_RING_SECTORS,
                                 s_log_erase_counts,
                                 s_log_sector_seq);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    /* Wear statistics first: they restore the ring erase counters. */
    err = s_ab_load(&s_wear, WEAR_LENGTH, s_wear_payload,
                    (uint16_t)WEAR_LENGTH, &seq, &length, &found);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }
    if (found == PLATFORM_TRUE)
    {
        s_apply_wear(s_wear_payload, length);
    }

    (void)s_ab_load(&s_blobs[SERVICE_STORAGE_BLOB_PARAM],
                    SERVICE_STORAGE_MAX_BLOB_LENGTH, NULL, 0u,
                    &seq, &length, &found);

    (void)s_ab_load(&s_blobs[SERVICE_STORAGE_BLOB_CALIB],
                    SERVICE_STORAGE_MAX_BLOB_LENGTH, NULL, 0u,
                    &seq, &length, &found);

    err = s_boot_count_update();
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    s_crash_scan();

    s_last_wear_log_erase = platform_logstore_erase_total(
                                &s_rings[SERVICE_STORAGE_RING_LOG]);
    s_last_wear_fault_erase = platform_logstore_erase_total(
                                &s_rings[SERVICE_STORAGE_RING_FAULT]);
    s_save_failed = PLATFORM_FALSE;
    s_clean = PLATFORM_TRUE;
    s_initialized = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t service_storage_deinit(void)
{
    if (s_initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    s_initialized = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

//******************************** Blobs ************************************//

platform_err_t service_storage_save_blob(service_storage_blob_t which,
                                         const uint8_t *data,
                                         uint16_t length,
                                         uint32_t now_ms)
{
    platform_err_t err;

    if ((which >= SERVICE_STORAGE_BLOB_COUNT) || (data == NULL) ||
        (length == 0u) || (length > SERVICE_STORAGE_MAX_BLOB_LENGTH))
    {
        return PLATFORM_ERR_PARAM;
    }
    if (s_initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = s_ab_store(&s_blobs[which], data, length, s_time_s(now_ms));
    if (PLATFORM_IS_ERR(err))
    {
        s_save_failed = PLATFORM_TRUE;
        return err;
    }

    if (which == SERVICE_STORAGE_BLOB_PARAM)
    {
        s_param_erases++;
    }
    else
    {
        s_calib_erases++;
    }

    s_save_failed = PLATFORM_FALSE;
    s_last_save_s = s_time_s(now_ms);

    /*
     * The wear record is a second sector erase (~50 ms). Do not do it here:
     * the blob seq is already published, so a host that polls REPORT_STORAGE
     * would see the new seq and send the next PARAM_SAVE while this call is
     * still busy, getting BUSY for a save that is in fact finished. The app
     * layer flushes the wear record after it has reported completion.
     */

    return PLATFORM_ERR_OK;
}

platform_err_t service_storage_load_blob(service_storage_blob_t which,
                                         uint8_t *buffer,
                                         uint16_t capacity,
                                         uint16_t *out_length,
                                         uint32_t *out_seq)
{
    platform_bool_t found;
    platform_err_t err;

    if ((which >= SERVICE_STORAGE_BLOB_COUNT) || (buffer == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }
    if (s_initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = s_ab_load(&s_blobs[which], SERVICE_STORAGE_MAX_BLOB_LENGTH,
                    buffer, capacity, out_seq, out_length, &found);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    return (found == PLATFORM_TRUE) ? PLATFORM_ERR_OK
                                    : PLATFORM_ERR_NO_RESOURCE;
}

platform_bool_t service_storage_blob_valid(service_storage_blob_t which)
{
    if (which >= SERVICE_STORAGE_BLOB_COUNT)
    {
        return PLATFORM_FALSE;
    }

    return s_blobs[which].valid;
}

uint32_t service_storage_blob_seq(service_storage_blob_t which)
{
    if (which >= SERVICE_STORAGE_BLOB_COUNT)
    {
        return 0u;
    }

    return s_blobs[which].seq;
}

void service_storage_set_clean(platform_bool_t clean)
{
    s_clean = (clean == PLATFORM_TRUE) ? PLATFORM_TRUE : PLATFORM_FALSE;
}

void service_storage_set_log_level(uint8_t level)
{
    s_log_level = level;
}

//******************************** Rings ************************************//

platform_err_t service_storage_append(service_storage_ring_t ring,
                                      uint8_t level,
                                      uint8_t tag,
                                      const char *text,
                                      uint16_t length,
                                      uint32_t now_ms)
{
    platform_err_t err;

    if (ring >= SERVICE_STORAGE_RING_COUNT)
    {
        return PLATFORM_ERR_PARAM;
    }
    if (s_initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }
    if ((text == NULL) && (length > 0u))
    {
        return PLATFORM_ERR_PARAM;
    }

    err = platform_logstore_append(&s_rings[ring], level, tag,
                                   s_time_s(now_ms),
                                   (const uint8_t *)text, length, NULL);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    s_wear_maybe_store(now_ms);

    return PLATFORM_ERR_OK;
}

platform_err_t service_storage_iter_begin(service_storage_ring_t ring,
                                          service_storage_iter_t *iter,
                                          uint32_t from_seq)
{
    if (ring >= SERVICE_STORAGE_RING_COUNT)
    {
        return PLATFORM_ERR_PARAM;
    }
    if (s_initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return platform_logstore_iter_begin(&s_rings[ring], iter, from_seq);
}

platform_err_t service_storage_iter_next(service_storage_iter_t *iter,
                                         service_storage_entry_t *entry)
{
    /* The iterator already carries the ring it walks; nothing else to check. */
    return platform_logstore_iter_next(iter, entry);
}

platform_err_t service_storage_ring_clear(service_storage_ring_t ring,
                                          uint32_t now_ms)
{
    platform_err_t err;

    if (ring >= SERVICE_STORAGE_RING_COUNT)
    {
        return PLATFORM_ERR_PARAM;
    }
    if (s_initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    err = platform_logstore_clear(&s_rings[ring]);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    (void)s_wear_store(now_ms);

    return PLATFORM_ERR_OK;
}

uint32_t service_storage_ring_used(service_storage_ring_t ring)
{
    if (ring >= SERVICE_STORAGE_RING_COUNT)
    {
        return 0u;
    }

    return platform_logstore_used_bytes(&s_rings[ring]);
}

uint32_t service_storage_ring_erase_total(service_storage_ring_t ring)
{
    if (ring >= SERVICE_STORAGE_RING_COUNT)
    {
        return 0u;
    }

    return platform_logstore_erase_total(&s_rings[ring]);
}

//******************************** Crash ************************************//

platform_err_t service_storage_crash_write(const uint8_t *data,
                                           uint16_t length,
                                           uint32_t now_ms)
{
    uint8_t header[SERVICE_STORAGE_RECORD_HEADER_SIZE];
    uint8_t tail[4];
    uint32_t slot;
    uint32_t best_slot;
    uint32_t best_seq;
    uint32_t min_seq;
    uint16_t flags;
    uint32_t crc;
    uint32_t address;
    uint32_t seq;
    platform_err_t err;

    if ((data == NULL) || (length > SERVICE_STORAGE_CRASH_MAX_LENGTH))
    {
        return PLATFORM_ERR_PARAM;
    }
    if (s_initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    /* Overwrite the oldest valid slot; an unusable slot is picked first. */
    best_slot = SERVICE_STORAGE_CRASH_SLOTS;
    best_seq = 0u;
    min_seq = 0xFFFFFFFFu;
    for (slot = 0u; slot < SERVICE_STORAGE_CRASH_SLOTS; slot++)
    {
        if (s_crash_state[slot] == 2u)
        {
            best_slot = slot;
            break;
        }
        if (s_crash_state[slot] != 1u)
        {
            best_slot = slot;
            break;
        }
        if (PLATFORM_IS_ERR(s_crash_validate((uint8_t)slot, NULL, &flags)))
        {
            continue;
        }
        /* Recover the seq cheaply from the header. */
        if (PLATFORM_IS_ERR(platform_flash_read(
                s_crash_slot_address(slot), header,
                SERVICE_STORAGE_RECORD_HEADER_SIZE)))
        {
            continue;
        }
        seq = s_get32(header + 8u);
        if (seq > best_seq)
        {
            best_seq = seq;
        }
        if (seq < min_seq)
        {
            min_seq = seq;
            if (best_slot == SERVICE_STORAGE_CRASH_SLOTS)
            {
                best_slot = slot;
            }
        }
    }
    if (best_slot == SERVICE_STORAGE_CRASH_SLOTS)
    {
        best_slot = 0u;
    }

    address = s_crash_slot_address(best_slot);

    s_build_header(header, SERVICE_STORAGE_MAGIC_CRASH,
                   SERVICE_STORAGE_RECORD_FLAG_VALID |
                       SERVICE_STORAGE_RECORD_FLAG_UNREAD,
                   best_seq + 1u, length, s_time_s(now_ms));
    crc = s_crash_crc_start(header);
    crc = platform_crc32_update(crc, data, length);
    crc = s_crc_finish(crc);

    err = platform_flash_erase(address, SERVICE_STORAGE_SECTOR_SIZE);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }
    s_crash_erases++;
    s_crash_state[best_slot] = 0u;

    s_put32(header + 16u, crc);

    err = s_write_at(address, header, SERVICE_STORAGE_RECORD_HEADER_SIZE);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }
    if (length > 0u)
    {
        err = s_write_at(address + SERVICE_STORAGE_RECORD_HEADER_SIZE,
                         data, length);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }
    }

    /* Commit marker, last write of the whole slot. */
    s_put32(tail, crc);
    err = s_write_at(address + SERVICE_STORAGE_SECTOR_SIZE - 4u, tail, 4u);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    s_crash_unread = PLATFORM_TRUE;
    s_crash_state[best_slot] = 1u;

    (void)s_wear_store(now_ms);

    return PLATFORM_ERR_OK;
}

platform_err_t service_storage_crash_info(uint8_t slot,
                                          uint16_t *out_length,
                                          uint8_t *out_flags)
{
    uint16_t flags;

    if (s_initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (PLATFORM_IS_ERR(s_crash_validate(slot, out_length, &flags)))
    {
        return PLATFORM_ERR_NO_RESOURCE;
    }
    if (out_flags != NULL)
    {
        *out_flags = (uint8_t)flags;
    }

    return PLATFORM_ERR_OK;
}

platform_err_t service_storage_crash_read(uint8_t slot,
                                          uint16_t offset,
                                          uint8_t *buffer,
                                          uint16_t capacity,
                                          uint16_t *out_length)
{
    uint16_t total;
    uint16_t count;
    platform_err_t err;

    if ((buffer == NULL) || (out_length == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    err = service_storage_crash_info(slot, &total, NULL);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }
    if (offset >= total)
    {
        *out_length = 0u;
        return PLATFORM_ERR_NO_RESOURCE;
    }

    count = (uint16_t)(total - offset);
    if (count > capacity)
    {
        count = capacity;
    }

    err = platform_flash_read(s_crash_slot_address(slot) +
                                  SERVICE_STORAGE_RECORD_HEADER_SIZE +
                                  offset,
                              buffer,
                              count);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    *out_length = count;

    return PLATFORM_ERR_OK;
}

platform_err_t service_storage_crash_mark_read(uint8_t slot)
{
    uint8_t flags[2];
    platform_err_t err;

    if (slot >= SERVICE_STORAGE_CRASH_SLOTS)
    {
        return PLATFORM_ERR_PARAM;
    }

    err = service_storage_crash_info(slot, NULL, NULL);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    /* Bits can only be cleared: keep VALID, drop UNREAD. */
    s_put16(flags, SERVICE_STORAGE_RECORD_FLAG_VALID);
    err = s_write_at(s_crash_slot_address(slot) + 6u, flags, 2u);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    s_crash_scan();

    return PLATFORM_ERR_OK;
}

platform_bool_t service_storage_crash_unread(void)
{
    return s_crash_unread;
}

/*
 * LOG_CLEAR for the crash region. Clearing the VALID bit is a 2-byte program
 * per slot; erasing sixteen 4 KiB sectors would take up to six seconds and
 * buys nothing, because a slot is erased again when it is reused.
 */
platform_err_t service_storage_crash_clear(void)
{
    /* Bytes, not a uint16_t: the encoding on flash is little-endian by rule. */
    uint8_t cleared[2] = { 0u, 0u };
    uint32_t slot;
    platform_err_t err;

    if (s_initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    for (slot = 0u; slot < SERVICE_STORAGE_CRASH_SLOTS; slot++)
    {
        if (s_crash_state[slot] != 1u)
        {
            continue;
        }

        err = s_write_at(s_crash_slot_address(slot) + 6u, cleared, 2u);
        if (PLATFORM_IS_ERR(err))
        {
            s_crash_scan();
            return err;
        }
    }

    s_crash_scan();

    return PLATFORM_ERR_OK;
}

/*
 * Both accessors answer from the cache filled by the mount scan, so a caller
 * that must not touch the bus (the protocol task, which owes the host an ACK
 * within 200 ms) can resolve CRASH_READ without waiting behind an erase.
 */
platform_bool_t service_storage_crash_slot_valid(uint8_t slot)
{
    if (slot >= SERVICE_STORAGE_CRASH_SLOTS)
    {
        return PLATFORM_FALSE;
    }

    return (s_crash_state[slot] == 1u) ? PLATFORM_TRUE : PLATFORM_FALSE;
}

platform_bool_t service_storage_crash_latest(uint8_t *out_slot)
{
    uint32_t slot;
    uint32_t best = SERVICE_STORAGE_CRASH_SLOTS;
    uint32_t best_seq = 0u;

    for (slot = 0u; slot < SERVICE_STORAGE_CRASH_SLOTS; slot++)
    {
        if ((s_crash_state[slot] == 1u) && (s_crash_seq[slot] >= best_seq))
        {
            best_seq = s_crash_seq[slot];
            best = slot;
        }
    }

    if (best == SERVICE_STORAGE_CRASH_SLOTS)
    {
        return PLATFORM_FALSE;
    }

    if (out_slot != NULL)
    {
        *out_slot = (uint8_t)best;
    }

    return PLATFORM_TRUE;
}

//******************************** Status ***********************************//

void service_storage_get_status(service_storage_status_t *status)
{
    uint32_t log_wrap;

    if (status == NULL)
    {
        return;
    }

    status->flags = 0u;
    if (s_clean == PLATFORM_TRUE)
    {
        status->flags |= SERVICE_STORAGE_FLAG_CLEAN;
    }
    if (s_blobs[SERVICE_STORAGE_BLOB_PARAM].valid == PLATFORM_TRUE)
    {
        status->flags |= SERVICE_STORAGE_FLAG_PARAM_VALID;
    }
    if (s_blobs[SERVICE_STORAGE_BLOB_CALIB].valid == PLATFORM_TRUE)
    {
        status->flags |= SERVICE_STORAGE_FLAG_CALIB_VALID;
    }
    if (s_save_failed == PLATFORM_TRUE)
    {
        status->flags |= SERVICE_STORAGE_FLAG_SAVE_FAILED;
    }
    if (s_crash_unread == PLATFORM_TRUE)
    {
        status->flags |= SERVICE_STORAGE_FLAG_CRASH_UNREAD;
    }

    log_wrap = platform_logstore_wrap_count(&s_rings[SERVICE_STORAGE_RING_LOG]);
    if (log_wrap > 0u)
    {
        status->flags |= SERVICE_STORAGE_FLAG_LOG_WRAPPED;
    }

    status->log_level = s_log_level;
    status->param_seq = (uint16_t)s_blobs[SERVICE_STORAGE_BLOB_PARAM].seq;
    status->calib_seq = (uint16_t)s_blobs[SERVICE_STORAGE_BLOB_CALIB].seq;
    status->reserved = 0u;
    status->boot_total = s_boot_total;
    status->log_used = platform_logstore_used_bytes(
                                &s_rings[SERVICE_STORAGE_RING_LOG]);
    status->log_erase = platform_logstore_erase_total(
                                &s_rings[SERVICE_STORAGE_RING_LOG]);
    status->last_save_s = s_last_save_s;
}

platform_err_t service_storage_flush_wear(uint32_t now_ms)
{
    if (s_initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    return s_wear_store(now_ms);
}
