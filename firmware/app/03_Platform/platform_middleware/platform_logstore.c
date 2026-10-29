/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_logstore.c
 *
 * @par dependencies
 * - platform_logstore.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement the append-only record ring.
 *
 * @version V1.0 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "platform_logstore.h"

//******************************** Defines **********************************//

#define LOGSTORE_SCRATCH_SIZE       ((uint32_t)32u)

//******************************** Declaring ********************************//

typedef struct
{
    uint32_t        records;
    uint32_t        last_seq;
    uint32_t        end_offset;
    platform_bool_t torn;
} s_scan_t;

typedef struct
{
    const uint32_t *sector_seq;
    uint32_t        want_seq;
} s_pick_ctx_t;

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

static uint32_t s_align4(uint32_t value)
{
    return (value + PLATFORM_LOGSTORE_RECORD_ALIGN - 1u) &
           ~(PLATFORM_LOGSTORE_RECORD_ALIGN - 1u);
}

static uint32_t s_sector_address(const platform_logstore_t *store,
                                 uint32_t index)
{
    return store->base_address + (index * store->sector_size);
}

static platform_bool_t s_all_erased(const uint8_t *data, uint32_t len)
{
    uint32_t i;

    for (i = 0u; i < len; i++)
    {
        if (data[i] != 0xFFu)
        {
            return PLATFORM_FALSE;
        }
    }

    return PLATFORM_TRUE;
}

static platform_bool_t s_sector_header_valid(const uint8_t *header)
{
    if (s_get16(header) != PLATFORM_LOGSTORE_SECTOR_MAGIC)
    {
        return PLATFORM_FALSE;
    }
    if (s_get16(header + 2u) != PLATFORM_LOGSTORE_SECTOR_OPEN)
    {
        return PLATFORM_FALSE;
    }
    if (s_get16(header + 10u) != platform_crc16_ccitt(header, 10u))
    {
        return PLATFORM_FALSE;
    }

    return PLATFORM_TRUE;
}

static void s_build_sector_header(uint8_t *header, uint32_t sector_seq)
{
    s_put16(header + 0u, PLATFORM_LOGSTORE_SECTOR_MAGIC);
    s_put16(header + 2u, PLATFORM_LOGSTORE_SECTOR_OPEN);
    s_put32(header + 4u, sector_seq);
    s_put16(header + 8u, (uint16_t)PLATFORM_LOGSTORE_SECTOR_HEADER_SIZE);
    s_put16(header + 10u, platform_crc16_ccitt(header, 10u));
    s_put32(header + 12u, 0xFFFFFFFFu);
}

static platform_bool_t s_record_header_plausible(const uint8_t *header,
                                                 uint32_t space_left,
                                                 uint16_t *len)
{
    uint32_t payload;

    if (s_get16(header) != PLATFORM_LOGSTORE_RECORD_MAGIC)
    {
        return PLATFORM_FALSE;
    }

    *len = s_get16(header + 8u);
    if (*len > PLATFORM_LOGSTORE_MAX_PAYLOAD)
    {
        return PLATFORM_FALSE;
    }

    payload = s_align4((uint32_t)*len);
    if ((PLATFORM_LOGSTORE_RECORD_HEADER_SIZE + payload) > space_left)
    {
        return PLATFORM_FALSE;
    }

    return PLATFORM_TRUE;
}

static uint16_t s_record_crc(const uint8_t *header,
                             const uint8_t *payload,
                             uint32_t len)
{
    uint16_t crc;

    crc = platform_crc16_ccitt_update(PLATFORM_CRC16_CCITT_INIT_VALUE,
                                      header, 10u);
    crc = platform_crc16_ccitt_update(crc, header + 12u, 4u);
    crc = platform_crc16_ccitt_update(crc, payload, len);

    return crc;
}

/*
 * Read a record body and check it against the header CRC. payload may be NULL
 * when the caller only wants validation; the body is then streamed through a
 * small scratch buffer so mount never needs a 240-byte stack frame per sector.
 */
static platform_err_t s_load_record(uint32_t address,
                                    const uint8_t *header,
                                    uint8_t *payload,
                                    uint16_t len,
                                    platform_bool_t *valid)
{
    uint8_t scratch[LOGSTORE_SCRATCH_SIZE];
    uint8_t *target;
    uint32_t chunk;
    uint32_t done;
    uint16_t crc;
    platform_err_t err;

    crc = platform_crc16_ccitt_update(PLATFORM_CRC16_CCITT_INIT_VALUE,
                                      header, 10u);
    crc = platform_crc16_ccitt_update(crc, header + 12u, 4u);

    done = 0u;
    while (done < len)
    {
        chunk = len - done;
        if (chunk > LOGSTORE_SCRATCH_SIZE)
        {
            chunk = LOGSTORE_SCRATCH_SIZE;
        }
        if (payload != NULL)
        {
            target = payload + done;
        }
        else
        {
            target = scratch;
        }

        err = platform_flash_read(address + done, target, chunk);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }

        crc = platform_crc16_ccitt_update(crc, target, chunk);
        done += chunk;
    }

    *valid = (crc == s_get16(header + 10u)) ? PLATFORM_TRUE : PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

/* Scan one sector from its header offset to the first slot that is not a
 * complete, CRC-valid record. */
static platform_err_t s_scan_sector(platform_logstore_t *store,
                                    uint32_t index,
                                    s_scan_t *scan)
{
    uint8_t header[PLATFORM_LOGSTORE_RECORD_HEADER_SIZE];
    uint32_t offset;
    uint32_t address;
    uint16_t len;
    platform_bool_t valid;
    platform_err_t err;

    scan->records = 0u;
    scan->last_seq = 0u;
    scan->end_offset = PLATFORM_LOGSTORE_SECTOR_HEADER_SIZE;
    scan->torn = PLATFORM_FALSE;

    address = s_sector_address(store, index);
    offset = PLATFORM_LOGSTORE_SECTOR_HEADER_SIZE;

    while ((offset + PLATFORM_LOGSTORE_RECORD_HEADER_SIZE) <=
           store->sector_size)
    {
        err = platform_flash_read(address + offset, header,
                                  PLATFORM_LOGSTORE_RECORD_HEADER_SIZE);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }

        if (s_record_header_plausible(
                header,
                store->sector_size - offset,
                &len) == PLATFORM_FALSE)
        {
            /*
             * An erased slot is the normal write position. Anything else is a
             * header that was cut short by a reset; that slot must never be
             * written again, because NOR bits can only be cleared.
             */
            if (s_all_erased(header,
                             PLATFORM_LOGSTORE_RECORD_HEADER_SIZE) ==
                PLATFORM_FALSE)
            {
                scan->torn = PLATFORM_TRUE;
            }
            break;
        }

        err = s_load_record(address + offset +
                                PLATFORM_LOGSTORE_RECORD_HEADER_SIZE,
                            header, NULL, len, &valid);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }
        if (valid == PLATFORM_FALSE)
        {
            scan->torn = PLATFORM_TRUE;
            break;
        }

        scan->records++;
        scan->last_seq = s_get32(header + 12u);
        offset += PLATFORM_LOGSTORE_RECORD_HEADER_SIZE + s_align4(len);
    }

    scan->end_offset = offset;

    return PLATFORM_ERR_OK;
}

static platform_err_t s_mount(platform_logstore_t *store)
{
    uint8_t header[PLATFORM_LOGSTORE_SECTOR_HEADER_SIZE];
    s_scan_t scan;
    uint32_t i;
    uint32_t seq;
    uint32_t top_index;
    uint32_t top_seq;
    uint32_t allocated;
    uint32_t floor_seq;
    platform_err_t err;

    top_index = PLATFORM_LOGSTORE_NONE;
    top_seq = 0u;
    allocated = 0u;

    for (i = 0u; i < store->sector_count; i++)
    {
        err = platform_flash_read(s_sector_address(store, i), header,
                                  PLATFORM_LOGSTORE_SECTOR_HEADER_SIZE);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }

        seq = s_sector_header_valid(header) ? s_get32(header + 4u) : 0u;
        if (seq == 0u)
        {
            store->sector_seq[i] = 0u;
            continue;
        }

        store->sector_seq[i] = seq;
        allocated++;
        if (seq > top_seq)
        {
            top_seq = seq;
            top_index = i;
        }
    }

    store->top_sector_seq = top_seq;
    store->allocated_sectors = allocated;
    store->last_sector = top_index;
    store->active_sector = PLATFORM_LOGSTORE_NONE;
    store->active_offset = 0u;
    store->used_bytes = 0u;
    store->next_seq = 1u;

    if (top_index == PLATFORM_LOGSTORE_NONE)
    {
        return PLATFORM_ERR_OK;
    }

    err = s_scan_sector(store, top_index, &scan);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    if (scan.records > 0u)
    {
        store->next_seq = scan.last_seq + 1u;
    }

    if (scan.torn == PLATFORM_FALSE)
    {
        store->active_sector = top_index;
        store->active_offset = scan.end_offset;
    }
    else
    {
        /* Seal: the failed slot cannot be rewritten, so waste the rest. */
        store->active_offset = store->sector_size;
    }

    store->used_bytes = (allocated - 1u) *
                        (store->sector_size -
                         PLATFORM_LOGSTORE_SECTOR_HEADER_SIZE);
    if (store->active_sector == top_index)
    {
        store->used_bytes += store->active_offset -
                             PLATFORM_LOGSTORE_SECTOR_HEADER_SIZE;
    }
    else
    {
        store->used_bytes += store->sector_size -
                             PLATFORM_LOGSTORE_SECTOR_HEADER_SIZE;
    }

    /*
     * next_seq was taken from the newest sector. When that sector is empty the
     * newest record is in an older sector, so keep walking down until one
     * reports a sequence number.
     */
    floor_seq = top_seq;
    while ((scan.records == 0u) && (floor_seq > 1u))
    {
        uint32_t best_index = PLATFORM_LOGSTORE_NONE;
        uint32_t best_seq = 0u;

        for (i = 0u; i < store->sector_count; i++)
        {
            seq = store->sector_seq[i];
            if ((seq != 0u) && (seq < floor_seq) && (seq > best_seq))
            {
                best_seq = seq;
                best_index = i;
            }
        }
        if (best_index == PLATFORM_LOGSTORE_NONE)
        {
            break;
        }

        err = s_scan_sector(store, best_index, &scan);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }
        if (scan.records > 0u)
        {
            store->next_seq = scan.last_seq + 1u;
            break;
        }

        floor_seq = best_seq;
    }

    return PLATFORM_ERR_OK;
}

/* Split a write at page boundaries; the driver rejects crossing ones. */
static platform_err_t s_write(platform_logstore_t *store,
                              uint32_t address,
                              const uint8_t *data,
                              uint32_t length)
{
    uint32_t chunk;
    uint32_t page_offset;
    platform_err_t err;

    while (length > 0u)
    {
        page_offset = address % store->page_size;
        chunk = store->page_size - page_offset;
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

static platform_bool_t s_pick_matches(uint32_t index, void *context)
{
    const s_pick_ctx_t *ctx = (const s_pick_ctx_t *)context;

    return (ctx->sector_seq[index] == ctx->want_seq) ? PLATFORM_TRUE
                                                     : PLATFORM_FALSE;
}

static platform_err_t s_open_sector(platform_logstore_t *store)
{
    uint8_t header[PLATFORM_LOGSTORE_SECTOR_HEADER_SIZE];
    s_pick_ctx_t ctx;
    uint32_t index;
    uint32_t hint;
    uint32_t new_seq;
    uint32_t i;
    platform_bool_t found;
    platform_bool_t was_full;
    platform_err_t err;

    ctx.sector_seq = store->sector_seq;
    was_full = (store->allocated_sectors >= store->sector_count)
                   ? PLATFORM_TRUE
                   : PLATFORM_FALSE;

    if (was_full == PLATFORM_TRUE)
    {
        /* Only the oldest sector may be discarded. */
        ctx.want_seq = 0xFFFFFFFFu;
        for (i = 0u; i < store->sector_count; i++)
        {
            if (store->sector_seq[i] < ctx.want_seq)
            {
                ctx.want_seq = store->sector_seq[i];
            }
        }
    }
    else
    {
        ctx.want_seq = 0u;
    }

    hint = (store->last_sector == PLATFORM_LOGSTORE_NONE)
               ? 0u
               : (store->last_sector + 1u);

    index = platform_wearlevel_pick_filtered(&store->wearlevel,
                                             hint,
                                             s_pick_matches,
                                             &ctx,
                                             &found);
    if (found == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_FAIL;
    }

    err = platform_flash_erase(s_sector_address(store, index),
                               store->sector_size);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }
    platform_wearlevel_note_erase(&store->wearlevel, index);

    new_seq = store->top_sector_seq + 1u;
    s_build_sector_header(header, new_seq);
    err = s_write(store, s_sector_address(store, index), header,
                  PLATFORM_LOGSTORE_SECTOR_HEADER_SIZE);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    store->sector_seq[index] = new_seq;
    store->top_sector_seq = new_seq;
    store->last_sector = index;
    store->active_sector = index;
    store->active_offset = PLATFORM_LOGSTORE_SECTOR_HEADER_SIZE;

    if (was_full == PLATFORM_TRUE)
    {
        store->wrap_count++;
        if (store->used_bytes >
            (store->sector_size - PLATFORM_LOGSTORE_SECTOR_HEADER_SIZE))
        {
            store->used_bytes -= store->sector_size -
                                 PLATFORM_LOGSTORE_SECTOR_HEADER_SIZE;
        }
        else
        {
            store->used_bytes = 0u;
        }
    }
    else
    {
        store->allocated_sectors++;
    }

    return PLATFORM_ERR_OK;
}

//******************************** Declaring ********************************//

platform_err_t platform_logstore_init(platform_logstore_t *store,
                                      uint32_t base_address,
                                      uint32_t sector_count,
                                      uint16_t *erase_counts,
                                      uint32_t *sector_seq)
{
    platform_flash_info_t info;
    platform_err_t err;
    uint32_t i;

    if ((store == NULL) || (erase_counts == NULL) || (sector_seq == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }
    if ((sector_count == 0u) ||
        (sector_count > PLATFORM_LOGSTORE_MAX_SECTORS))
    {
        return PLATFORM_ERR_PARAM;
    }

    err = platform_flash_get_info(&info);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }
    if ((info.sector_size_bytes == 0u) || (info.page_size_bytes == 0u))
    {
        return PLATFORM_ERR_PARAM;
    }
    if ((uint64_t)base_address +
        ((uint64_t)sector_count * info.sector_size_bytes) >
        (uint64_t)info.capacity_bytes)
    {
        return PLATFORM_ERR_PARAM;
    }

    store->base_address = base_address;
    store->sector_size = info.sector_size_bytes;
    store->page_size = info.page_size_bytes;
    store->sector_count = sector_count;
    store->erase_counts = erase_counts;
    store->sector_seq = sector_seq;

    err = platform_wearlevel_init(&store->wearlevel,
                                  base_address,
                                  info.sector_size_bytes,
                                  sector_count,
                                  erase_counts);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    for (i = 0u; i < sector_count; i++)
    {
        sector_seq[i] = 0u;
    }

    store->active_sector = PLATFORM_LOGSTORE_NONE;
    store->active_offset = 0u;
    store->top_sector_seq = 0u;
    store->next_seq = 1u;
    store->allocated_sectors = 0u;
    store->used_bytes = 0u;
    store->written_bytes = 0u;
    store->wrap_count = 0u;
    store->last_sector = PLATFORM_LOGSTORE_NONE;
    store->mounted = PLATFORM_FALSE;

    err = s_mount(store);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    store->mounted = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_logstore_append(platform_logstore_t *store,
                                        uint8_t level,
                                        uint8_t tag,
                                        uint32_t ts,
                                        const uint8_t *payload,
                                        uint16_t len,
                                        uint32_t *out_seq)
{
    uint8_t header[PLATFORM_LOGSTORE_RECORD_HEADER_SIZE];
    uint32_t record_size;
    uint32_t address;
    uint32_t offset;
    uint32_t seq;
    platform_err_t err;

    if (store == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }
    if (store->mounted == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }
    if (len > PLATFORM_LOGSTORE_MAX_PAYLOAD)
    {
        return PLATFORM_ERR_PARAM;
    }
    if ((payload == NULL) && (len > 0u))
    {
        return PLATFORM_ERR_PARAM;
    }

    record_size = PLATFORM_LOGSTORE_RECORD_HEADER_SIZE + s_align4(len);
    if ((PLATFORM_LOGSTORE_SECTOR_HEADER_SIZE + record_size) >
        store->sector_size)
    {
        return PLATFORM_ERR_PARAM;
    }

    if ((store->active_sector == PLATFORM_LOGSTORE_NONE) ||
        ((store->active_offset + record_size) > store->sector_size))
    {
        err = s_open_sector(store);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }
    }

    seq = store->next_seq;
    offset = store->active_offset;
    address = s_sector_address(store, store->active_sector);

    s_put16(header + 0u, PLATFORM_LOGSTORE_RECORD_MAGIC);
    header[2] = level;
    header[3] = tag;
    s_put32(header + 4u, ts);
    s_put16(header + 8u, len);
    s_put16(header + 10u, 0u);
    s_put32(header + 12u, seq);
    s_put16(header + 10u, s_record_crc(header, payload, len));

    err = s_write(store,
                  address + offset,
                  header,
                  PLATFORM_LOGSTORE_RECORD_HEADER_SIZE);
    if (PLATFORM_IS_ERR(err))
    {
        store->active_offset = store->sector_size;
        return err;
    }

    if (len > 0u)
    {
        err = s_write(store,
                      address + offset +
                          PLATFORM_LOGSTORE_RECORD_HEADER_SIZE,
                      payload,
                      len);
        if (PLATFORM_IS_ERR(err))
        {
            store->active_offset = store->sector_size;
            return err;
        }
    }

    store->active_offset = offset + record_size;
    store->used_bytes += record_size;
    store->written_bytes += record_size;
    store->next_seq = seq + 1u;

    if (out_seq != NULL)
    {
        *out_seq = seq;
    }

    return PLATFORM_ERR_OK;
}

platform_err_t platform_logstore_clear(platform_logstore_t *store)
{
    static const uint8_t s_zero_magic[2] = {0x00u, 0x00u};
    uint32_t i;
    platform_err_t err;

    if (store == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }
    if (store->mounted == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    for (i = 0u; i < store->sector_count; i++)
    {
        if (store->sector_seq[i] == 0u)
        {
            continue;
        }

        err = s_write(store, s_sector_address(store, i), s_zero_magic, 2u);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }
        store->sector_seq[i] = 0u;
    }

    store->active_sector = PLATFORM_LOGSTORE_NONE;
    store->active_offset = 0u;
    store->allocated_sectors = 0u;
    store->used_bytes = 0u;
    store->last_sector = PLATFORM_LOGSTORE_NONE;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_logstore_iter_begin(platform_logstore_t *store,
                                            platform_logstore_iter_t *iter,
                                            uint32_t from_seq)
{
    uint32_t i;
    uint32_t best_index;
    uint32_t best_seq;

    if ((store == NULL) || (iter == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }
    if (store->mounted == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    best_index = PLATFORM_LOGSTORE_NONE;
    best_seq = 0xFFFFFFFFu;
    for (i = 0u; i < store->sector_count; i++)
    {
        if ((store->sector_seq[i] != 0u) &&
            (store->sector_seq[i] < best_seq))
        {
            best_seq = store->sector_seq[i];
            best_index = i;
        }
    }

    iter->store = store;
    iter->from_seq = from_seq;
    iter->sector = best_index;
    iter->sector_seq = best_seq;
    iter->offset = PLATFORM_LOGSTORE_SECTOR_HEADER_SIZE;
    iter->finished = (best_index == PLATFORM_LOGSTORE_NONE) ? PLATFORM_TRUE
                                                            : PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

static void s_iter_next_sector(platform_logstore_iter_t *iter)
{
    platform_logstore_t *store = iter->store;
    uint32_t i;
    uint32_t best_index;
    uint32_t best_seq;

    best_index = PLATFORM_LOGSTORE_NONE;
    best_seq = 0xFFFFFFFFu;
    for (i = 0u; i < store->sector_count; i++)
    {
        if ((store->sector_seq[i] > iter->sector_seq) &&
            (store->sector_seq[i] < best_seq))
        {
            best_seq = store->sector_seq[i];
            best_index = i;
        }
    }

    if (best_index == PLATFORM_LOGSTORE_NONE)
    {
        iter->finished = PLATFORM_TRUE;
        return;
    }

    iter->sector = best_index;
    iter->sector_seq = best_seq;
    iter->offset = PLATFORM_LOGSTORE_SECTOR_HEADER_SIZE;
}

platform_err_t platform_logstore_iter_next(platform_logstore_iter_t *iter,
                                           platform_logstore_entry_t *entry)
{
    platform_logstore_t *store;
    uint8_t header[PLATFORM_LOGSTORE_RECORD_HEADER_SIZE];
    uint32_t address;
    uint32_t space_left;
    uint16_t len;
    platform_bool_t valid;
    platform_err_t err;

    if ((iter == NULL) || (entry == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }
    store = iter->store;

    while (iter->finished == PLATFORM_FALSE)
    {
        if ((iter->offset + PLATFORM_LOGSTORE_RECORD_HEADER_SIZE) >
            store->sector_size)
        {
            s_iter_next_sector(iter);
            continue;
        }

        address = s_sector_address(store, iter->sector);
        space_left = store->sector_size - iter->offset;

        err = platform_flash_read(address + iter->offset, header,
                                  PLATFORM_LOGSTORE_RECORD_HEADER_SIZE);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }

        if (s_record_header_plausible(header, space_left, &len) ==
            PLATFORM_FALSE)
        {
            s_iter_next_sector(iter);
            continue;
        }

        err = s_load_record(address + iter->offset +
                                PLATFORM_LOGSTORE_RECORD_HEADER_SIZE,
                            header,
                            entry->payload,
                            len,
                            &valid);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }
        if (valid == PLATFORM_FALSE)
        {
            s_iter_next_sector(iter);
            continue;
        }

        iter->offset += PLATFORM_LOGSTORE_RECORD_HEADER_SIZE + s_align4(len);

        if (s_get32(header + 12u) < iter->from_seq)
        {
            continue;
        }

        entry->level = header[2];
        entry->tag = header[3];
        entry->ts = s_get32(header + 4u);
        entry->len = len;
        entry->seq = s_get32(header + 12u);

        return PLATFORM_ERR_OK;
    }

    return PLATFORM_ERR_NO_RESOURCE;
}

uint32_t platform_logstore_used_bytes(const platform_logstore_t *store)
{
    return (store == NULL) ? 0u : store->used_bytes;
}

uint32_t platform_logstore_written_bytes(const platform_logstore_t *store)
{
    return (store == NULL) ? 0u : store->written_bytes;
}

uint32_t platform_logstore_wrap_count(const platform_logstore_t *store)
{
    return (store == NULL) ? 0u : store->wrap_count;
}

uint32_t platform_logstore_erase_total(const platform_logstore_t *store)
{
    uint32_t i;
    uint32_t sum;

    if (store == NULL)
    {
        return 0u;
    }

    sum = 0u;
    for (i = 0u; i < store->sector_count; i++)
    {
        sum += platform_wearlevel_get_count(&store->wearlevel, i);
    }

    return sum;
}

uint32_t platform_logstore_allocated_sectors(const platform_logstore_t *store)
{
    return (store == NULL) ? 0u : store->allocated_sectors;
}
