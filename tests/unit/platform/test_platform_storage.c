/* L1 unit test for the storage middleware (wear levelling + log store).
 * Runs on the PC against a RAM fake that behaves like NOR flash: erase sets
 * 0xFF, writes can only clear bits, and a write that crosses a page boundary
 * is rejected exactly as the W25Q64 driver rejects it.
 *
 * Build (MinGW or any host gcc):
 *   gcc -std=c99 -Wall -Wextra \
 *       -I firmware/app/03_Platform/platform_common \
 *       -I firmware/app/03_Platform/platform_bsp \
 *       -I firmware/app/03_Platform/platform_middleware \
 *       -I firmware/app/04_Impl \
 *       tests/unit/platform/test_platform_storage.c \
 *       firmware/app/03_Platform/platform_middleware/platform_wearlevel.c \
 *       firmware/app/03_Platform/platform_middleware/platform_logstore.c \
 *       firmware/app/03_Platform/platform_middleware/platform_crc.c \
 *       -o build/test_platform_storage
 */

#include <stdio.h>
#include <string.h>

#include "platform_crc.h"
#include "platform_wearlevel.h"
#include "platform_logstore.h"

static int g_failed = 0;

#define CHECK(cond, msg)                                                  \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);        \
            g_failed++;                                                   \
        }                                                                 \
    } while (0)

/* --------------------------------------------------------------------- */
/* Fake W25Q64: 16 KiB, 256 B sectors, 64 B pages.                       */
/* --------------------------------------------------------------------- */

#define FAKE_CAPACITY   ((uint32_t)(16u * 1024u))
#define FAKE_SECTOR     ((uint32_t)256u)
#define FAKE_PAGE       ((uint32_t)64u)

static uint8_t  s_flash[FAKE_CAPACITY];
static uint32_t s_and_violations;

platform_err_t platform_flash_read(uint32_t address,
                                          uint8_t *data,
                                          uint32_t length)
{
    if ((data == NULL) || ((address + length) > FAKE_CAPACITY))
    {
        return PLATFORM_ERR_PARAM;
    }

    memcpy(data, &s_flash[address], length);

    return PLATFORM_ERR_OK;
}

platform_err_t platform_flash_write(uint32_t address,
                                           const uint8_t *data,
                                           uint32_t length)
{
    uint32_t i;

    if ((data == NULL) || ((address + length) > FAKE_CAPACITY))
    {
        return PLATFORM_ERR_PARAM;
    }
    if (((address % FAKE_PAGE) + length) > FAKE_PAGE)
    {
        return PLATFORM_ERR_PARAM; /* page crossing, same rule as the driver */
    }

    /* NOR programming may only clear bits; setting one means the caller wrote
     * over a region that was never erased. */
    for (i = 0u; i < length; i++)
    {
        if ((~s_flash[address + i] & data[i]) != 0u)
        {
            s_and_violations++;
        }
        s_flash[address + i] &= data[i];
    }

    return PLATFORM_ERR_OK;
}

platform_err_t platform_flash_erase(uint32_t address, uint32_t length)
{
    if ((address % FAKE_SECTOR) != 0u)
    {
        return PLATFORM_ERR_PARAM;
    }
    if ((length % FAKE_SECTOR) != 0u)
    {
        return PLATFORM_ERR_PARAM;
    }
    if ((address + length) > FAKE_CAPACITY)
    {
        return PLATFORM_ERR_PARAM;
    }

    memset(&s_flash[address], 0xFF, length);

    return PLATFORM_ERR_OK;
}

platform_err_t platform_flash_get_info(platform_flash_info_t *info)
{
    if (info == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    info->capacity_bytes    = FAKE_CAPACITY;
    info->sector_size_bytes = FAKE_SECTOR;
    info->page_size_bytes   = FAKE_PAGE;
    info->manufacturer_id   = 0xEFu;
    info->device_id         = 0x4017u;

    return PLATFORM_ERR_OK;
}

static void fake_reset(void)
{
    memset(s_flash, 0xFF, sizeof(s_flash));
    s_and_violations = 0u;
}

/* --------------------------------------------------------------------- */
/* CRC-32                                                                */
/* --------------------------------------------------------------------- */

static void test_crc32_selfcheck(void)
{
    CHECK(platform_crc32((const uint8_t *)"123456789", 9u) == 0xCBF43926u,
          "CRC-32 自检值应为 0xCBF43926");

    CHECK(platform_crc32_update(PLATFORM_CRC32_INIT_VALUE,
                                (const uint8_t *)"1234", 4u) ==
          (platform_crc32((const uint8_t *)"1234", 4u) ^
           PLATFORM_CRC32_XOROUT_VALUE),
          "增量 CRC-32 应与一次性计算一致");
}

/* --------------------------------------------------------------------- */
/* Wear levelling                                                        */
/* --------------------------------------------------------------------- */

static platform_bool_t even_filter(uint32_t index, void *context)
{
    const uint8_t *flags = (const uint8_t *)context;

    return (flags[index] != 0u) ? PLATFORM_TRUE : PLATFORM_FALSE;
}

static void test_wearlevel_round_robin(void)
{
    uint16_t counts[8];
    platform_wearlevel_t wl;
    uint32_t i;

    CHECK(platform_wearlevel_init(&wl, 0x1000u, 4096u, 8u, counts) ==
          PLATFORM_ERR_OK, "磨损均衡初始化应成功");

    /* All equal: the hint is kept, so the ring stays sequential. */
    CHECK(platform_wearlevel_pick(&wl, 3u) == 3u,
          "计数相同时应返回 hint");
    CHECK(platform_wearlevel_sector_address(&wl, 2u) == 0x3000u,
          "扇区地址应为 base + index * sector_size");

    for (i = 0u; i < 3u; i++)
    {
        platform_wearlevel_note_erase(&wl, 3u);
    }

    /* Give every other sector a higher count so sectors 0 and 3 are the only
     * cold ones; the hint then decides which of them is reached first. */
    for (i = 0u; i < 8u; i++)
    {
        if ((i != 0u) && (i != 3u))
        {
            platform_wearlevel_set_count(&wl, i, 2u);
        }
    }

    CHECK(platform_wearlevel_min(&wl) == 0u, "最小值应为 0");
    CHECK(platform_wearlevel_max(&wl) == 3u, "最大值应为 3");
    CHECK(platform_wearlevel_pick(&wl, 0u) == 0u,
          "应选中擦除次数最少的扇区");
    CHECK(platform_wearlevel_pick(&wl, 4u) == 0u,
          "从 hint 4 出发应回绕到冷扇区 0");

    /* Saturating counter. */
    platform_wearlevel_set_count(&wl, 5u, 70000u);
    CHECK(platform_wearlevel_get_count(&wl, 5u) ==
          (uint32_t)PLATFORM_WEARLEVEL_COUNT_MAX,
          "计数应饱和在 0xFFFF");
    platform_wearlevel_set_count(&wl, 5u, 1u);
}

static void test_wearlevel_filtered(void)
{
    uint16_t counts[4];
    uint8_t flags[4] = { 0u, 1u, 0u, 1u };
    platform_wearlevel_t wl;
    platform_bool_t found;

    CHECK(platform_wearlevel_init(&wl, 0u, 256u, 4u, counts) ==
          PLATFORM_ERR_OK, "磨损均衡初始化应成功");

    platform_wearlevel_set_count(&wl, 0u, 0u);
    platform_wearlevel_set_count(&wl, 1u, 9u);
    platform_wearlevel_set_count(&wl, 2u, 1u);
    platform_wearlevel_set_count(&wl, 3u, 4u);

    /* Globally coldest is 0, but it is not a candidate. */
    CHECK(platform_wearlevel_pick_filtered(&wl, 0u, even_filter, flags,
                                           &found) == 3u,
          "过滤后应选中最冷且被允许的扇区");
    CHECK(found == PLATFORM_TRUE, "应报告找到候选");

    flags[1] = 0u;
    flags[3] = 0u;
    (void)platform_wearlevel_pick_filtered(&wl, 0u, even_filter, flags,
                                           &found);
    CHECK(found == PLATFORM_FALSE, "没有候选时应报告未找到");
}

/* --------------------------------------------------------------------- */
/* Log store                                                             */
/* --------------------------------------------------------------------- */

#define STORE_SECTORS   ((uint32_t)4u)

static platform_logstore_t s_store;
static uint16_t s_counts[STORE_SECTORS];
static uint32_t s_seqs[STORE_SECTORS];

static platform_err_t store_mount(void)
{
    return platform_logstore_init(&s_store, 0u, STORE_SECTORS,
                                  s_counts, s_seqs);
}

static void test_logstore_append_and_iterate(void)
{
    platform_logstore_iter_t iter;
    platform_logstore_entry_t entry;
    uint32_t seq;
    uint32_t seen;

    fake_reset();
    CHECK(store_mount() == PLATFORM_ERR_OK, "日志存储挂载应成功");
    CHECK(platform_logstore_used_bytes(&s_store) == 0u, "空白环已用应为 0");

    CHECK(platform_logstore_append(&s_store, 1u, 2u, 111u,
                                   (const uint8_t *)"first", 5u, &seq) ==
          PLATFORM_ERR_OK, "写入第一条应成功");
    CHECK(seq == 1u, "首条序号应为 1");
    CHECK(platform_logstore_append(&s_store, 2u, 3u, 222u,
                                   (const uint8_t *)"second", 6u, NULL) ==
          PLATFORM_ERR_OK, "写入第二条应成功");

    seen = 0u;
    CHECK(platform_logstore_iter_begin(&s_store, &iter, 0u) ==
          PLATFORM_ERR_OK, "迭代器初始化应成功");
    while (platform_logstore_iter_next(&iter, &entry) == PLATFORM_ERR_OK)
    {
        seen++;
        if (seen == 1u)
        {
            CHECK(entry.seq == 1u, "第一条序号应为 1");
            CHECK(entry.level == 1u, "第一条等级应为 1");
            CHECK(entry.ts == 111u, "第一条时间戳应为 111");
            CHECK((entry.len == 5u) &&
                  (memcmp(entry.payload, "first", 5u) == 0),
                  "第一条负载应为 first");
        }
        else
        {
            CHECK(entry.seq == 2u, "第二条序号应为 2");
            CHECK((entry.len == 6u) &&
                  (memcmp(entry.payload, "second", 6u) == 0),
                  "第二条负载应为 second");
        }
    }
    CHECK(seen == 2u, "应读出 2 条记录");

    /* Remount: the scan must recover the write position and the sequence. */
    s_and_violations = 0u;
    CHECK(store_mount() == PLATFORM_ERR_OK, "重新挂载应成功");
    CHECK(s_store.next_seq == 3u, "重新挂载后下一条序号应为 3");
    CHECK(s_store.allocated_sectors == 1u, "应识别出 1 个已分配扇区");
    CHECK(platform_logstore_append(&s_store, 1u, 2u, 333u,
                                   (const uint8_t *)"third", 5u, &seq) ==
          PLATFORM_ERR_OK, "重新挂载后继续写入应成功");
    CHECK(seq == 3u, "续写序号应为 3");
    CHECK(s_and_violations == 0u, "不得写入未擦除的字节");
}

static void test_logstore_wrap_and_order(void)
{
    uint32_t i;
    uint32_t seq;
    uint32_t seen;
    uint32_t last_seq;
    uint8_t text[8];
    platform_logstore_iter_t iter;
    platform_logstore_entry_t entry;

    fake_reset();
    CHECK(store_mount() == PLATFORM_ERR_OK, "日志存储挂载应成功");

    /* 8-byte payload -> 24-byte record -> exactly 10 records per sector. */
    for (i = 0u; i < 45u; i++)
    {
        memset(text, (int)('A' + (i % 26u)), sizeof(text));
        CHECK(platform_logstore_append(&s_store, 1u, 1u, i, text,
                                       (uint16_t)sizeof(text), &seq) ==
              PLATFORM_ERR_OK, "批量写入应成功");
        CHECK(seq == (i + 1u), "序号应连续递增");
    }

    CHECK(platform_logstore_wrap_count(&s_store) == 1u, "应回卷 1 次");
    CHECK(platform_logstore_allocated_sectors(&s_store) == STORE_SECTORS,
          "四个扇区都应被分配");

    seen = 0u;
    last_seq = 0u;
    CHECK(platform_logstore_iter_begin(&s_store, &iter, 0u) ==
          PLATFORM_ERR_OK, "迭代器初始化应成功");
    while (platform_logstore_iter_next(&iter, &entry) == PLATFORM_ERR_OK)
    {
        CHECK(entry.seq > last_seq, "迭代必须按序号递增");
        last_seq = entry.seq;
        seen++;
    }
    CHECK(seen == 35u, "回卷后应保留 35 条记录");
    CHECK(last_seq == 45u, "最后一条序号应为 45");

    /* from_seq skips the old records. */
    seen = 0u;
    CHECK(platform_logstore_iter_begin(&s_store, &iter, 41u) ==
          PLATFORM_ERR_OK, "迭代器初始化应成功");
    while (platform_logstore_iter_next(&iter, &entry) == PLATFORM_ERR_OK)
    {
        seen++;
    }
    CHECK(seen == 5u, "from_seq=41 应只返回 5 条");

    /* Remount after a full wrap: same picture from flash alone. */
    CHECK(store_mount() == PLATFORM_ERR_OK, "回卷后重新挂载应成功");
    seen = 0u;
    CHECK(platform_logstore_iter_begin(&s_store, &iter, 0u) ==
          PLATFORM_ERR_OK, "迭代器初始化应成功");
    while (platform_logstore_iter_next(&iter, &entry) == PLATFORM_ERR_OK)
    {
        seen++;
    }
    CHECK(seen == 35u, "重新挂载后仍应保留 35 条记录");
}

static void test_logstore_torn_write(void)
{
    uint32_t addr;
    platform_logstore_iter_t iter;
    platform_logstore_entry_t entry;
    uint32_t seen;
    uint32_t seq;

    fake_reset();
    CHECK(store_mount() == PLATFORM_ERR_OK, "日志存储挂载应成功");

    (void)platform_logstore_append(&s_store, 1u, 1u, 1u,
                                   (const uint8_t *)"aaaa", 4u, NULL);
    (void)platform_logstore_append(&s_store, 1u, 1u, 2u,
                                   (const uint8_t *)"bbbb", 4u, NULL);
    (void)platform_logstore_append(&s_store, 1u, 1u, 3u,
                                   (const uint8_t *)"cccc", 4u, NULL);

    /* Corrupt the payload of the last record: a power cut in the middle of
     * the page program looks exactly like this. */
    addr = (s_store.active_sector * FAKE_SECTOR) + s_store.active_offset - 20u;
    s_flash[addr + PLATFORM_LOGSTORE_RECORD_HEADER_SIZE] ^= 0xFFu;

    CHECK(store_mount() == PLATFORM_ERR_OK, "损坏后重新挂载应成功");
    CHECK(s_store.active_sector == PLATFORM_LOGSTORE_NONE,
          "损坏的扇区应被封口");

    seen = 0u;
    CHECK(platform_logstore_iter_begin(&s_store, &iter, 0u) ==
          PLATFORM_ERR_OK, "迭代器初始化应成功");
    while (platform_logstore_iter_next(&iter, &entry) == PLATFORM_ERR_OK)
    {
        seen++;
    }
    CHECK(seen == 2u, "损坏记录之后的数据不应被读出");

    /* Writing continues in a new sector rather than overwriting the torn slot. */
    CHECK(platform_logstore_append(&s_store, 1u, 1u, 4u,
                                   (const uint8_t *)"dddd", 4u, &seq) ==
          PLATFORM_ERR_OK, "封口后继续写入应成功");
    CHECK(seq == 3u, "损坏的记录不占用序号");
    CHECK(s_store.allocated_sectors == 2u, "应启用第二个扇区");
    CHECK(s_and_violations == 0u, "不得写入未擦除的字节");
}

static void test_logstore_clear(void)
{
    uint32_t i;
    uint32_t seen;
    platform_logstore_iter_t iter;
    platform_logstore_entry_t entry;

    fake_reset();
    CHECK(store_mount() == PLATFORM_ERR_OK, "日志存储挂载应成功");

    for (i = 0u; i < 25u; i++)
    {
        (void)platform_logstore_append(&s_store, 1u, 1u, i,
                                       (const uint8_t *)"0123456789", 10u,
                                       NULL);
    }

    CHECK(platform_logstore_clear(&s_store) == PLATFORM_ERR_OK,
          "清空应成功");
    CHECK(platform_logstore_used_bytes(&s_store) == 0u, "清空后已用应为 0");
    CHECK(platform_logstore_allocated_sectors(&s_store) == 0u,
          "清空后不应有已分配扇区");

    CHECK(platform_logstore_iter_begin(&s_store, &iter, 0u) ==
          PLATFORM_ERR_OK, "迭代器初始化应成功");
    seen = 0u;
    while (platform_logstore_iter_next(&iter, &entry) == PLATFORM_ERR_OK)
    {
        seen++;
    }
    CHECK(seen == 0u, "清空后不应读出任何记录");

    /* The ring is usable again and restarts at the coldest free sector. */
    CHECK(platform_logstore_append(&s_store, 1u, 1u, 99u,
                                   (const uint8_t *)"after", 5u, NULL) ==
          PLATFORM_ERR_OK, "清空后写入应成功");
    CHECK(platform_logstore_used_bytes(&s_store) == 24u,
          "清空后已用应只包含新记录");
    CHECK(s_and_violations == 0u, "不得写入未擦除的字节");

    CHECK(store_mount() == PLATFORM_ERR_OK, "清空后重新挂载应成功");
    seen = 0u;
    CHECK(platform_logstore_iter_begin(&s_store, &iter, 0u) ==
          PLATFORM_ERR_OK, "迭代器初始化应成功");
    while (platform_logstore_iter_next(&iter, &entry) == PLATFORM_ERR_OK)
    {
        seen++;
    }
    CHECK(seen == 1u, "清空后重新挂载应只看到 1 条");
}

static void test_logstore_limits(void)
{
    uint8_t big[PLATFORM_LOGSTORE_MAX_PAYLOAD + 1u];

    fake_reset();
    CHECK(store_mount() == PLATFORM_ERR_OK, "日志存储挂载应成功");

    memset(big, 0x5A, sizeof(big));
    CHECK(platform_logstore_append(&s_store, 1u, 1u, 0u, big,
                                   (uint16_t)sizeof(big), NULL) ==
          PLATFORM_ERR_PARAM, "超长负载应被拒绝");
    CHECK(platform_logstore_append(&s_store, 1u, 1u, 0u, NULL, 4u, NULL) ==
          PLATFORM_ERR_PARAM, "负载指针为空应被拒绝");
    CHECK(platform_logstore_append(&s_store, 1u, 1u, 0u, NULL, 0u, NULL) ==
          PLATFORM_ERR_OK, "空负载记录应允许");

    CHECK(platform_logstore_init(&s_store, FAKE_CAPACITY - FAKE_SECTOR,
                                 STORE_SECTORS, s_counts, s_seqs) ==
          PLATFORM_ERR_PARAM, "超出器件容量的分区应被拒绝");
}

static void test_wear_even_after_fill(void)
{
    uint32_t i;
    uint32_t low;
    uint32_t high;

    fake_reset();
    CHECK(store_mount() == PLATFORM_ERR_OK, "日志存储挂载应成功");

    /* Fill the whole ring once so every sector has been erased once:
     * 10-byte payloads -> 28-byte records -> 8 records per sector. */
    for (i = 0u; i < 32u; i++)
    {
        (void)platform_logstore_append(&s_store, 1u, 1u, i,
                                       (const uint8_t *)"0123456789", 10u,
                                       NULL);
    }
    CHECK(platform_logstore_wrap_count(&s_store) == 0u,
          "刚好写满不应计为回卷");

    CHECK(platform_logstore_erase_total(&s_store) == STORE_SECTORS,
          "每个扇区应各擦除一次");

    low = platform_wearlevel_min(&s_store.wearlevel);
    high = platform_wearlevel_max(&s_store.wearlevel);
    CHECK((high - low) <= 1u, "擦除次数应保持均匀");
}

int main(void)
{
    test_crc32_selfcheck();
    test_wearlevel_round_robin();
    test_wearlevel_filtered();
    test_logstore_append_and_iterate();
    test_logstore_wrap_and_order();
    test_logstore_torn_write();
    test_logstore_clear();
    test_logstore_limits();
    test_wear_even_after_fill();

    if (g_failed == 0)
    {
        printf("test_platform_storage: all checks passed\n");
        return 0;
    }

    printf("test_platform_storage: %d check(s) failed\n", g_failed);
    return 1;
}
