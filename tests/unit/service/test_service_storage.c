/* L1 unit test for service_storage. Runs on the PC against an 8 MiB RAM fake
 * that follows the W25Q64 rules: erase sets 0xFF, programming can only clear
 * bits, and crossing a 256 B page is rejected.
 *
 * Build (MinGW or any host gcc):
 *   gcc -std=c99 -Wall -Wextra \
 *       -I firmware/app/03_Platform/platform_common \
 *       -I firmware/app/03_Platform/platform_bsp \
 *       -I firmware/app/03_Platform/platform_middleware \
 *       -I firmware/app/04_Impl \
 *       -I firmware/app/02_Service/service_storage \
 *       tests/unit/service/test_service_storage.c \
 *       firmware/app/02_Service/service_storage/service_storage.c \
 *       firmware/app/03_Platform/platform_middleware/platform_logstore.c \
 *       firmware/app/03_Platform/platform_middleware/platform_wearlevel.c \
 *       firmware/app/03_Platform/platform_middleware/platform_crc.c \
 *       -o build/test_service_storage
 */

#include <stdio.h>
#include <string.h>

#include "service_storage.h"

static int g_failed = 0;

#define CHECK(cond, msg)                                                  \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);        \
            g_failed++;                                                   \
        }                                                                 \
    } while (0)

/* --------------------------------------------------------------------- */
/* 8 MiB fake W25Q64                                                     */
/* --------------------------------------------------------------------- */

#define FAKE_CAPACITY   ((uint32_t)0x800000u)
#define FAKE_SECTOR     ((uint32_t)4096u)
#define FAKE_PAGE       ((uint32_t)256u)

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
        return PLATFORM_ERR_PARAM;
    }

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
    if (((address % FAKE_SECTOR) != 0u) || ((length % FAKE_SECTOR) != 0u))
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
/* Blobs                                                                 */
/* --------------------------------------------------------------------- */

static void test_blank_board(void)
{
    service_storage_status_t status;

    fake_reset();
    CHECK(service_storage_init(1000u) == PLATFORM_ERR_OK, "空白板上初始化应成功");
    CHECK(service_storage_blob_valid(SERVICE_STORAGE_BLOB_PARAM) ==
          PLATFORM_FALSE, "空白板参数区应无效");
    CHECK(service_storage_blob_valid(SERVICE_STORAGE_BLOB_CALIB) ==
          PLATFORM_FALSE, "空白板标定区应无效");

    service_storage_get_status(&status);
    CHECK(status.boot_total == 1u, "首次启动计数应为 1");
    CHECK((status.flags & SERVICE_STORAGE_FLAG_PARAM_VALID) == 0u,
          "REPORT_STORAGE 参数有效位应为 0");
    CHECK((status.flags & SERVICE_STORAGE_FLAG_SAVE_FAILED) == 0u,
          "未保存过不应置保存失败位");
    CHECK(status.log_used == 0u, "日志已用应为 0");
    CHECK(status.param_seq == 0u, "空白板参数序号应为 0");

    CHECK(service_storage_deinit() == PLATFORM_ERR_OK, "反初始化应成功");
}

static void test_blob_save_and_load(void)
{
    uint8_t in[32];
    uint8_t out[32];
    uint16_t length;
    uint32_t seq;
    static const char first[] = "first-parameter-set";
    static const char second[] = "second-parameter-set";

    fake_reset();
    CHECK(service_storage_init(1000u) == PLATFORM_ERR_OK, "初始化应成功");

    memset(in, 0, sizeof(in));
    memcpy(in, first, sizeof(first));

    CHECK(service_storage_save_blob(SERVICE_STORAGE_BLOB_PARAM, in,
                                    (uint16_t)sizeof(first), 2000u) ==
          PLATFORM_ERR_OK, "首次保存参数应成功");
    CHECK(service_storage_blob_valid(SERVICE_STORAGE_BLOB_PARAM) ==
          PLATFORM_TRUE, "保存后参数区应有效");
    CHECK(service_storage_blob_seq(SERVICE_STORAGE_BLOB_PARAM) == 1u,
          "首次保存序号应为 1");

    memset(out, 0, sizeof(out));
    CHECK(service_storage_load_blob(SERVICE_STORAGE_BLOB_PARAM, out,
                                    (uint16_t)sizeof(out), &length, &seq) ==
          PLATFORM_ERR_OK, "读取参数应成功");
    CHECK((length == sizeof(first)) && (memcmp(out, first, length) == 0),
          "读回内容应与写入一致");
    CHECK(seq == 1u, "读回序号应为 1");

    /* Second save must go to the other copy. */
    memset(in, 0, sizeof(in));
    memcpy(in, second, sizeof(second));
    CHECK(service_storage_save_blob(SERVICE_STORAGE_BLOB_PARAM, in,
                                    (uint16_t)sizeof(second), 3000u) ==
          PLATFORM_ERR_OK, "第二次保存参数应成功");
    CHECK(service_storage_blob_seq(SERVICE_STORAGE_BLOB_PARAM) == 2u,
          "第二次保存序号应为 2");

    memset(out, 0, sizeof(out));
    CHECK(service_storage_load_blob(SERVICE_STORAGE_BLOB_PARAM, out,
                                    (uint16_t)sizeof(out), &length, &seq) ==
          PLATFORM_ERR_OK, "第二次读取应成功");
    CHECK((length == sizeof(second)) && (memcmp(out, second, length) == 0),
          "读回内容应为第二次写入的数据");
    CHECK(seq == 2u, "读回序号应为 2");

    /* Corrupt the newer copy: the reader must fall back to the older one. */
    s_flash[SERVICE_STORAGE_PARAM_B_ADDRESS +
            SERVICE_STORAGE_RECORD_HEADER_SIZE + 3u] ^= 0xFFu;

    memset(out, 0, sizeof(out));
    CHECK(service_storage_load_blob(SERVICE_STORAGE_BLOB_PARAM, out,
                                    (uint16_t)sizeof(out), &length, &seq) ==
          PLATFORM_ERR_OK, "新副本损坏时应回退到旧副本");
    CHECK((length == sizeof(first)) && (memcmp(out, first, length) == 0),
          "回退后内容应为第一次写入的数据");
    CHECK(seq == 1u, "回退后序号应为 1");

    /* Reboot: the same picture must come back from flash. */
    CHECK(service_storage_deinit() == PLATFORM_ERR_OK, "反初始化应成功");
    CHECK(service_storage_init(5000u) == PLATFORM_ERR_OK, "重启初始化应成功");
    CHECK(service_storage_blob_valid(SERVICE_STORAGE_BLOB_PARAM) ==
          PLATFORM_TRUE, "重启后参数区应有效");
    CHECK(service_storage_blob_seq(SERVICE_STORAGE_BLOB_PARAM) == 1u,
          "重启后序号应仍为 1");

    memset(out, 0, sizeof(out));
    CHECK(service_storage_load_blob(SERVICE_STORAGE_BLOB_PARAM, out,
                                    (uint16_t)sizeof(out), &length, &seq) ==
          PLATFORM_ERR_OK, "重启后读取应成功");
    CHECK(memcmp(out, first, length) == 0, "重启后内容应一致");

    /* Calibration is a separate region and stays untouched. */
    CHECK(service_storage_blob_valid(SERVICE_STORAGE_BLOB_CALIB) ==
          PLATFORM_FALSE, "标定区不应跟随参数区变化");

    CHECK(s_and_violations == 0u, "不得写入未擦除的字节");
    CHECK(service_storage_deinit() == PLATFORM_ERR_OK, "反初始化应成功");
}

static void test_blob_limits(void)
{
    uint8_t data[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };

    fake_reset();
    CHECK(service_storage_init(1000u) == PLATFORM_ERR_OK, "初始化应成功");
    CHECK(service_storage_save_blob(SERVICE_STORAGE_BLOB_PARAM, data, 0u,
                                    1000u) == PLATFORM_ERR_PARAM,
          "空负载应被拒绝");
    CHECK(service_storage_save_blob(SERVICE_STORAGE_BLOB_COUNT, data, 8u,
                                    1000u) == PLATFORM_ERR_PARAM,
          "非法区域应被拒绝");
    CHECK(service_storage_deinit() == PLATFORM_ERR_OK, "反初始化应成功");
}

/* --------------------------------------------------------------------- */
/* Rings                                                                 */
/* --------------------------------------------------------------------- */

static void test_ring_append_and_clear(void)
{
    platform_logstore_iter_t iter;
    platform_logstore_entry_t entry;
    uint32_t seen;
    service_storage_status_t status;

    fake_reset();
    CHECK(service_storage_init(1000u) == PLATFORM_ERR_OK, "初始化应成功");

    CHECK(service_storage_append(SERVICE_STORAGE_RING_LOG, 1u, 2u,
                                 "boot done", 9u, 2500u) ==
          PLATFORM_ERR_OK, "写日志应成功");
    CHECK(service_storage_append(SERVICE_STORAGE_RING_LOG, 2u, 2u,
                                 "warn", 4u, 3000u) == PLATFORM_ERR_OK,
          "写第二条日志应成功");

    seen = 0u;
    CHECK(service_storage_iter_begin(SERVICE_STORAGE_RING_LOG, &iter, 0u) ==
          PLATFORM_ERR_OK, "迭代器初始化应成功");
    while (platform_logstore_iter_next(&iter, &entry) == PLATFORM_ERR_OK)
    {
        seen++;
        if (seen == 2u)
        {
            CHECK((entry.len == 4u) &&
                  (memcmp(entry.payload, "warn", 4u) == 0),
                  "第二条日志内容应为 warn");
            CHECK(entry.ts == 3u, "日志时间戳应为 3 s");
        }
    }
    CHECK(seen == 2u, "应读出两条日志");

    CHECK(service_storage_ring_used(SERVICE_STORAGE_RING_LOG) > 0u,
          "日志已用应大于 0");

    CHECK(service_storage_ring_clear(SERVICE_STORAGE_RING_LOG, 4000u) ==
          PLATFORM_ERR_OK, "清空日志应成功");
    CHECK(service_storage_ring_used(SERVICE_STORAGE_RING_LOG) == 0u,
          "清空后日志已用应为 0");

    seen = 0u;
    CHECK(service_storage_iter_begin(SERVICE_STORAGE_RING_LOG, &iter, 0u) ==
          PLATFORM_ERR_OK, "清空后迭代器初始化应成功");
    while (platform_logstore_iter_next(&iter, &entry) == PLATFORM_ERR_OK)
    {
        seen++;
    }
    CHECK(seen == 0u, "清空后不应读出记录");

    /* Fault ring is independent. */
    CHECK(service_storage_append(SERVICE_STORAGE_RING_FAULT, 2u, 4u,
                                 "low battery", 11u, 5000u) ==
          PLATFORM_ERR_OK, "写故障记录应成功");
    CHECK(service_storage_ring_used(SERVICE_STORAGE_RING_FAULT) > 0u,
          "故障环应有数据");
    CHECK(service_storage_ring_used(SERVICE_STORAGE_RING_LOG) == 0u,
          "清空日志不应影响故障环");

    service_storage_get_status(&status);
    CHECK(status.log_used == 0u, "状态中日志已用应为 0");
    CHECK((status.flags & SERVICE_STORAGE_FLAG_LOG_WRAPPED) == 0u,
          "未回卷不应置位");

    CHECK(service_storage_deinit() == PLATFORM_ERR_OK, "反初始化应成功");
}

/* --------------------------------------------------------------------- */
/* Crash slots                                                           */
/* --------------------------------------------------------------------- */

static void test_crash_slot(void)
{
    uint8_t scene[100];
    uint8_t out[64];
    uint16_t length;
    uint16_t total;
    uint8_t flags;
    uint32_t i;

    fake_reset();
    CHECK(service_storage_init(1000u) == PLATFORM_ERR_OK, "初始化应成功");
    CHECK(service_storage_crash_unread() == PLATFORM_FALSE,
          "空白板不应有未读崩溃");

    for (i = 0u; i < sizeof(scene); i++)
    {
        scene[i] = (uint8_t)(i * 7u);
    }

    CHECK(service_storage_crash_write(scene, (uint16_t)sizeof(scene),
                                      7000u) == PLATFORM_ERR_OK,
          "写崩溃现场应成功");
    CHECK(service_storage_crash_unread() == PLATFORM_TRUE,
          "写后应报告未读");

    CHECK(service_storage_crash_info(0u, &total, &flags) == PLATFORM_ERR_OK,
          "读取崩溃信息应成功");
    CHECK(total == sizeof(scene), "崩溃长度应为 100");
    CHECK((flags & SERVICE_STORAGE_RECORD_FLAG_UNREAD) != 0u,
          "崩溃标志应为未读");

    CHECK(service_storage_crash_read(0u, 0u, out, 32u, &length) ==
          PLATFORM_ERR_OK, "读第一片应成功");
    CHECK((length == 32u) && (memcmp(out, scene, 32u) == 0),
          "第一片内容应一致");

    CHECK(service_storage_crash_read(0u, 32u, out, 64u, &length) ==
          PLATFORM_ERR_OK, "读第二片应成功");
    CHECK((length == 64u) && (memcmp(out, &scene[32], 64u) == 0),
          "第二片内容应一致");

    CHECK(service_storage_crash_read(0u, 96u, out, 64u, &length) ==
          PLATFORM_ERR_OK, "读尾片应成功");
    CHECK((length == 4u) && (memcmp(out, &scene[96], 4u) == 0),
          "尾片应截断到剩余长度");

    CHECK(service_storage_crash_mark_read(0u) == PLATFORM_ERR_OK,
          "标记已读应成功");
    CHECK(service_storage_crash_unread() == PLATFORM_FALSE,
          "标记后不应再有未读");

    /*
     * Reboot must re-verify the record from flash. The in-session cache hides
     * a wrong header CRC, so this is the only case that proves the commit
     * marker and the header field agree.
     */
    CHECK(service_storage_deinit() == PLATFORM_ERR_OK, "反初始化应成功");
    CHECK(service_storage_init(8000u) == PLATFORM_ERR_OK, "重启初始化应成功");
    CHECK(service_storage_crash_info(0u, &total, &flags) == PLATFORM_ERR_OK,
          "重启后崩溃记录仍应有效");
    CHECK(total == sizeof(scene), "重启后长度应不变");
    CHECK((flags & SERVICE_STORAGE_RECORD_FLAG_UNREAD) == 0u,
          "已读标志应跨重启保留");
    CHECK(service_storage_crash_read(0u, 0u, out, 32u, &length) ==
          PLATFORM_ERR_OK, "重启后读崩溃记录应成功");
    CHECK((length == 32u) && (memcmp(out, scene, 32u) == 0),
          "重启后内容应一致");

    /* A world without the commit CRC must not count as a crash scene. */
    s_flash[SERVICE_STORAGE_CRASH_ADDRESS + FAKE_SECTOR - 4u] = 0x00u;
    CHECK(service_storage_crash_info(0u, &total, NULL) ==
          PLATFORM_ERR_NO_RESOURCE, "缺少提交 CRC 的记录应作废");

    /* Reboot sees no valid slot. */
    CHECK(service_storage_deinit() == PLATFORM_ERR_OK, "反初始化应成功");
    CHECK(service_storage_init(9000u) == PLATFORM_ERR_OK, "再次初始化应成功");
    CHECK(service_storage_crash_unread() == PLATFORM_FALSE,
          "重启后不应有未读崩溃");
    CHECK(service_storage_crash_info(0u, &total, NULL) ==
          PLATFORM_ERR_NO_RESOURCE, "提交 CRC 损坏后重启应作废");

    CHECK(service_storage_deinit() == PLATFORM_ERR_OK, "反初始化应成功");
}

/* --------------------------------------------------------------------- */
/* Boot counter and wear statistics                                      */
/* --------------------------------------------------------------------- */

static void test_boot_counter_and_wear(void)
{
    uint8_t data[16];
    service_storage_status_t status;
    uint32_t i;

    fake_reset();
    memset(data, 0x11, sizeof(data));

    for (i = 1u; i <= 3u; i++)
    {
        CHECK(service_storage_init(i * 1000u) == PLATFORM_ERR_OK,
              "第 N 次启动初始化应成功");
        service_storage_get_status(&status);
        CHECK(status.boot_total == i, "启动计数应逐次加一");
        CHECK(service_storage_deinit() == PLATFORM_ERR_OK,
              "反初始化应成功");
    }

    CHECK(service_storage_init(10000u) == PLATFORM_ERR_OK, "初始化应成功");
    CHECK(service_storage_save_blob(SERVICE_STORAGE_BLOB_CALIB, data,
                                    (uint16_t)sizeof(data), 10500u) ==
          PLATFORM_ERR_OK, "保存标定应成功");
    CHECK(service_storage_flush_wear(11000u) == PLATFORM_ERR_OK,
          "落盘寿命统计应成功");
    CHECK(service_storage_deinit() == PLATFORM_ERR_OK, "反初始化应成功");
    CHECK(service_storage_init(12000u) == PLATFORM_ERR_OK,
          "保存后重启初始化应成功");
    CHECK(service_storage_blob_valid(SERVICE_STORAGE_BLOB_CALIB) ==
          PLATFORM_TRUE, "重启后标定区应有效");
    CHECK(service_storage_blob_seq(SERVICE_STORAGE_BLOB_CALIB) == 1u,
          "重启后标定序号应为 1");

    service_storage_get_status(&status);
    CHECK(status.boot_total == 5u, "启动计数应连续累加");
    CHECK(status.last_save_s != 0u, "保存时间应已记录");

    CHECK(s_and_violations == 0u, "不得写入未擦除的字节");
    CHECK(service_storage_deinit() == PLATFORM_ERR_OK, "反初始化应成功");
}

int main(void)
{
    test_blank_board();
    test_blob_save_and_load();
    test_blob_limits();
    test_ring_append_and_clear();
    test_crash_slot();
    test_boot_counter_and_wear();

    if (g_failed == 0)
    {
        printf("test_service_storage: all checks passed\n");
        return 0;
    }

    printf("test_service_storage: %d check(s) failed\n", g_failed);
    return 1;
}
