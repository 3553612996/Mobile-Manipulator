/* L1 unit test for 06_BSP/bsp_flash_w25q64.c. Runs on the PC, no hardware.
 *
 * The driver only talks to the platform SPI contract, so this test replaces
 * platform_spi_* with an in-memory W25Q64 model:
 *   - 8 MiB array, sector erase sets 0xFF, page program ANDs bits;
 *   - command/address parsing driven by chip-select, like the real chip;
 *   - a status register with BUSY and WEL that takes a couple of polls to
 *     clear, so the driver's wait loops are actually exercised;
 *   - a JEDEC ID and two failure switches (wrong ID, WEL never latches).
 *
 * Build (MinGW or any host gcc):
 *   gcc -std=c99 -Wall -Wextra \
 *       -I firmware/app/03_Platform/platform_common \
 *       -I firmware/app/03_Platform/platform_mcu \
 *       -I firmware/app/03_Platform/platform_bsp \
 *       -I firmware/app/04_Impl \
 *       -I firmware/app/06_BSP \
 *       tests/unit/bsp/test_bsp_flash_w25q64.c \
 *       firmware/app/06_BSP/bsp_flash_w25q64.c \
 *       -o build/test_bsp_flash_w25q64
 */

#include <stdio.h>
#include <string.h>

#include "bsp_flash_w25q64.h"

static int g_failed = 0;

#define CHECK(cond, msg)                                                  \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);        \
            g_failed++;                                                   \
        }                                                                 \
    } while (0)

/* --------------------------------------------------------------------- */
/* In-memory W25Q64                                                      */
/* --------------------------------------------------------------------- */

#define CHIP_CAPACITY   ((uint32_t)0x800000u)
#define CHIP_SECTOR     ((uint32_t)4096u)

static struct
{
    uint8_t         mem[CHIP_CAPACITY];
    uint8_t         command;
    uint32_t        address;
    uint32_t        header_len;
    uint32_t        header_total;
    uint32_t        data_index;
    uint8_t         wel;
    uint32_t        busy_reads;
    uint8_t         jedec[3];
    platform_bool_t jedec_bad;
    platform_bool_t ignore_write_enable;
    uint32_t        status_reads;
    uint32_t        cs_low_count;
} s_chip;

static struct
{
    platform_spi_cfg_t cfg;
    platform_bool_t    open;
    uint32_t           lock_takes;
    uint32_t           lock_releases;
    uint32_t           deinits;
    platform_gpio_level_t cs_level;
} s_bus;

void platform_delay_ms(uint32_t ms)
{
    (void)ms;
}

void platform_delay_us(uint32_t us)
{
    (void)us;
}

static void chip_reset(void)
{
    memset(s_chip.mem, 0xFF, sizeof(s_chip.mem));
    s_chip.command = 0xFFu;
    s_chip.address = 0u;
    s_chip.header_len = 0u;
    s_chip.header_total = 0u;
    s_chip.data_index = 0u;
    s_chip.wel = 0u;
    s_chip.busy_reads = 0u;
    s_chip.jedec[0] = 0xEFu;
    s_chip.jedec[1] = 0x40u;
    s_chip.jedec[2] = 0x17u;
    s_chip.jedec_bad = PLATFORM_FALSE;
    s_chip.ignore_write_enable = PLATFORM_FALSE;
    s_chip.status_reads = 0u;
    s_chip.cs_low_count = 0u;

    memset(&s_bus, 0, sizeof(s_bus));
    s_bus.cs_level = PLATFORM_GPIO_LEVEL_HIGH;
}

/* The driver never asserts CS around a status read? It does; this runs on the
 * falling edge and resets the command parser, exactly like the real device. */
static void chip_end_transaction(void)
{
    if (s_chip.header_len == s_chip.header_total)
    {
        if (s_chip.command == 0x06u)
        {
            /* The real chip latches WEL on the rising edge of CS. */
            if (s_chip.ignore_write_enable == PLATFORM_FALSE)
            {
                s_chip.wel = 1u;
            }
        }
        else if (s_chip.command == 0x02u)
        {
            /* Programming already happened byte by byte. */
            s_chip.wel = 0u;
            s_chip.busy_reads = 2u;
        }
        else if (s_chip.command == 0x20u)
        {
            uint32_t base = s_chip.address & ~(CHIP_SECTOR - 1u);

            if ((base + CHIP_SECTOR) <= CHIP_CAPACITY)
            {
                memset(&s_chip.mem[base], 0xFF, CHIP_SECTOR);
            }
            s_chip.wel = 0u;
            s_chip.busy_reads = 2u;
        }
        else
        {
            /* Read/write-enable/JEDEC: nothing to commit. */
        }
    }

    s_chip.command = 0xFFu;
    s_chip.header_len = 0u;
    s_chip.header_total = 0u;
    s_chip.data_index = 0u;
}

static void chip_header_byte(uint8_t byte)
{
    if (s_chip.header_len == 0u)
    {
        s_chip.command = byte;
        s_chip.header_total =
            ((byte == 0x03u) || (byte == 0x02u) || (byte == 0x20u)) ? 4u : 1u;
        s_chip.address = 0u;
        s_chip.header_len = 1u;
        return;
    }

    if (s_chip.header_len < s_chip.header_total)
    {
        s_chip.address = (s_chip.address << 8) | (uint32_t)byte;
        s_chip.header_len++;
    }
}

static uint8_t chip_read_byte(void)
{
    if (s_chip.header_len < s_chip.header_total)
    {
        return 0xFFu;
    }

    if (s_chip.command == 0x05u)
    {
        uint8_t status;

        s_chip.status_reads++;

        if (s_chip.busy_reads > 0u)
        {
            s_chip.busy_reads--;
            return 0x01u;       /* BUSY */
        }

        status = (s_chip.wel != 0u) ? 0x02u : 0x00u;
        return status;
    }

    if (s_chip.command == 0x9Fu)
    {
        uint8_t value = 0xFFu;

        if (s_chip.data_index < 3u)
        {
            value = (s_chip.jedec_bad == PLATFORM_TRUE)
                        ? 0x00u : s_chip.jedec[s_chip.data_index];
        }

        s_chip.data_index++;
        return value;
    }

    if (s_chip.command == 0x03u)
    {
        uint32_t index = s_chip.address + s_chip.data_index;
        uint8_t  value = 0xFFu;

        if (index < CHIP_CAPACITY)
        {
            value = s_chip.mem[index];
        }

        s_chip.data_index++;
        return value;
    }

    return 0xFFu;
}

/* --------------------------- platform SPI ----------------------------- */

platform_err_t platform_spi_init(const platform_spi_cfg_t *cfg)
{
    if (cfg == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    s_bus.cfg = *cfg;
    s_bus.open = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_spi_deinit(platform_spi_bus_t bus)
{
    (void)bus;

    s_bus.open = PLATFORM_FALSE;
    s_bus.deinits++;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_spi_transfer(platform_spi_bus_t bus,
                                     const uint8_t *tx_data,
                                     uint8_t *rx_data,
                                     uint32_t len,
                                     uint32_t timeout_ms)
{
    (void)bus;
    (void)tx_data;
    (void)rx_data;
    (void)len;
    (void)timeout_ms;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t platform_spi_write(platform_spi_bus_t bus,
                                  const uint8_t *tx_data,
                                  uint32_t len,
                                  uint32_t timeout_ms)
{
    uint32_t i;

    (void)bus;
    (void)timeout_ms;

    if ((tx_data == NULL) || (s_bus.open == PLATFORM_FALSE))
    {
        return PLATFORM_ERR_PARAM;
    }

    for (i = 0u; i < len; i++)
    {
        uint8_t byte = tx_data[i];

        /* header_len == 0 means the previous byte began a new transaction. */
        if ((s_chip.header_len == 0u) ||
            (s_chip.header_len < s_chip.header_total))
        {
            chip_header_byte(byte);
            continue;
        }

        if (s_chip.command == 0x06u)
        {
            /* Write-enable carries no data; WEL latches on CS high. */
        }
        else if (s_chip.command == 0x02u)
        {
            uint32_t index = s_chip.address + s_chip.data_index;

            if (index < CHIP_CAPACITY)
            {
                s_chip.mem[index] &= byte;
            }
            s_chip.data_index++;
        }
        else
        {
            s_chip.data_index++;
        }
    }

    return PLATFORM_ERR_OK;
}

platform_err_t platform_spi_read(platform_spi_bus_t bus,
                                 uint8_t *rx_data,
                                 uint32_t len,
                                 uint32_t timeout_ms)
{
    uint32_t i;

    (void)bus;
    (void)timeout_ms;

    if ((rx_data == NULL) || (s_bus.open == PLATFORM_FALSE))
    {
        return PLATFORM_ERR_PARAM;
    }

    for (i = 0u; i < len; i++)
    {
        rx_data[i] = chip_read_byte();
    }

    return PLATFORM_ERR_OK;
}

platform_err_t platform_spi_write_async(platform_spi_bus_t bus,
                                        const uint8_t *tx_data,
                                        uint32_t len)
{
    (void)bus;
    (void)tx_data;
    (void)len;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t platform_spi_register_complete_cb(
                                        platform_spi_bus_t bus,
                                        platform_spi_complete_cb_t callback,
                                        void *user_data)
{
    (void)bus;
    (void)callback;
    (void)user_data;

    return PLATFORM_ERR_NOT_SUPPORTED;
}

platform_err_t platform_spi_cs_set(platform_spi_bus_t bus,
                                   platform_gpio_level_t level)
{
    (void)bus;

    if (s_bus.open == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if (level == PLATFORM_GPIO_LEVEL_LOW)
    {
        s_chip.cs_low_count++;
        s_chip.command = 0xFFu;
        s_chip.header_len = 0u;
        s_chip.header_total = 0u;
        s_chip.data_index = 0u;
    }
    else
    {
        chip_end_transaction();
    }

    s_bus.cs_level = level;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_spi_lock(platform_spi_bus_t bus, uint32_t timeout_ms)
{
    (void)bus;
    (void)timeout_ms;

    s_bus.lock_takes++;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_spi_unlock(platform_spi_bus_t bus)
{
    (void)bus;

    s_bus.lock_releases++;

    return PLATFORM_ERR_OK;
}

/* --------------------------------------------------------------------- */
/* Helpers                                                               */
/* --------------------------------------------------------------------- */

static void make_flash(bsp_flash_w25q64_t *flash)
{
    memset(flash, 0, sizeof(*flash));
    flash->cfg.bus                 = PLATFORM_SPI_BUS_0;
    flash->cfg.cs_pin.port         = 1u;
    flash->cfg.cs_pin.pin          = 12u;
    flash->cfg.max_frequency_hz    = BSP_FLASH_W25Q64_DEFAULT_HZ;
    flash->cfg.transfer_timeout_ms = 100u;
}

/* --------------------------------------------------------------------- */
/* Tests                                                                 */
/* --------------------------------------------------------------------- */

static void test_init(void)
{
    bsp_flash_w25q64_t flash;
    platform_flash_info_t info;

    chip_reset();
    make_flash(&flash);

    CHECK(bsp_flash_w25q64_init(&flash) == PLATFORM_ERR_OK, "初始化应成功");
    CHECK(flash.initialized == PLATFORM_TRUE, "初始化后应置位");
    CHECK((flash.jedec_id[0] == 0xEFu) && (flash.jedec_id[1] == 0x40u) &&
          (flash.jedec_id[2] == 0x17u), "应读到 EF 40 17");

    CHECK(s_bus.cfg.bus == PLATFORM_SPI_BUS_0, "应使用逻辑总线 0");
    CHECK(s_bus.cfg.mode == PLATFORM_SPI_MODE_0, "应为 SPI 模式 0");
    CHECK(s_bus.cfg.data_width == PLATFORM_SPI_DATA_8BIT, "应为 8 位数据");
    CHECK(s_bus.cfg.msb_first == PLATFORM_TRUE, "应高位先发");
    CHECK(s_bus.cfg.cs_mode == PLATFORM_SPI_CS_SOFTWARE, "应软件片选");
    CHECK((s_bus.cfg.cs_pin.port == 1u) && (s_bus.cfg.cs_pin.pin == 12u),
          "片选应为 PB12");
    CHECK(s_bus.cfg.max_frequency_hz == BSP_FLASH_W25Q64_DEFAULT_HZ,
          "应使用默认 4.5 MHz");
    CHECK(s_bus.cs_level == PLATFORM_GPIO_LEVEL_HIGH, "初始片选应为高");

    CHECK(bsp_flash_w25q64_get_info(&flash, &info) == PLATFORM_ERR_OK,
          "取几何参数应成功");
    CHECK(info.capacity_bytes == CHIP_CAPACITY, "容量应为 8 MiB");
    CHECK(info.sector_size_bytes == 4096u, "扇区应为 4 KiB");
    CHECK(info.page_size_bytes == 256u, "页应为 256 B");
    CHECK((info.manufacturer_id == 0x00EFu) && (info.device_id == 0x4017u),
          "ID 字段应为 EF / 4017");

    CHECK(bsp_flash_w25q64_init(&flash) == PLATFORM_ERR_ALREADY_INIT,
          "重复初始化应被拒绝");

    CHECK(bsp_flash_w25q64_deinit(&flash) == PLATFORM_ERR_OK, "反初始化应成功");
    CHECK(s_bus.deinits == 1u, "应释放一次 SPI");
}

static void test_init_wrong_id(void)
{
    bsp_flash_w25q64_t flash;

    chip_reset();
    s_chip.jedec_bad = PLATFORM_TRUE;
    make_flash(&flash);

    CHECK(bsp_flash_w25q64_init(&flash) == PLATFORM_ERR_FAIL,
          "ID 不符时初始化应失败");
    CHECK(flash.initialized == PLATFORM_FALSE, "失败后不应置位");
    CHECK(s_bus.deinits == 1u, "失败时应释放 SPI");
}

static void test_read(void)
{
    bsp_flash_w25q64_t flash;
    uint8_t out[32];
    uint32_t i;

    chip_reset();
    make_flash(&flash);
    CHECK(bsp_flash_w25q64_init(&flash) == PLATFORM_ERR_OK, "初始化应成功");

    for (i = 0u; i < sizeof(out); i++)
    {
        s_chip.mem[0x1000u + i] = (uint8_t)(0xA0u + i);
    }

    memset(out, 0, sizeof(out));
    CHECK(bsp_flash_w25q64_read(&flash, 0x1000u, out, sizeof(out)) ==
          PLATFORM_ERR_OK, "读应成功");
    for (i = 0u; i < sizeof(out); i++)
    {
        if (out[i] != (uint8_t)(0xA0u + i))
        {
            CHECK(0, "读回内容应一致");
            break;
        }
    }

    CHECK(bsp_flash_w25q64_read(&flash, CHIP_CAPACITY - 4u, out, 8u) ==
          PLATFORM_ERR_PARAM, "越界读应被拒绝");
    CHECK(bsp_flash_w25q64_read(&flash, 0x10u, NULL, 4u) ==
          PLATFORM_ERR_PARAM, "空指针应被拒绝");

    CHECK(bsp_flash_w25q64_deinit(&flash) == PLATFORM_ERR_OK, "反初始化应成功");
}

static void test_write_and_erase(void)
{
    bsp_flash_w25q64_t flash;
    uint8_t data[16];
    uint8_t out[16];
    uint32_t i;

    chip_reset();
    make_flash(&flash);
    CHECK(bsp_flash_w25q64_init(&flash) == PLATFORM_ERR_OK, "初始化应成功");

    /* Dirty the sector first, then erase it and check it reads 0xFF. */
    memset(s_chip.mem + 0x2000u, 0x00, 4096u);
    CHECK(bsp_flash_w25q64_erase(&flash, 0x2000u, 4096u) == PLATFORM_ERR_OK,
          "扇区擦除应成功");
    CHECK(s_chip.mem[0x2000u] == 0xFFu, "擦除后首字节应为 0xFF");
    CHECK(s_chip.mem[0x2FFFu] == 0xFFu, "擦除后末字节应为 0xFF");
    CHECK(s_chip.status_reads > 0u, "擦除应轮询状态寄存器");

    for (i = 0u; i < sizeof(data); i++)
    {
        data[i] = (uint8_t)(i + 1u);
    }

    s_chip.status_reads = 0u;
    CHECK(bsp_flash_w25q64_write(&flash, 0x2010u, data, sizeof(data)) ==
          PLATFORM_ERR_OK, "页内写应成功");
    CHECK(s_chip.status_reads >= 2u, "写前应确认 WEL、写后应等待 BUSY");

    memset(out, 0, sizeof(out));
    CHECK(bsp_flash_w25q64_read(&flash, 0x2010u, out, sizeof(out)) ==
          PLATFORM_ERR_OK, "写后读应成功");
    CHECK(memcmp(out, data, sizeof(data)) == 0, "写后内容应一致");
    CHECK(s_chip.mem[0x200Fu] == 0xFFu, "页前字节不应被改");

    /* Programming is an AND: writing over data can only clear bits. */
    CHECK(bsp_flash_w25q64_write(&flash, 0x2010u, data, sizeof(data)) ==
          PLATFORM_ERR_OK, "重复写应成功");
    CHECK(s_chip.mem[0x2010u] == data[0], "按位与后内容应不变");

    /* Page crossing is rejected, not split. */
    CHECK(bsp_flash_w25q64_write(&flash, 0x20F8u, data, sizeof(data)) ==
          PLATFORM_ERR_PARAM, "跨页写应被拒绝");
    CHECK(bsp_flash_w25q64_write(&flash, 0x2010u, data, 0u) ==
          PLATFORM_ERR_OK, "零长度写应成功");
    CHECK(bsp_flash_w25q64_write(&flash, 0x2010u, NULL, 4u) ==
          PLATFORM_ERR_PARAM, "空指针应被拒绝");

    /* Erase alignment is enforced. */
    CHECK(bsp_flash_w25q64_erase(&flash, 0x2001u, 4096u) ==
          PLATFORM_ERR_PARAM, "未对齐擦除应被拒绝");
    CHECK(bsp_flash_w25q64_erase(&flash, 0x2000u, 100u) ==
          PLATFORM_ERR_PARAM, "非整扇区长度应被拒绝");
    CHECK(bsp_flash_w25q64_erase(&flash, CHIP_CAPACITY - 4096u, 8192u) ==
          PLATFORM_ERR_PARAM, "越界擦除应被拒绝");

    CHECK(bsp_flash_w25q64_deinit(&flash) == PLATFORM_ERR_OK, "反初始化应成功");
}

static void test_write_enable_failure(void)
{
    bsp_flash_w25q64_t flash;
    uint8_t data[4] = { 1u, 2u, 3u, 4u };

    chip_reset();
    make_flash(&flash);
    CHECK(bsp_flash_w25q64_init(&flash) == PLATFORM_ERR_OK, "初始化应成功");

    s_chip.ignore_write_enable = PLATFORM_TRUE;

    CHECK(bsp_flash_w25q64_write(&flash, 0x3000u, data, sizeof(data)) ==
          PLATFORM_ERR_TIMEOUT, "WEL 不置位时写应超时");
    CHECK(bsp_flash_w25q64_erase(&flash, 0x3000u, 4096u) ==
          PLATFORM_ERR_TIMEOUT, "WEL 不置位时擦除应超时");

    CHECK(bsp_flash_w25q64_deinit(&flash) == PLATFORM_ERR_OK, "反初始化应成功");
}

static void test_bus_usage(void)
{
    bsp_flash_w25q64_t flash;
    uint8_t out[4];

    chip_reset();
    make_flash(&flash);
    CHECK(bsp_flash_w25q64_init(&flash) == PLATFORM_ERR_OK, "初始化应成功");

    CHECK(bsp_flash_w25q64_read(&flash, 0x0u, out, sizeof(out)) ==
          PLATFORM_ERR_OK, "读应成功");
    CHECK((s_bus.lock_takes == 1u) && (s_bus.lock_releases == 1u),
          "一次读应加解锁各一次");

    CHECK(bsp_flash_w25q64_deinit(&flash) == PLATFORM_ERR_OK, "反初始化应成功");
    CHECK(bsp_flash_w25q64_read(&flash, 0x0u, out, sizeof(out)) ==
          PLATFORM_ERR_NOT_INITIALIZED, "反初始化后读应报未初始化");
    CHECK(bsp_flash_w25q64_get_info(&flash, NULL) == PLATFORM_ERR_PARAM,
          "空信息指针应被拒绝");
}

int main(void)
{
    test_init();
    test_init_wrong_id();
    test_read();
    test_write_and_erase();
    test_write_enable_failure();
    test_bus_usage();

    if (g_failed == 0)
    {
        printf("test_bsp_flash_w25q64: all checks passed\n");
        return 0;
    }

    printf("test_bsp_flash_w25q64: %d check(s) failed\n", g_failed);
    return 1;
}
