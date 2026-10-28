/* L1 unit test for platform_middleware. Runs on the PC, no hardware needed.
 *
 * Covers the shared algorithms every layer above uses: the lock-free ring
 * buffer that carries UART bytes out of the interrupt, CRC-16/CCITT-FALSE, and
 * the filters applied to sampled values.
 *
 * Build (MinGW or any host gcc):
 *   gcc -std=c99 -Wall -Wextra \
 *       -I firmware/app/03_Platform/platform_common \
 *       -I firmware/app/04_Impl \
 *       -I firmware/app/03_Platform/platform_middleware \
 *       tests/test_platform_middleware.c \
 *       firmware/app/03_Platform/platform_middleware/platform_ringbuf.c \
 *       firmware/app/03_Platform/platform_middleware/platform_crc.c \
 *       firmware/app/03_Platform/platform_middleware/platform_filter.c \
 *       -o build/test_platform_middleware
 */

#include <stdio.h>
#include "platform_ringbuf.h"
#include "platform_crc.h"
#include "platform_filter.h"

static int g_failed = 0;

#define CHECK(cond, msg)                                                  \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);        \
            g_failed++;                                                   \
        }                                                                 \
    } while (0)

static void test_crc_selfcheck(void)
{
    CHECK(platform_crc16_ccitt((const uint8_t *)"123456789", 9u) == 0x29B1u,
          "CRC 自检值应为 0x29B1");
}

static void test_crc_protocol_frame(void)
{
    /* LEN..PAYLOAD of the SET_VELOCITY example in docs/protocol.md. */
    const uint8_t body[] = { 0x06, 0x01, 0x01, 0x2C, 0x01, 0xF4, 0x01 };

    CHECK(platform_crc16_ccitt(body, (uint32_t)sizeof(body)) == 0x3F72u,
          "SET_VELOCITY 帧 CRC 应为 0x3F72");

    CHECK(platform_crc16_ccitt_update(PLATFORM_CRC16_CCITT_INIT_VALUE,
                                      body, 3u) ==
          platform_crc16_ccitt(body, 3u),
          "增量 CRC 应与一次性计算一致");
}

static void test_ringbuf_basic(void)
{
    uint8_t storage[8];
    platform_ringbuf_t rb;
    uint8_t out[16];
    const uint8_t pattern[12] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

    CHECK(platform_ringbuf_init(&rb, storage, 8u) == PLATFORM_ERR_OK,
          "环形缓冲区初始化应成功");
    CHECK(platform_ringbuf_capacity(&rb) == 8u, "容量应为 8");
    CHECK(platform_ringbuf_is_empty(&rb) == PLATFORM_TRUE,
          "初始化后应为空");

    CHECK(platform_ringbuf_write(&rb, pattern, 5u) == 5u,
          "应写入 5 字节");
    CHECK(platform_ringbuf_count(&rb) == 5u, "已用应为 5");
    CHECK(platform_ringbuf_free(&rb) == 3u, "剩余应为 3");

    /* Short write is reported instead of silently truncating. */
    CHECK(platform_ringbuf_write(&rb, &pattern[5], 5u) == 3u,
          "空间不足时应只写入 3 字节");
    CHECK(platform_ringbuf_is_full(&rb) == PLATFORM_TRUE, "此时应为满");

    CHECK(platform_ringbuf_read(&rb, out, 4u) == 4u, "应读出 4 字节");
    CHECK((out[0] == 0u) && (out[1] == 1u) && (out[2] == 2u) && (out[3] == 3u),
          "读出的前 4 字节内容应正确");
}

static void test_ringbuf_wrap(void)
{
    uint8_t storage[8];
    platform_ringbuf_t rb;
    uint8_t out[16];
    uint8_t byte = 0u;
    const uint8_t pattern[12] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

    CHECK(platform_ringbuf_init(&rb, storage, 8u) == PLATFORM_ERR_OK,
          "环形缓冲区初始化应成功");

    (void)platform_ringbuf_write(&rb, pattern, 8u);
    (void)platform_ringbuf_read(&rb, out, 4u);

    /* Write past the end of the storage array to force wrapping. */
    CHECK(platform_ringbuf_write(&rb, &pattern[8], 4u) == 4u,
          "回绕前应写入 4 字节");
    CHECK(platform_ringbuf_read(&rb, out, 8u) == 8u, "回绕后应读出 8 字节");
    CHECK((out[0] == 4u) && (out[1] == 5u) && (out[2] == 6u) && (out[3] == 7u),
          "回绕数据前半段应正确");
    CHECK((out[4] == 8u) && (out[5] == 9u) && (out[6] == 10u) && (out[7] == 11u),
          "回绕数据后半段应正确");

    CHECK(platform_ringbuf_read_byte(&rb, &byte) == 0u, "空缓冲区读字节应失败");
    CHECK(platform_ringbuf_write_byte(&rb, 0xAAu) == 1u, "写单字节应成功");
    CHECK(platform_ringbuf_peek(&rb, out, 1u) == 1u, "peek 应返回 1 字节");
    CHECK(out[0] == 0xAAu, "peek 不应移动读指针");
    CHECK(platform_ringbuf_drop(&rb, 1u) == 1u, "drop 应丢弃 1 字节");
    CHECK(platform_ringbuf_is_empty(&rb) == PLATFORM_TRUE,
          "丢弃后应为空");
}

static void test_ringbuf_param(void)
{
    uint8_t storage[8];
    platform_ringbuf_t rb;

    CHECK(platform_ringbuf_init(&rb, storage, 0u) == PLATFORM_ERR_PARAM,
          "容量为 0 应被拒绝");
    CHECK(platform_ringbuf_init(&rb, storage, 6u) == PLATFORM_ERR_PARAM,
          "非 2 的幂容量应被拒绝");
    CHECK(platform_ringbuf_init(NULL, storage, 8u) == PLATFORM_ERR_PARAM,
          "空句柄应被拒绝");
}

static void test_filters(void)
{
    int32_t storage[4];
    platform_filter_moving_average_t ma;
    platform_filter_lowpass_t lp;

    CHECK(platform_filter_moving_average_init(&ma, storage, 4u) ==
          PLATFORM_ERR_OK, "滑动平均初始化应成功");
    CHECK(platform_filter_moving_average_get(&ma) == 0,
          "无样本时输出应为 0");

    (void)platform_filter_moving_average_update(&ma, 1);
    (void)platform_filter_moving_average_update(&ma, 2);
    (void)platform_filter_moving_average_update(&ma, 3);
    CHECK(platform_filter_moving_average_update(&ma, 4) == 2,
          "1,2,3,4 的整数平均应为 2");
    CHECK(platform_filter_moving_average_update(&ma, 5) == 3,
          "窗口滑动后 2,3,4,5 的平均应为 3");

    platform_filter_moving_average_reset(&ma);
    CHECK(platform_filter_moving_average_get(&ma) == 0,
          "reset 后输出应回到 0");

    CHECK(platform_filter_lowpass_init(&lp, 0.5f) == PLATFORM_ERR_OK,
          "低通初始化应成功");
    CHECK(platform_filter_lowpass_update(&lp, 10.0f) == 10.0f,
          "第一个样本应直接作为初值");
    CHECK(platform_filter_lowpass_update(&lp, 20.0f) == 15.0f,
          "alpha=0.5 时第二个样本应为 15");

    CHECK(platform_filter_lowpass_init(&lp, 0.0f) == PLATFORM_ERR_PARAM,
          "alpha=0 应被拒绝");
    CHECK(platform_filter_lowpass_init(&lp, 1.5f) == PLATFORM_ERR_PARAM,
          "alpha>1 应被拒绝");
}

int main(void)
{
    test_crc_selfcheck();
    test_crc_protocol_frame();
    test_ringbuf_basic();
    test_ringbuf_wrap();
    test_ringbuf_param();
    test_filters();

    if (g_failed != 0)
    {
        printf("\n%d 项失败\n", g_failed);
        return 1;
    }

    printf("\n全部通过\n");
    return 0;
}
