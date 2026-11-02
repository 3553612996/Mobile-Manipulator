/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_crc.c
 *
 * @par dependencies
 * - platform_crc.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement CRC-16/CCITT-FALSE and CRC-32/ISO-HDLC.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "platform_crc.h"
#include "platform_def.h"

//******************************** Functions ********************************//

uint16_t platform_crc16_ccitt_update(uint16_t crc,
                                     const uint8_t *data,
                                     uint32_t len)
{
    uint32_t i;
    uint8_t bit;

    if (data == NULL)
    {
        return crc;
    }

    for (i = 0u; i < len; i++)
    {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);

        for (bit = 0u; bit < 8u; bit++)
        {
            if ((crc & 0x8000u) != 0u)
            {
                crc = (uint16_t)((crc << 1) ^ PLATFORM_CRC16_CCITT_POLYNOMIAL);
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }

    return crc;
}

uint16_t platform_crc16_ccitt(const uint8_t *data, uint32_t len)
{
    return platform_crc16_ccitt_update(PLATFORM_CRC16_CCITT_INIT_VALUE,
                                       data,
                                       len);
}

/*
 * 16-entry nibble table for the reflected polynomial. One table lookup per
 * four bits costs 16 words of flash instead of 256 and is still about four
 * times faster than the bit-at-a-time loop; a 32 KB parameter record then
 * takes roughly 2 ms at 72 MHz, which matters because the storage task writes
 * while the control loop is running.
 */
static const uint32_t s_crc32_nibble_table[16] =
{
    0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu,
    0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
    0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu,
    0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu
};

uint32_t platform_crc32_update(uint32_t crc,
                               const uint8_t *data,
                               uint32_t len)
{
    uint32_t i;

    if (data == NULL)
    {
        return crc;
    }

    for (i = 0u; i < len; i++)
    {
        crc ^= (uint32_t)data[i];
        crc = (crc >> 4) ^ s_crc32_nibble_table[crc & 0x0Fu];
        crc = (crc >> 4) ^ s_crc32_nibble_table[crc & 0x0Fu];
    }

    return crc;
}

uint32_t platform_crc32(const uint8_t *data, uint32_t len)
{
    return platform_crc32_update(PLATFORM_CRC32_INIT_VALUE, data, len) ^
           PLATFORM_CRC32_XOROUT_VALUE;
}
