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
 * @brief Implement CRC-16/CCITT-FALSE.
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
