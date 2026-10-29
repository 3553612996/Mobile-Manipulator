/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_crc.h
 *
 * @par dependencies
 * - platform_type.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the CRC-16/CCITT-FALSE algorithm used by the host protocol.
 *
 * Parameters:
 *   polynomial : 0x1021
 *   init       : 0xFFFF
 *   refin      : no
 *   refout     : no
 *   xorout     : 0x0000
 *   check      : "123456789" -> 0x29B1
 *
 * This is a pure function with no hardware or OS dependency, so the same
 * implementation can be compiled on the PC for unit tests.
 *
 * CRC-32/ISO-HDLC (the zlib / PNG / Ethernet variant) is used by the flash
 * record headers:
 *   polynomial : 0x04C11DB7 (reflected 0xEDB88320)
 *   init       : 0xFFFFFFFF
 *   refin/refout: yes
 *   xorout     : 0xFFFFFFFF
 *   check      : "123456789" -> 0xCBF43926
 *
 * @version V1.1 2026-10-29
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_CRC_H__
#define __PLATFORM_CRC_H__

//******************************** Includes *********************************//

#include "platform_type.h"

//******************************** Defines **********************************//

#define PLATFORM_CRC16_CCITT_INIT_VALUE     ((uint16_t)0xFFFFu)
#define PLATFORM_CRC16_CCITT_POLYNOMIAL     ((uint16_t)0x1021u)
#define PLATFORM_CRC16_CCITT_CHECK_VALUE    ((uint16_t)0x29B1u)

#define PLATFORM_CRC32_INIT_VALUE           ((uint32_t)0xFFFFFFFFu)
#define PLATFORM_CRC32_XOROUT_VALUE         ((uint32_t)0xFFFFFFFFu)
#define PLATFORM_CRC32_POLYNOMIAL           ((uint32_t)0xEDB88320u)
#define PLATFORM_CRC32_CHECK_VALUE          ((uint32_t)0xCBF43926u)

//******************************** Declaring ********************************//

/* One-shot calculation starting from the standard init value. */
uint16_t platform_crc16_ccitt(const uint8_t *data, uint32_t len);

/*
 * Incremental calculation. Pass PLATFORM_CRC16_CCITT_INIT_VALUE for the first
 * chunk and the previous return value for each following chunk.
 */
uint16_t platform_crc16_ccitt_update(uint16_t crc,
                                     const uint8_t *data,
                                     uint32_t len);

/* One-shot CRC-32. The init/xorout are applied here, not by the caller. */
uint32_t platform_crc32(const uint8_t *data, uint32_t len);

/*
 * Incremental CRC-32 over the raw register state: start with
 * PLATFORM_CRC32_INIT_VALUE, feed each chunk, then XOR the final value with
 * PLATFORM_CRC32_XOROUT_VALUE. This split exists so a record payload can be
 * hashed while it is still in RAM, before the header that precedes it is
 * known.
 */
uint32_t platform_crc32_update(uint32_t crc,
                               const uint8_t *data,
                               uint32_t len);

#endif /* __PLATFORM_CRC_H__ */
