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
 * @version V1.0 2026-10-28
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

#endif /* __PLATFORM_CRC_H__ */
