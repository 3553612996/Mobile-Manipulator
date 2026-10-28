/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file board_types.h
 *
 * @par dependencies
 * - none
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Fix the concrete compiler types for the current board.
 *
 * This file is the only place where the concrete compiler types are fixed.
 * platform_common/platform_type.h re-exports them as int8_t/uint8_t/... .
 * Upper layers must not include this file directly.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __BOARD_TYPES_H__
#define __BOARD_TYPES_H__

//******************************** Includes *********************************//

//******************************** Defines **********************************//

/*
 * Target: STM32F103RCT6 (32-bit ARM Cortex-M3, no FPU).
 * int64 must be long long: long is 32-bit on this ABI.
 */
typedef signed char         int8;
typedef unsigned char       uint8;
typedef signed short        int16;
typedef unsigned short      uint16;
typedef signed int          int32;
typedef unsigned int        uint32;
typedef signed long long    int64;
typedef unsigned long long  uint64;

typedef float               float32;
typedef double              float64;

#endif /* __BOARD_TYPES_H__ */
