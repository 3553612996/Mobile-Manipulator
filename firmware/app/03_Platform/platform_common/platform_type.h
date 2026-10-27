/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file platform_type.h
 *
 * @par dependencies
 * - impl_board/board_types.h
 *
 * @author Jack | R&D Dept. | EternalChip
 *
 * @brief Provide the platform base types.
 *
 * platform_type.h is the Platform layer type outlet. Upper layers include this
 * file instead of including board_types.h directly.
 *
 * @version V1.0 2026-05-10
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_TYPE_H__
#define __PLATFORM_TYPE_H__

//******************************** Includes *********************************//

#include "impl_board/board_types.h"

//******************************** Declaring ********************************//

typedef int8      int8_t;
typedef uint8     uint8_t;
typedef int16     int16_t;
typedef uint16    uint16_t;
typedef int32     int32_t;
typedef uint32    uint32_t;
typedef int64     int64_t;
typedef uint64    uint64_t;

typedef float32   float_t;
typedef float64   double_t;

typedef char      char_t;
typedef uint8     uchar_t;

typedef uint8     bool_t;

#endif /* __PLATFORM_TYPE_H__ */
