/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file platform_error.h
 *
 * @par dependencies
 * - platform_type.h
 *
 * @author Jack | R&D Dept. | EternalChip
 *
 * @brief Provide the platform error definitions.
 *
 * @version V1.0 2026-05-10
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_ERROR_H__
#define __PLATFORM_ERROR_H__

//******************************** Includes *********************************//

#include "platform_type.h"

//******************************** Declaring ********************************//

typedef enum
{
    PLATFORM_ERR_OK              = 0,
    PLATFORM_ERR_GENERAL         = 1,
    PLATFORM_ERR_TIMEOUT         = 2,
    PLATFORM_ERR_PARAM           = 3,
    PLATFORM_ERR_NO_MEMORY       = 4,
    PLATFORM_ERR_NO_RESOURCE     = 5,
    PLATFORM_ERR_NOT_SUPPORTED   = 6,
    PLATFORM_ERR_NOT_INITIALIZED = 7,
    PLATFORM_ERR_ALREADY_INIT    = 8,
    PLATFORM_ERR_BUSY            = 9,
    PLATFORM_ERR_FAIL            = 10,
    PLATFORM_ERR_RESERVED        = 0x7FFFFFFF
} platform_err_t;

#define PLATFORM_IS_ERR(err)    ((err) != PLATFORM_ERR_OK)
#define PLATFORM_IS_OK(err)     ((err) == PLATFORM_ERR_OK)

#endif /* __PLATFORM_ERROR_H__ */
