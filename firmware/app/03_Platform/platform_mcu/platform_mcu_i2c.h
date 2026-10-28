/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_mcu_i2c.h
 *
 * @par dependencies
 * - platform_error.h
 * - platform_def.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform I2C interface.
 *
 * Device addresses are always 7-bit (or 10-bit when configured) unshifted
 * values. 04_Impl converts them to the HAL's 8-bit form, which is the typical
 * parameter adaptation this layer exists for.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_MCU_I2C_H__
#define __PLATFORM_MCU_I2C_H__

//******************************** Includes *********************************//

#include "platform_error.h"
#include "platform_def.h"

//******************************** Defines **********************************//

#define PLATFORM_I2C_BUS_0          ((platform_i2c_bus_t)0u)
#define PLATFORM_I2C_BUS_1          ((platform_i2c_bus_t)1u)
#define PLATFORM_I2C_BUS_2          ((platform_i2c_bus_t)2u)
#define PLATFORM_I2C_BUS_3          ((platform_i2c_bus_t)3u)
#define PLATFORM_I2C_BUS_INVALID    ((platform_i2c_bus_t)0xFFu)

#define PLATFORM_I2C_MEM_ADDR_1BYTE 1u
#define PLATFORM_I2C_MEM_ADDR_2BYTE 2u

//******************************** Declaring ********************************//

typedef uint8_t platform_i2c_bus_t;

typedef enum
{
    PLATFORM_I2C_ADDRESS_7BIT = 0,
    PLATFORM_I2C_ADDRESS_10BIT
} platform_i2c_addressing_t;

typedef struct
{
    platform_i2c_bus_t          bus;
    uint32_t                    clock_hz;
    platform_i2c_addressing_t   addressing;
} platform_i2c_cfg_t;

platform_err_t platform_i2c_init(const platform_i2c_cfg_t *cfg);
platform_err_t platform_i2c_deinit(platform_i2c_bus_t bus);

/*
 * Plain transfers. dev_addr is the unshifted address (e.g. 0x68 for MPU6050).
 */
platform_err_t platform_i2c_write(platform_i2c_bus_t bus,
                                  uint16_t dev_addr,
                                  const uint8_t *data,
                                  uint32_t len,
                                  uint32_t timeout_ms);
platform_err_t platform_i2c_read(platform_i2c_bus_t bus,
                                 uint16_t dev_addr,
                                 uint8_t *data,
                                 uint32_t len,
                                 uint32_t timeout_ms);

/*
 * Register (memory) transfers. mem_addr_size_bytes is 1 or 2; the address is
 * sent most-significant byte first as required by common I2C devices.
 */
platform_err_t platform_i2c_mem_write(platform_i2c_bus_t bus,
                                      uint16_t dev_addr,
                                      uint16_t mem_addr,
                                      uint8_t mem_addr_size_bytes,
                                      const uint8_t *data,
                                      uint32_t len,
                                      uint32_t timeout_ms);
platform_err_t platform_i2c_mem_read(platform_i2c_bus_t bus,
                                     uint16_t dev_addr,
                                     uint16_t mem_addr,
                                     uint8_t mem_addr_size_bytes,
                                     uint8_t *data,
                                     uint32_t len,
                                     uint32_t timeout_ms);

/*
 * Probe a device. *ready is TRUE only if the device acknowledged within the
 * configured trials. A completed probe returns PLATFORM_ERR_OK even when
 * *ready is FALSE; an error means the parameters or the bus are invalid.
 */
platform_err_t platform_i2c_is_ready(platform_i2c_bus_t bus,
                                     uint16_t dev_addr,
                                     uint32_t trials,
                                     uint32_t timeout_ms,
                                     platform_bool_t *ready);

/*
 * Optional bus lock for composite transactions. The implementation may use an
 * OS mutex; callers must not hold it in an ISR.
 */
platform_err_t platform_i2c_lock(platform_i2c_bus_t bus,
                                 uint32_t timeout_ms);
platform_err_t platform_i2c_unlock(platform_i2c_bus_t bus);

#endif /* __PLATFORM_MCU_I2C_H__ */
