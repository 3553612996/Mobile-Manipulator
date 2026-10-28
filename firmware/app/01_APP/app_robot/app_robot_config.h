/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file app_robot_config.h
 *
 * @par dependencies
 * - none
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Board-specific numbers for the robot application.
 *
 * Every value that must be measured on the real chassis lives here, not in
 * the services. The services validate against these limits; they never guess
 * geometry.
 *
 * TODO(measure): half_length / half_width / counts_per_mm. The values below
 * come from the C25 V2.4 drawing and a 60 mm mecanum wheel with the 1:90
 * gearbox; a ruler test on the assembled robot is still required. Odometry
 * scales linearly with counts_per_mm, so a wrong value shows up as a constant
 * distance error, not as drift.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __APP_ROBOT_CONFIG_H__
#define __APP_ROBOT_CONFIG_H__

//******************************** Includes *********************************//

#include "platform_mcu_uart.h"

//******************************** Defines **********************************//

/* ------------------------------- Transport ------------------------------ */

#define APP_ROBOT_UART_BUS              PLATFORM_UART_BUS_0
#define APP_ROBOT_UART_BAUDRATE         ((uint32_t)115200u)

/* ------------------------------- Geometry ------------------------------- */

#define APP_CHASSIS_HALF_LENGTH_MM      ((int16_t)150)
#define APP_CHASSIS_HALF_WIDTH_MM       ((int16_t)120)

/*
 * counts_per_mm * 1000:
 *   13 lines * 4 (quadrature) * 90 (gearbox) / (pi * 60 mm) = 24.828 counts/mm
 *
 * 4680 counts per wheel revolution. A 65 mm wheel would give 22.918, which is
 * 8% low on distance -- the constant is only as good as the tyre measured.
 */
#define APP_CHASSIS_COUNTS_PER_MM_X1000 ((int32_t)24828)

/* --------------------------- Motion limits ------------------------------ */

#define APP_CHASSIS_MAX_VX_MM_S         ((int16_t)500)
#define APP_CHASSIS_MAX_VY_MM_S         ((int16_t)500)
#define APP_CHASSIS_MAX_WZ_MRAD_S       ((int16_t)1500)

/* Wheel speed that maps to +-1000 permille in the open-loop placeholder. */
#define APP_CHASSIS_MAX_WHEEL_MM_S      ((int16_t)500)

#define APP_CHASSIS_CMD_TIMEOUT_MS      ((uint32_t)200u)

/* ------------------------------- Timing --------------------------------- */

#define APP_COMM_PERIOD_MS              ((uint32_t)10u)
#define APP_CONTROL_PERIOD_MS           ((uint32_t)10u)
#define APP_ODOM_PERIOD_MS              ((uint32_t)20u)    /* 50 Hz. */
#define APP_ARM_PERIOD_MS               ((uint32_t)50u)    /* 20 Hz. */
#define APP_STATUS_PERIOD_MS            ((uint32_t)100u)   /* 10 Hz. */
#define APP_HEARTBEAT_PERIOD_MS         ((uint32_t)100u)   /* 10 Hz. */

/* 7.4 V (2S) pack: warn at 3.4 V/cell, well above the 3.0 V/cell floor. */
#define APP_BATTERY_LOW_MV              ((uint16_t)6800u)

/* ------------------------------- Key ------------------------------------ */

#define APP_KEY_LONG_PRESS_MS           ((uint32_t)1500u)
#define APP_KEY_FORWARD_MM_S            ((uint16_t)150u)
#define APP_KEY_BEEP_MS                 ((uint32_t)300u)

/* -------------------------------- Arm ----------------------------------- */

#define APP_ARM_MIN_CENTI_DEG           ((int16_t)-9000)
#define APP_ARM_MAX_CENTI_DEG           ((int16_t)9000)
#define APP_ARM_MOVE_TIME_MS            ((uint16_t)800u)

/* ------------------------------ Task stacks ----------------------------- */

#define APP_COMM_TASK_STACK_BYTES       ((uint32_t)1024u)
#define APP_CONTROL_TASK_STACK_BYTES    ((uint32_t)1024u)

#endif /* __APP_ROBOT_CONFIG_H__ */
