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
 * TODO(measure): half_length / half_width / counts_per_mm / PID gains. The
 * geometry values come from the C25 V2.4 drawing and a 60 mm mecanum wheel
 * with the 1:90 gearbox; a ruler test on the assembled robot is still
 * required. Odometry scales linearly with counts_per_mm, so a wrong value
 * shows up as a constant distance error, not as drift. The PID gains are only
 * a conservative starting point until the tyres are fitted.
 *
 * @version V1.1 2026-10-29
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

/*
 * Counts per wheel revolution, the constant SET_PARAM 0x02 uses when the host
 * declares a wheel diameter: counts_per_mm = counts_per_rev / (pi * d).
 * 13 lines * 4 (quadrature) * 90 (gearbox) = 4680. With the 60 mm tyre this
 * reproduces APP_CHASSIS_COUNTS_PER_MM_X1000 exactly, so writing the default
 * diameter is a no-op.
 */
#define APP_CHASSIS_COUNTS_PER_REV      ((int32_t)4680)
#define APP_CHASSIS_WHEEL_DIAMETER_MM   ((int32_t)60)

/*
 * Reliable start duty (deadzone), per wheel in FL/FR/RL/RR order. Measured on
 * the raised bench at 7.81 V on 2026-10-29, see docs/bench/pid-tuning.md 3.3:
 * A 75/67, B 67/75, C 82/80, D 80/100 forward/reverse.
 *
 * These are the fallback for a blank or corrupt calibration record. A saved
 * calibration blob wins at boot, so the values written to flash must be kept
 * in sync with these (PARAM_SAVE scope=1).
 */
#define APP_CHASSIS_DEADZONE_FWD_PERMILLE   { 75, 67, 82, 80 }
#define APP_CHASSIS_DEADZONE_REV_PERMILLE   { 67, 75, 80, 100 }

/* --------------------------- Motion limits ------------------------------ */

#define APP_CHASSIS_MAX_VX_MM_S         ((int16_t)500)
#define APP_CHASSIS_MAX_VY_MM_S         ((int16_t)500)
#define APP_CHASSIS_MAX_WZ_MRAD_S       ((int16_t)1500)

/* Wheel-speed limit; also the full scale if speed feedback is unavailable. */
#define APP_CHASSIS_MAX_WHEEL_MM_S      ((int16_t)500)

/*
 * Speed-loop gains, from the raised-bench tuning of 2026-10-29
 * (docs/bench/pid-tuning.md 4.4, 200 mm/s, all four wheels alike). These are
 * the fallback for a blank or corrupt parameter record; a saved blob loaded at
 * boot wins. Keep the stored blob in sync with these (PARAM_SAVE scope=0).
 *
 * Kd stays at 0 until the encoder speed is filtered; differentiating a 10 ms
 * speed sample mostly amplifies quantization noise.
 */
#define APP_CHASSIS_LEFT_PID_KP         ((float_t)5.0f)
#define APP_CHASSIS_LEFT_PID_KI         ((float_t)6.0f)
#define APP_CHASSIS_LEFT_PID_KD         ((float_t)0.0f)

#define APP_CHASSIS_RIGHT_PID_KP        ((float_t)5.0f)
#define APP_CHASSIS_RIGHT_PID_KI        ((float_t)6.0f)
#define APP_CHASSIS_RIGHT_PID_KD        ((float_t)0.0f)

#define APP_CHASSIS_CMD_TIMEOUT_MS      ((uint32_t)200u)

/*
 * Runtime PID tuning policy. SET_PARAM carries gain * 1000; these are the
 * accepted maxima. They are intentionally the same order as the starting
 * gains, so a slider accident cannot command an absurd duty.
 */
#define APP_PID_GAIN_MAX_X1000          ((int32_t)10000)
#define APP_PID_KD_MAX_X1000            ((int32_t)5000)

/* One gain change per wheel; one SET_PARAM may enqueue two (left/right). */
#define APP_PID_QUEUE_LEN               ((uint32_t)8u)

/*
 * Bench open-loop duty cap, permille. Deliberately equal to the service cap so
 * the handler rejects an out-of-range sweep with ACK 0x01 instead of letting
 * the service silently clamp it: a bench script must not believe it drove
 * 600 permille when the motor saw 400.
 */
#define APP_TEST_DUTY_MAX_PERMILLE      ((int32_t)400)

/* ------------------------------- Timing --------------------------------- */

#define APP_COMM_PERIOD_MS              ((uint32_t)10u)
/*
 * Speed loop at 10 ms. At 1 ms one encoder count is about 40 mm/s, which is
 * too coarse for PID feedback without a filter; 10 ms also keeps the float
 * odometry and four loops comfortably inside the control task budget.
 */
#define APP_CONTROL_PERIOD_MS           ((uint32_t)10u)
#define APP_ODOM_PERIOD_MS              ((uint32_t)20u)    /* 50 Hz. */
#define APP_WHEEL_SPEED_PERIOD_MS       ((uint32_t)20u)    /* 50 Hz. */
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

/* ------------------------------- Storage -------------------------------- */

/*
 * The storage task is the only writer of the external flash and the only
 * caller of service_storage_*, so nothing outside it can block behind a
 * 1 s sector erase.
 */
#define APP_STORAGE_TASK_STACK_BYTES    ((uint32_t)1024u)

/*
 * Requests are descriptors (about 24 B), never payloads: one queue slot of the
 * static pool is 512 B for the whole queue, so eight small descriptors fit
 * while a 4 KB blob never could.
 */
#define APP_STORAGE_QUEUE_LEN           ((uint32_t)8u)

/* Storage task -> comm task: records ready for transmission. */
#define APP_STORAGE_EXPORT_QUEUE_LEN    ((uint32_t)4u)

/*
 * Small records (log lines, fault events) are copied into these slots when
 * they are queued: the caller's buffer may be a stack local that is gone long
 * before the storage task runs. Two slots are enough because appends are
 * event-driven; when both are busy the append is dropped and counted.
 */
#define APP_STORAGE_STAGING_SLOTS       ((uint32_t)2u)
#define APP_STORAGE_STAGING_BYTES       ((uint32_t)240u)

/*
 * Idle wake-up of the storage task. It is also the quantization of the export
 * pacing, so it must divide APP_STORAGE_EXPORT_PERIOD_MS; 10 ms keeps the
 * export at an even 20 Hz instead of drifting to 16 Hz.
 */
#define APP_STORAGE_TICK_MS             ((uint32_t)10u)

/* REPORT_LOG / REPORT_CRASH rate cap, protocol.md 5.5.4: 20 Hz. */
#define APP_STORAGE_EXPORT_PERIOD_MS    ((uint32_t)50u)

/* Stop an export when the host has been silent this long. */
#define APP_STORAGE_EXPORT_IDLE_MS      ((uint32_t)500u)

/* ------------------------------ Task stacks ----------------------------- */

#define APP_COMM_TASK_STACK_BYTES       ((uint32_t)1024u)
#define APP_CONTROL_TASK_STACK_BYTES    ((uint32_t)1024u)

#endif /* __APP_ROBOT_CONFIG_H__ */
