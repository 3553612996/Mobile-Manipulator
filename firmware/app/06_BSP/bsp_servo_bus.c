/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file bsp_servo_bus.c
 *
 * @par dependencies
 * - bsp_servo_bus.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement the YH bus-servo state machine.
 *
 * @version V1.0 2026-10-31
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "bsp_servo_bus.h"

#include <string.h>

//******************************** Defines **********************************//

#define BSP_SERVO_AWAIT_NONE        ((uint8_t)0u)
#define BSP_SERVO_AWAIT_ANGLE       ((uint8_t)1u)
#define BSP_SERVO_AWAIT_TEMP        ((uint8_t)2u)
#define BSP_SERVO_AWAIT_ID          ((uint8_t)3u)

#define BSP_SERVO_ENABLE_NONE       ((uint8_t)0u)
#define BSP_SERVO_ENABLE_OFF        ((uint8_t)1u)
#define BSP_SERVO_ENABLE_ON         ((uint8_t)2u)

//******************************** Functions ********************************//

static void bus_put_digits(char *dst, uint16_t value, uint8_t width)
{
    uint8_t i;

    for (i = width; i > 0u; i--)
    {
        dst[i - 1u] = (char)('0' + (value % 10u));
        value = (uint16_t)(value / 10u);
    }
}

static platform_bool_t bus_is_digit(char c)
{
    return ((c >= '0') && (c <= '9')) ? PLATFORM_TRUE : PLATFORM_FALSE;
}

/* Parse an unsigned run of digits; advances *cursor past it. */
static platform_bool_t bus_parse_uint(const char **cursor, uint32_t *out)
{
    const char *p = *cursor;
    uint32_t value = 0u;
    uint8_t digits = 0u;

    while ((bus_is_digit(*p) == PLATFORM_TRUE) && (digits < 6u))
    {
        value = (value * 10u) + (uint32_t)(*p - '0');
        p++;
        digits++;
    }

    if (digits == 0u)
    {
        return PLATFORM_FALSE;
    }

    *cursor = p;
    *out = value;

    return PLATFORM_TRUE;
}

/* "#IDPpppp!" -> pppp. Rejects the "PDA..." variant on purpose. */
static platform_bool_t bus_parse_angle(const char *line, uint32_t *pulse)
{
    const char *p = strchr(line, 'P');

    if (p == NULL)
    {
        return PLATFORM_FALSE;
    }

    p++;

    if (bus_parse_uint(&p, pulse) == PLATFORM_FALSE)
    {
        return PLATFORM_FALSE;
    }

    return ((*pulse >= 400u) && (*pulse <= 2600u)) ? PLATFORM_TRUE
                                                   : PLATFORM_FALSE;
}

/* "T25" / "T-3" -> degrees Celsius. */
static platform_bool_t bus_parse_temperature(const char *line, int32_t *out)
{
    const char *p = strchr(line, 'T');
    uint32_t value;
    int32_t sign = 1;

    if (p == NULL)
    {
        return PLATFORM_FALSE;
    }

    p++;

    if (*p == '-')
    {
        sign = -1;
        p++;
    }

    if (bus_parse_uint(&p, &value) == PLATFORM_FALSE)
    {
        return PLATFORM_FALSE;
    }

    *out = sign * (int32_t)value;

    return PLATFORM_TRUE;
}

/* "V07" -> 7000 mV, "V5.1" -> 5100 mV. */
static platform_bool_t bus_parse_voltage(const char *line, uint16_t *out_mv)
{
    const char *p = strchr(line, 'V');
    uint32_t whole;
    uint32_t frac = 0u;
    uint32_t value;
    uint8_t digits = 0u;

    if (p == NULL)
    {
        return PLATFORM_FALSE;
    }

    p++;

    if (bus_parse_uint(&p, &whole) == PLATFORM_FALSE)
    {
        return PLATFORM_FALSE;
    }

    if (*p == '.')
    {
        p++;

        while ((bus_is_digit(*p) == PLATFORM_TRUE) && (digits < 3u))
        {
            frac = (frac * 10u) + (uint32_t)(*p - '0');
            p++;
            digits++;
        }
    }

    while (digits < 3u)
    {
        frac *= 10u;
        digits++;
    }

    value = (whole * 1000u) + frac;

    if (value > 30000u)
    {
        return PLATFORM_FALSE;
    }

    *out_mv = (uint16_t)value;

    return PLATFORM_TRUE;
}

/* Bus address of a "#000..." reply; broadcast when the frame has no address. */
static uint8_t bus_line_id(const char *line)
{
    uint32_t id;

    if ((line[0] != '#') ||
        (bus_is_digit(line[1]) == PLATFORM_FALSE) ||
        (bus_is_digit(line[2]) == PLATFORM_FALSE) ||
        (bus_is_digit(line[3]) == PLATFORM_FALSE))
    {
        return BSP_SERVO_BUS_ID_BROADCAST;
    }

    id = (uint32_t)(line[1] - '0') * 100u +
         (uint32_t)(line[2] - '0') * 10u +
         (uint32_t)(line[3] - '0');

    return (id <= 254u) ? (uint8_t)id : BSP_SERVO_BUS_ID_BROADCAST;
}

static int16_t bus_pulse_to_angle(const bsp_servo_bus_t *bus,
                                  uint16_t pulse,
                                  uint8_t joint)
{
    int32_t span = ((int32_t)bus->cfg.pulse_max_us -
                    (int32_t)bus->cfg.pulse_min_us) / 2;
    int32_t raw;
    int32_t value;

    if (span <= 0)
    {
        span = 1000;
    }

    raw = ((int32_t)pulse - (int32_t)BSP_SERVO_BUS_PULSE_CENTER_US) *
          (int32_t)bus->cfg.angle_limit_centi_deg / span;

    value = raw - (int32_t)bus->cfg.offset_centi_deg[joint];

    if (value > (int32_t)bus->cfg.angle_limit_centi_deg)
    {
        value = (int32_t)bus->cfg.angle_limit_centi_deg;
    }
    else if (value < -(int32_t)bus->cfg.angle_limit_centi_deg)
    {
        value = -(int32_t)bus->cfg.angle_limit_centi_deg;
    }

    return (int16_t)value;
}

static uint16_t bus_angle_to_pulse(const bsp_servo_bus_t *bus,
                                   int16_t angle_centi_deg,
                                   uint8_t joint)
{
    int32_t span = ((int32_t)bus->cfg.pulse_max_us -
                    (int32_t)bus->cfg.pulse_min_us) / 2;
    int32_t angle;
    int32_t pulse;

    if (span <= 0)
    {
        span = 1000;
    }

    angle = (int32_t)angle_centi_deg + (int32_t)bus->cfg.offset_centi_deg[joint];

    pulse = (int32_t)BSP_SERVO_BUS_PULSE_CENTER_US +
            (angle * span) / (int32_t)bus->cfg.angle_limit_centi_deg;

    if (pulse < (int32_t)bus->cfg.pulse_min_us)
    {
        pulse = (int32_t)bus->cfg.pulse_min_us;
    }
    else if (pulse > (int32_t)bus->cfg.pulse_max_us)
    {
        pulse = (int32_t)bus->cfg.pulse_max_us;
    }

    return (uint16_t)pulse;
}

static void bus_finish_config(bsp_servo_bus_t *bus, platform_err_t result,
                              int32_t value)
{
    bus->cfg_out = value;
    bus->cfg_result = result;
    bus->cfg_done = 1u;
}

/*
 * One complete reply line. Anything that is not the awaited transaction is
 * ignored: the bus is shared and a late reply must not move a joint's cache.
 */
static void bus_handle_line(bsp_servo_bus_t *bus, const char *line,
                            uint32_t now_ms)
{
    uint8_t joint = bus->await_joint;
    uint8_t id = bus_line_id(line);
    uint32_t pulse;
    int32_t temperature;
    uint16_t voltage_mv;
    platform_bool_t valid;

    if (bus->await_kind == BSP_SERVO_AWAIT_NONE)
    {
        return;
    }

    if ((id != BSP_SERVO_BUS_ID_BROADCAST) &&
        (id != bus->cfg.servo_id[joint]))
    {
        /* Some other device answered; keep waiting for ours. */
        return;
    }

    switch (bus->await_kind)
    {
        case BSP_SERVO_AWAIT_ANGLE:
            if (bus_parse_angle(line, &pulse) == PLATFORM_TRUE)
            {
                bus->angle_centi_deg[joint] =
                    bus_pulse_to_angle(bus, (uint16_t)pulse, joint);
                bus->angle_ms[joint] = now_ms;
                bus->misses[joint] = 0u;
                bus->online[joint] = PLATFORM_TRUE;
                bus->fault_bits[joint] &=
                    (uint8_t)~(BSP_SERVO_BUS_FAULT_OFFLINE |
                               BSP_SERVO_BUS_FAULT_REPLY);
            }
            else
            {
                bus->fault_bits[joint] |= BSP_SERVO_BUS_FAULT_REPLY;
            }
            bus->await_kind = BSP_SERVO_AWAIT_NONE;
            break;

        case BSP_SERVO_AWAIT_TEMP:
            valid = PLATFORM_FALSE;

            if (bus_parse_temperature(line, &temperature) == PLATFORM_TRUE)
            {
                if ((temperature >= -40) && (temperature <= 150))
                {
                    bus->temperature_c[joint] = (int16_t)temperature;
                    valid = PLATFORM_TRUE;
                }
            }

            if (bus_parse_voltage(line, &voltage_mv) == PLATFORM_TRUE)
            {
                bus->voltage_mv[joint] = voltage_mv;
                valid = PLATFORM_TRUE;
            }

            if (valid == PLATFORM_TRUE)
            {
                bus->misses[joint] = 0u;
                bus->online[joint] = PLATFORM_TRUE;
                bus->fault_bits[joint] &=
                    (uint8_t)~(BSP_SERVO_BUS_FAULT_OFFLINE |
                               BSP_SERVO_BUS_FAULT_REPLY);
            }
            else
            {
                bus->fault_bits[joint] |= BSP_SERVO_BUS_FAULT_REPLY;
            }
            bus->await_kind = BSP_SERVO_AWAIT_NONE;
            break;

        case BSP_SERVO_AWAIT_ID:
            if (bus->cfg_done == 0u)
            {
                if (strstr(line, "OK") != NULL)
                {
                    bus->cfg.servo_id[joint] = (uint8_t)bus->cfg_value;
                    bus_finish_config(bus, PLATFORM_ERR_OK, bus->cfg_value);
                }
                else
                {
                    bus_finish_config(bus, PLATFORM_ERR_FAIL, 0);
                }
            }
            bus->await_kind = BSP_SERVO_AWAIT_NONE;
            break;

        default:
            bus->await_kind = BSP_SERVO_AWAIT_NONE;
            break;
    }
}

static void bus_drain_rx(bsp_servo_bus_t *bus, uint32_t now_ms)
{
    uint8_t chunk[24];
    uint32_t got;
    uint32_t i;
    char c;
    platform_err_t err;

    for (;;)
    {
        err = platform_uart_rx_read(bus->cfg.bus, chunk,
                                    (uint32_t)sizeof(chunk), &got);
        if (PLATFORM_IS_ERR(err) || (got == 0u))
        {
            break;
        }

        for (i = 0u; i < got; i++)
        {
            c = (char)chunk[i];

            if ((c == '\r') || (c == '\n') || (c == ' '))
            {
                continue;
            }

            if (c == '#')
            {
                bus->rx_line[0] = '#';
                bus->rx_len = 1u;
                continue;
            }

            if (bus->rx_len == 0u)
            {
                continue;
            }

            if (bus->rx_len < (BSP_SERVO_BUS_REPLY_MAX - 1u))
            {
                bus->rx_line[bus->rx_len] = c;
                bus->rx_len++;
            }

            if (c == '!')
            {
                bus->rx_line[bus->rx_len] = '\0';
                bus_handle_line(bus, bus->rx_line, now_ms);
                bus->rx_len = 0u;
            }
        }
    }
}

static void bus_check_timeout(bsp_servo_bus_t *bus, uint32_t now_ms)
{
    uint32_t limit;
    uint8_t joint;

    if (bus->await_kind == BSP_SERVO_AWAIT_NONE)
    {
        return;
    }

    limit = (bus->await_kind == BSP_SERVO_AWAIT_ID)
                ? BSP_SERVO_BUS_ID_TIMEOUT_MS
                : (uint32_t)bus->cfg.reply_timeout_ms;

    if ((now_ms - bus->sent_ms) < limit)
    {
        return;
    }

    joint = bus->await_joint;

    if (bus->await_kind == BSP_SERVO_AWAIT_ID)
    {
        if (bus->cfg_done == 0u)
        {
            bus_finish_config(bus, PLATFORM_ERR_TIMEOUT, 0);
        }
    }
    else
    {
        if (bus->misses[joint] < 255u)
        {
            bus->misses[joint]++;
        }

        if (bus->misses[joint] >= bus->cfg.offline_misses)
        {
            bus->online[joint] = PLATFORM_FALSE;
            bus->fault_bits[joint] |= BSP_SERVO_BUS_FAULT_OFFLINE;
        }
    }

    bus->await_kind = BSP_SERVO_AWAIT_NONE;
}

static platform_err_t bus_write(bsp_servo_bus_t *bus, const char *text,
                                uint16_t length, uint32_t now_ms)
{
    platform_err_t err;

    err = platform_uart_write(bus->cfg.bus, (const uint8_t *)text, length,
                              BSP_SERVO_BUS_TX_TIMEOUT_MS);
    if (PLATFORM_IS_OK(err))
    {
        bus->last_tx_ms = now_ms;
        bus->tx_seen = PLATFORM_TRUE;
    }

    return err;
}

static platform_bool_t bus_gap_elapsed(const bsp_servo_bus_t *bus,
                                       uint32_t now_ms)
{
    if (bus->tx_seen == PLATFORM_FALSE)
    {
        return PLATFORM_TRUE;
    }

    return ((now_ms - bus->last_tx_ms) >= (uint32_t)bus->cfg.bus_gap_ms)
               ? PLATFORM_TRUE
               : PLATFORM_FALSE;
}

static platform_bool_t bus_send_move(bsp_servo_bus_t *bus,
                                     const int16_t target[BSP_SERVO_BUS_JOINT_MAX],
                                     uint16_t move_ms,
                                     uint32_t now_ms)
{
    char frame[BSP_SERVO_BUS_TX_MAX];
    uint16_t length = 0u;
    uint8_t i;

    if (move_ms > 9999u)
    {
        move_ms = 9999u;
    }

    frame[length++] = '{';

    for (i = 0u; i < bus->cfg.joint_count; i++)
    {
        frame[length++] = '#';
        bus_put_digits(&frame[length], bus->cfg.servo_id[i], 3u);
        length = (uint16_t)(length + 3u);
        frame[length++] = 'P';
        bus_put_digits(&frame[length],
                       bus_angle_to_pulse(bus, target[i], i), 4u);
        length = (uint16_t)(length + 4u);
        frame[length++] = 'T';
        bus_put_digits(&frame[length], move_ms, 4u);
        length = (uint16_t)(length + 4u);
        frame[length++] = '!';
    }

    frame[length++] = '}';

    if (PLATFORM_IS_ERR(bus_write(bus, frame, length, now_ms)))
    {
        return PLATFORM_FALSE;
    }

    for (i = 0u; i < bus->cfg.joint_count; i++)
    {
        bus->last_target[i] = target[i];
    }

    bus->last_move_ms = move_ms;
    bus->has_last = PLATFORM_TRUE;

    return PLATFORM_TRUE;
}

static platform_bool_t bus_send_query(bsp_servo_bus_t *bus, uint8_t joint,
                                      uint8_t kind, uint32_t now_ms)
{
    char frame[16];
    uint16_t length = 0u;

    frame[length++] = '#';
    bus_put_digits(&frame[length], bus->cfg.servo_id[joint], 3u);
    length = (uint16_t)(length + 3u);

    if (kind == BSP_SERVO_AWAIT_ANGLE)
    {
        memcpy(&frame[length], "PRAD!", 5u);
        length = (uint16_t)(length + 5u);
    }
    else
    {
        memcpy(&frame[length], "PRTV!", 5u);
        length = (uint16_t)(length + 5u);
    }

    if (PLATFORM_IS_ERR(bus_write(bus, frame, length, now_ms)))
    {
        return PLATFORM_FALSE;
    }

    bus->await_kind = kind;
    bus->await_joint = joint;
    bus->sent_ms = now_ms;

    return PLATFORM_TRUE;
}

/* Disable: stop and hold where they are. Enable: restore torque. */
static void bus_send_torque_state(bsp_servo_bus_t *bus, uint32_t now_ms)
{
    static const char stop_frame[] = "#255PDST!";
    static const char restore_frame[] = "#255PULR!";

    if (bus->enabled == PLATFORM_TRUE)
    {
        (void)bus_write(bus, restore_frame,
                        (uint16_t)(sizeof(restore_frame) - 1u), now_ms);
    }
    else
    {
        (void)bus_write(bus, stop_frame,
                        (uint16_t)(sizeof(stop_frame) - 1u), now_ms);
        bus->req_consumed = bus->req_seq;
    }
}

static platform_bool_t bus_handle_enable_request(bsp_servo_bus_t *bus,
                                                 uint32_t now_ms)
{
    uint8_t request = bus->enable_req;

    if (request == BSP_SERVO_ENABLE_NONE)
    {
        return PLATFORM_FALSE;
    }

    bus->enable_req = BSP_SERVO_ENABLE_NONE;

    if (request == BSP_SERVO_ENABLE_ON)
    {
        bus->enabled = PLATFORM_TRUE;
        bus_send_torque_state(bus, now_ms);

        /*
         * Re-assert the last target. The restore frame just used the slot, so
         * the move goes out after the normal bus gap.
         */
        if (bus->has_last == PLATFORM_TRUE)
        {
            uint8_t i;

            bus->req_seq++;
            for (i = 0u; i < bus->cfg.joint_count; i++)
            {
                bus->req_target[i] = bus->last_target[i];
            }
            bus->req_move_ms = bus->last_move_ms;
            bus->req_seq++;
        }
    }
    else
    {
        bus->enabled = PLATFORM_FALSE;
        bus_send_torque_state(bus, now_ms);
    }

    return PLATFORM_TRUE;
}

/*
 * Execute at most one step of the outstanding configuration request.
 * Local operations (calibrate/clear) finish without touching the bus.
 */
static platform_bool_t bus_service_config(bsp_servo_bus_t *bus,
                                          uint32_t now_ms)
{
    char frame[16];
    uint16_t length;
    uint8_t joint;
    uint8_t op;
    int32_t value;
    int32_t raw;

    if ((bus->cfg_pending == 0u) || (bus->cfg_done != 0u))
    {
        return PLATFORM_FALSE;
    }

    joint = bus->cfg_joint;
    op = bus->cfg_op;
    value = bus->cfg_value;

    if (op == BSP_SERVO_BUS_CFG_CALIBRATE)
    {
        if ((bus->online[joint] == PLATFORM_FALSE) ||
            ((now_ms - bus->angle_ms[joint]) > 3000u))
        {
            bus_finish_config(bus, PLATFORM_ERR_FAIL, 0);
            return PLATFORM_TRUE;
        }

        raw = (int32_t)bus->angle_centi_deg[joint] +
              (int32_t)bus->cfg.offset_centi_deg[joint];

        if (raw > 32767)
        {
            raw = 32767;
        }
        else if (raw < -32768)
        {
            raw = -32768;
        }

        bus->cfg.offset_centi_deg[joint] = (int16_t)raw;
        bus_finish_config(bus, PLATFORM_ERR_OK, raw);
        return PLATFORM_TRUE;
    }

    if (op == BSP_SERVO_BUS_CFG_CLEAR_CALIB)
    {
        bus->cfg.offset_centi_deg[joint] = 0;
        bus_finish_config(bus, PLATFORM_ERR_OK, 0);
        return PLATFORM_TRUE;
    }

    if (bus_gap_elapsed(bus, now_ms) == PLATFORM_FALSE)
    {
        return PLATFORM_TRUE;
    }

    length = 0u;
    frame[length++] = '#';
    bus_put_digits(&frame[length], bus->cfg.servo_id[joint], 3u);
    length = (uint16_t)(length + 3u);

    if (op == BSP_SERVO_BUS_CFG_SET_ID)
    {
        memcpy(&frame[length], "PID", 3u);
        length = (uint16_t)(length + 3u);
        bus_put_digits(&frame[length], (uint16_t)value, 3u);
        length = (uint16_t)(length + 3u);
        frame[length++] = '!';

        if (PLATFORM_IS_ERR(bus_write(bus, frame, length, now_ms)))
        {
            bus_finish_config(bus, PLATFORM_ERR_FAIL, 0);
            return PLATFORM_TRUE;
        }

        bus->await_kind = BSP_SERVO_AWAIT_ID;
        bus->await_joint = joint;
        bus->sent_ms = now_ms;
        return PLATFORM_TRUE;
    }

    if (op == BSP_SERVO_BUS_CFG_SET_BIAS)
    {
        memcpy(&frame[length], "PSCK", 4u);
        length = (uint16_t)(length + 4u);
        frame[length++] = (value < 0) ? '-' : '+';
        bus_put_digits(&frame[length],
                       (uint16_t)((value < 0) ? -value : value), 3u);
        length = (uint16_t)(length + 3u);
        frame[length++] = '!';

        bus_finish_config(bus,
                          PLATFORM_IS_ERR(bus_write(bus, frame, length,
                                                    now_ms))
                              ? PLATFORM_ERR_FAIL
                              : PLATFORM_ERR_OK,
                          value);
        return PLATFORM_TRUE;
    }

    /* BSP_SERVO_BUS_CFG_TORQUE */
    memcpy(&frame[length], (value != 0) ? "PULR!" : "PULK!", 5u);
    length = (uint16_t)(length + 5u);

    bus_finish_config(bus,
                      PLATFORM_IS_ERR(bus_write(bus, frame, length, now_ms))
                          ? PLATFORM_ERR_FAIL
                          : PLATFORM_ERR_OK,
                      value);
    return PLATFORM_TRUE;
}

static platform_bool_t bus_service_move(bsp_servo_bus_t *bus, uint32_t now_ms)
{
    int16_t target[BSP_SERVO_BUS_JOINT_MAX];
    uint32_t sequence;
    uint16_t move_ms;
    uint8_t i;

    sequence = bus->req_seq;

    if ((sequence == bus->req_consumed) || ((sequence & 1u) != 0u))
    {
        return PLATFORM_FALSE;
    }

    for (i = 0u; i < bus->cfg.joint_count; i++)
    {
        target[i] = bus->req_target[i];
    }
    move_ms = bus->req_move_ms;

    if (sequence != bus->req_seq)
    {
        /* The writer moved underneath us; try again on the next slot. */
        return PLATFORM_FALSE;
    }

    bus->req_consumed = sequence;

    if (bus->enabled == PLATFORM_FALSE)
    {
        return PLATFORM_TRUE;
    }

    (void)bus_send_move(bus, target, move_ms, now_ms);

    return PLATFORM_TRUE;
}

//******************************** Functions ********************************//

platform_err_t bsp_servo_bus_init(bsp_servo_bus_t *bus)
{
    uint8_t i;

    if (bus == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    if ((bus->cfg.joint_count == 0u) ||
        (bus->cfg.joint_count > BSP_SERVO_BUS_JOINT_MAX))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (bus->cfg.angle_limit_centi_deg <= 0)
    {
        return PLATFORM_ERR_PARAM;
    }

    for (i = 0u; i < BSP_SERVO_BUS_JOINT_MAX; i++)
    {
        bus->angle_centi_deg[i] = 0;
        bus->temperature_c[i] = BSP_SERVO_BUS_TEMP_INVALID;
        bus->voltage_mv[i] = BSP_SERVO_BUS_VOLTAGE_INVALID;

        /*
         * Do not start with OFFLINE set: the first scan needs a few slots and
         * latching a fault before the bus has been polled once would force a
         * CLEAR_FAULT after every boot. offline starts FALSE and is raised
         * only after cfg.offline_misses unanswered queries.
         */
        bus->fault_bits[i] = 0u;
        bus->online[i] = PLATFORM_FALSE;
        bus->angle_ms[i] = 0u;
        bus->misses[i] = 0u;
    }

    bus->req_seq = 0u;
    bus->req_consumed = 0u;
    bus->req_move_ms = 0u;
    bus->has_last = PLATFORM_FALSE;
    bus->enable_req = BSP_SERVO_ENABLE_NONE;
    bus->enabled = PLATFORM_FALSE;
    bus->cfg_pending = 0u;
    bus->cfg_done = 0u;
    bus->cfg_result = PLATFORM_ERR_OK;
    bus->cfg_out = 0;
    bus->rx_len = 0u;
    bus->await_kind = BSP_SERVO_AWAIT_NONE;
    bus->await_joint = 0u;
    bus->sent_ms = 0u;
    bus->last_tx_ms = 0u;
    bus->tx_seen = PLATFORM_FALSE;
    bus->poll_joint = 0u;
    bus->poll_temp_joint = 0u;
    bus->poll_query = BSP_SERVO_AWAIT_ANGLE;
    bus->queries_since_move = 0u;
    bus->initialized = PLATFORM_TRUE;

    return PLATFORM_ERR_OK;
}

platform_err_t bsp_servo_bus_deinit(bsp_servo_bus_t *bus)
{
    if (bus == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    bus->initialized = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

platform_err_t bsp_servo_bus_set_enable(bsp_servo_bus_t *bus,
                                        platform_bool_t enable)
{
    if ((bus == NULL) || (bus->initialized == PLATFORM_FALSE))
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    bus->enable_req = (enable == PLATFORM_TRUE) ? BSP_SERVO_ENABLE_ON
                                                : BSP_SERVO_ENABLE_OFF;

    return PLATFORM_ERR_OK;
}

platform_err_t bsp_servo_bus_request_move(
                    bsp_servo_bus_t *bus,
                    const int16_t joint_centi_deg[BSP_SERVO_BUS_JOINT_MAX],
                    uint16_t move_time_ms)
{
    uint8_t i;

    if ((bus == NULL) || (joint_centi_deg == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (bus->initialized == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    bus->req_seq++;
    for (i = 0u; i < bus->cfg.joint_count; i++)
    {
        bus->req_target[i] = joint_centi_deg[i];
    }
    bus->req_move_ms = move_time_ms;
    bus->req_seq++;

    return PLATFORM_ERR_OK;
}

void bsp_servo_bus_poll(bsp_servo_bus_t *bus, uint32_t now_ms)
{
    if ((bus == NULL) || (bus->initialized == PLATFORM_FALSE))
    {
        return;
    }

    bus_drain_rx(bus, now_ms);
    bus_check_timeout(bus, now_ms);

    if ((bus->await_kind != BSP_SERVO_AWAIT_NONE))
    {
        return;
    }

    if (bus_handle_enable_request(bus, now_ms) == PLATFORM_TRUE)
    {
        return;
    }

    if (bus_service_config(bus, now_ms) == PLATFORM_TRUE)
    {
        return;
    }

    if (bus_gap_elapsed(bus, now_ms) == PLATFORM_FALSE)
    {
        return;
    }

    /*
     * Fair share of the 100 ms bus: a move needs at least one query since the
     * previous move, so a host streaming targets at 20 Hz still gets every
     * joint's angle refreshed roughly once a second.
     */
    if ((bus->queries_since_move > 0u) &&
        (bus_service_move(bus, now_ms) == PLATFORM_TRUE))
    {
        bus->queries_since_move = 0u;
        return;
    }

    if (bus->poll_query == BSP_SERVO_AWAIT_ANGLE)
    {
        if (bus_send_query(bus, bus->poll_joint, BSP_SERVO_AWAIT_ANGLE,
                           now_ms) == PLATFORM_TRUE)
        {
            bus->queries_since_move++;
            bus->poll_joint =
                (uint8_t)((bus->poll_joint + 1u) % bus->cfg.joint_count);

            /* A full angle round is done: one joint reports T and V next. */
            if (bus->poll_joint == 0u)
            {
                bus->poll_query = BSP_SERVO_AWAIT_TEMP;
            }
        }
    }
    else if (bus_send_query(bus, bus->poll_temp_joint,
                            BSP_SERVO_AWAIT_TEMP, now_ms) == PLATFORM_TRUE)
    {
        bus->queries_since_move++;
        bus->poll_temp_joint =
            (uint8_t)((bus->poll_temp_joint + 1u) % bus->cfg.joint_count);
        bus->poll_query = BSP_SERVO_AWAIT_ANGLE;
    }
    else
    {
        /* TX failed; retry the same query on the next slot. */
    }
}

void bsp_servo_bus_get_states(
                    const bsp_servo_bus_t *bus,
                    int16_t angle_centi_deg[BSP_SERVO_BUS_JOINT_MAX],
                    int16_t temperature_c[BSP_SERVO_BUS_JOINT_MAX],
                    uint16_t voltage_mv[BSP_SERVO_BUS_JOINT_MAX],
                    uint8_t fault_bits[BSP_SERVO_BUS_JOINT_MAX],
                    platform_bool_t online[BSP_SERVO_BUS_JOINT_MAX])
{
    uint8_t i;

    if (bus == NULL)
    {
        return;
    }

    for (i = 0u; i < BSP_SERVO_BUS_JOINT_MAX; i++)
    {
        if (angle_centi_deg != NULL)
        {
            angle_centi_deg[i] = bus->angle_centi_deg[i];
        }
        if (temperature_c != NULL)
        {
            temperature_c[i] = bus->temperature_c[i];
        }
        if (voltage_mv != NULL)
        {
            voltage_mv[i] = bus->voltage_mv[i];
        }
        if (fault_bits != NULL)
        {
            fault_bits[i] = bus->fault_bits[i];
        }
        if (online != NULL)
        {
            online[i] = bus->online[i];
        }
    }
}

void bsp_servo_bus_set_map(
                    bsp_servo_bus_t *bus,
                    const uint8_t servo_id[BSP_SERVO_BUS_JOINT_MAX],
                    const int16_t offset_centi_deg[BSP_SERVO_BUS_JOINT_MAX])
{
    uint8_t i;

    if ((bus == NULL) || (servo_id == NULL) || (offset_centi_deg == NULL))
    {
        return;
    }

    for (i = 0u; i < BSP_SERVO_BUS_JOINT_MAX; i++)
    {
        bus->cfg.servo_id[i] = servo_id[i];
        bus->cfg.offset_centi_deg[i] = offset_centi_deg[i];
    }
}

platform_err_t bsp_servo_bus_request_config(bsp_servo_bus_t *bus,
                                            uint8_t joint,
                                            uint8_t op,
                                            int32_t value)
{
    uint8_t i;

    if ((bus == NULL) || (bus->initialized == PLATFORM_FALSE))
    {
        return PLATFORM_ERR_NOT_INITIALIZED;
    }

    if ((joint >= bus->cfg.joint_count) || (op < BSP_SERVO_BUS_CFG_SET_ID) ||
        (op > BSP_SERVO_BUS_CFG_TORQUE))
    {
        return PLATFORM_ERR_PARAM;
    }

    switch (op)
    {
        case BSP_SERVO_BUS_CFG_SET_ID:
            if ((value < 0) || (value >= (int32_t)BSP_SERVO_BUS_ID_BROADCAST))
            {
                return PLATFORM_ERR_PARAM;
            }

            for (i = 0u; i < bus->cfg.joint_count; i++)
            {
                if ((i != joint) &&
                    (bus->cfg.servo_id[i] == (uint8_t)value))
                {
                    return PLATFORM_ERR_PARAM;
                }
            }
            break;

        case BSP_SERVO_BUS_CFG_SET_BIAS:
            if ((value < -BSP_SERVO_BUS_BIAS_LIMIT) ||
                (value > BSP_SERVO_BUS_BIAS_LIMIT))
            {
                return PLATFORM_ERR_PARAM;
            }
            break;

        case BSP_SERVO_BUS_CFG_TORQUE:
            if ((value != 0) && (value != 1))
            {
                return PLATFORM_ERR_PARAM;
            }
            break;

        default:
            break;
    }

    if ((bus->cfg_pending != 0u) || (bus->cfg_done != 0u))
    {
        return PLATFORM_ERR_BUSY;
    }

    bus->cfg_joint = joint;
    bus->cfg_op = op;
    bus->cfg_value = value;
    bus->cfg_result = PLATFORM_ERR_OK;
    bus->cfg_out = 0;
    bus->cfg_done = 0u;
    bus->cfg_pending = 1u;

    return PLATFORM_ERR_OK;
}

platform_bool_t bsp_servo_bus_config_ready(const bsp_servo_bus_t *bus)
{
    if (bus == NULL)
    {
        return PLATFORM_FALSE;
    }

    return (bus->cfg_done != 0u) ? PLATFORM_TRUE : PLATFORM_FALSE;
}

platform_err_t bsp_servo_bus_config_take(bsp_servo_bus_t *bus,
                                         int32_t *out_value)
{
    platform_err_t result;

    if ((bus == NULL) || (bus->cfg_done == 0u))
    {
        return PLATFORM_ERR_BUSY;
    }

    result = bus->cfg_result;

    if (out_value != NULL)
    {
        *out_value = bus->cfg_out;
    }

    bus->cfg_done = 0u;
    bus->cfg_pending = 0u;

    return result;
}

void bsp_servo_bus_config_cancel(bsp_servo_bus_t *bus)
{
    if (bus == NULL)
    {
        return;
    }

    if (bus->cfg_pending != 0u)
    {
        bus_finish_config(bus, PLATFORM_ERR_FAIL, 0);
    }
}
