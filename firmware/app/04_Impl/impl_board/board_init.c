/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file board_init.c
 *
 * @par dependencies
 * - impl_board/board_init.h
 * - impl_bsp/impl_bsp_motor.h
 * - impl_bsp/impl_bsp_servo.h
 * - impl_bsp/impl_bsp_key.h
 * - impl_bsp/impl_bsp_led.h
 * - impl_bsp/impl_bsp_buzzer.h
 * - impl_bsp/impl_bsp_battery.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Instantiate the board devices and register them with the platform.
 *
 * This is the only file that knows the C25 V2.4 wiring:
 *
 *   wheel A = TIM8 CH1/CH2 (PC6/PC7), encoder TIM5
 *   wheel B = TIM8 CH3/CH4 (PC8/PC9), encoder TIM4
 *   wheel C = TIM1 CH2N/CH3N (PB0/PB1), encoder TIM3
 *   wheel D = TIM1 CH1/CH4 (PA8/PA11), encoder TIM2
 *   servos  = PC13, PC14, PC15, PC0, PC1, PC2 on the TIM7 soft-PWM chain
 *   key PB2, LED PC4 (active low), buzzer PC5 (active high)
 *   battery divider x11 on ADC1_IN13 / PC3
 *
 * For every wheel the "reverse" input is the one the vendor holds at full
 * compare; the "forward" input is chopped (see bsp_motor_dual_pwm.h).
 * Wheels B and D are mounted turned around, so their invert_direction is set:
 * with the same positive duty all four wheels then roll the vehicle forward.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "impl_board/board_init.h"

#include "impl_bsp/impl_bsp_motor.h"
#include "impl_bsp/impl_bsp_servo.h"
#include "impl_bsp/impl_bsp_key.h"
#include "impl_bsp/impl_bsp_led.h"
#include "impl_bsp/impl_bsp_buzzer.h"
#include "impl_bsp/impl_bsp_battery.h"

#include "bsp_motor_dual_pwm.h"
#include "bsp_servo_softpwm.h"
#include "bsp_key_gpio.h"
#include "bsp_led_gpio.h"
#include "bsp_buzzer_gpio.h"
#include "bsp_battery_adc.h"

//******************************** Defines **********************************//

#define BOARD_MOTOR_COUNT       PLATFORM_MOTOR_ID_MAX

#define BOARD_GPIO_PORT_A       ((uint8_t)0u)
#define BOARD_GPIO_PORT_B       ((uint8_t)1u)
#define BOARD_GPIO_PORT_C       ((uint8_t)2u)

#define BOARD_ADC_CHANNEL       PLATFORM_ADC_CHANNEL_0
#define BOARD_ADC_DIVIDER       ((uint16_t)11u)

//******************************** Declaring ********************************//

typedef struct
{
    uint8_t         pwm_forward;
    uint8_t         pwm_reverse;
    uint8_t         encoder;
    platform_bool_t invert_direction;
} board_motor_wiring_t;

//******************************** Variables ********************************//

static bsp_motor_dual_pwm_t s_motor[BOARD_MOTOR_COUNT];
static bsp_servo_softpwm_t  s_servo;
static bsp_key_gpio_t       s_key;
static bsp_led_gpio_t       s_led;
static bsp_buzzer_gpio_t    s_buzzer;
static bsp_battery_adc_t    s_battery;

/*
 * pwm_forward is the bridge input the vendor chops for a positive speed, i.e.
 * the channel that gets "1000 - duty" permille.
 *
 * invert_direction records the mounting, not the algorithm: B and D sit on the
 * opposite side of the chassis and are turned around, so the same electrical
 * "forward" drives the vehicle backwards.  Bench test 2026-10-28: with
 * +300 permille on every wheel only A and C rolled the vehicle forward.
 *
 * It flips the encoder count as well, so "positive duty means increasing
 * count" still holds for every wheel and the control layer needs no per-wheel
 * sign. Swapping the two PWM channels instead would not do: the drive would
 * reverse while the encoder kept its old sign.
 */
static const board_motor_wiring_t s_motor_wiring[BOARD_MOTOR_COUNT] =
{
    /* A */ { PLATFORM_PWM_CHANNEL_0, PLATFORM_PWM_CHANNEL_1,
              PLATFORM_ENCODER_CHANNEL_3, PLATFORM_FALSE },
    /* B */ { PLATFORM_PWM_CHANNEL_2, PLATFORM_PWM_CHANNEL_3,
              PLATFORM_ENCODER_CHANNEL_2, PLATFORM_TRUE  },
    /* C */ { PLATFORM_PWM_CHANNEL_5, PLATFORM_PWM_CHANNEL_6,
              PLATFORM_ENCODER_CHANNEL_1, PLATFORM_FALSE },
    /* D */ { PLATFORM_PWM_CHANNEL_4, PLATFORM_PWM_CHANNEL_7,
              PLATFORM_ENCODER_CHANNEL_0, PLATFORM_TRUE  },
};

static const platform_gpio_t s_servo_pin[BSP_SERVO_JOINT_COUNT] =
{
    { BOARD_GPIO_PORT_C, 13u },
    { BOARD_GPIO_PORT_C, 14u },
    { BOARD_GPIO_PORT_C, 15u },
    { BOARD_GPIO_PORT_C, 0u },
    { BOARD_GPIO_PORT_C, 1u },
    { BOARD_GPIO_PORT_C, 2u },
};

//******************************** Functions ********************************//

static platform_err_t board_register_motors(void)
{
    platform_err_t err;
    uint8_t        i;

    for (i = 0u; i < BOARD_MOTOR_COUNT; i++)
    {
        s_motor[i].cfg.pwm_forward      = s_motor_wiring[i].pwm_forward;
        s_motor[i].cfg.pwm_reverse      = s_motor_wiring[i].pwm_reverse;
        s_motor[i].cfg.encoder          = s_motor_wiring[i].encoder;
        s_motor[i].cfg.invert_direction = s_motor_wiring[i].invert_direction;
        s_motor[i].cfg.pwm_frequency_hz = BSP_MOTOR_PWM_FREQUENCY_HZ;

        err = platform_motor_register((platform_motor_id_t)i,
                                      &impl_bsp_motor_ops,
                                      &s_motor[i]);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }
    }

    return PLATFORM_ERR_OK;
}

static platform_err_t board_register_servo(void)
{
    uint8_t i;

    for (i = 0u; i < BSP_SERVO_JOINT_COUNT; i++)
    {
        s_servo.cfg.pin[i] = s_servo_pin[i];
    }

    s_servo.cfg.timer                  = PLATFORM_TIMER_ID_0;
    s_servo.cfg.pulse_center_us        = BSP_SERVO_PULSE_CENTER_US;
    s_servo.cfg.pulse_span_us          = BSP_SERVO_PULSE_SPAN_US;
    s_servo.cfg.angle_limit_centi_deg  = BSP_SERVO_ANGLE_LIMIT;

    return platform_servo_register(&impl_bsp_servo_ops, &s_servo);
}

static platform_err_t board_register_switches(void)
{
    platform_err_t err;

    s_key.cfg.pin.port    = BOARD_GPIO_PORT_B;
    s_key.cfg.pin.pin     = 2u;
    s_key.cfg.pull        = PLATFORM_GPIO_PULL_DOWN;
    s_key.cfg.active_high = PLATFORM_TRUE;
    s_key.cfg.debounce_ms = BSP_KEY_DEBOUNCE_MS_DEFAULT;

    err = platform_key_register(PLATFORM_KEY_ID_0, &impl_bsp_key_ops, &s_key);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    s_led.cfg.pin.port   = BOARD_GPIO_PORT_C;
    s_led.cfg.pin.pin    = 4u;
    s_led.cfg.active_low = PLATFORM_TRUE;

    err = platform_led_register(&impl_bsp_led_ops, &s_led);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    s_buzzer.cfg.pin.port    = BOARD_GPIO_PORT_C;
    s_buzzer.cfg.pin.pin     = 5u;
    s_buzzer.cfg.active_high = PLATFORM_TRUE;

    return platform_buzzer_register(&impl_bsp_buzzer_ops, &s_buzzer);
}

static platform_err_t board_register_battery(void)
{
    s_battery.cfg.channel       = BOARD_ADC_CHANNEL;
    s_battery.cfg.divider_ratio = BOARD_ADC_DIVIDER;
    s_battery.cfg.reference_mv  = 0u;   /* Platform default (3.3 V). */

    return platform_battery_register(&impl_bsp_battery_ops, &s_battery);
}

platform_err_t board_init(void)
{
    platform_err_t err;
    uint8_t        i;

    err = board_register_motors();
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = board_register_servo();
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = board_register_switches();
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = board_register_battery();
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    /* Motors come up initialized but disabled: the control task enables them. */
    for (i = 0u; i < BOARD_MOTOR_COUNT; i++)
    {
        err = platform_motor_init((platform_motor_id_t)i);
        if (PLATFORM_IS_ERR(err))
        {
            return err;
        }
    }

    /*
     * The arm has no holding torque without a pulse train, and the vendor board
     * starts the waveform on boot at the center pulse, so do the same.
     */
    err = platform_servo_init();
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = platform_servo_set_enable(PLATFORM_TRUE);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = platform_key_init(PLATFORM_KEY_ID_0);
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = platform_led_init();
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    err = platform_buzzer_init();
    if (PLATFORM_IS_ERR(err))
    {
        return err;
    }

    return platform_battery_init();
}
