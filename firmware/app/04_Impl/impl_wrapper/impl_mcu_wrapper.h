/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_mcu_wrapper.h
 *
 * @par dependencies
 * - platform_mcu.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Declare the Impl wrapper entry points for the MCU platform.
 *
 * 03_Platform/platform_mcu/platform_mcu_*.c calls only these functions.
 * A wrapper does two things and nothing else: emit trace hooks around the
 * call, then call the
 * implementation adapter in 04_Impl/impl_mcu. Parameter validation, unit
 * conversion and vendor API calls belong to the implementation layer.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __IMPL_MCU_WRAPPER_H__
#define __IMPL_MCU_WRAPPER_H__

//******************************** Includes *********************************//

#include "platform_mcu.h"

//******************************** Declaring ********************************//

/* -------------------------------- GPIO --------------------------------- */

platform_err_t impl_gpio_init_wrap(const platform_gpio_cfg_t *cfg);
platform_err_t impl_gpio_deinit_wrap(platform_gpio_t pin);
platform_err_t impl_gpio_write_wrap(platform_gpio_t pin,
                                    platform_gpio_level_t level);
platform_err_t impl_gpio_read_wrap(platform_gpio_t pin,
                                   platform_gpio_level_t *level);
platform_err_t impl_gpio_toggle_wrap(platform_gpio_t pin);
platform_err_t impl_gpio_irq_attach_wrap(platform_gpio_t pin,
                                         platform_gpio_irq_edge_t edge,
                                         platform_gpio_irq_cb_t callback,
                                         void *user_data);
platform_err_t impl_gpio_irq_detach_wrap(platform_gpio_t pin);

/* -------------------------------- UART --------------------------------- */

platform_err_t impl_uart_init_wrap(const platform_uart_cfg_t *cfg);
platform_err_t impl_uart_deinit_wrap(platform_uart_bus_t bus);
platform_err_t impl_uart_write_wrap(platform_uart_bus_t bus,
                                    const uint8_t *data,
                                    uint32_t len,
                                    uint32_t timeout_ms);
platform_err_t impl_uart_read_wrap(platform_uart_bus_t bus,
                                   uint8_t *data,
                                   uint32_t len,
                                   uint32_t timeout_ms);
platform_err_t impl_uart_write_async_wrap(platform_uart_bus_t bus,
                                          const uint8_t *data,
                                          uint32_t len);
platform_err_t impl_uart_read_async_wrap(platform_uart_bus_t bus,
                                         uint8_t *data,
                                         uint32_t max_len);
platform_err_t impl_uart_get_rx_count_wrap(platform_uart_bus_t bus,
                                           uint32_t *rx_len);
platform_err_t impl_uart_rx_read_wrap(platform_uart_bus_t bus,
                                      uint8_t *data,
                                      uint32_t max_len,
                                      uint32_t *out_len);
platform_err_t impl_uart_rx_available_wrap(platform_uart_bus_t bus,
                                           uint32_t *count);
platform_err_t impl_uart_register_event_cb_wrap(
                                        platform_uart_bus_t bus,
                                        platform_uart_event_cb_t callback,
                                        void *user_data);
platform_err_t impl_uart_flush_rx_wrap(platform_uart_bus_t bus);

/* --------------------------------- SPI --------------------------------- */

platform_err_t impl_spi_init_wrap(const platform_spi_cfg_t *cfg);
platform_err_t impl_spi_deinit_wrap(platform_spi_bus_t bus);
platform_err_t impl_spi_transfer_wrap(platform_spi_bus_t bus,
                                      const uint8_t *tx_data,
                                      uint8_t *rx_data,
                                      uint32_t len,
                                      uint32_t timeout_ms);
platform_err_t impl_spi_write_wrap(platform_spi_bus_t bus,
                                   const uint8_t *tx_data,
                                   uint32_t len,
                                   uint32_t timeout_ms);
platform_err_t impl_spi_read_wrap(platform_spi_bus_t bus,
                                  uint8_t *rx_data,
                                  uint32_t len,
                                  uint32_t timeout_ms);
platform_err_t impl_spi_write_async_wrap(platform_spi_bus_t bus,
                                         const uint8_t *tx_data,
                                         uint32_t len);
platform_err_t impl_spi_register_complete_cb_wrap(
                                        platform_spi_bus_t bus,
                                        platform_spi_complete_cb_t callback,
                                        void *user_data);
platform_err_t impl_spi_cs_set_wrap(platform_spi_bus_t bus,
                                    platform_gpio_level_t level);
platform_err_t impl_spi_lock_wrap(platform_spi_bus_t bus,
                                  uint32_t timeout_ms);
platform_err_t impl_spi_unlock_wrap(platform_spi_bus_t bus);

/* --------------------------------- I2C --------------------------------- */

platform_err_t impl_i2c_init_wrap(const platform_i2c_cfg_t *cfg);
platform_err_t impl_i2c_deinit_wrap(platform_i2c_bus_t bus);
platform_err_t impl_i2c_write_wrap(platform_i2c_bus_t bus,
                                   uint16_t dev_addr,
                                   const uint8_t *data,
                                   uint32_t len,
                                   uint32_t timeout_ms);
platform_err_t impl_i2c_read_wrap(platform_i2c_bus_t bus,
                                  uint16_t dev_addr,
                                  uint8_t *data,
                                  uint32_t len,
                                  uint32_t timeout_ms);
platform_err_t impl_i2c_mem_write_wrap(platform_i2c_bus_t bus,
                                       uint16_t dev_addr,
                                       uint16_t mem_addr,
                                       uint8_t mem_addr_size_bytes,
                                       const uint8_t *data,
                                       uint32_t len,
                                       uint32_t timeout_ms);
platform_err_t impl_i2c_mem_read_wrap(platform_i2c_bus_t bus,
                                      uint16_t dev_addr,
                                      uint16_t mem_addr,
                                      uint8_t mem_addr_size_bytes,
                                      uint8_t *data,
                                      uint32_t len,
                                      uint32_t timeout_ms);
platform_err_t impl_i2c_is_ready_wrap(platform_i2c_bus_t bus,
                                      uint16_t dev_addr,
                                      uint32_t trials,
                                      uint32_t timeout_ms,
                                      platform_bool_t *ready);
platform_err_t impl_i2c_lock_wrap(platform_i2c_bus_t bus,
                                  uint32_t timeout_ms);
platform_err_t impl_i2c_unlock_wrap(platform_i2c_bus_t bus);

/* --------------------------------- PWM --------------------------------- */

platform_err_t impl_pwm_init_wrap(const platform_pwm_cfg_t *cfg);
platform_err_t impl_pwm_deinit_wrap(platform_pwm_channel_t channel);
platform_err_t impl_pwm_start_wrap(platform_pwm_channel_t channel);
platform_err_t impl_pwm_stop_wrap(platform_pwm_channel_t channel);
platform_err_t impl_pwm_set_duty_wrap(platform_pwm_channel_t channel,
                                      uint16_t duty_permille);
platform_err_t impl_pwm_set_frequency_wrap(platform_pwm_channel_t channel,
                                            uint32_t frequency_hz);
platform_err_t impl_pwm_get_duty_wrap(platform_pwm_channel_t channel,
                                      uint16_t *duty_permille);

/* --------------------------------- ADC --------------------------------- */

platform_err_t impl_adc_init_wrap(const platform_adc_cfg_t *cfg);
platform_err_t impl_adc_deinit_wrap(platform_adc_channel_t channel);
platform_err_t impl_adc_read_raw_wrap(platform_adc_channel_t channel,
                                      uint16_t *raw_value);
platform_err_t impl_adc_read_mv_wrap(platform_adc_channel_t channel,
                                     uint32_t *millivolts);

/* ------------------------------- ENCODER -------------------------------- */

platform_err_t impl_encoder_init_wrap(const platform_encoder_cfg_t *cfg);
platform_err_t impl_encoder_deinit_wrap(platform_encoder_channel_t channel);
platform_err_t impl_encoder_read_delta_wrap(platform_encoder_channel_t channel,
                                            int16_t *delta);
platform_err_t impl_encoder_get_count_wrap(platform_encoder_channel_t channel,
                                           int16_t *count);
platform_err_t impl_encoder_reset_wrap(platform_encoder_channel_t channel);

/* -------------------------------- TIMER --------------------------------- */

platform_err_t impl_timer_init_wrap(const platform_timer_cfg_t *cfg);
platform_err_t impl_timer_deinit_wrap(platform_timer_id_t timer);
platform_err_t impl_timer_start_wrap(platform_timer_id_t timer);
platform_err_t impl_timer_stop_wrap(platform_timer_id_t timer);
platform_err_t impl_timer_set_next_interval_us_wrap(platform_timer_id_t timer,
                                                    uint32_t interval_us);

/* --------------------------------- WDG --------------------------------- */

platform_err_t impl_wdg_init_wrap(uint32_t timeout_ms);
platform_err_t impl_wdg_feed_wrap(void);
platform_err_t impl_wdg_get_reset_reason_wrap(
                                        platform_wdg_reset_reason_t *reason);

#endif /* __IMPL_MCU_WRAPPER_H__ */
