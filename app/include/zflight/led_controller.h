/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 *
 * @brief Public zflight LED Controller API
 */

#ifndef ZFLIGHT_APP_INCLUDE_LED_CONTROLLER_H_
#define ZFLIGHT_APP_INCLUDE_LED_CONTROLLER_H_

#include <zflight/led_manager.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/ring_buffer.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ZFLIGHT_LED_COMMAND_QUEUE_LEN   4

struct zflight_led_command {
    uint32_t pattern    : 16;
    uint32_t mode       : 16;   
};

struct zflight_led_controller {
    struct gpio_dt_spec led;
    struct zflight_led_command active_cmd;
    struct zflight_led_command base_cmd;
    unsigned int command_index;
    struct ring_buf led_commands;
    uint32_t led_commands_buf[ZFLIGHT_LED_COMMAND_QUEUE_LEN];
    struct k_work_delayable led_toggle_work;
};

int zflight_led_controller_set_pattern(
    struct zflight_led_controller *controller,
    enum zflight_led_pattern_type pattern,
    enum zflight_led_pattern_mode mode);

int zflight_led_controller_init(
    struct zflight_led_controller *controller,
    const struct gpio_dt_spec *led);

#ifdef __cplusplus
}
#endif

#endif /* ZFLIGHT_APP_INCLUDE_LED_CONTROLLER_H_ */