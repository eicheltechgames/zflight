/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zflight/led_manager.h>
#include <zflight/led_controller.h>

#include <zephyr/drivers/gpio.h>

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec status_led =
    GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static struct zflight_led_controller led_controllers[ZFLIGHT_NUM_LEDS];

int zflight_led_manager_set_pattern(
    enum zflight_led led,
    enum zflight_led_pattern_type pattern,
    enum zflight_led_pattern_mode mode)
{
    if (led >= ZFLIGHT_NUM_LEDS) {
        return -EINVAL;
    }

    return zflight_led_controller_set_pattern(
        &led_controllers[led], pattern, mode);
}

int zflight_led_manager_init()
{
    return zflight_led_controller_init(
        &led_controllers[ZFLIGHT_STATUS_LED], &status_led);
}