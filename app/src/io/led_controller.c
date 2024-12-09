/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zflight/led_controller.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/ring_buffer.h>

#include <string.h>

#include "led_patterns.h"

void led_toggle_handler(struct k_work *work)
{
    int err;
    bool try_dequeue_next_command = true;
    struct k_work_delayable *work_delayable = 
        k_work_delayable_from_work(work);
    struct zflight_led_controller *controller = 
        CONTAINER_OF(work_delayable,
            struct zflight_led_controller,
            led_toggle_work);
    const struct led_pattern *curr_patt =
        get_led_pattern(controller->active_cmd.pattern);

    if (curr_patt) {
        try_dequeue_next_command = 
            controller->command_index >= curr_patt->num_toggles;
    }
    
    if (try_dequeue_next_command) {
        controller->command_index = 0;
        // Store the base command for future use if it is a repeater
        if (controller->active_cmd.mode == ZFLIGHT_LED_PATT_MODE_REPEAT) {
            controller->base_cmd = controller->active_cmd;
        }

        // Try to get the next command from the ring buffer
        struct zflight_led_command next_cmd;
        uint16_t tmp1 = 0;
        uint8_t tmp2 = 0;
        uint8_t sz = 1;
        err = ring_buf_item_get(&controller->led_commands,
            &tmp1, &tmp2, (uint32_t *)&next_cmd, &sz);

        // If there is no next command, then use the base command
        controller->active_cmd = err ? controller->base_cmd : next_cmd;

        // All patterns start with LED on, except LED off
        gpio_pin_set_dt(&controller->led,
            controller->active_cmd.mode != LED_OFF);
    } else {
        gpio_pin_toggle_dt(&controller->led);
    }

    if (controller->active_cmd.pattern < NUM_LED_PATTERN) {
        curr_patt = get_led_pattern(controller->active_cmd.pattern);
        k_work_reschedule(work_delayable,
            K_MSEC(curr_patt->toggle_ms[controller->command_index]));
        controller->command_index++;
    }
}

int zflight_led_controller_set_pattern(
    struct zflight_led_controller *controller,
    enum zflight_led_pattern_type pattern,
    enum zflight_led_pattern_mode mode)
{
    int err;
    if (!controller) {
        return -EINVAL;
    }

    struct zflight_led_command command = {
        .pattern = pattern,
        .mode = mode
    };

    err = ring_buf_item_put(&controller->led_commands,
        0, 0, (uint32_t *)&command, 1);
    if (err) {
        return err;
    }

    err = k_work_schedule(&controller->led_toggle_work, K_NO_WAIT);
    if (err < 0) {
        return err;
    }

    return 0;
}

// @todo bring up with system init from device tree
int zflight_led_controller_init(struct zflight_led_controller *controller,
                                const struct gpio_dt_spec *led)
{
    int err;

    if (!controller || !led) {
        return -EINVAL;
    }

    if (!gpio_is_ready_dt(led)) {
        return -EIO;
    }

    err = gpio_pin_configure_dt(led, GPIO_OUTPUT_ACTIVE);
    if (err) {
        return err;
    }

    err = gpio_pin_set_dt(led, 0);
    if (err) {
        return err;
    }

    struct zflight_led_command led_off = {
        .pattern = LED_OFF,
        .mode = ZFLIGHT_LED_PATT_MODE_REPEAT,
    };

    memcpy(&controller->led, led, sizeof(controller->led));
    controller->active_cmd = led_off;
    controller->base_cmd = led_off;
    controller->command_index = 0;
    ring_buf_item_init(&controller->led_commands,
                  ZFLIGHT_LED_COMMAND_QUEUE_LEN,
                  (uint32_t *)&controller->led_commands_buf[0]);
    memset(&controller->led_commands_buf, 0,
        sizeof(controller->led_commands_buf));
    k_work_init_delayable(&controller->led_toggle_work, led_toggle_handler);

    return 0;
}