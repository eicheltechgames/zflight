/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zflight/flight_task.h>
#include <zflight/esc_manager.h>

#include <zephyr/drivers/counter.h>

#include "flight_control.h"

#define FLIGHT_COUNTER_NODE DT_CHOSEN(zflight_flight_counter)

void flight_task(const struct device *dev, void *user_data);

// @todo need to check device_is_ready() during init
static const struct device *counter_dev = DEVICE_DT_GET(FLIGHT_COUNTER_NODE);
struct counter_top_cfg counter_config = {
    .callback = flight_task,
    .user_data = NULL,
    .flags = COUNTER_CONFIG_INFO_COUNT_UP,
};
static bool running = false;

void flight_task(const struct device *dev, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    struct flight_control_inputs set_points = { 0 };
    struct flight_control_IMU_inputs imu_inputs = { 0 };
    float throttles[CONFIG_ZFLIGHT_NUM_MOTORS] = { 0 };

    // @todo aquire IMU data

    // @todo acquire Control Task setpoints

    flight_control(&set_points, &imu_inputs, throttles);

    // @todo ESC telemetry

    esc_manager_set_throttles(throttles);
    // @todo -EBUSY means this loop is running faster
    // than the ESC can handle
}

int flight_task_start(uint16_t task_frequency)
{
    if (running) {
        return -EALREADY;
    }

    if (!IN_RANGE(task_frequency, 100, 10000)) {
        return -EINVAL;
    }

    counter_config.ticks = counter_us_to_ticks(counter_dev,
        MHZ(1)/task_frequency);
    uint32_t freq = counter_get_frequency(counter_dev);

    // @note this resets the counter to 0
    int err = counter_set_top_value(counter_dev, &counter_config);
    if (err) {
        return err;
    }

    running = true;
    err = counter_start(counter_dev);
    if (err) {
        return err;
    }

    return freq;
}

void flight_task_stop(void)
{
    counter_stop(counter_dev);
    running = false;
}
