/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util.h>

#include "led_patterns.h"

static const uint16_t init_toggles[] = {
        // on
    63, // off
    62, // on
    62, // off
    63, // on
    63, // off
    // = 313 ms
};

static const uint16_t idle_toggles[] = {
          // on
    1000, // off
    1000, // on
    // = 2 seconds
};

static const uint16_t maint_toggles[] = {
         // on
    500, // off
    500, // on
    // = 1 second
};

static const uint16_t arm_fail_toggles[] = {
         // on
    250, // off
    250, // on
    250, // off
    250, // on
    250, // off
    250, // on
    250, // off
    250, // on
    // = 2 seconds
};

static const uint16_t error_toggles[] = {
        // on
    63, // off
    62, // on
    // = 125 ms
};

static const struct led_pattern led_patterns[NUM_LED_PATTERN] = {
/*   type                   num_toggles                     toggle_ms   */
    {LED_PATTERN_INIT,      ARRAY_SIZE(init_toggles),       init_toggles},
    {LED_PATTERN_IDLE,      ARRAY_SIZE(idle_toggles),       idle_toggles},
    {LED_PATTERN_MAINT,     ARRAY_SIZE(maint_toggles),      maint_toggles},
    {LED_PATTERN_ARM_FAIL,  ARRAY_SIZE(arm_fail_toggles),   arm_fail_toggles},
    {LED_PATTERN_ERROR,     ARRAY_SIZE(error_toggles),      error_toggles},
};

const struct led_pattern *
    get_led_pattern(enum zflight_led_pattern_type pattern)
{
    return pattern >= NUM_LED_PATTERN ? NULL : &led_patterns[pattern];
}