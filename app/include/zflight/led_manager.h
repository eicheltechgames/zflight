/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 *
 * @brief Public zflight LED Manager API
 */

#ifndef ZFLIGHT_APP_INCLUDE_LED_MANAGER_H_
#define ZFLIGHT_APP_INCLUDE_LED_MANAGER_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

enum zflight_led {
    ZFLIGHT_STATUS_LED,
    ZFLIGHT_NUM_LEDS
};

enum zflight_led_pattern_type {
    LED_PATTERN_INIT = 0,
    LED_PATTERN_IDLE,
    LED_PATTERN_MAINT,
    LED_PATTERN_ARM_FAIL,
    LED_PATTERN_ERROR,
    NUM_LED_PATTERN,
    LED_OFF = NUM_LED_PATTERN,
    LED_ON
};

enum zflight_led_pattern_mode {
    ZFLIGHT_LED_PATT_MODE_REPEAT,
    ZFLIGHT_LED_PATT_MODE_SINGLE,
};

int zflight_led_manager_set_pattern(
    enum zflight_led led,
    enum zflight_led_pattern_type pattern,
    enum zflight_led_pattern_mode mode);

static inline int zflight_led_manager_set(enum zflight_led led, bool on)
{
    return zflight_led_manager_set_pattern(led,
        on ? LED_ON : LED_OFF, ZFLIGHT_LED_PATT_MODE_REPEAT);
}

int zflight_led_manager_init();

#ifdef __cplusplus
}
#endif

#endif /* ZFLIGHT_APP_INCLUDE_LED_MANAGER_H_ */