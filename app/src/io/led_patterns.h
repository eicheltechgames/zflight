/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 *
 * @brief Private zflight led patterns API
 */

#ifndef ZFLIGHT_APP_SRC_IO_LED_PATTERNS_H_
#define ZFLIGHT_APP_SRC_IO_LED_PATTERNS_H_

#include <zflight/led_controller.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct led_pattern {
    uint8_t type;
    uint8_t num_toggles;
    const uint16_t *toggle_ms;
};

const struct led_pattern *get_led_pattern(enum zflight_led_pattern_type pattern);

#ifdef __cplusplus
}
#endif

#endif /* ZFLIGHT_APP_SRC_IO_LED_PATTERNS_H_ */