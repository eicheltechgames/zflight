/**
 * @file
 *
 * @brief Public APIs for the ESC Manager.
 */

/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZFLIGHT_INCLUDE_ESC_MANAGER_H_
#define ZFLIGHT_INCLUDE_ESC_MANAGER_H_

#include <errno.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int esc_manager_init(void);

// will override throttles when terminal is active
int esc_manager_set_throttles(const float throttles[CONFIG_ZFLIGHT_NUM_MOTORS]);

// @todo get ESC telemetry

#ifdef __cplusplus
}
#endif

#endif /* ZFLIGHT_INCLUDE_ESC_MANAGER_H_ */