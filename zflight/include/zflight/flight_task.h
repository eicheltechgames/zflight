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

#ifndef ZFLIGHT_INCLUDE_FLIGHT_TASK_H_
#define ZFLIGHT_INCLUDE_FLIGHT_TASK_H_

#include <stdint.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

int flight_task_start(uint16_t task_frequency);

void flight_task_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* ZFLIGHT_INCLUDE_FLIGHT_TASK_H_ */