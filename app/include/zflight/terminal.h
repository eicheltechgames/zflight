/*
 * Copyright (c) 2016, Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 *
 * @brief Public kernel APIs.
 */

#ifndef ZFLIGHT_INCLUDE_TERMINAL_H_
#define ZFLIGHT_INCLUDE_TERMINAL_H_

#ifdef __cplusplus
extern "C" {
#endif

enum zflight_terminal_events {
    ZFLIGHT_TERMINAL_DISCONNECTED = 0,
    ZFLIGHT_TERMINAL_CONNECTED
};

int zflight_terminal_start(void);

int zflight_terminal_stop(void);

int zflight_terminal_init(void);

#ifdef __cplusplus
}
#endif

#endif /* ZFLIGHT_INCLUDE_TERMINAL_H_ */