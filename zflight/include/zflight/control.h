/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZFLIGHT_INCLUDE_CONTROL_H_
#define ZFLIGHT_INCLUDE_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

enum zflight_state
{
    ZFLIGHT_STATE_IDLE,
    ZFLIGHT_STATE_MAINT,
    ZFLIGHT_STATE_FLIGHT
};

int zflight_init(void);

int zflight_run(void);

enum zflight_state zflight_get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* ZFLIGHT_INCLUDE_CONTROL_H_ */