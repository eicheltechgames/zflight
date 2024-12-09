/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 *
 * @brief Private zflight init state API
 */

#ifndef ZFLIGHT_APP_SRC_STATES_INIT_STATE_H_
#define ZFLIGHT_APP_SRC_STATES_INIT_STATE_H_

#ifdef __cplusplus
extern "C" {
#endif

void zflight_init_state_entry(void *app_ptr);

void zflight_init_state_run(void *app_ptr);

void zflight_init_state_exit(void *app_ptr);

#ifdef __cplusplus
}
#endif

#endif /* ZFLIGHT_APP_SRC_STATES_INIT_STATE_H_ */