/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 *
 * @brief Public zflight app API
 */

#ifndef ZFLIGHT_APP_INCLUDE_APP_H_
#define ZFLIGHT_APP_INCLUDE_APP_H_

#include <zephyr/kernel.h>
#include <zephyr/smf.h>
#include <zephyr/sys/atomic.h>

#ifdef __cplusplus
extern "C" {
#endif

enum zflight_app_state {
    ZFLIGHT_APP_STATE_INIT = 0,
    ZFLIGHT_APP_STATE_IDLE,
    ZFLIGHT_APP_STATE_MAINT,
    ZFLIGHT_APP_STATE_FLIGHT,
    ZFLIGHT_APP_NUM_STATES
};

enum zflight_app_event {
    ZFLIGHT_APP_EVENT_TERMINAL = 0,
    ZFLIGHT_APP_EVENT_ARM_SWITCH,
    ZFLIGHT_APP_NUM_EVENTS
};

enum zflight_app_signal {
    ZFLIGHT_APP_SIGNAL_TERMINAL = 0,
    ZFLIGHT_APP_SIGNAL_ARMED,
    ZFLIGHT_APP_NUM_SIGNALS
};

struct zflight_app {
    struct smf_ctx ctx;
    enum zflight_app_state state;
    struct k_poll_signal signals[ZFLIGHT_APP_NUM_SIGNALS];
    atomic_t arm_flags;
    int err;
};

int zflight_app_init();

int zflight_app_run();

void zflight_app_notify_event(enum zflight_app_event event, int event_data);

int zflight_app_arming_prohinited();

#ifdef __cplusplus
}
#endif

#endif /* ZFLIGHT_APP_INCLUDE_APP_H_ */