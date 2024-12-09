/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zflight/app.h>
#include "states/state_machine.h"

struct zflight_app app;

int zflight_app_init()
{
    // app.ctx init with zflight_smf_init() below
    app.state = ZFLIGHT_APP_STATE_INIT;
    for (int i = 0; i < ZFLIGHT_APP_NUM_SIGNALS; ++i) {
        k_poll_signal_init(&app.signals[i]);
    }
    app.arm_flags = 0;

    return zflight_smf_init(&app);
}

int zflight_app_run()
{
    int err = 0;

    while(1) {
        err = zflight_smf_run(&app);
        if (err) {
            break;
        }
    }

    // @todo save error to flash?

    return err;
}

static inline void handle_terminal_event(int event_data)
{
    (void)k_poll_signal_raise(
        &app.signals[ZFLIGHT_APP_SIGNAL_TERMINAL], event_data);
}

static inline void handle_arm_switch_event(int event_data)
{
    (void)k_poll_signal_raise(
        &app.signals[ZFLIGHT_APP_SIGNAL_ARMED], event_data);
}

void zflight_app_notify_event(enum zflight_app_event event, int event_data)
{
    switch (event)
    {
    case ZFLIGHT_APP_EVENT_TERMINAL:
        handle_terminal_event(event_data);
        break;
    case ZFLIGHT_APP_EVENT_ARM_SWITCH:
        handle_arm_switch_event(event_data);
        break;
    default:
        break;
    }
}

int zflight_app_arming_prohinited()
{
    return 1;
}