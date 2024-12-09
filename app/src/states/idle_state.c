/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zflight/app.h>
#include <zflight/terminal.h>
#include <zflight/led_manager.h>

#include <zephyr/kernel.h>

#include "idle_state.h"
#include "state_machine.h"

void zflight_idle_state_entry(void *app_ptr)
{
    struct zflight_app *app = app_ptr;
    app->state = ZFLIGHT_APP_STATE_IDLE;
    (void)zflight_led_manager_set_pattern(
        ZFLIGHT_STATUS_LED, LED_PATTERN_IDLE, ZFLIGHT_LED_PATT_MODE_REPEAT);
}

void zflight_idle_state_run(void *app_ptr)
{
    int err;
    int signaled;
    int event_data;
    struct zflight_app *app = app_ptr;

    struct k_poll_event idle_events[] = {
        K_POLL_EVENT_INITIALIZER(
            K_POLL_TYPE_SIGNAL,
            K_POLL_MODE_NOTIFY_ONLY,
            &app->signals[ZFLIGHT_APP_SIGNAL_ARMED]),
        K_POLL_EVENT_INITIALIZER(
            K_POLL_TYPE_SIGNAL,
            K_POLL_MODE_NOTIFY_ONLY,
            &app->signals[ZFLIGHT_APP_SIGNAL_TERMINAL]),
    };

    err = k_poll(idle_events, ARRAY_SIZE(idle_events), K_FOREVER);
    if (err) {
        app->err = err;
        return;
    }

    // Check if arm switch was set
    k_poll_signal_check(
        &app->signals[ZFLIGHT_APP_SIGNAL_ARMED], &signaled, &event_data);
    if (signaled && event_data == 1 /* @todo replace with receiver value*/) {
        k_poll_signal_reset(&app->signals[ZFLIGHT_APP_SIGNAL_ARMED]);
        if (!zflight_app_arming_prohinited()) {
            zflight_smf_set_state(app, ZFLIGHT_APP_STATE_FLIGHT);
            return;
        } else {
            // @todo play armed failed sequence
        }
    }

    // Check if terminal was connected
    k_poll_signal_check(
        &app->signals[ZFLIGHT_APP_SIGNAL_TERMINAL], &signaled, &event_data);
    if (signaled && event_data == ZFLIGHT_TERMINAL_CONNECTED) {
        k_poll_signal_reset(&app->signals[ZFLIGHT_APP_SIGNAL_TERMINAL]);
        zflight_smf_set_state(app, ZFLIGHT_APP_STATE_MAINT);
    }
}

void zflight_idle_state_exit(void *app_ptr)
{
    ARG_UNUSED(app_ptr);
}
