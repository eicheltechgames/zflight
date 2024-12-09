/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zflight/app.h>
#include <zflight/terminal.h>
#include <zflight/led_manager.h>

#include "maint_state.h"
#include "state_machine.h"

void zflight_maint_state_entry(void *app_ptr)
{
    int err;
    struct zflight_app *app = app_ptr;

    app->state = ZFLIGHT_APP_STATE_MAINT;
    (void)zflight_led_manager_set_pattern(
        ZFLIGHT_STATUS_LED, LED_PATTERN_MAINT, ZFLIGHT_LED_PATT_MODE_REPEAT);
    err = zflight_terminal_start();
    if (err) {
        app->err = err;
        return;
    }
}

void zflight_maint_state_run(void *app_ptr)
{
    int err;
    int signaled;
    int event_data;
    struct zflight_app *app = app_ptr;

    struct k_poll_event maint_events[] = {
        K_POLL_EVENT_INITIALIZER(
            K_POLL_TYPE_SIGNAL,
            K_POLL_MODE_NOTIFY_ONLY,
            &app->signals[ZFLIGHT_APP_SIGNAL_TERMINAL]),
    };

    err = k_poll(maint_events, ARRAY_SIZE(maint_events), K_FOREVER);
    if (err) {
        app->err = err;
        return;
    }

    // Check if terminal was disconnected
    k_poll_signal_check(
        &app->signals[ZFLIGHT_APP_SIGNAL_TERMINAL], &signaled, &event_data);
    if (signaled && event_data == ZFLIGHT_TERMINAL_DISCONNECTED) {
        k_poll_signal_reset(&app->signals[ZFLIGHT_APP_SIGNAL_TERMINAL]);
        zflight_smf_set_state(app, ZFLIGHT_APP_STATE_IDLE);
    }
}

void zflight_maint_state_exit(void *app_ptr)
{
    int err;
    struct zflight_app *app = app_ptr;

    err = zflight_terminal_stop();
    if (err) {
        app->err = err;
        return;
    }
}