/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zflight/app.h>
#include <zflight/led_manager.h>
#include <zflight/terminal.h>

#include "init_state.h"
#include "state_machine.h"

void zflight_init_state_entry(void *app_ptr)
{
    int err;
    struct zflight_app *app = app_ptr;
    app->state = ZFLIGHT_APP_STATE_INIT;

    // init leds
    err = zflight_led_manager_init();
    if (err) {
        app->err = err;
        return;
    }
    (void)zflight_led_manager_set_pattern(
        ZFLIGHT_STATUS_LED, LED_PATTERN_INIT, ZFLIGHT_LED_PATT_MODE_SINGLE);

    // init terminal
    err = zflight_terminal_init();
    if (err) {
        app->err = err;
        return;
    }
}

void zflight_init_state_run(void *app_ptr)
{
    struct zflight_app *app = app_ptr;

    // @todo wait for init events to complete
    zflight_smf_set_state(app, ZFLIGHT_APP_STATE_IDLE);
}

void zflight_init_state_exit(void *app_ptr)
{
    ARG_UNUSED(app_ptr);
}