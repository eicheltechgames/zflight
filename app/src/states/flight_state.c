/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zflight/app.h>
#include <zflight/led_manager.h>

#include "flight_state.h"
#include "state_machine.h"

void zflight_flight_entry(void *app_ptr)
{
    struct zflight_app *app = app_ptr;
    app->state = ZFLIGHT_APP_STATE_FLIGHT;
    (void)zflight_led_manager_set(ZFLIGHT_STATUS_LED, true);
}

void zflight_flight_run(void *app_ptr)
{
    ARG_UNUSED(app_ptr);
    // struct zflight_app *app = app_ptr;

    // @todo wait for disarmed
}

void zflight_flight_exit(void *app_ptr)
{
    ARG_UNUSED(app_ptr);
}