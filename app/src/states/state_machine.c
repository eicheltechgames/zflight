/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zflight/app.h>
#include <zephyr/smf.h>

#include "state_machine.h"
#include "init_state.h"
#include "idle_state.h"
#include "maint_state.h"
#include "flight_state.h"

/* Populate state table */
static const struct smf_state zflight_states[] = {
    [ZFLIGHT_APP_STATE_INIT] = SMF_CREATE_STATE(zflight_init_state_entry,
                                                zflight_init_state_run,
                                                zflight_init_state_exit,
                                                NULL, NULL),
    [ZFLIGHT_APP_STATE_IDLE] = SMF_CREATE_STATE(zflight_idle_state_entry,
                                                zflight_idle_state_run,
                                                zflight_idle_state_exit,
                                                NULL, NULL),
    [ZFLIGHT_APP_STATE_MAINT] = SMF_CREATE_STATE(zflight_maint_state_entry,
                                                zflight_maint_state_run,
                                                zflight_maint_state_exit,
                                                NULL, NULL),
    [ZFLIGHT_APP_STATE_FLIGHT] = SMF_CREATE_STATE(zflight_flight_entry,
                                                zflight_flight_run,
                                                zflight_flight_exit,
                                                NULL, NULL),
};

int zflight_smf_init(struct zflight_app *app)
{
    smf_set_initial(SMF_CTX(app), &zflight_states[app->state]);
    return app->err;
}

int zflight_smf_run(struct zflight_app *app)
{
    int err = 0;

    err = smf_run_state(SMF_CTX(app));
    if (err) {
        err = app->err;
    }
    
    return err;
}

void zflight_smf_set_state(struct zflight_app *app,
                        enum zflight_app_state new_state)
{
    smf_set_state(SMF_CTX(app), &zflight_states[new_state]);
}