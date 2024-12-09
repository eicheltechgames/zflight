/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 *
 * @brief Private zflight state machine API
 */

#ifndef ZFLIGHT_APP_SRC_STATES_STATE_MACHINE_H_
#define ZFLIGHT_APP_SRC_STATES_STATE_MACHINE_H_

#include <zflight/app.h>

#ifdef __cplusplus
extern "C" {
#endif

int zflight_smf_init(struct zflight_app *app);

int zflight_smf_run(struct zflight_app *app);

void zflight_smf_set_state(struct zflight_app *app,
                        enum zflight_app_state new_state);

static inline enum zflight_app_state
    zflight_smf_get_state(struct zflight_app *app)
{
    return app->state;
}

#ifdef __cplusplus
}
#endif

#endif /* ZFLIGHT_APP_SRC_STATES_STATE_MACHINE_H_ */