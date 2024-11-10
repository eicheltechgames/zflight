/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zflight/control.h>
#include <zflight/flight_task.h>

#include <zephyr/smf.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_backend.h>

/* Forward declaration of state table */
static const struct smf_state zflight_states[];
enum zflight_state curr_state;

/* User defined object */
struct s_object {
        /* This must be first */
        struct smf_ctx ctx;

        /* Other state specific data add here */
} s_obj;

/* IDLE State */
static void zflight_idle_entry(void *o)
{
    curr_state = ZFLIGHT_STATE_IDLE;
    /* Start idle LED pattern */
}

static void zflight_idle_run(void *o)
{
    /* Poll for arm conditions */
    /* Poll for terminal conditions */

    // temp
    smf_set_state(SMF_CTX(&s_obj), &zflight_states[ZFLIGHT_STATE_MAINT]);
}

static void zflight_idle_exit(void *o)
{
    /* If armed, start armed LED pattern, else stop LEDs */
}

/* MAINT State */
static void zflight_maint_entry(void *o)
{
    curr_state = ZFLIGHT_STATE_MAINT;
    /* Start maint LED pattern */
    shell_start(shell_backend_get(0));
    flight_task_start(1000);
}

static void zflight_maint_run(void *o)
{
    /* TBD */
}

static void zflight_maint_exit(void *o)
{
    flight_task_stop();
    shell_stop(shell_backend_get(0));
    /* Stop LEDs */
}

/* FLIGHT State */
static void zflight_flight_entry(void *o)
{
    curr_state = ZFLIGHT_STATE_FLIGHT;
    /* Start flight task */
}

static void zflight_flight_run(void *o)
{
    /* Execute control modes (substates?) */
}

static void zflight_flight_exit(void *o)
{
    /* Stop flight task */
}

/* Populate state table */
static const struct smf_state zflight_states[] = {
        [ZFLIGHT_STATE_IDLE] = SMF_CREATE_STATE(zflight_idle_entry,
                                                zflight_idle_run,
                                                zflight_idle_exit,
                                                NULL, NULL),
        [ZFLIGHT_STATE_MAINT] = SMF_CREATE_STATE(zflight_maint_entry,
                                                zflight_maint_run,
                                                zflight_maint_exit,
                                                NULL, NULL),
        [ZFLIGHT_STATE_FLIGHT] = SMF_CREATE_STATE(zflight_flight_entry,
                                                zflight_flight_run,
                                                zflight_flight_exit,
                                                NULL, NULL),
};

enum zflight_state zflight_get_state(void)
{
    return curr_state;
}

int zflight_init(void)
{
    smf_set_initial(SMF_CTX(&s_obj), &zflight_states[ZFLIGHT_STATE_IDLE]);
    return 0;
}

int zflight_run(void)
{
    int ret = smf_run_state(SMF_CTX(&s_obj));
    return ret;
}
