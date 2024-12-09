/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zflight/app.h>

int main(void)
{
    int err;

    err = zflight_app_init();
    if (!err) {
        err = zflight_app_run();
    }

    /* Should never get here */
    return err;
}
