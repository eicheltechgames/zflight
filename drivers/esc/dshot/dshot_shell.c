/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief DShot shell commands.
 */

#include <stdlib.h>

#include <zephyr/shell/shell.h>
#include <zephyr/kernel.h>

#include <zflight/drivers/esc/dshot.h>

#include "dshot_common.h"

#define DSHOT_SHELL_THREAD_PERIOD_MSEC  10

#define DSHOT_NODE DT_ALIAS(dshot0)

/*
 * A build error on this line means your board is unsupported.
 * Add dshot0 alias on board overlay to fix this.
 */
static const struct device *dshot_dev = DEVICE_DT_GET(DSHOT_NODE);

// types
enum dshot_shell_update_type {
    DSHOT_SHELL_UPDATE_NONE,
    DSHOT_SHELL_UPDATE_THROTTLE,
    DSHOT_SHELL_UPDATE_COMMAND,
};

static const char update_type_desc[][10] = {
    [DSHOT_SHELL_UPDATE_NONE] = "NONE",
    [DSHOT_SHELL_UPDATE_THROTTLE] = "THROTTLE",
    [DSHOT_SHELL_UPDATE_COMMAND] = "COMMAND",
};

// global data
static bool run_dshot_shell_update = false;
static enum dshot_shell_update_type update_type = DSHOT_SHELL_UPDATE_NONE;
static uint32_t update_channel = 0;
static uint16_t update_throttle = 0;
static uint16_t update_command = 0;
K_MUTEX_DEFINE(update_mutex);

// dshot shell thread
static void dshot_shell_update(void *, void *, void *);
#define STACK_SIZE  1024
#define PRIORITY    5
K_KERNEL_THREAD_DEFINE(dshot_shell_tid, STACK_SIZE, \
    dshot_shell_update, NULL, NULL, NULL,           \
    PRIORITY, 0, SYS_FOREVER_MS);                   \

static void dshot_shell_update(void *, void *, void *)
{
    uint32_t channel;
    uint16_t throttle;
    uint16_t command;
    enum dshot_shell_update_type type;
    bool printed_err = false;

    while (run_dshot_shell_update) {
        int err = 0;

        k_mutex_lock(&update_mutex, K_FOREVER);
        channel = update_channel;
        throttle = update_throttle;
        command = update_command;
        type = update_type;
        update_type = DSHOT_SHELL_UPDATE_NONE;
        k_mutex_unlock(&update_mutex);

#ifdef CONFIG_DSHOT_BIDIR

#endif

        int output = 0;
        if (!dshot_command_in_progress(dshot_dev, channel)) {
            switch (type)
            {
            case DSHOT_SHELL_UPDATE_THROTTLE:
                err = dshot_set_throttle(dshot_dev, channel, throttle);
                output = throttle;
                break;
            case DSHOT_SHELL_UPDATE_COMMAND:
                err = dshot_set_command(dshot_dev, channel, command);
                output = command;
                break;
            default:
                break;
            }
            type = DSHOT_SHELL_UPDATE_NONE;

            if (err && err != -EACCES) {
                printk("Error setting %s to %d (err %d)\n",
                    update_type_desc[type], output, err);
            }
        }

        err = dshot_send(dshot_dev);
        if (err) {
            if (!printed_err) {
                printk("Send Error (err %d)\n", err);
                printed_err = true;
            }
        } else {
            printed_err = false;
        }

        k_sleep(K_MSEC(DSHOT_SHELL_THREAD_PERIOD_MSEC));
    }
}

static int cmd_set_enabled(const struct shell *sh, size_t argc, char **argv)
{
    const int arg_idx_enabled = 1;

    int err;
    bool enabled;

    enabled = strtoul(argv[arg_idx_enabled], NULL, 0) > 0;

    if (enabled == run_dshot_shell_update) {
        return 0;
    }

    run_dshot_shell_update = enabled;
    err = dshot_set_enabled(dshot_dev, enabled);
    if (err) {
        shell_error(sh, "Failed to %s device (err %d)",
            enabled ? "enable" : "disable", err);
        return err;
    }

    if (enabled) {
        k_thread_start(dshot_shell_tid);
    } else {
        err = k_thread_join(dshot_shell_tid, K_MSEC(DSHOT_SHELL_THREAD_PERIOD_MSEC + 1));
        if (err) {
            shell_error(sh, "Failed to stop dshot shell thread (%d)", err);
            return err;
        }
    }

    return err;
}

static int cmd_set_type(const struct shell *sh, size_t argc, char **argv)
{
    const int arg_idx_type = 1;

    int err;
    unsigned type;
    bool type_valid;

    type = strtoul(argv[arg_idx_type], NULL, 0);
    switch (type)
    {
    case DSHOT_150:
    case DSHOT_300:
    case DSHOT_600:
        type_valid = true;
        break;
    default:
        type_valid = false;
        break;
    }

    if (!type_valid) {
        shell_error(sh, "Invalid DShot type");
        return -EINVAL;
    }

    err = dshot_set_type(dshot_dev, (enum dshot_type)type);
    if (err) {
        shell_error(sh, "Failed to set type (err %d)", err);
        return err;
    }

    return 0;
}

static int cmd_set_mode(const struct shell *sh, size_t argc, char **argv)
{
    const int arg_idx_mode = 1;

    int err;
    unsigned mode;
    bool mode_valid;

    mode = strtoul(argv[arg_idx_mode], NULL, 0);
    switch (mode)
    {
    case DSHOT_MODE_NORMAL:
    case DSHOT_MODE_BIDIRECTIONAL:
        mode_valid = true;
        break;
    default:
        mode_valid = false;
        break;
    }

    if (!mode_valid) {
        shell_error(sh, "Invalid DShot mode");
        return -EINVAL;
    }

    err = dshot_set_mode(dshot_dev, (enum dshot_mode)mode);
    if (err) {
        shell_error(sh, "Failed to set type (err %d)", err);
        return err;
    }

    return 0;
}

static int cmd_set_throttle(const struct shell *sh, size_t argc, char **argv)
{
    const int arg_idx_channel = 1;
    const int arg_idx_throttle = 2;

    uint32_t channel;
    uint32_t throttle;

    channel = strtoul(argv[arg_idx_channel], NULL, 0);
    throttle = strtoul(argv[arg_idx_throttle], NULL, 0);
    throttle = CLAMP(throttle, 0, 100) * (UINT16_MAX / 100);

    k_mutex_lock(&update_mutex, K_FOREVER);
    update_channel = channel;
    update_throttle = throttle;
    update_type = DSHOT_SHELL_UPDATE_THROTTLE;
    k_mutex_unlock(&update_mutex);

    return 0;
}

static int cmd_set_command(const struct shell *sh, size_t argc, char **argv)
{
    const int arg_idx_channel = 1;
    const int arg_idx_cmd = 2;

    uint32_t channel;
    uint32_t cmd;

    channel = strtoul(argv[arg_idx_channel], NULL, 0);
    cmd = strtoul(argv[arg_idx_cmd], NULL, 0);
    cmd = CLAMP(cmd, 0, DSHOT_CMD_MAX);
    if (!dshot_cmd_settings[cmd].enabled) {
        shell_error(sh, "DShot command not enabled");
        return -EINVAL;
    }

    k_mutex_lock(&update_mutex, K_FOREVER);
    update_channel = channel;
    update_command = cmd;
    update_type = DSHOT_SHELL_UPDATE_COMMAND;
    k_mutex_unlock(&update_mutex);

    return 0;
}

static int cmd_request_telem(const struct shell *sh, size_t argc, char **argv)
{
    const int arg_idx_channel = 1;

    int err;
    uint32_t channel;

    channel = strtoul(argv[arg_idx_channel], NULL, 0);
    err = dshot_set_request_telem(dshot_dev, channel);
    if (err) {
        shell_error(sh, "Unable to request telemetry (%d)", err);
        return err;
    }

    return 0;
}

// @todo telemetry

SHELL_STATIC_SUBCMD_SET_CREATE(dshot_cmds,
    SHELL_CMD_ARG(set_enabled, NULL, "<enabled>", cmd_set_enabled, 2, 0),
    SHELL_CMD_ARG(set_type, NULL, "<dshot_type>", cmd_set_type, 2, 0),
    SHELL_CMD_ARG(set_mode, NULL, "<dshot_mode>", cmd_set_mode, 2, 0),
    SHELL_CMD_ARG(set_throttle, NULL, "<channel> <throttle 0-100>", cmd_set_throttle, 3, 0),
    SHELL_CMD_ARG(set_command, NULL, "<channel> <dshot_command>", cmd_set_command, 3, 0),
    SHELL_CMD_ARG(req_telem, NULL, "<channel>", cmd_request_telem, 2, 0),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(dshot, &dshot_cmds, "DShot shell commands", NULL);
