/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief DShot shell commands.
 */

#include <zephyr/shell/shell.h>
#include <zephyr/kernel.h>
#include <stdlib.h>

#include <zflight/drivers/esc/dshot.h>

#define DSHOT_SHELL_THREAD_PERIOD_MSEC  10

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

// @todo global data

static const struct device *update_dev = NULL;
static uint32_t update_channel = 0;
static uint16_t update_throttle = 0;
static uint16_t update_command = 0;
static enum dshot_shell_update_type update_type = DSHOT_SHELL_UPDATE_NONE;
static bool run_dshot_shell_update = false;
K_MUTEX_DEFINE(update_mutex);

// @todo thread function
static void dshot_shell_update(void *, void *, void *)
{
    const struct device *dev;
    uint32_t channel;
    uint16_t throttle;
    uint16_t command;
    enum dshot_shell_update_type type;
    bool printed_err = false;

    while (run_dshot_shell_update) {
        int err = 0;

        k_mutex_lock(&update_mutex, K_FOREVER);
        dev = update_dev;
        channel = update_channel;
        throttle = update_throttle;
        command = update_command;
        type = update_type;
        update_type = DSHOT_SHELL_UPDATE_NONE;
        k_mutex_unlock(&update_mutex);

#ifdef CONFIG_DSHOT_BIDIR

#endif

        int output = 0;
        if (!dshot_command_in_progress(dev, channel)) {
            switch (type)
            {
            case DSHOT_SHELL_UPDATE_THROTTLE:
                err = dshot_set_throttle(dev, channel, throttle);
                output = throttle;
                break;
            case DSHOT_SHELL_UPDATE_COMMAND:
                err = dshot_set_command(dev, channel, command);
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

        err = dshot_send(dev);
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

#define STACK_SIZE  2048
#define PRIORITY    5
K_KERNEL_THREAD_DEFINE(dshot_shell_tid, STACK_SIZE, \
    dshot_shell_update, NULL, NULL, NULL,           \
    PRIORITY, 0, SYS_FOREVER_MS);                   \

static int cmd_start_thread(const struct shell *sh, size_t argc, char **argv)
{
    static const int arg_idx_run = 1;

    bool run = strtoul(argv[arg_idx_run], NULL, 0) > 0;

    if (run == run_dshot_shell_update) {
        return 0;
    }

    run_dshot_shell_update = run;
    if (run) {
        k_thread_start(dshot_shell_tid);
    } else {
        k_thread_join(dshot_shell_tid, K_FOREVER);
    }
    return 0;
}

static int cmd_set_enabled(const struct shell *sh, size_t argc, char **argv)
{
    static const int arg_idx_device = 1;
    static const int arg_idx_enabled = 2;

    int err;
    const struct device *dev;
    bool enabled;

    dev = device_get_binding(argv[arg_idx_device]);
    if (!dev) {
		shell_error(sh, "DShot device not found");
		return -EINVAL;
	}
    enabled = strtoul(argv[arg_idx_enabled], NULL, 0) > 0;

    err = dshot_set_enabled(dev, enabled);
	if (err) {
		shell_error(sh, "Failed to %s device (err %d)",
			enabled ? "enable" : "disable", err);
		return err;
	}

	return 0;
}

static int cmd_set_type(const struct shell *sh, size_t argc, char **argv)
{
    static const int arg_idx_device = 1;
    static const int arg_idx_type = 2;

    int err;
    const struct device *dev;
    unsigned type;
    bool type_valid;

    dev = device_get_binding(argv[arg_idx_device]);
    if (!dev) {
		shell_error(sh, "DShot device not found");
		return -EINVAL;
	}

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

    err = dshot_set_type(dev, (enum dshot_type)type);
	if (err) {
		shell_error(sh, "Failed to set type (err %d)", err);
		return err;
	}

	return 0;
}

static int cmd_set_mode(const struct shell *sh, size_t argc, char **argv)
{
    static const int arg_idx_device = 1;
    static const int arg_idx_mode = 2;

    int err;
    const struct device *dev;
    unsigned mode;
    bool mode_valid;

    dev = device_get_binding(argv[arg_idx_device]);
    if (!dev) {
		shell_error(sh, "DShot device not found");
		return -EINVAL;
	}

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

    err = dshot_set_mode(dev, (enum dshot_mode)mode);
	if (err) {
		shell_error(sh, "Failed to set type (err %d)", err);
		return err;
	}

	return 0;
}

static int cmd_set_throttle(const struct shell *sh, size_t argc, char **argv)
{
    static const int arg_idx_device = 1;
    static const int arg_idx_channel = 2;
    static const int arg_idx_throttle = 3;

    const struct device *dev;
    uint32_t channel;
    uint32_t throttle;

    dev = device_get_binding(argv[arg_idx_device]);
    if (!dev) {
		shell_error(sh, "DShot device not found");
		return -EINVAL;
	}

    channel = strtoul(argv[arg_idx_channel], NULL, 0);
    throttle = strtoul(argv[arg_idx_throttle], NULL, 0);
    throttle = CLAMP(throttle, 0, 100) * (UINT16_MAX / 100);

    k_mutex_lock(&update_mutex, K_FOREVER);
    update_dev = dev;
    update_channel = channel;
    update_throttle = throttle;
    update_type = DSHOT_SHELL_UPDATE_THROTTLE;
    k_mutex_unlock(&update_mutex);

	return 0;
}

// static int cmd_set_command(const struct shell *sh, size_t argc, char **argv)
// {

// }

// static int cmd_request_telem(const struct shell *sh, size_t argc, char **argv)
// {

// }

// @todo telemetry

SHELL_STATIC_SUBCMD_SET_CREATE(dshot_cmds,
	SHELL_CMD_ARG(start, NULL, "<run>", cmd_start_thread, 2, 0),
    SHELL_CMD_ARG(enable, NULL, "<device> <enabled>", cmd_set_enabled, 3, 0),
    SHELL_CMD_ARG(type, NULL, "<device> <dshot_type>", cmd_set_type, 3, 0),
    SHELL_CMD_ARG(mode, NULL, "<device> <dshot_mode>", cmd_set_mode, 3, 0),
    SHELL_CMD_ARG(throttle, NULL, "<device> <channel> <throttle 0-100>", cmd_set_throttle, 4, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(dshot, &dshot_cmds, "DShot shell commands", NULL);
