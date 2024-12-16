/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief RC Link shell commands.
 */

#include <zflight/drivers/rc_link.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>

const struct device *rc_link_dev = DEVICE_DT_GET(DT_CHOSEN(zflight_rx));

static int cmd_get_enabled(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);

    if (rc_link_get_enabled(rc_link_dev)) {
        shell_print(sh, "status = enabled");
    } else {
        shell_print(sh, "status = disabled");
    }

    return 0;
}

static int cmd_set_enabled(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);

    bool enabled = strtoul(argv[1], NULL, 0) > 0;
    rc_link_set_enabled(rc_link_dev, enabled);
    cmd_get_enabled(sh, argc, argv);

    return 0;
}

static int cmd_get_channels(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);

    uint16_t channels[RC_LINK_MAX_RC_CHANNELS];
    int num_ch = rc_link_get_channels(rc_link_dev, channels, RC_LINK_MAX_RC_CHANNELS);
    if (num_ch < 0) {
        shell_error(sh, "Error getting channels: %d", num_ch);
        return num_ch;
    }

    shell_print(sh, "Ch:\tVal:");
    for (int i = 0; i < num_ch; ++i) {
        shell_print(sh, "%d\t%d", i, channels[i]);
    }

    return 0;
}

static int cmd_set_channels(const struct shell *sh, size_t argc, char **argv)
{
    uint16_t channels[RC_LINK_MAX_RC_CHANNELS];

    int num_ch = MIN(RC_LINK_MAX_RC_CHANNELS, argc - 2);
    for (int i = 0; i < num_ch; ++i) {
        channels[i] = strtoul(argv[i + 2], NULL, 0);
    }

    num_ch = rc_link_set_channels(rc_link_dev, channels, num_ch);
    if (num_ch < 0) {
        shell_error(sh, "Error setting channels: %d", num_ch);
        return num_ch;
    } else {
        shell_print(sh, "Set %d channels", num_ch);
    }

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(rc_link_enabled_cmds,
    SHELL_CMD_ARG(get, NULL,
        "Print the RC Link enabled status to the console\n"
        "Usage: rc_link enabled get <device>\n"
        "example: rc_link enabled csrf_rx",
        cmd_get_enabled, 1, 0),
    SHELL_CMD_ARG(set, NULL,
        "Set the RC Link enabled status\n"
        "Usage: rc_link enabled set <device> <enabled>\n"
        "<enabled> - 1 = enabled, 0 = disabled\n"
        "example: rc_link enabled set csrf_rx 1",
        cmd_set_enabled, 2, 0),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(rc_link_channels_cmds,
    SHELL_CMD_ARG(get, NULL,
        "Print the RC Link enabled status to the console\n"
        "Usage: rc_link enabled get <device>\n"
        "example: rc_link enabled csrf_rx",
        cmd_get_channels, 1, 0),
    SHELL_CMD_ARG(set, NULL,
        "Set the RC Link enabled status\n"
        "Usage: rc_link enabled set <device> <enabled>\n"
        "<enabled> - 1 = enabled, 0 = disabled\n"
        "example: rc_link enabled set csrf_rx 1",
        cmd_set_channels, 1, RC_LINK_MAX_RC_CHANNELS),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(rc_link_cmds,
    SHELL_CMD(enabled, &rc_link_enabled_cmds, "RC Link enabled commands: get set", NULL),
    SHELL_CMD(channels, &rc_link_channels_cmds, "RC Link channels commands: get set", NULL),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(rc_link, &rc_link_cmds, "RC Link commands", NULL);