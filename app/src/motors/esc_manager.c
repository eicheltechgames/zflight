/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <string.h>

#include <zflight/esc_manager.h>
#include <zflight/drivers/esc.h>
#ifdef CONFIG_ZFLIGHT_DSHOT
#include <zflight/drivers/esc/dshot.h>
#endif

#include <zephyr/sys/atomic.h>
#include <zephyr/init.h>
#include <zephyr/shell/shell.h>

// #define ESC_MANAGER_NODE    DT_CHOSEN(zflight_esc_manager)

// #define STORE_ESC_DT_SPEC(node)                                             \
//     [DT_PROP(node, motor_index)] = ESC_DT_SPEC_GET(DT_PHANDLE(node, esc)),  \

// #define FOREACH_ESC_DT_SPEC_STATUS_OKAY(node)                       \
//     COND_CODE_1(ESC_DT_SPEC_HAS_STATUS_OKAY(DT_PHANDLE(node, esc)), \
//         STORE_ESC_DT_SPEC(node), EMPTY)                             \

// #define DEFAULT_MOTOR_CONFIG(i, _)  \
//     {                               \
//         .esc = &escs[i],            \
//         .poles = 14,                \
//     }                               \

// struct motor {
//     const struct esc_dt_spec *esc;
//     uint8_t poles;
// };

// static const struct esc_dt_spec escs[] = {
//     DT_FOREACH_CHILD(ESC_MANAGER_NODE, STORE_ESC_DT_SPEC)
// };

// static struct motor motors[CONFIG_ZFLIGHT_NUM_MOTORS] = {
//     LISTIFY(CONFIG_ZFLIGHT_NUM_MOTORS, DEFAULT_MOTOR_CONFIG, (,))
// };

// static atomic_t terminal_rw_lock = 0;
// static bool terminal_throttle_enable = false;
// static uint16_t terminal_throttles[CONFIG_ZFLIGHT_NUM_MOTORS] = { 0 };

// #ifdef CONFIG_ZFLIGHT_DSHOT
// static int terminal_dshot_commands[CONFIG_ZFLIGHT_NUM_MOTORS] = { 0 };

// static inline int handle_dshot_terminal(int motor_idx)
// {
//     if (terminal_dshot_commands[motor_idx] >= 0) {
//         int err = dshot_set_command(motors[motor_idx].esc->dev,
//             motors[motor_idx].esc->channel, terminal_dshot_commands[motor_idx]);
//         if (err) {
//             return err;
//         }
//     }

//     return dshot_command_in_progress(
//         motors[motor_idx].esc->dev,
//         motors[motor_idx].esc->channel) >= 0 ?
//         -EALREADY : 0;
// }
// #endif

// int esc_manager_set_throttles(const float throttles[CONFIG_ZFLIGHT_NUM_MOTORS])
// {
//     // @todo error metrics
//     int err = 0;
//     int tmp_err = 0;

//     bool terminal_active = true; // @todo get if the terminal is active

//     if (!terminal_active) {
//         for (int i = 0; i < CONFIG_ZFLIGHT_NUM_MOTORS; ++i) {
//             uint16_t throttle = CLAMP(throttles[i], 0.0f, 1.0f) * UINT16_MAX;
//             // @todo set to 0 if not armed
//             tmp_err = esc_set_throttle(motors[i].esc->dev,
//                 motors[i].esc->channel, throttle);

//             if (tmp_err) {
//                 err = tmp_err;
//             }
//         }

//     // Skip if terminal currently has lock, we'll get the data next frame
//     // Okay to resend previous frame.
//     } else if (!atomic_get(&terminal_rw_lock)) {
//         // @note terminal does all error checking for valid inputs
//         for (int i = 0; i < CONFIG_ZFLIGHT_NUM_MOTORS; ++i) {
//             enum esc_type type = esc_get_type(motors[i].esc->dev);
//             switch (type)
//             {
// #ifdef CONFIG_ZFLIGHT_DSHOT
//             case ESC_TYPE_DSHOT:
//                 tmp_err = handle_dshot_terminal(i);
//                 break;
// #endif
//             default:
//                 tmp_err = -ENOSYS;
//                 break;
//             }

//             if (terminal_throttle_enable && !tmp_err) {
//                 tmp_err = esc_set_throttle(motors[i].esc->dev,
//                     motors[i].esc->channel, terminal_throttles[i]);
//             }

//             if (tmp_err && tmp_err != -EALREADY) {
//                 err = tmp_err;
//             }
//         }
//     }

//     if (err) {
//         return err;
//     }

//     tmp_err = 0;
//     for (int i = 0; i < CONFIG_ZFLIGHT_NUM_MOTORS; ++i) {
//         tmp_err = esc_send(motors[i].esc->dev);
//         if (tmp_err && tmp_err != -EALREADY) {
//             err = tmp_err;
//         }
//     }

//     return err;
// }

// int esc_manager_init(void) {
//     // @todo read flash for config for motor map, poles, dshot config
//     return 0;
// }

// SYS_INIT(esc_manager_init, APPLICATION, 0);

// #ifdef CONFIG_ZFLIGHT_DSHOT
// static int cmd_dshot_set_type(const struct shell *sh, size_t argc, char **argv)
// {
//     ARG_UNUSED(argc);

//     int err;
//     int tmp_err;
//     int type;
//     bool type_valid;

//     type = strtol(argv[1], NULL, 0);
//     switch (type)
//     {
//     case DSHOT_150:
//     case DSHOT_300:
//     case DSHOT_600:
//         type_valid = true;
//         break;
//     default:
//         type_valid = false;
//         break;
//     }

//     if (!type_valid) {
//         shell_error(sh, "Invalid type. Types are: %d, %d, %d",
//             DSHOT_150, DSHOT_300, DSHOT_600);
//         return -EINVAL;
//     }

//     ARRAY_FOR_EACH(motors, i) {
//         if (ESC_TYPE_DSHOT == esc_get_type(motors[i].esc->dev)) {
//             tmp_err = dshot_set_type(motors[i].esc->dev, (enum dshot_type)type);
//             if (tmp_err) {
//                 shell_error(sh, "Failed to set type on motor %d (err %d)",
//                     i + 1, tmp_err);
//                 err = tmp_err;
//             }
//         }
//     }

//     if (err) {
//         shell_warn(sh, "Error occured while setting type... skipping save config to flash");
//         return err;
//     }

//     // @todo save to flash

//     return err;
// }

// static int cmd_dshot_set_mode(const struct shell *sh, size_t argc, char **argv)
// {
//     static const char *normal_str = "normal";
//     static const char *bidir_str = "bidir";
//     static const char *bidirectional_str = "bidirectional";
//     int err;
//     int tmp_err;
//     enum dshot_mode mode;
//     bool mode_valid;

//     if (!strcmp(argv[1], normal_str)) {
//         mode = DSHOT_MODE_NORMAL;
//         mode_valid = true;
//     } else if (!(strcmp(argv[1], bidir_str)  ||
//                  strcmp(argv[1], bidirectional_str))) {
//         mode = DSHOT_MODE_BIDIRECTIONAL;
//         mode_valid = true;
//     } else {
//         mode_valid = false;
//     }

//     if (!mode_valid) {
//         shell_error(sh, "Invalid DShot mode. Modes are: %s, %s, %s",
//             normal_str, bidir_str, bidirectional_str);
//         return -EINVAL;
//     }

//     ARRAY_FOR_EACH(motors, i) {
//         if (ESC_TYPE_DSHOT == esc_get_type(motors[i].esc->dev)) {
//             tmp_err = dshot_set_mode(motors[i].esc->dev, mode);
//             if (tmp_err) {
//                 shell_error(sh, "Failed to set mode on motor %d (err %d)",
//                     i + 1, tmp_err);
//                 err = tmp_err;
//             }
//         }
//     }

//     if (err) {
//         shell_warn(sh, "Error occured while setting mode... skipping save config to flash");
//         return err;
//     }

//     // @todo save to flash

//     return 0;
// }

// static int cmd_dshot_set_command(const struct shell *sh, size_t argc, char **argv)
// {
//     int err = 0;
//     int command = 0;

//     command = strtol(argv[1], NULL, 0);
//     if (!IN_RANGE(command, 0, DSHOT_CMD_MAX)) {
//         shell_error(sh, "Invalid DShot command");
//         return -EINVAL;
//     }

//     if (!dshot_cmd_settings[command].enabled) {
//         shell_error(sh, "DShot command not enabled");
//         return -EINVAL;
//     }

//     atomic_set(&terminal_rw_lock, 1);
//     if (argc < 3) {
//         ARRAY_FOR_EACH(terminal_dshot_commands, i) {
//             terminal_dshot_commands[i] = command;
//         }
//     } else {
//         int motor = strtol(argv[2], NULL, 0);
//         motor--;
//         if (motor >= 0 && motor < CONFIG_ZFLIGHT_NUM_MOTORS) {
//             terminal_dshot_commands[motor] = command;
//         } else {
//             shell_error(sh, "Invalid motor");
//             err = -EINVAL;
//         }
//     }
//     atomic_set(&terminal_rw_lock, 0);
//     return err;
// }
// #endif

// SHELL_STATIC_SUBCMD_SET_CREATE(dshot_cmds,
// #ifdef CONFIG_ZFLIGHT_DSHOT
//     SHELL_CMD_ARG(set_type, NULL,
//         "Set the DShot type (150, 300, 600) for all DShot ESCs\n"
//         "Usage: set_type <dshot_type>",
//         cmd_dshot_set_type, 2, 0),
//     SHELL_CMD_ARG(set_mode, NULL,
//         "Set the DShot Mode (normal, bidir) for all DShot ESCs\n"
//         "Usage: set_mode <dshot_mode>",
//         cmd_dshot_set_mode, 2, 0),
//     SHELL_CMD_ARG(set_command, NULL,
//         "Set the DShot command for a motor\n"
//         "If motor is not specified, command will be applied to ALL motors\n"
//         "Usage: set_command <command (0-47)> [<motor (1-NUM_MOTORS)>]",
//         cmd_dshot_set_command, 3, 0),
// #endif
//     SHELL_SUBCMD_SET_END
// );

// static int cmd_enable_throttle(const struct shell *sh, size_t argc, char **argv)
// {
//     ARG_UNUSED(argc);
//     ARG_UNUSED(argv);

//     // @todo determine conditions to allow this

//     shell_warn(sh, "ENSURE PROPS ARE REMOVED AND USE WITH CAUTION!");
//     memset(terminal_throttles, 0, sizeof(terminal_throttles));
//     terminal_throttle_enable = true;
//     return 0;
// }

// static int cmd_disable_throttle(const struct shell *sh, size_t argc, char **argv)
// {
//     ARG_UNUSED(sh);
//     ARG_UNUSED(argc);
//     ARG_UNUSED(argv);

//     terminal_throttle_enable = false;
//     return 0;
// }

// static int cmd_set_throttle(const struct shell *sh, size_t argc, char **argv)
// {
//     int err = 0;
//     int throttle = 0;

//     if (!terminal_throttle_enable) {
//         shell_error(sh,
//             "Must enable terminal throttle control before setting throttles");
//         return -EPERM;
//     }

//     throttle = strtol(argv[1], NULL, 0);
//     throttle = CLAMP(throttle, 0, 100) * (UINT16_MAX / 100);

//     atomic_set(&terminal_rw_lock, 1);
//     if (argc < 3) {
//         ARRAY_FOR_EACH(terminal_throttles, i) {
//             terminal_throttles[i] = throttle;
//         }
//     } else {
//         int motor = strtol(argv[2], NULL, 0);
//         motor--;
//         if (motor >= 0 && motor < CONFIG_ZFLIGHT_NUM_MOTORS) {
//             terminal_throttles[motor] = throttle;
//         } else {
//             shell_error(sh, "Invalid motor");
//             err = -EINVAL;
//         }
//     }
//     atomic_set(&terminal_rw_lock, 0);
//     return err;
// }

// int cmd_set_motor_poles(const struct shell *sh, size_t argc, char **argv)
// {
//     // @todo set poles
//     // @todo write to flash

//     return 0;
// }

// int cmd_remap_motors(const struct shell *sh, size_t argc, char **argv)
// {
//     int err;
//     int map[CONFIG_ZFLIGHT_NUM_MOTORS] = { 0 };
//     bool map_verification[CONFIG_ZFLIGHT_NUM_MOTORS] = { false };

//     // @todo determine other conditions to allow this
//     if (terminal_throttle_enable) {
//         shell_error(sh,
//             "Cannot remap motors while terminal throttle control is enabled");
//         return -EPERM;
//     }

//     for (int i = 1; i < argc; ++i) {
//         map[i - 1] = strtol(argv[i], NULL, 0);
//         map[i - 1]--;
//     }

//     // verify a valid motor map
//     for (int i = 0; i < CONFIG_ZFLIGHT_NUM_MOTORS; ++i) {
//         if (map[i] < 0 || map[i] >= CONFIG_ZFLIGHT_NUM_MOTORS) {
//             shell_error(sh, "Invalid index for motor %d", i + 1);
//             err = -EINVAL;
//             break;
//         }
//         if (map_verification[map[i]]) {
//             shell_error(sh, "Duplicate index detected");
//             err = -EINVAL;
//             break;
//         }
//         map_verification[map[i]] = true;
//     }

//     if (err) {
//         shell_warn(sh, "Error occured while remapping... skipping save config to flash");
//         return err;
//     }

//     // @todo save motor map to flash
//     return 0;
// }

// SHELL_STATIC_SUBCMD_SET_CREATE(esc_cmds,
//     SHELL_CMD(enable_throttle, NULL,
//         "Enables terminal control of motor throttles\n"
//         "!!!ENSURE PROPS ARE REMOVED AND USE WITH CAUTION!!!\n",
//         cmd_enable_throttle),
//     SHELL_CMD(disable_throttle, NULL,
//         "Disables terminal control of motor throttles and stops the motors",
//         cmd_disable_throttle),
//     SHELL_CMD_ARG(set_throttle, NULL,
//         "Set the %% throttle for a motor\n"
//         "If motor is not specified, throttle will be applied to ALL motors\n"
//         "Usage: set_throttle <throttle (0-100)> [<motor (1-NUM_MOTORS)>]",
//         cmd_set_throttle, 2, 1),
//     SHELL_CMD_ARG(set_motor_poles, NULL,
//         "Set the number of poles for a motor\n"
//         "If motor is not specified, poles will be applied to ALL motors\n"
//         "Usage: set_motor_poles <poles> [<motor (1-NUM_MOTORS)>]",
//         cmd_set_motor_poles, 2, 1),
//     SHELL_CMD_ARG(remap_motors, NULL,
//         "Remap the motors by index\n"
//         "Motors indexes increment starting from 1 at 6:00\n"
//         "NOTE: requires reboot to take effect\n"
//         "Usage: remap_motors <m_1> <m_2> ... <m_n>\n"
//         "Example: 'remap_motors 4 3 2 1' will reverse the order of the motors",
//         cmd_remap_motors, CONFIG_ZFLIGHT_NUM_MOTORS + 1, 0),
//     // @todo command to print motor config
//     SHELL_COND_CMD(CONFIG_ZFLIGHT_DSHOT, dshot, &dshot_cmds, "DShot commands", NULL),
//     SHELL_SUBCMD_SET_END
// );

// SHELL_CMD_REGISTER(esc, &esc_cmds, "ESC commands", NULL);
