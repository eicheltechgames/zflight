/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZFLIGHT_INCLUDE_DRIVERS_ESC_DSHOT_H_
#define ZFLIGHT_INCLUDE_DRIVERS_ESC_DSHOT_H_

#include <zflight/drivers/esc.h>

#ifdef __cplusplus
extern "C" {
#endif

enum dshot_type {
    DSHOT_150 = 150,
    DSHOT_300 = 300,
    DSHOT_600 = 600
};

enum dshot_mode {
    DSHOT_MODE_NORMAL,
    DSHOT_MODE_BIDIRECTIONAL
};

enum dshot_command {
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEEP1,
    DSHOT_CMD_BEEP2,
    DSHOT_CMD_BEEP3,
    DSHOT_CMD_BEEP4,
    DSHOT_CMD_BEEP5,
    DSHOT_CMD_ESC_INFO,
    DSHOT_CMD_SPIN_DIRECTION_1,
    DSHOT_CMD_SPIN_DIRECTION_2,
    DSHOT_CMD_3D_MODE_OFF,
    DSHOT_CMD_3D_MODE_ON,
    DSHOT_CMD_SETTINGS_REQUEST,
    DSHOT_CMD_SAVE_SETTINGS,
    DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE,
    DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED,
    DSHOT_CMD_LED0_ON, // BLHeli32 only
    DSHOT_CMD_LED1_ON, // BLHeli32 only
    DSHOT_CMD_LED2_ON, // BLHeli32 only
    DSHOT_CMD_LED3_ON, // BLHeli32 only
    DSHOT_CMD_LED0_OFF, // BLHeli32 only
    DSHOT_CMD_LED1_OFF, // BLHeli32 only
    DSHOT_CMD_LED2_OFF, // BLHeli32 only
    DSHOT_CMD_LED3_OFF, // BLHeli32 only
    DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF, // KISS audio Stream mode on/Off
    DSHOT_CMD_SILENT_MODE_ON_OFF, // KISS silent Mode on/Off
#ifdef CONFIG_DSHOT_BIDIR
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE,
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE,
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY,
#ifdef CONFIG_DSHOT_EDT
    DSHOT_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY = 42,
    DSHOT_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_CURRENT_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_ERPM_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY,
#endif /* CONFIG_DSHOT_EDT */
#endif /* CONFIG_DSHOT_BIDIR */
    DSHOT_CMD_MAX = 47
};

struct dshot_bidir_telem {
    uint16_t erpm;
#ifdef CONFIG_DSHOT_EDT
    uint16_t temperature;
    uint16_t current;
    uint16_t voltage;
    uint16_t debug_1;
    uint16_t debug_2;
    uint16_t debug_3;
    uint16_t state_event;
#endif
};

/** @cond INTERNAL_HIDDEN */
/**
 * @brief ESC driver API call to get the type of ESC.
 * @see esc_get_type() for argument description.
 */
typedef enum dshot_type (*dshot_get_type_t)(const struct device *dev);

/**
 * @brief ESC driver API call to get the type of ESC.
 * @see esc_get_type() for argument description.
 */
typedef int (*dshot_set_type_t)(const struct device *dev, enum dshot_type type);

/**
 * @brief ESC driver API call to get the type of ESC.
 * @see esc_get_type() for argument description.
 */
typedef enum dshot_mode (*dshot_get_mode_t)(const struct device *dev);

/**
 * @brief ESC driver API call to get the type of ESC.
 * @see esc_get_type() for argument description.
 */
typedef int (*dshot_set_mode_t)(const struct device *dev, enum dshot_mode mode);

/**
 * @brief ESC driver API call to set the throttle for a channel.
 * @see esc_set_throttle_direct() for argument description.
 */
typedef int (*dshot_set_command_t)(const struct device *dev, uint32_t channel, enum dshot_command command);

/**
 * @brief ESC driver API call to set the throttle for a channel.
 * @see esc_set_throttle_direct() for argument description.
 */
typedef int (*dshot_command_in_progress_t)(const struct device *dev, uint32_t channel);

/**
 * @brief ESC driver API call to set the device enabled.
 * @see esc_set_enabled() for argument description.
 */
typedef int (*dshot_set_request_telem_t)(const struct device *dev, uint32_t channel);

/**
 * @brief ESC driver API call to send the throttle commands.
 * @see esc_send() for argument description.
 */
typedef void (*dshot_get_bidir_telem_t)(const struct device *dev, uint32_t channel, struct dshot_bidir_telem *out_telem);

/** @brief ESC driver API definition. */
__subsystem struct dshot_driver_api {
	dshot_get_type_t get_type;
    dshot_set_type_t set_type;
    dshot_get_mode_t get_mode;
#ifdef CONFIG_DSHOT_BIDIR
    dshot_set_mode_t set_mode;
#endif
    dshot_set_command_t set_command;
    dshot_command_in_progress_t command_in_progress;
    dshot_set_request_telem_t set_request_telem;
#ifdef CONFIG_DSHOT_BIDIR
    dshot_get_bidir_telem_t get_telem;
#endif
};
/** @endcond */

static inline enum dshot_type dshot_get_type(struct device *dev)
{
    const struct esc_driver_api *esc_api = dev->api;
    const struct dshot_driver_api *dshot_api = esc_api->extended_driver_api;

    return dshot_api->get_type(dev);
}

static inline int dshot_set_type(struct device *dev, enum dshot_type type)
{
    const struct esc_driver_api *esc_api = dev->api;
    const struct dshot_driver_api *dshot_api = esc_api->extended_driver_api;

    if (dshot_get_enabled(dev)) {
        return -EACCES;
    }

    return dshot_api->set_type(dev, type);
}

static inline bool dshot_get_enabled(struct device *dev)
{
    return esc_get_enabled(dev);
}

static inline bool dshot_set_enabled(struct device *dev, bool enabled)
{
    return esc_set_enabled(dev, enabled);
}

static inline enum dshot_mode dshot_get_mode(struct device *dev)
{
    const struct esc_driver_api *esc_api = dev->api;
    const struct dshot_driver_api *dshot_api = esc_api->extended_driver_api;

    return dshot_api->get_mode(dev);
}

static inline int dshot_set_mode(struct device *dev, enum dshot_mode mode)
{
#ifndef CONFIG_DSHOT_BIDIR
    ARG_UNUSED(dev);
    ARG_UNUSED(mode);
    return -ENOTSUP;
#else
    const struct esc_driver_api *esc_api = dev->api;
    const struct dshot_driver_api *dshot_api = esc_api->extended_driver_api;

    if (dshot_get_enabled(dev)) {
        return -EACCES;
    }

    return dshot_api->set_mode(dev, mode);
#endif
}

static inline int dshot_set_throttle(struct device *dev, uint32_t channel, uint16_t throttle)
{
    return esc_set_throttle(dev, channel, throttle);
}

static inline int dshot_set_command(struct device *dev, uint32_t channel, enum dshot_command command)
{
    const struct esc_driver_api *esc_api = dev->api;
    const struct dshot_driver_api *dshot_api = esc_api->extended_driver_api;

    return dshot_api->set_command(dev, channel, command);
}

static inline int dshot_set_request_telem(struct device *dev, uint32_t channel)
{
    const struct esc_driver_api *esc_api = dev->api;
    const struct dshot_driver_api *dshot_api = esc_api->extended_driver_api;

    return dshot_api->set_request_telem(dev, channel);
}

static inline int dshot_send(struct device *dev)
{
    return esc_send(dev);
}

static inline int dshot_command_in_progress(struct device *dev, uint32_t channel)
{
    const struct esc_driver_api *esc_api = dev->api;
    const struct dshot_driver_api *dshot_api = esc_api->extended_driver_api;

    return dshot_api->command_in_progress(dev, channel);
}

static inline void dshot_get_bidir_telem(struct device *dev, uint32_t channel, struct dshot_bidir_telem *out_telem)
{
#ifndef CONFIG_DSHOT_BIDIR
    ARG_UNUSED(dev);
    ARG_UNUSED(channel);
    ARG_UNUSED(out_telem);
    return -ENOTSUP;
#else
    const struct esc_driver_api *esc_api = dev->api;
    const struct dshot_driver_api *dshot_api = esc_api->extended_driver_api;

    dshot_api->get_telem(dev, channel, out_telem);
#endif
}

#ifdef __cplusplus
}
#endif

#endif /* ZFLIGHT_INCLUDE_DRIVERS_ESC_DSHOT_H_ */