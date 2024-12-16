/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public APIs for Radio Control (RC) Link
 */

#ifndef ZFLIGHT_INCLUDE_DRIVERS_RC_LINK_H_
#define ZFLIGHT_INCLUDE_DRIVERS_RC_LINK_H_

/**
 * @brief RC Link Interface
 * @defgroup rc_link_interface RC Link Interface
 * @since 1.0
 * @version 1.0.0
 * @ingroup io_interfaces
 * @{
 */

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RC_LINK_MAX_RC_CHANNELS 16
#define RC_LINK_CHANNEL_VAL_MIN 987
#define RC_LINK_CHANNEL_VAL_MAX 2012

#define RC_LINK_STATUS_DOWNLINK_FLAG        BIT(0)
#define RC_LINK_STATUS_RSSI_PCT_FLAG        BIT(1)
#define RC_LINK_STATUS_LINK_QUALITY_FLAG    BIT(2)
#define RC_LINK_STATUS_RSSI_DBM_FLAG        BIT(3)
#define RC_LINK_STATUS_SNR_FLAG             BIT(4)
#define RC_LINK_STATUS_POWER_FLAG           BIT(5)

#define RC_LINK_TELEM_VBATT_FLAG            BIT(0)
#define RC_LINK_TELEM_CURRENT_FLAG          BIT(1)
#define RC_LINK_TELEM_BATT_CAPACITY_FLAG    BIT(2)
#define RC_LINK_TELEM_BATT_PERCENT_FLAG     BIT(3)

enum rc_link_type {
    RC_LINK_TYPE_PWM,
    RC_LINK_TYPE_CRSF,
    RC_LINK_NUM_TYPES
};

enum rc_link_dir {
    RC_LINK_UPLINK,
    RC_LINK_DOWNLINK,
};

struct rc_link_dir_status {
    uint8_t  rssi_pct;
#ifdef CONFIG_RC_LINK_STATUS_HAS_LINK_QUALITY
    uint8_t  link_quality_pct;
#endif // CONFIG_RC_LINK_STATUS_HAS_LINK_QUALITY
#ifdef CONFIG_RC_LINK_STATUS_HAS_RSSI_DBM
    int32_t rssi_dbm;
#endif // CONFIG_RC_LINK_STATUS_HAS_RSSI_DBM
#ifdef CONFIG_RC_LINK_STATUS_HAS_SNR
    int32_t snr_dbm;
#endif // CONFIG_RC_LINK_STATUS_HAS_SNR
#ifdef CONFIG_RC_LINK_STATUS_HAS_POWER
    uint32_t power_mW;
#endif // CONFIG_RC_LINK_STATUS_HAS_POWER
};

struct rc_link_status {
    struct rc_link_dir_status uplink;
#ifdef CONFIG_RC_LINK_DOWNLINK_STATUS
    struct rc_link_dir_status downlink;
#endif // CONFIG_RC_LINK_DOWNLINK_STATUS
};

struct rc_link_telemetry {
#ifdef CONFIG_RC_LINK_TELEM_HAS_VBATT
    uint32_t vbatt_mV;
#endif // CONFIG_RC_LINK_TELEM_HAS_VBATT
#ifdef CONFIG_RC_LINK_TELEM_HAS_CURRENT
    uint32_t current_mA;
#endif // CONFIG_RC_LINK_TELEM_HAS_CURR
#ifdef CONFIG_RC_LINK_TELEM_HAS_BATT_CAPACITY
    uint32_t batt_capacity_mAh;
#endif // CONFIG_RC_LINK_TELEM_HAS_BATT_CAPACITY
#ifdef CONFIG_RC_LINK_TELEM_HAS_BATT_PERCENT
    uint8_t batt_pct;
#endif // CONFIG_RC_LINK_TELEM_HAS_BATT_PERCENT
};

struct rc_link_config {
    enum rc_link_type type;
    uint8_t num_channels;
    uint32_t supported_uplink_status;
#ifdef CONFIG_RC_LINK_DOWNLINK_STATUS
    uint32_t supported_downlink_status;
#endif // CONFIG_RC_LINK_DOWNLINK_STATUS
#ifdef CONFIG_RC_LINK_TELEMETRY
    uint32_t supported_telemetry;
#endif // CONFIG_RC_LINK_TELEMETRY
};

typedef void (*rc_link_on_channels_received_cb_t)(const struct device *dev,
    const uint16_t *channels, uint8_t num_channels, void *user_data);

typedef void (*rc_link_on_telem_received_cb_t)(const struct device *dev,
    const struct rc_link_telemetry *telem, void *user_data);

/**
 * @cond INTERNAL_HIDDEN
 *
 * These are for internal use only, so skip these in
 * public documentation.
 */

typedef int  (*rc_link_api_get_config)
    (const struct device *dev, struct rc_link_config *cfg);
typedef int  (*rc_link_api_set_channels_callback_t)
    (const struct device *dev, rc_link_on_channels_received_cb_t cb, void *user_data);
typedef int  (*rc_link_api_set_telem_callback_t)
    (const struct device *dev, rc_link_on_telem_received_cb_t cb, void *user_data);
typedef void (*rc_link_api_set_enabled_t)(const struct device *dev, bool enabled);
typedef bool (*rc_link_api_get_enabled_t)(const struct device *dev);
typedef int  (*rc_link_api_get_channels_t)
    (const struct device *dev, uint16_t *channels, uint8_t num_channels);
typedef int  (*rc_link_api_set_channels_t)
    (const struct device *dev, const uint16_t *channels, uint8_t num_channels);
typedef int  (*rc_link_api_get_telem_t)
    (const struct device *dev, struct rc_link_telemetry *telem);
typedef int  (*rc_link_api_set_telem_t)
    (const struct device *dev, const struct rc_link_telemetry *telem);
typedef int  (*rc_link_api_get_link_status_t)
    (const struct device *dev, struct rc_link_status *status);
typedef int  (*rc_link_api_send_data_t)(const struct device *dev, int data_type);

__subsystem struct rc_link_driver_api {
    rc_link_api_get_config get_config;
    rc_link_api_set_channels_callback_t set_channels_callback;
    rc_link_api_set_enabled_t set_enabled;
    rc_link_api_get_enabled_t get_enabled;
    rc_link_api_get_link_status_t get_status;
    rc_link_api_get_channels_t get_channels;
    rc_link_api_set_channels_t set_channels;
#ifdef CONFIG_RC_LINK_TELEMETRY
    rc_link_api_set_telem_callback_t set_telem_callback;
    rc_link_api_set_telem_t set_telem;
    rc_link_api_get_telem_t get_telem;
#endif // CONFIG_RC_LINK_TELEMETRY
};

/**
 * @endcond
 */

static inline int rc_link_config_get(const struct device *dev,
                                            struct rc_link_config *cfg)
{
    const struct rc_link_driver_api *api =
                (const struct rc_link_driver_api *)dev->api;

    if (cfg == NULL) {
        return -EINVAL;
    }

    return api->get_config(dev, cfg);
}

static inline int rc_link_set_channels_callback(const struct device *dev,
                    rc_link_on_channels_received_cb_t cb, void *user_data)
{
    const struct rc_link_driver_api *api =
        (const struct rc_link_driver_api *)dev->api;

    if (api->set_channels_callback == NULL) {
        return -ENOSYS;
    }

    return api->set_channels_callback(dev, cb, user_data);
}

static inline void rc_link_set_enabled(const struct device *dev, bool enabled)
{
    const struct rc_link_driver_api *api =
        (const struct rc_link_driver_api *)dev->api;

    api->set_enabled(dev, enabled);
}

static inline bool rc_link_get_enabled(const struct device *dev)
{
    const struct rc_link_driver_api *api =
        (const struct rc_link_driver_api *)dev->api;

    return api->get_enabled(dev);
}

static inline int rc_link_get_status(const struct device *dev,
                                    struct rc_link_status *status)
{
    const struct rc_link_driver_api *api =
        (const struct rc_link_driver_api *)dev->api;

    if (api->get_status == NULL) {
        return -ENOSYS;
    }

    return api->get_status(dev, status);
}

static inline int rc_link_get_status_rssi_pct(
    const struct rc_link_config *config,
    const struct rc_link_status *status,
    enum rc_link_dir dir)
{
    uint32_t flags;

    if (config == NULL || status == NULL) {
        return -EINVAL;
    }

    if (dir == RC_LINK_UPLINK) {
        // uplink always supports RSSI percent
        return status->uplink.rssi_pct;
    } else {
#ifdef CONFIG_RC_LINK_DOWNLINK_STATUS
        flags = RC_LINK_STATUS_RSSI_PCT_FLAG | RC_LINK_STATUS_DOWNLINK_FLAG;
        if ((config->supported_downlink_status & flags) == flags) {
            return status->downlink.rssi_pct;
        }
#else
        return -ENOTSUP;
#endif // CONFIG_RC_LINK_DOWNLINK_STATUS
    }
}

static inline int rc_link_get_status_link_quality_pct(
    const struct rc_link_config *config,
    const struct rc_link_status *status,
    enum rc_link_dir dir)
{
#ifdef CONFIG_RC_LINK_STATUS_HAS_LINK_QUALITY
    uint32_t flags = RC_LINK_STATUS_RSSI_PCT_FLAG;

    if (config == NULL || status == NULL) {
        return -EINVAL;
    }

    if (dir == RC_LINK_UPLINK) {
        if ((config->supported_uplink_status & flags) == flags) {
            return status->uplink.link_quality_pct;
        }
    } else {
#ifdef CONFIG_RC_LINK_DOWNLINK_STATUS
        flags |= RC_LINK_STATUS_DOWNLINK_FLAG;
        if ((config->supported_downlink_status & flags) == flags) {
            return status->downlink.link_quality_pct;
        }
#else
        return -ENOTSUP;
#endif // CONFIG_RC_LINK_DOWNLINK_STATUS
    }
#else
    ARG_UNUSED(config);
    ARG_UNUSED(status);
    ARG_UNUSED(dir);

    return -ENOTSUP;
#endif // CONFIG_RC_LINK_STATUS_HAS_LINK_QUALITY
}

static inline int rc_link_get_channels(const struct device *dev,
                        uint16_t *channels, uint8_t num_channels)
{
    const struct rc_link_driver_api *api =
        (const struct rc_link_driver_api *)dev->api;

    if (channels == NULL || num_channels == 0) {
        return -EINVAL;
    }

    return api->get_channels(dev, channels, num_channels);
}


static inline int rc_link_get_channel(const struct device *dev, uint8_t channel)
{
    const struct rc_link_config *config = dev->config;
    uint16_t channels[RC_LINK_MAX_RC_CHANNELS] = {0};

    if (channel >= config->num_channels) {
        return -EINVAL;
    }

    int err = rc_link_get_channels(dev, channels, RC_LINK_MAX_RC_CHANNELS);
    if (err <= 0) {
        return err;
    }

    return channels[channel];
}

static inline int rc_link_set_channels(const struct device *dev,
                                const uint16_t *channels, uint8_t num_channels)
{
    const struct rc_link_driver_api *api =
        (const struct rc_link_driver_api *)dev->api;

    if (channels == NULL || num_channels == 0) {
        return -EINVAL;
    }

    return api->set_channels(dev, channels, num_channels);
}

static inline int rc_link_set_telem_callback(const struct device *dev,
                    rc_link_on_telem_received_cb_t cb, void *user_data)
{
#ifdef CONFIG_RC_LINK_TELEMETRY
    const struct rc_link_driver_api *api =
        (const struct rc_link_driver_api *)dev->api;

    if (api->set_telem_callback == NULL) {
        return -ENOSYS;
    }

    return api->set_telem_callback(dev, cb, user_data);
#else
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(user_data);

    return -ENOTSUP;
#endif // CONFIG_RC_LINK_TELEMETRY
}

static inline int rc_link_set_telemetry(const struct device *dev,
                                        const struct rc_link_telemetry *telem)
{
#ifdef CONFIG_RC_LINK_TELEMETRY
    const struct rc_link_driver_api *api =
        (const struct rc_link_driver_api *)dev->api;;

    if (api->set_telem == NULL) {
        return -ENOSYS;
    }

    if (telem == NULL) {
        return -EINVAL;
    }

    return api->set_telem(dev, telem);
#else
    ARG_UNUSED(dev);
    ARG_UNUSED(telem);

    return -ENOTSUP;
#endif // CONFIG_RC_LINK_TELEMETRY
}

static inline int rc_link_get_telemetry(const struct device *dev,
                                        struct rc_link_telemetry *telem)
{
#ifdef CONFIG_RC_LINK_TELEMETRY
    const struct rc_link_driver_api *api =
        (const struct rc_link_driver_api *)dev->api;

    if (api->get_telem == NULL) {
        return -ENOSYS;
    }

    if (telem == NULL) {
        return -EINVAL;
    }

    return api->get_telem(dev, telem);
#else
    ARG_UNUSED(dev);
    ARG_UNUSED(telem);

    return -ENOTSUP;
#endif // CONFIG_RC_LINK_TELEMETRY
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZFLIGHT_INCLUDE_DRIVERS_RC_LINK_H_ */
