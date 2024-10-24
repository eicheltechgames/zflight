/**
 * @file
 *
 * @brief Public APIs for the ESC drivers.
 */

/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZFLIGHT_INCLUDE_DRIVERS_ESC_H_
#define ZFLIGHT_INCLUDE_DRIVERS_ESC_H_

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ESC Interface
 * @defgroup esc_interface ESC Interface
 * @since 1.0
 * @version 1.0.0
 * @ingroup io_interfaces
 * @{
 */

enum esc_type {
    ESC_TYPE_DSHOT,
    NUM_ESC_TYPE
};

/** @cond INTERNAL_HIDDEN */
/**
 * @brief ESC driver API call to get the type of ESC.
 * @see esc_get_type() for argument description.
 */
typedef enum esc_type (*esc_get_type_t)(const struct device *dev);

/**
 * @brief ESC driver API call to get if the device is enabled.
 * @see esc_get_enabled() for argument description.
 */
typedef bool (*esc_get_enabled_t)(const struct device *dev);

/**
 * @brief ESC driver API call to set the device enabled.
 * @see esc_set_enabled() for argument description.
 */
typedef int (*esc_set_enabled_t)(const struct device *dev, bool enabled);

/**
 * @brief ESC driver API call to set the throttle for a channel.
 * @see esc_set_throttle_direct() for argument description.
 */
typedef int (*esc_set_throttle_t)(const struct device *dev, uint32_t channel, uint16_t throttle);

/**
 * @brief ESC driver API call to send the throttle commands.
 * @see esc_send() for argument description.
 */
typedef int (*esc_send_t)(const struct device *dev);

/** @brief ESC driver API definition. */
__subsystem struct esc_driver_api {
    esc_get_type_t get_type;
    esc_get_enabled_t get_enabled;
    esc_set_enabled_t set_enabled;
    esc_set_throttle_t set_throttle;
    esc_send_t send;
#if defined(CONFIG_ESC_DSHOT)
    void *extended_driver_api;
#endif
};
/** @endcond */

/**
 * @brief ESC driver API call to get the type of ESC.
 * @see esc_get_type() for argument description.
 */
static inline enum esc_type esc_get_type(const struct device *dev)
{
    const struct esc_driver_api *api = dev->api;

    return api->get_type(dev);
}

/**
 * @brief ESC driver API call to get if the device is enabled.
 * @see esc_get_enabled() for argument description.
 */
static inline bool esc_get_enabled(const struct device *dev)
{
    const struct esc_driver_api *api = dev->api;

    return api->get_enabled(dev);
}

/**
 * @brief ESC driver API call to set the device enabled.
 * @see esc_set_enabled() for argument description.
 */
static inline int esc_set_enabled(const struct device *dev, bool enabled)
{
    const struct esc_driver_api *api = dev->api;

    return api->set_enabled(dev, enabled);
}

/**
 * @brief ESC driver API call to set the throttle for a channel.
 * @see esc_set_throttle_direct() for argument description.
 */
static inline int esc_set_throttle(const struct device *dev, uint32_t channel, uint16_t throttle)
{
    const struct esc_driver_api *api = dev->api;

    return api->set_throttle(dev, channel, throttle);
}

/**
 * @brief ESC driver API call to send the throttle commands.
 * @see esc_send() for argument description.
 */
static inline int esc_send(const struct device *dev)
{
    const struct esc_driver_api *api = dev->api;

    if (!esc_get_enabled(dev)) {
        return -EACCES;
    }

    return api->send(dev);
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZFLIGHT_INCLUDE_DRIVERS_ESC_H_ */