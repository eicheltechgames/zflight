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
    ESC_TYPE_PPM,
    ESC_TYPE_DSHOT,
    NUM_ESC_TYPE
};

struct esc_dt_spec {
    const struct device *dev;
    uint8_t channel;
};

#define ESC_DT_SPEC_GET(node)                   \
    {                                           \
        .dev = DEVICE_DT_GET(DT_PARENT(node)),  \
        .channel = DT_PROP(node, channel)       \
    }                                           \

#define ESC_DT_SPEC_HAS_STATUS_OKAY(node)                   \
    (DT_NODE_HAS_STATUS(node, okay) &&                      \
     DT_NODE_HAS_STATUS(DT_PARENT(DT_PARENT(node)), okay))  \

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
typedef int (*esc_set_throttle_t)(const struct device *dev,
    uint32_t channel, uint16_t throttle);

/**
 * @brief ESC driver API call to send commands to the ESC(s).
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
};
/** @endcond */

/**
 * @brief Get the ESC driver type.
 *
 * @param[in] dev ESC device instance.
 *
 * @return The ESC type as indicated in esc_type enum.
 */
static inline enum esc_type esc_get_type(const struct device *dev)
{
    const struct esc_driver_api *api = dev->api;

    return api->get_type(dev);
}

/**
 * @brief Get if the ESC device is enabled.
 *
 * @see esc_set_enabled() for the characteristics of an
 * enabled/disabled device.
 *
 * @param[in] dev ESC device instance.
 *
 * @return True if @p dev is enabled, otherwise false.
 */
static inline bool esc_get_enabled(const struct device *dev)
{
    const struct esc_driver_api *api = dev->api;

    return api->get_enabled(dev);
}

/**
 * @brief Set the ESC device enabled/disabled. The ESC driver will only
 * send commands to the ESC if it is enabled.
 *
 * @note Some ESC specific device APIs can only be called when the device
 * is disabled.
 *
 * @param[in] dev ESC device instance.
 * @param enabled Input to enable/disable @p dev.
 *
 * @retval 1 If device is enabled.
 * @retval 0 If device is disabled.
 * @retval -errno Negative errno code on failure.
 */
static inline int esc_set_enabled(const struct device *dev, bool enabled)
{
    const struct esc_driver_api *api = dev->api;

    return api->set_enabled(dev, enabled);
}

/**
 * @brief Set the throttle for a single channel on the ESC device which will
 * be sent to the ESC upon the next call to esc_send(). Setting the throttle
 * to 0 will cause the device to stop sending commands for this channel.
 *
 * @see esc_send()
 *
 * @note While setting the throttle to 0 will stop the ESC device from sending
 * commands from this channel, the device remains enabled and will continue
 * sending commands from the other channels. Upon setting a non-zero ESC
 * throttle for this channel, the device will automatically resume sending its
 * commands.
 *
 * @param[in] dev ESC device instance.
 * @param channel DShot device channel.
 * @param throttle
 *
 * @retval 0 If successful.
 * @retval -EINVAL If the channel is inavlid.
 * @retval -EBUSY If the the device is currently sending commands.
 * @retval -errno Negative errno code on failure.
 */
static inline int esc_set_throttle(const struct device *dev,
                                   uint32_t channel, uint16_t throttle)
{
    const struct esc_driver_api *api = dev->api;

    return api->set_throttle(dev, channel, throttle);
}

/**
 * @brief Send the throttle commands to the ESCs on all channels.
 *
 * @param[in] dev ESC device instance.
 *
 * @retval 0 If successful.
 * @retval -ALREADY If the the device is currently sending commands.
 * @retval -errno Negative errno code on failure.
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