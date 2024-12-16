/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public APIs for the Team Blacksheep (TBS) Crossfire (CRSF) RC Link
 */

#ifndef ZFLIGHT_INCLUDE_DRIVERS_RC_LINK_CRSF_H_
#define ZFLIGHT_INCLUDE_DRIVERS_RC_LINK_CRSF_H_

#include <zflight/drivers/rc_link.h>
#include <zflight/drivers/rc_link/protocols/crsf_protocol.h>

#ifdef __cplusplus
extern "C" {
#endif

const struct crsf_device_info_frame * rc_link_crsf_get_local_device_info(const struct device *dev);

void rc_link_crsf_get_rx_device_info(const struct device *dev, struct crsf_device_info_frame *device_info);

void rc_link_crsf_get_channels_raw(const struct device *dev, struct crsf_rc_channels_payload *channels);

void rc_link_crsf_get_link_statistics_raw(const struct device *dev, struct crsf_link_statistics_payload *stats);

// TODO: add other crsf packets

#ifdef __cplusplus
}
#endif

#endif // ZFLIGHT_INCLUDE_DRIVERS_RC_LINK_CRSF_H_