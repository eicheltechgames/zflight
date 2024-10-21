/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_dshot

#include <errno.h>

#include <stm32_ll_tim.h>
#include <stm32_ll_dma.h>
#include <zflight/drivers/esc.h>
#include <zflight/drivers/esc/dshot.h>

LOG_MODULE_REGISTER(dshot_stm32, CONFIG_DSHOT_LOG_LEVEL);

/** DShot data. */
struct dshot_stm32_data {

};

/** DShot configuration. */
struct dshot_stm32_config {
	TIM_TypeDef *timer;
};

static enum esc_type dshot_stm32_get_type(const struct device *dev) const
{
    return ESC_TYPE_DSHOT;
}

static bool dshot_stm32_get_enabled(const struct device *dev)
{
    return false;
}

static int dshot_stm32_set_enabled(const struct device *dev, bool enabled)
{
    return 0;
}

static int dshot_stm32_set_throttle(const struct device *dev, uint32_t channel, uint16_t throttle)
{
    return 0;
}

static int dshot_stm32_send(const struct device *dev)
{
    return 0;
}

static const struct esc_driver_api dshot_stm32_driver_api = {
    .get_type = dshot_stm32_get_type,
    .get_enabled = dshot_stm32_get_enabled,
    .set_enabled = dshot_stm32_set_enabled,
    .set_throttle = dshot_stm32_set_throttle,
    .send = dshot_stm32_send,
};

static int pwm_stm32_init(const struct device *dev)
{
    return 0;
}