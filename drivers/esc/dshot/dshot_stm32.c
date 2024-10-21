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

#include <zephyr/drivers/clock_control/stm32_clock_control.h>

LOG_MODULE_REGISTER(dshot_stm32, CONFIG_DSHOT_LOG_LEVEL);

/** DShot data. */
struct dshot_stm32_data {
    enum dshot_type type;
    enum dshot_mode mode;
    bool enabled;
    /** Timer clock (Hz). */
    uint32_t tim_clk;
};

/** DShot configuration. */
struct dshot_stm32_config {
    TIM_TypeDef *timer;
    struct stm32_pclken pclken;
};

enum dshot_type dshot_get_type(struct device *dev) const
{
    const struct dshot_stm32_data *data = dev->data;
    return data->type;
}

int dshot_set_type(struct device *dev, enum dshot_type type)
{
    return 0;
}

enum dshot_mode dshot_get_mode(struct device *dev) const
{
    const struct dshot_stm32_data *data = dev->data;
    return data->mode;
}

#ifdef CONFIG_DSHOT_BIDIR
int dshot_set_mode(struct device *dev, enum dshot_mode mode)
{
    return 0;
}

void dshot_get_bidir_telem(struct device *dev, uint32_t channel, struct dshot_bidir_telem *out_telem)
{
    
}
#endif

void dshot_request_telem(struct device *dev, uint32_t channel, enum dshot_telem_type type)
{
    
}

int dshot_set_command(struct device *dev, uint32_t channel, enum dshot_command command)
{
    return 0;
}

bool dshot_command_in_progress(struct device *dev, uint32_t channel) const
{
    return false;
}

static enum esc_type dshot_stm32_get_type(const struct device *dev) const
{
    return ESC_TYPE_DSHOT;
}

static bool dshot_stm32_get_enabled(const struct device *dev) const
{
    const struct dshot_stm32_data *data = dev->data;
    return data->enabled;
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

static int dshot_stm32_init(const struct device *dev)
{
    struct dshot_stm32_data *data = dev->data;
    const struct dshot_stm32_config *cfg = dev->config;

    int r;
    const struct device *clk;
    LL_TIM_InitTypeDef init;

    /* enable clock and store its speed */
    clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

    if (!device_is_ready(clk)) {
        LOG_ERR("clock control device not ready");
        return -ENODEV;
    }

    r = clock_control_on(clk, (clock_control_subsys_t)&cfg->pclken);
    if (r < 0) {
        LOG_ERR("Could not initialize clock (%d)", r);
        return r;
    }

    r = get_tim_clk(&cfg->pclken, &data->tim_clk);
    if (r < 0) {
        LOG_ERR("Could not obtain timer clock (%d)", r);
        return r;
    }

    /* Reset timer to default state using RCC */
    // (void)reset_line_toggle_dt(&data->reset);

    /* configure pinmux */
    // r = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
    // if (r < 0) {
    // 	LOG_ERR("PWM pinctrl setup failed (%d)", r);
    // 	return r;
    // }

    /* initialize timer */
    LL_TIM_StructInit(&init);

    init.Prescaler = cfg->prescaler; // @todo: default to DSHOT 150
    init.CounterMode = cfg->countermode; // @todo: replace with UP
    init.Autoreload = 0u; // @todo: default to DSHOT 150
    init.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1; // @todo: default to DSHOT 150

    if (LL_TIM_Init(cfg->timer, &init) != SUCCESS) {
        LOG_ERR("Could not initialize timer");
        return -EIO;
    }

    cfg->irq_config_func(dev); // @todo: DMA irqs

    return 0;
}