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
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>

#include <zephyr/drivers/clock_control/stm32_clock_control.h>

#include "dshot_common.h"

LOG_MODULE_REGISTER(dshot_stm32, CONFIG_DSHOT_LOG_LEVEL);

/* L0 series MCUs only have 16-bit timers and don't have below macro defined */
#ifndef IS_TIM_32B_COUNTER_INSTANCE
#define IS_TIM_32B_COUNTER_INSTANCE(INSTANCE) (0)
#endif

/** Maximum number of timer channels : some stm32 soc have 6 else only 4 */
#if defined(LL_TIM_CHANNEL_CH6)
#define TIMER_HAS_6CH 1
#define TIMER_MAX_CH 6u
#else
#define TIMER_HAS_6CH 0
#define TIMER_MAX_CH 4u
#endif

/** DShot channel data. */
struct dshot_stm32_channel_data {
    bool pending_telem_req;
    uint16_t timer_counts[16];
};

/** DShot data. */
struct dshot_stm32_data {
    enum dshot_type type;
    enum dshot_mode mode;
    bool enabled;
    /** Timer clock (Hz). */
    uint32_t tim_clk;
    struct dshot_stm32_channel_data channel_data[TIMER_MAX_CH];
};

/** DShot channel config. */
struct dshot_stm32_channel_config {
    bool complementary;
};

/** DShot configuration. */
struct dshot_stm32_config {
    TIM_TypeDef *timer;
    struct stm32_pclken pclken;
    const struct pinctrl_dev_config *pcfg;
    struct dshot_stm32_channel_config channel_config[TIMER_MAX_CH];
};

/** Channel to LL mapping. */
static const uint32_t ch2ll[TIMER_MAX_CH] = {
	LL_TIM_CHANNEL_CH1, LL_TIM_CHANNEL_CH2,
	LL_TIM_CHANNEL_CH3, LL_TIM_CHANNEL_CH4,
#if TIMER_HAS_6CH
	LL_TIM_CHANNEL_CH5, LL_TIM_CHANNEL_CH6
#endif
};

/** Some stm32 mcus have complementary channels : 3 or 4 */
static const uint32_t ch2ll_n[] = {
#if defined(LL_TIM_CHANNEL_CH1N)
	LL_TIM_CHANNEL_CH1N,
	LL_TIM_CHANNEL_CH2N,
	LL_TIM_CHANNEL_CH3N,
#if defined(LL_TIM_CHANNEL_CH4N)
/** stm32g4x and stm32u5x have 4 complementary channels */
	LL_TIM_CHANNEL_CH4N,
#endif /* LL_TIM_CHANNEL_CH4N */
#endif /* LL_TIM_CHANNEL_CH1N */
};
/** Maximum number of complemented timer channels is ARRAY_SIZE(ch2ll_n)*/

/**
 * Obtain timer clock speed.
 *
 * @param pclken  Timer clock control subsystem.
 * @param tim_clk Where computed timer clock will be stored.
 *
 * @return 0 on success, error code otherwise.
 */
static int get_tim_clk(const struct stm32_pclken *pclken, uint32_t *tim_clk)
{
	int r;
	const struct device *clk;
	uint32_t bus_clk, apb_psc;

	clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	r = clock_control_get_rate(clk, (clock_control_subsys_t)pclken,
				   &bus_clk);
	if (r < 0) {
		return r;
	}

#if defined(CONFIG_SOC_SERIES_STM32H7X)
	if (pclken->bus == STM32_CLOCK_BUS_APB1) {
		apb_psc = STM32_D2PPRE1;
	} else {
		apb_psc = STM32_D2PPRE2;
	}
#else
	if (pclken->bus == STM32_CLOCK_BUS_APB1) {
#if defined(CONFIG_SOC_SERIES_STM32MP1X)
		apb_psc = (uint32_t)(READ_BIT(RCC->APB1DIVR, RCC_APB1DIVR_APB1DIV));
#else
		apb_psc = STM32_APB1_PRESCALER;
#endif
	}
#if !defined(CONFIG_SOC_SERIES_STM32C0X) && !defined(CONFIG_SOC_SERIES_STM32F0X) &&                \
	!defined(CONFIG_SOC_SERIES_STM32G0X)
	else {
#if defined(CONFIG_SOC_SERIES_STM32MP1X)
		apb_psc = (uint32_t)(READ_BIT(RCC->APB2DIVR, RCC_APB2DIVR_APB2DIV));
#else
		apb_psc = STM32_APB2_PRESCALER;
#endif
	}
#endif
#endif

#if defined(RCC_DCKCFGR_TIMPRE) || defined(RCC_DCKCFGR1_TIMPRE) || \
	defined(RCC_CFGR_TIMPRE)
	/*
	 * There are certain series (some F4, F7 and H7) that have the TIMPRE
	 * bit to control the clock frequency of all the timers connected to
	 * APB1 and APB2 domains.
	 *
	 * Up to a certain threshold value of APB{1,2} prescaler, timer clock
	 * equals to HCLK. This threshold value depends on TIMPRE setting
	 * (2 if TIMPRE=0, 4 if TIMPRE=1). Above threshold, timer clock is set
	 * to a multiple of the APB domain clock PCLK{1,2} (2 if TIMPRE=0, 4 if
	 * TIMPRE=1).
	 */

	if (LL_RCC_GetTIMPrescaler() == LL_RCC_TIM_PRESCALER_TWICE) {
		/* TIMPRE = 0 */
		if (apb_psc <= 2u) {
			LL_RCC_ClocksTypeDef clocks;

			LL_RCC_GetSystemClocksFreq(&clocks);
			*tim_clk = clocks.HCLK_Frequency;
		} else {
			*tim_clk = bus_clk * 2u;
		}
	} else {
		/* TIMPRE = 1 */
		if (apb_psc <= 4u) {
			LL_RCC_ClocksTypeDef clocks;

			LL_RCC_GetSystemClocksFreq(&clocks);
			*tim_clk = clocks.HCLK_Frequency;
		} else {
			*tim_clk = bus_clk * 4u;
		}
	}
#else
	/*
	 * If the APB prescaler equals 1, the timer clock frequencies
	 * are set to the same frequency as that of the APB domain.
	 * Otherwise, they are set to twice (×2) the frequency of the
	 * APB domain.
	 */
	if (apb_psc == 1u) {
		*tim_clk = bus_clk;
	} else {
		*tim_clk = bus_clk * 2u;
	}
#endif

	return 0;
}

static enum esc_type dshot_stm32_get_esc_type(const struct device *dev) const
{
    return ESC_TYPE_DSHOT;
}

static enum dshot_type dshot_stm32_get_type(struct device *dev) const
{
    const struct dshot_stm32_data *data = dev->data;
    return data->type;
}

static int dshot_stm32_set_type(struct device *dev, enum dshot_type type)
{
    return 0;
}

static enum dshot_mode dshot_stm32_get_mode(struct device *dev) const
{
    const struct dshot_stm32_data *data = dev->data;
    return data->mode;
}

static int dshot_stm32_set_mode(struct device *dev, enum dshot_mode mode)
{
    if (mode == DSHOT_MODE_NORMAL) {
        LL_TIM_OC_SetPolarity(cfg->timer, current_ll_channel, get_polarity(flags));
        
    } else {

    }
    return 0;
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

static int dshot_stm32_set_packet(const struct device *dev, uint32_t channel, uint16_t payload)
{
    struct dshot_stm32_data *data = dev->data;

    if (channel < 1u || channel > TIMER_MAX_CH) {
        return -EINVAL;
    }
    struct dshot_stm32_channel_data *channel_data = &data->channel_data[channel - 1u];

    uint16_t packet = dshot_common_make_packet(payload,
                        channel_data->pending_telem_req,
                        data->mode == DSHOT_MODE_BIDIRECTIONAL);
    channel_data->pending_telem_req = false;
}

static int dshot_stm32_set_throttle(const struct device *dev, uint32_t channel, uint16_t throttle)
{
    return dshot_stm32_set_packet(dev, channel, dshot_common_quantize_throttle(throttle));
}

static int dshot_stm32_set_command(struct device *dev, uint32_t channel, enum dshot_command command)
{
    return dshot_stm32_set_packet(dev, channel, (uint16_t)command);
}

static int dshot_stm32_set_request_telem(struct device *dev, uint32_t channel)
{
    struct dshot_stm32_data *data = dev->data;

    if (channel < 1u || channel > TIMER_MAX_CH) {
        return -EINVAL;
    }
    const int ch_idx = channel - 1;
    data->channel_data[ch_idx].pending_telem_req = true;
    return 0;
}

static int dshot_stm32_send(const struct device *dev)
{
    return 0;
}

static bool dshot_stm32_command_in_progress(struct device *dev, uint32_t channel) const
{
    return false;
}

static void dshot_stm32_get_telem(struct device *dev, uint32_t channel, struct dshot_bidir_telem *out_telem)
{
    
}

static const struct dshot_driver_api dshot_stm32_driver_api = {
    .get_type = dshot_stm32_get_type,
    .set_type = dshot_stm32_set_type,
    .get_mode = dshot_stm32_get_mode,
#ifdef CONFIG_DSHOT_BIDIR
    .set_mode = dshot_stm32_set_mode,
#endif
    .set_command = dshot_stm32_set_command,
    .command_in_progress = dshot_stm32_command_in_progress,
    .set_request_telem = dshot_stm32_set_request_telem,
#ifdef CONFIG_DSHOT_BIDIR
    .get_telem = dshot_stm32_get_telem,
#endif
};

static const struct esc_driver_api dshot_esc_stm32_driver_api = {
    .get_type = dshot_stm32_get_esc_type,
    .get_enabled = dshot_stm32_get_enabled,
    .set_enabled = dshot_stm32_set_enabled,
    .set_throttle = dshot_stm32_set_throttle,
    .send = dshot_stm32_send,
    .extended_driver_api = dshot_stm32_driver_api,
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