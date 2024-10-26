/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_dshot

#include <errno.h>

#include <stm32_ll_tim.h>
#include <zflight/drivers/esc.h>
#include <zflight/drivers/esc/dshot.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>

#include <zephyr/drivers/clock_control/stm32_clock_control.h>

#include "dshot_common.h"

LOG_MODULE_REGISTER(dshot_stm32, CONFIG_DSHOT_LOG_LEVEL);

/* L0 series MCUs only have 16-bit timers and don't have below macro defined */
#ifndef IS_TIM_32B_COUNTER_INSTANCE
#define IS_TIM_32B_COUNTER_INSTANCE(INSTANCE) (0)
#endif

/** Maximum number of timer channels : only 4 supported */
#define TIMER_MAX_CH        4u

static enum dshot_stm32_dir {
    INPUT_DIR = 0b01,
    OUTPUT_DIR = 0b10,
    ANY_DIR = 0b11,
};

/** DShot DMA */
struct dshot_stm32_dma {
    const struct device *dev;
    struct dma_config config;
    struct dma_block_config block;
};

/** DShot channel data. */
struct dshot_stm32_channel_data {
    bool pending_telem_req;
    struct dshot_command_settings cmd_status;
    uint16_t dshot_tim_buf[DSHOT_PACKET_BITS];
#ifdef CONFIG_DSHOT_BIDIR
    LL_TIM_OC_InitTypeDef oc_init;
    uint16_t telem_cnts_buf[TELEM_PACKET_BITS];
#endif
    struct dshot_stm32_dma dma;
}; // @todo make non-cachable for DMA

/** DShot data. */
struct dshot_stm32_data {
    uint32_t last_send_timestamp;
#ifdef CONFIG_DSHOT_BIDIR
    enum dshot_stm32_dir curr_dir;
    LL_TIM_IC_InitTypeDef ic_init;
#endif
    struct dshot_stm32_channel_data *channel_data[TIMER_MAX_CH];
};

/** DShot channel config. */
struct dshot_stm32_channel_config {
    const struct pinctrl_dev_config *pcfg;
    uint32_t ll_channel;
};

/** DShot configuration. */
struct dshot_stm32_config {
    TIM_TypeDef *timer;
    struct stm32_pclken pclken;
    struct dshot_stm32_channel_config *channel_config[TIMER_MAX_CH];
};

static inline bool is_complementary_channel(uint32_t ll_channel)
{
    switch (ll_channel)
    {
    case LL_TIM_CHANNEL_CH1N:
    case LL_TIM_CHANNEL_CH2N:
    case LL_TIM_CHANNEL_CH3N:
#if defined(LL_TIM_CHANNEL_CH4N)
    case LL_TIM_CHANNEL_CH4N:
#endif
        return true;
    default:
        return false;
    }
}

static inline uint32_t positive_channel(uint32_t ll_channel)
{
    switch (ll_channel)
    {
    case LL_TIM_CHANNEL_CH1N:
        return LL_TIM_CHANNEL_CH1;
    case LL_TIM_CHANNEL_CH2N:
        return LL_TIM_CHANNEL_CH2;
    case LL_TIM_CHANNEL_CH3N:
        return LL_TIM_CHANNEL_CH3;
#if defined(LL_TIM_CHANNEL_CH4N)
    case LL_TIM_CHANNEL_CH4N:
        return LL_TIM_CHANNEL_CH4;
#endif
    default:
        return ll_channel;
    }
}

static inline bool get_tim_busy(const struct device *dev, enum dshot_stm32_dir dir)
{
#ifndef CONFIG_DSHOT_BIDIR
    ARG_UNUSED(dir);
#endif
    const struct dshot_stm32_config *cfg = dev->config;

    if (LL_TIM_IsEnabledCounter(cfg->timer)) {
#ifdef CONFIG_DSHOT_BIDIR
        const struct dshot_stm32_data *data = dev->data;
        if (data->curr_dir & dir)
#endif
        {
            return true;
        }
    }
    return false;
}

static int get_channel_data(struct dshot_stm32_data *data,
    uint32_t channel, struct dshot_stm32_channel_data** out_ch_data)
{
    if (channel < 1u || channel > TIMER_MAX_CH) {
        LOG_ERR("Channel %d out of bounds", channel);
        return -EINVAL;
    }
    *out_ch_data = data->channel_data[channel - 1u];

    if (*out_ch_data == NULL) {
        LOG_ERR("Channel %d was not configured", channel);
        return -EINVAL;
    }
    return 0;
}

static int get_channel_config(const struct dshot_stm32_config *cfg,
    uint32_t channel, const struct dshot_stm32_channel_config** out_ch_cfg)
{
    if (channel < 1u || channel > TIMER_MAX_CH) {
        LOG_ERR("Channel %d out of bounds", channel);
        return -EINVAL;
    }
    *out_ch_cfg = cfg->channel_config[channel - 1u];

    if (*out_ch_cfg == NULL) {
        LOG_ERR("Channel %d was not configured", channel);
        return -EINVAL;
    }
    return 0;
}

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

#ifdef CONFIG_DSHOT_BIDIR
static void dshot_stm32_set_direction_output(const struct device *dev)
{
    const struct dshot_stm32_config *cfg = dev->config;
    struct dshot_stm32_data *data = dev->data;

    for (int ch_idx = 0; ch_idx < TIMER_MAX_CH; ++ch_idx) {
        const struct dshot_stm32_channel_config *ch_cfg =
            cfg->channel_config[ch_idx];
        struct dshot_stm32_channel_data *ch_data =
            data->channel_data[ch_idx];

        if (ch_cfg == NULL || ch_data == NULL) {
            continue;
        }

        // @todo disable dma

        const uint32_t channel = positive_channel(ch_cfg->ll_channel);
        LL_TIM_OC_DisablePreload(cfg->timer, channel);
        LL_TIM_OC_Init(cfg->timer, channel, ch_data->oc_init);
        LL_TIM_OC_EnablePreload(cfg->timer, channel);

        // @todo configure dma
    }

    LL_TIM_SetCounter(cfg->timer, 0);
    data->curr_dir = INPUT_DIR;
}

static void dshot_stm32_set_direction_input(const struct device *dev)
{
    const struct dshot_stm32_config *cfg = dev->config;
    struct dshot_stm32_data *data = dev->data;

    LL_TIM_EnableARRPreload(cfg->timer); // Only update the period once all channels are done
    if (IS_TIM_32B_COUNTER_INSTANCE(cfg->timer)) {
        LL_TIM_SetAutoReload(cfg->timer, UINT32_MAX);
    } else {
        LL_TIM_SetAutoReload(cfg->timer, UINT16_MAX);
    }

    for (int ch_idx = 0; ch_idx < TIMER_MAX_CH; ++ch_idx) {
        const struct dshot_stm32_channel_config *ch_cfg =
            cfg->channel_config[ch_idx];
        struct dshot_stm32_channel_data *ch_data =
            data->channel_data[ch_idx];

        if (ch_cfg == NULL || ch_data == NULL) {
            continue;
        }

        // @todo disable dma

        const uint32_t channel = positive_channel(ch_cfg->ll_channel);
        LL_TIM_IC_Init(cfg->timer, channel, data->ic_init);

        // @todo configure dma
    }

    LL_TIM_SetCounter(cfg->timer, 0);
    data->curr_dir = OUTPUT_DIR;
}
#endif

static enum esc_type dshot_stm32_get_esc_type(const struct device *dev)
{
    ARG_UNUSED(dev);
    return ESC_TYPE_DSHOT;
}

static enum dshot_type dshot_stm32_get_type(const struct device *dev)
{
    const struct dshot_stm32_config *cfg = dev->config;
    uint32_t tim_clk;

    get_tim_clk(&cfg->pclken, &tim_clk);
    const uint16_t prescaler = LL_TIM_GetPrescaler(cfg->timer) + 1;
    const uint32_t dshot = tim_clk / prescaler / DSHOT_TIM_CNTS_PER_BIT;
    return dshot / KHZ(1);
}

static int dshot_stm32_set_type(const struct device *dev, enum dshot_type type)
{
    const struct dshot_stm32_config *cfg = dev->config;
    uint32_t tim_clk;
    int err = 0;

    if (get_tim_busy(dev, ANY_DIR)) {
        return -EBUSY;
    }

    err = get_tim_clk(&cfg->pclken, &tim_clk);
    if (err) {
        LOG_ERR("Could not obtain timer clock (%d)", err);
        return err;
    }

    const uint16_t prescaler = tim_clk / DSHOT_TIM_CNTS_PER_BIT * DSHOT_BIT_RATE(type);
    const uint16_t rem = tim_clk % DSHOT_TIM_CNTS_PER_BIT * DSHOT_BIT_RATE(type);
    if (rem) {
        LOG_ERR("Timer clock not configured for dshot (%d)", tim_clk);
        return -EIO;
    }

    LL_TIM_SetPrescaler(cfg->timer, prescaler - 1);
    return 0;
}

static enum dshot_mode dshot_stm32_get_mode(const struct device *dev)
{
#ifdef CONFIG_DSHOT_BIDIR
    struct dshot_stm32_data *data = dev->data;

    for (int ch_idx = 0; ch_idx < TIMER_MAX_CH; ++ch_idx) {
        struct dshot_stm32_channel_data *ch_data =
            data->channel_data[ch_idx];

        if (ch_data == NULL) {
            continue;
        }


        if (ch_data->oc_init.OCState == LL_TIM_OCSTATE_ENABLE) {
            return ch_data->oc_init.OCPolarity == LL_TIM_OCPOLARITY_HIGH ?
                DSHOT_MODE_NORMAL : DSHOT_MODE_BIDIRECTIONAL;
        } else {
            return ch_data->oc_init.OCNPolarity == LL_TIM_OCPOLARITY_HIGH ?
                DSHOT_MODE_NORMAL : DSHOT_MODE_BIDIRECTIONAL;
        }
    }
#endif
    return DSHOT_MODE_NORMAL;
}

#ifdef CONFIG_DSHOT_BIDIR
static int dshot_stm32_set_mode(const struct device *dev, enum dshot_mode mode)
{
    struct dshot_stm32_data *data = dev->data;

    if (get_tim_busy(dev, ANY_DIR)) {
        return -EBUSY;
    }

    for (int ch_idx = 0; ch_idx < TIMER_MAX_CH; ++ch_idx) {
        struct dshot_stm32_channel_data *ch_data =
            data->channel_data[ch_idx];

        if (ch_data == NULL) {
            continue;
        }

        /* Channels are all same mode */
        if (ch_data->oc_init.OCState == LL_TIM_OCSTATE_ENABLE) {
            ch_data->oc_init.OCPolarity = mode == DSHOT_MODE_NORMAL ?
                LL_TIM_OCPOLARITY_HIGH : LL_TIM_OCPOLARITY_LOW;
        } else {
            ch_data->oc_init.OCNPolarity = mode == DSHOT_MODE_NORMAL ?
                LL_TIM_OCPOLARITY_HIGH : LL_TIM_OCPOLARITY_LOW;
        }
    }
    return 0;
}
#endif

static bool dshot_stm32_get_enabled(const struct device *dev)
{
    const struct dshot_stm32_config *cfg = dev->config;

    for (int ch_idx = 0; ch_idx < TIMER_MAX_CH; ++ch_idx) {
        const struct dshot_stm32_channel_config *ch_cfg =
            cfg->channel_config[ch_idx];

        if (ch_cfg == NULL) {
            continue;
        }

        /* Channels are either all enabled or all disabled */
        return LL_TIM_CC_IsEnabledChannel(cfg->timer, ch_cfg->ll_channel);
    }
    return false;
}

static int dshot_stm32_set_enabled(const struct device *dev, bool enabled)
{
    const struct dshot_stm32_config *cfg = dev->config;

    for (int ch_idx = 0; ch_idx < TIMER_MAX_CH; ++ch_idx) {
        const struct dshot_stm32_channel_config *ch_cfg =
            cfg->channel_config[ch_idx];

        if (ch_cfg == NULL) {
            continue;
        }

        if (enabled) {
            LL_TIM_CC_EnableChannel(cfg->timer, ch_cfg->ll_channel);
        } else {
            LL_TIM_CC_DisableChannel(cfg->timer, ch_cfg->ll_channel);
        }
    }
    return 0;
}

static int dshot_stm32_set_packet(const struct device *dev, uint32_t channel, uint16_t payload)
{
    const struct dshot_stm32_config *cfg = dev->config;
    struct dshot_stm32_data *data = dev->data;
    const struct dshot_stm32_channel_config *ch_cfg;
    struct dshot_stm32_channel_data *ch_data;
    int err;

    err = get_channel_config(cfg, channel, &ch_cfg);
    if (err) {
        return err;
    }
    err = get_channel_data(data, channel, &ch_data);
    if (err) {
        return err;
    }

    if (get_tim_busy(dev, OUTPUT_DIR)) {
        return -EBUSY;
    }

    uint16_t packet = dshot_common_make_packet(payload,
                        ch_data->pending_telem_req,
                        dshot_stm32_get_mode(dev) == DSHOT_MODE_BIDIRECTIONAL);
    ch_data->pending_telem_req = false;

    dshot_common_load_dshot_buffer(&ch_data->dshot_tim_buf[0], packet);
    return 0;
}

static int dshot_stm32_set_throttle(const struct device *dev, uint32_t channel, uint16_t throttle)
{
    static const struct dshot_command_settings reset = { 0 };

    int err = dshot_stm32_set_packet(dev, channel, dshot_common_quantize_throttle(throttle));

    if (!err) {
        struct dshot_stm32_data *data = dev->data;
        struct dshot_stm32_channel_data *ch_data = &data->channel_data[channel - 1u];
        ch_data->cmd_status = reset;
    }
    return err;
}

static int dshot_stm32_set_command(const struct device *dev, uint32_t channel, enum dshot_command command)
{
    int err = dshot_stm32_set_packet(dev, channel, (uint16_t)command);

    if (!err) {
        struct dshot_stm32_data *data = dev->data;
        struct dshot_stm32_channel_data *ch_data = &data->channel_data[channel - 1u];
        ch_data->cmd_status = command_settings[command];
    }
    return err;
}

static int dshot_stm32_set_request_telem(const struct device *dev, uint32_t channel)
{
    struct dshot_stm32_data *data = dev->data;
    struct dshot_stm32_channel_data *ch_data;

    int err = get_channel_data(data, channel, &ch_data);
    if (err) {
        return err;
    }

    ch_data->pending_telem_req = true;
    return 0;
}

static int dshot_stm32_send(const struct device *dev)
{
    const struct dshot_stm32_config *cfg = dev->config;
    struct dshot_stm32_data *data = dev->data;

    if (get_tim_busy(dev, ANY_DIR)) {
        return -EBUSY;
    }

    LL_TIM_EnableCounter(cfg->timer);

    uint32_t curr_timestamp = k_uptime_get_32();
    for (int ch_idx = 0; ch_idx < TIMER_MAX_CH; ++ch_idx) {
        struct dshot_stm32_channel_data *ch_data =
            data->channel_data[ch_idx];

        if (ch_data == NULL) {
            continue;
        }

        if (ch_data->cmd_status.repeat > 0) {
            ch_data->cmd_status.repeat--;
        }
        if (ch_data->cmd_status.delay_ms > 0) {
            int32_t delta = ch_data->cmd_status.delay_ms
                - (curr_timestamp - data->last_send_timestamp);
            ch_data->cmd_status.delay_ms = MAX(delta, 0);
        }
    }
    data->last_send_timestamp = curr_timestamp;

    return 0;
}

static bool dshot_stm32_command_in_progress(const struct device *dev, uint32_t channel)
{
    struct dshot_stm32_data *data = dev->data;
    struct dshot_stm32_channel_data *ch_data;
    int err = get_channel_data(data, channel, &ch_data);
    if (err) {
        return err;
    }

    return ch_data->cmd_status.repeat > 0 || ch_data->cmd_status.delay_ms > 0;
}

static int dshot_stm32_stop_receive(const struct device *dev)
{
    const struct dshot_stm32_config *cfg = dev->config;
    LL_TIM_DisableCounter(cfg->timer);
    // @ todo stop DMA
    dshot_stm32_set_direction_output(dev);
}

static int dshot_stm32_decode_telem(const struct device *dev, uint32_t channel, uint16_t *out_telem)
{
    const struct dshot_stm32_config *cfg = dev->config;
    struct dshot_stm32_data *data = dev->data;
    struct dshot_stm32_channel_data *ch_data;
    int err;

    if (get_tim_busy(dev, INPUT_DIR)) {
        return -EBUSY;
    }

    err = get_channel_data(data, channel, &ch_data);
    if (err) {
        return err;
    }

    // @ todo decode
}

static const struct dshot_driver_api dshot_stm32_driver_api = {
    .esc.get_type = dshot_stm32_get_esc_type,
    .esc.get_enabled = dshot_stm32_get_enabled,
    .esc.set_enabled = dshot_stm32_set_enabled,
    .esc.set_throttle = dshot_stm32_set_throttle,
    .esc.send = dshot_stm32_send,
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
    .stop_receive = dshot_stm32_stop_receive,
    .decode_telem = dshot_stm32_decode_telem,
#endif
};

static void dshot_stm32_dma_init(struct dshot_stm32_channel_data *ch_data, uint32_t *CCR)
{
    struct dshot_stm32_dma *dma = &ch_data->dma;

    /* DShot Output direction */
    dma->block.source_address = (uint32_t)&ch_data->dshot_tim_buf[0];
    dma->block.dest_address = (uint32_t)CCR;
    dma->block.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
    dma->block.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
    dma->block.block_size = sizeof(ch_data->dshot_tim_buf);
    dma->block.next_block = NULL;

    /* RX disable circular buffer */
    dma->block.source_reload_en  = 0;
    dma->block.dest_reload_en = 0;

    dma->config.head_block = &dma->block;
}

static int dshot_stm32_init(const struct device *dev)
{
    struct dshot_stm32_data *data = dev->data;
    const struct dshot_stm32_config *cfg = dev->config;

    int err;
    const struct device *clk;
    LL_TIM_InitTypeDef init;

    /* enable clock and store its speed */
    clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

    if (!device_is_ready(clk)) {
        LOG_ERR("clock control device not ready");
        return -ENODEV;
    }

    err = clock_control_on(clk, (clock_control_subsys_t)&cfg->pclken);
    if (err) {
        LOG_ERR("Could not initialize clock (%d)", err);
        return err;
    }

    /* Reset timer to default state using RCC */
    (void)reset_line_toggle_dt(&data->reset);

    /* verify channels and configure pinmux */
    for (int i = 0; i < TIMER_MAX_CH; ++i) {
        const struct dshot_stm32_channel_config *ch_cfg = cfg->channel_config[i];
        struct dshot_stm32_channel_data *ch_data = data->channel_data[i];

        if ((ch_cfg->pcfg != NULL || ch_cfg != NULL || ch_data != NULL) &&
            (ch_cfg->pcfg != NULL && ch_cfg != NULL && ch_data != NULL)) {
            // @todo log warning about mis-configure
            data->channel_data[i] = NULL;
            continue;
        }

        /* configure pinmux */
        err = pinctrl_apply_state(ch_cfg->pcfg, PINCTRL_STATE_DEFAULT);
        if (err < 0) {
            LOG_ERR("PWM pinctrl setup failed (%d)", err);
            return err;
        }
    }

    /* initialize timer */
    LL_TIM_StructInit(&init);

    init.Prescaler = 0;
    init.CounterMode = LL_TIM_COUNTERMODE_UP;
    init.Autoreload = DSHOT_TIM_CNTS_PER_BIT;
    init.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_DisableCounter(cfg->timer);

    if (LL_TIM_Init(cfg->timer, &init) != SUCCESS) {
        LOG_ERR("Could not initialize timer");
        return -EIO;
    }

    err = dshot_set_type(dev, DSHOT_300);
    if (err) {
        LOG_ERR("Could not set Type = DSHOT_300 (%d)", err);
        return err;
    }

    /* configure channels */
    LL_TIM_OC_InitTypeDef *oc_init;
#ifndef CONFIG_DSHOT_BIDIR
    LL_TIM_OC_InitTypeDef oc_init_data;
    oc_init = &oc_init_data;
#else
    /* configure input compare */
    LL_TIM_IC_StructInit(&data->ic_init);
    data->ic_init->icInitStruct.ICPolarity = LL_TIM_IC_POLARITY_BOTHEDGE;
    data->ic_init->icInitStruct.ICPrescaler = LL_TIM_ICPSC_DIV1;
    data->ic_init->icInitStruct.ICFilter = 2;
#endif
    for (int i = 0; i < TIMER_MAX_CH; ++i) {
        const struct dshot_stm32_channel_config *ch_cfg = cfg->channel_config[i];
        struct dshot_stm32_channel_data *ch_data = data->channel_data[i];

        if (ch_cfg == NULL || ch_data == NULL) {
            continue;
        }

        /* configure output compare */
#ifdef CONFIG_DSHOT_BIDIR
        oc_init = &ch_data->oc_init;
#endif
        LL_TIM_OC_StructInit(oc_init);
        oc_init->OCMode = LL_TIM_OCMODE_PWM1;
        if (is_complementary_channel(ch_cfg->ll_channel)) {
            oc_init->OCNState = LL_TIM_OCSTATE_ENABLE;
            oc_init->OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
            oc_init->OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
        } else {
            oc_init->OCState = LL_TIM_OCSTATE_ENABLE;
            oc_init->OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
            oc_init->OCPolarity =  LL_TIM_OCPOLARITY_HIGH;
        }

        const uint32_t channel = positive_channel(ch_cfg->ll_channel);
#ifndef CONFIG_DSHOT_BIDIR
        LL_TIM_OC_Init(cfg->timer, channel, oc_init);
        LL_TIM_OC_EnablePreload(cfg->timer, channel);
#endif
        LL_TIM_OC_DisableFast(cfg->timer, channel);

        /* configure dma */
        dshot_stm32_dma_init(ch_data, &(cfg->timer->CCR1) + i);
    }

#ifdef CONFIG_DSHOT_BIDIR
    dshot_stm32_set_direction_output(dev);
#endif

    cfg->irq_config_func(dev); // @todo: DMA irqs

    return 0;
}