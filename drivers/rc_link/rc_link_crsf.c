/*
 * Copyright 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zflight/drivers/rc_link/rc_link_crsf.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys_clock.h>
#include <zephyr/irq.h>
#ifdef CONFIG_DCACHE
#include <zephyr/dt-bindings/memory-attr/memory-attr-arm.h>
#endif /* CONFIG_DCACHE */
#ifdef CONFIG_NOCACHE_MEMORY
#include <zephyr/linker/linker-defs.h>
#else
#include <zephyr/arch/cache.h>
#endif /* CONFIG_NOCACHE_MEMORY */
#include <zephyr/arch/cache.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/crc.h>

#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(crsf, CONFIG_RC_LINK_LOG_LEVEL);

#define DT_DRV_COMPAT   tbs_crsf

#if defined(CONFIG_UART_EXCLUSIVE_API_CALLBACKS) && defined(CONFIG_UART_ASYNC_API)
#error "CONFIG_UART_EXCLUSIVE_API_CALLBACKS" must =n if CONFIG_UART_ASYNC_API=y
#endif

#if defined(CONFIG_UART_ASYNC_API) && defined(CONFIG_DCACHE) && !defined(CONFIG_NOCACHE_MEMORY)
#define RC_LINK_CRSF_MANUAL_CACHE_COHERENCY_REQUIRED	1
#else
#define RC_LINK_CRSF_MANUAL_CACHE_COHERENCY_REQUIRED	0
#endif

#ifdef CONFIG_DCACHE
#define RC_LINK_CRSF_BUF_SIZE   \
    ROUND_UP(CRSF_MAX_PACKET_LEN, CONFIG_DCACHE_LINE_SIZE)
#else
#define RC_LINK_CRSF_BUF_SIZE   CRSF_MAX_PACKET_LEN
// override for interoperability
#define CONFIG_DCACHE_LINE_SIZE 4
#endif

/* 8 data bits + 1 stop bit per byte */
#define CRSF_BITS_PER_BYTE      9

#define CRSF_DEFAULT_BAUDRATE   416666
#define ELRS_DEFAULT_BAUDRATE   420000

#define CRSF_CHANNEL_SCALED_MIN  172 // 987us - actual CRSF min is 0 with E.Limits on
#define CRSF_CHANNEL_VALUE_1000  191
#define CRSF_CHANNEL_VALUE_MID   992
#define CRSF_CHANNEL_VALUE_2000  1792
#define CRSF_CHANNEL_SCALED_MAX  1811 // 2012us - actual CRSF max is 1984 with E.Limits on

#define CRSF_RSSI_MIN (-130)
#define CRSF_RSSI_MAX  0
#define CRSF_SNR_MIN  (-30)
#define CRSF_SNR_MAX   20

#define CRSF_TELEM_TX_PERIOD_MS     1000
#define CRSF_CHANNELS_TX_PERIOD_MS  100

#define SUPPORTED_UPLINK    RC_LINK_STATUS_RSSI_PCT_FLAG | \
                            RC_LINK_STATUS_LINK_QUALITY_FLAG | \
                            RC_LINK_STATUS_RSSI_DBM_FLAG | \
                            RC_LINK_STATUS_SNR_FLAG | \
                            RC_LINK_STATUS_POWER_FLAG

#define SUPPORTED_DOWNLINK  RC_LINK_STATUS_DOWNLINK_FLAG | \
                            RC_LINK_STATUS_RSSI_PCT_FLAG | \
                            RC_LINK_STATUS_LINK_QUALITY_FLAG | \
                            RC_LINK_STATUS_RSSI_DBM_FLAG | \
                            RC_LINK_STATUS_SNR_FLAG

#define SUPPORTED_TELEM     RC_LINK_TELEM_VBATT_FLAG | \
                            RC_LINK_TELEM_CURRENT_FLAG | \
                            RC_LINK_TELEM_BATT_CAPACITY_FLAG | \
                            RC_LINK_TELEM_BATT_PERCENT_FLAG

struct rc_link_crsf_config {
    const struct rc_link_config base_config;
    const struct device *uart;
    enum crsf_addr dev_addr;
    const struct crsf_device_info_frame device_info;
    uint32_t tx_period;
};

struct rc_link_crsf_data {
    const struct device *dev;
    atomic_t enabled;
    bool is_ELRS;
    struct k_mutex lock;
    struct k_work work_process_rx;
    struct k_work_delayable work_process_tx;
    rc_link_on_channels_received_cb_t channels_cb;
    void *channels_user_data;
    rc_link_on_telem_received_cb_t telem_cb;
    void *telem_user_data;
    union crsf_frame_buffer *rx_buf_ptr;
    union crsf_frame_buffer *tx_buf_ptr;
    struct crsf_device_info_frame rx_device_info;
    struct crsf_rc_channels_payload channels;
    struct crsf_link_statistics_payload link_stats;
#ifdef CONFIG_RC_LINK_TELEMETRY
    struct crsf_sensor_battery_payload batt_data;
    struct crsf_sensor_baro_vario_payload bario_vario_data;
    uint8_t telem_sched;
#endif // CONFIG_RC_LINK_TELEMETRY
    uint32_t last_rx_us;
    uint32_t rx_byte_tol_us;
    uint32_t rx_tol_us;
    uint8_t num_rx_bytes;
    uint8_t tx_address;
    uint8_t tx_message_type;
};

static const int telem_schedule[] = {
    CRSF_FRAMETYPE_GPS,
    CRSF_FRAMETYPE_VARIO, // see INDEX_OF_VARIO_TELEM_SCHED
    CRSF_FRAMETYPE_BATTERY_SENSOR,
    CRSF_FRAMETYPE_BARO_ALTITUDE,
    CRSF_FRAMETYPE_ATTITUDE,
    CRSF_FRAMETYPE_FLIGHT_MODE,
};
#define INDEX_OF_VARIO_TELEM_SCHED  1

static inline uint16_t scale_range(uint16_t x, uint16_t srcFrom, uint16_t srcTo, uint16_t destFrom, uint16_t destTo)
{
    uint32_t a = ((uint32_t) destTo - (uint32_t) destFrom) *
        ((uint32_t) x - (uint32_t) srcFrom);
    uint32_t b = (uint32_t) srcTo - (uint32_t) srcFrom;
    return (a / b) + destFrom;
}

static inline uint8_t calculate_packet_crc(const union crsf_frame_buffer *frame_buffer)
{
    const uint8_t crc_len =
        crsf_frame_get_frame_len(frame_buffer) - CRSF_FRAME_CRC_SIZE;
    return crc8(&frame_buffer->bytes[CRSF_HEADER_TYPE_INDEX],
        crc_len, CRSF_CRC_POLY, 0, false);
}

static inline int check_packet_integrity(const union crsf_frame_buffer *frame_buffer, uint8_t rx_len)
{
    const uint8_t packet_len = crsf_frame_get_packet_size(frame_buffer);
    if (packet_len != rx_len) {
        return -EMSGSIZE;
    }

    const uint8_t crc = calculate_packet_crc(frame_buffer);
    if (crc != crsf_frame_get_crc(frame_buffer)) {
        return -EIO;
    }

    return 0;
}

static inline int check_packet_address(const union crsf_frame_buffer *frame_buffer, uint8_t dev_addr)
{
    enum crsf_addr packet_addr = crsf_frame_get_frame_device_addr(frame_buffer);
    bool packet_addr_valid = packet_addr == CRSF_SYNC_BYTE ||
                             packet_addr == dev_addr ||
                             packet_addr == CRSF_ADDRESS_BROADCAST;
    if (crsf_frame_is_extended_frame(frame_buffer)) {
        packet_addr = crsf_frame_get_frame_dst_addr(frame_buffer);
        packet_addr_valid |= packet_addr == dev_addr ||
                             packet_addr == CRSF_ADDRESS_BROADCAST;
    }

    return packet_addr_valid ? packet_addr : -packet_addr;
}

static void reset_rx(const struct device *dev)
{
    const struct rc_link_crsf_config *config = dev->config;
    struct rc_link_crsf_data *data = dev->data;

    data->rx_tol_us = 0;
    data->num_rx_bytes = 0;
    while (1) {
        unsigned char dummy;
        if (uart_fifo_read(config->uart, &dummy, 1) <= 0) {
            break;
        }
    }
    if (atomic_get(&data->enabled)) {
        uart_irq_rx_enable(config->uart);
    }
}

static inline int unpack_battery_sensor_frame(const union crsf_frame_buffer *frame_buffer,
                                              struct rc_link_crsf_data *data)
{
#ifdef CONFIG_RC_LINK_TELEMETRY
    const struct crsf_sensor_battery_payload *payload = NULL;
    uint8_t payload_size;

    crsf_get_payload(frame_buffer, (uint8_t const **)&payload, &payload_size);
    if (payload_size != CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE) {
        return -EBADMSG;
    }

     // mutex locked in caller
    data->batt_data = *payload;
#endif // CONFIG_RC_LINK_TELEMETRY
    return 0;
}

static inline void pack_battery_sensor_frame(union crsf_frame_buffer *frame_buffer,
                                             const struct rc_link_crsf_data *data)
{
#ifdef CONFIG_RC_LINK_TELEMETRY
    struct crsf_frame *frame = &frame_buffer->frame;

    frame->header.device_addr = CRSF_SYNC_BYTE;
    frame->header.type = CRSF_FRAMETYPE_BATTERY_SENSOR;
     // mutex locked in caller
    crsf_set_payload(frame_buffer, (uint8_t *)&data->batt_data, CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE);
    const uint8_t crc = calculate_packet_crc(frame_buffer);
    crsf_frame_set_crc(frame_buffer, crc);
#endif // CONFIG_RC_LINK_TELEMETRY
}

static inline int unpack_link_statistics_frame(const union crsf_frame_buffer *frame_buffer,
                                               struct rc_link_crsf_data *data)
{
    const struct crsf_link_statistics_payload *payload = NULL;
    uint8_t payload_size;

    crsf_get_payload(frame_buffer, (uint8_t const **)&payload, &payload_size);
    if (payload_size != CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE) {
        return -EBADMSG;
    }

     // mutex locked in caller
    data->link_stats = *payload;
    return 0;
}

static inline int unpack_channels_frame(const union crsf_frame_buffer *frame_buffer,
                                        struct rc_link_crsf_data *data)
{
    const struct crsf_rc_channels_payload *payload = NULL;
    uint8_t payload_size = 0;

    crsf_get_payload(frame_buffer, (uint8_t const **)&payload, &payload_size);
    if (payload_size != CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE) {
        return -EBADMSG;
    }

     // mutex locked in caller
    data->channels = *payload;
    return 0;
}

static inline void pack_channels_frame(union crsf_frame_buffer *frame_buffer,
                                       const struct rc_link_crsf_data *data)
{
    struct crsf_frame *frame = &frame_buffer->frame;

    frame->header.device_addr = CRSF_SYNC_BYTE;
    frame->header.type = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    // mutex locked in caller
    crsf_set_payload(frame_buffer, (uint8_t *)&data->channels, CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE);
    const uint8_t crc = calculate_packet_crc(frame_buffer);
    crsf_frame_set_crc(frame_buffer, crc);
}

static inline int handle_ping_frame(const union crsf_frame_buffer *frame_buffer,
                                    struct rc_link_crsf_data *data)
{
    const uint8_t payload_size = crsf_frame_get_payload_size(frame_buffer);
    if (payload_size == CRSF_FRAME_DEVICE_PING_PAYLOAD_SIZE) {
         // mutex locked in caller
        data->tx_address = crsf_frame_get_frame_src_addr(frame_buffer);
        data->tx_message_type = CRSF_FRAMETYPE_DEVICE_INFO;
        k_work_reschedule(&data->work_process_tx, K_NO_WAIT);
        // @todo schedule work response
        return 0;
    } else {
        return -EBADMSG;
    }
}

static inline int unpack_device_info_frame(const union crsf_frame_buffer *frame_buffer,
                                           struct rc_link_crsf_data *data)
{
    if (crsf_get_device_info_payload(frame_buffer, &data->rx_device_info)) {
#ifdef CONFIG_RC_LINK_CRSF_AUTO_DETECT_ELRS
         // mutex locked in caller
        data->is_ELRS = memcmp(data->rx_device_info.name_payload, "ELRS", 4) == 0;
#endif // CONFIG_RC_LINK_CRSF_AUTO_DETECT_ELRS
        return 0;
    } else {
        return -EBADMSG;
    }
}

static inline void pack_device_info_frame(union crsf_frame_buffer *frame_buffer,
                                          const struct rc_link_crsf_config *config,
                                          const struct rc_link_crsf_data *data)
{
     // mutex locked in caller
    frame_buffer->ext_frame.header.device_addr = CRSF_SYNC_BYTE;
    frame_buffer->ext_frame.header.type = CRSF_FRAMETYPE_DEVICE_INFO;
    frame_buffer->ext_frame.header.dest_addr = data->tx_address;
    frame_buffer->ext_frame.header.orig_addr = config->dev_addr;
    crsf_set_device_info_payload(frame_buffer, &config->device_info);
    const uint8_t crc = calculate_packet_crc(frame_buffer);
    crsf_frame_set_crc(frame_buffer, crc);
}

// @todo change to a "send" schedule (for ground or air unit)
static inline int update_telem_schedule(const struct device *dev)
{
#ifdef CONFIG_RC_LINK_TELEMETRY
    struct rc_link_crsf_data *data = dev->data;

    data->telem_sched++;
#ifdef CONFIG_RC_LINK_CRSF_AUTO_DETECT_ELRS
    if (data->is_ELRS && data->telem_sched == INDEX_OF_VARIO_TELEM_SCHED) {
        data->telem_sched++;
    }
#else
    const struct rc_link_crsf_config *config = dev->config;
    if (config->is_ELRS && data->telem_sched == INDEX_OF_VARIO_TELEM_SCHED) {
        data->telem_sched++;
    }
#endif // CONFIG_RC_LINK_CRSF_AUTO_DETECT_ELRS
    if (data->telem_sched == ARRAY_SIZE(telem_schedule)) {
        data->telem_sched = 0;
    }

    return telem_schedule[data->telem_sched];
#else
    return 0;
#endif // CONFIG_RC_LINK_TELEMETRY
}

static inline void transmit_bytes(const struct device *uart, uint8_t *bytes, int len)
{
    k_sched_lock();
    while (len--) {
        uart_poll_out(uart, *bytes++);
    }
    k_sched_unlock();
}

static int rc_link_crsf_get_config(const struct device *dev,
                                    struct rc_link_config *cfg)
{
    const struct rc_link_crsf_config *config = dev->config;
    memcpy(cfg, &config->base_config, sizeof(struct rc_link_config));
    return 0;
}

static int rc_link_crsf_set_channels_callback(const struct device *dev,
                    rc_link_on_channels_received_cb_t cb, void *user_data)
{
    struct rc_link_crsf_data *data = dev->data;

    (void)k_mutex_lock(&data->lock, K_FOREVER);
    data->channels_cb = cb;
    data->channels_user_data = user_data;
    (void)k_mutex_unlock(&data->lock);

    return 0;
}

static bool rc_link_crsf_get_enabled(const struct device *dev)
{
    struct rc_link_crsf_data *data = dev->data;
    bool enabled = atomic_get(&data->enabled);
    return enabled;
}

static void rc_link_crsf_set_enabled(const struct device *dev, bool enabled)
{
    const struct rc_link_crsf_config *config = dev->config;
    struct rc_link_crsf_data *data = dev->data;

    bool was_enabled = atomic_set(&data->enabled, enabled);

    if (enabled == was_enabled) {
        return;
    }

    if (enabled) {
        reset_rx(dev);
    } else {
        uart_irq_rx_disable(config->uart);
#ifdef CONFIG_UART_ASYNC_API
        (void)uart_rx_disable(config->uart);
        (void)uart_tx_abort(config->uart);
#endif
        (void)k_work_cancel(&data->work_process_rx);
        (void)k_work_cancel_delayable(&data->work_process_tx);
    }
}

static int rc_link_crsf_get_status(const struct device *dev, struct rc_link_status *status)
{
    struct rc_link_crsf_data *data = dev->data;

    (void)k_mutex_lock(&data->lock, K_FOREVER);
    int rssi_dbm = -1 * data->link_stats.active_antenna == 0 ?
        data->link_stats.uplink_RSSI_1 : data->link_stats.uplink_RSSI_2;
    status->uplink.rssi_pct = scale_range(rssi_dbm, CRSF_RSSI_MIN, CRSF_RSSI_MAX, 0, 100);
    status->uplink.link_quality_pct = data->link_stats.uplink_Link_quality;
    status->uplink.rssi_dbm = rssi_dbm;
    status->uplink.snr_dbm = data->link_stats.uplink_SNR;
    status->uplink.power_mW = data->link_stats.uplink_TX_Power;
#ifdef CONFIG_RC_LINK_DOWNLINK_STATUS
    status->downlink.rssi_pct = data->link_stats.downlink_RSSI;
    status->downlink.link_quality_pct = data->link_stats.downlink_Link_quality;
    status->downlink.rssi_dbm = -1 * data->link_stats.downlink_RSSI;
    status->downlink.snr_dbm = data->link_stats.downlink_SNR;
#endif // CONFIG_RC_LINK_DOWNLINK_STATUS
    (void)k_mutex_unlock(&data->lock);
    return 0;
}

static int rc_link_crsf_get_channels(const struct device *dev, uint16_t *channels, uint8_t num_channels)
{
    struct rc_link_crsf_data *data = dev->data;

    num_channels = MIN(CRSF_MAX_RC_CHANNELS, num_channels);
    (void)k_mutex_lock(&data->lock, K_FOREVER);
    switch (num_channels - 1) {
        case 15 : channels[15] = data->channels.ch15;
        case 14 : channels[14] = data->channels.ch14;
        case 13 : channels[13] = data->channels.ch13;
        case 12 : channels[12] = data->channels.ch12;
        case 11 : channels[11] = data->channels.ch11;
        case 10 : channels[10] = data->channels.ch10;
        case  9 : channels[9]  = data->channels.ch9;
        case  8 : channels[8]  = data->channels.ch8;
        case  7 : channels[7]  = data->channels.ch7;
        case  6 : channels[6]  = data->channels.ch6;
        case  5 : channels[5]  = data->channels.ch5;
        case  4 : channels[4]  = data->channels.ch4;
        case  3 : channels[3]  = data->channels.ch3;
        case  2 : channels[2]  = data->channels.ch2;
        case  1 : channels[1]  = data->channels.ch1;
        case  0 : channels[0]  = data->channels.ch0;
        default: break;
    }
    (void)k_mutex_unlock(&data->lock);

    for (int i = 0; i < num_channels; ++i) {
        channels[i] = CLAMP(channels[i], CRSF_CHANNEL_SCALED_MIN, CRSF_CHANNEL_SCALED_MAX);
        channels[i] = scale_range(channels[i],
            CRSF_CHANNEL_SCALED_MIN, CRSF_CHANNEL_SCALED_MAX,
            RC_LINK_CHANNEL_VAL_MIN, RC_LINK_CHANNEL_VAL_MAX);
    }

    return num_channels;
}

static int rc_link_crsf_set_channels(const struct device *dev, const uint16_t *channels, uint8_t num_channels)
{
    struct rc_link_crsf_data *data = dev->data;
    uint16_t temp_channels[CRSF_MAX_RC_CHANNELS];

    num_channels = MIN(CRSF_MAX_RC_CHANNELS, num_channels);
    for (int i = 0; i < num_channels; ++i) {
        temp_channels[i] = CLAMP(channels[i], RC_LINK_CHANNEL_VAL_MIN, RC_LINK_CHANNEL_VAL_MAX);
        temp_channels[i] = scale_range(temp_channels[i],
            RC_LINK_CHANNEL_VAL_MIN, RC_LINK_CHANNEL_VAL_MAX,
            CRSF_CHANNEL_SCALED_MIN, CRSF_CHANNEL_SCALED_MAX);
    }

    (void)k_mutex_lock(&data->lock, K_FOREVER);
    switch (num_channels - 1) {
        case 15 : data->channels.ch15 = temp_channels[15];
        case 14 : data->channels.ch14 = temp_channels[14];
        case 13 : data->channels.ch13 = temp_channels[13];
        case 12 : data->channels.ch12 = temp_channels[12];
        case 11 : data->channels.ch11 = temp_channels[11];
        case 10 : data->channels.ch10 = temp_channels[10];
        case  9 : data->channels.ch9  = temp_channels[9];
        case  8 : data->channels.ch8  = temp_channels[8];
        case  7 : data->channels.ch7  = temp_channels[7];
        case  6 : data->channels.ch6  = temp_channels[6];
        case  5 : data->channels.ch5  = temp_channels[5];
        case  4 : data->channels.ch4  = temp_channels[4];
        case  3 : data->channels.ch3  = temp_channels[3];
        case  2 : data->channels.ch2  = temp_channels[2];
        case  1 : data->channels.ch1  = temp_channels[1];
        case  0 : data->channels.ch0  = temp_channels[0];
        default: break;
    }
    (void)k_mutex_unlock(&data->lock);

    return num_channels;
}

#ifdef CONFIG_RC_LINK_TELEMETRY
static int rc_link_crsf_set_telem_callback(const struct device *dev,
                        rc_link_on_telem_received_cb_t cb, void *user_data)
{
    struct rc_link_crsf_data *data = dev->data;

    (void)k_mutex_lock(&data->lock, K_FOREVER);
    data->telem_cb = cb;
    data->telem_user_data = user_data;
    (void)k_mutex_unlock(&data->lock);

    return 0;
}

static int rc_link_crsf_get_telem(const struct device *dev, struct rc_link_telemetry *telem)
{
    struct rc_link_crsf_data *data = dev->data;

    (void)k_mutex_lock(&data->lock, K_FOREVER);
    telem->vbatt_mV = data->batt_data.voltage * 100;
    telem->current_mA = data->batt_data.current * 100;
    telem->batt_capacity_mAh = data->batt_data.capacity;
    telem->batt_pct = data->batt_data.remaining;
    (void)k_mutex_unlock(&data->lock);
    return 0;
}

static int rc_link_crsf_set_telem(const struct device *dev, const struct rc_link_telemetry *telem)
{
    struct rc_link_crsf_data *data = dev->data;

    (void)k_mutex_lock(&data->lock, K_FOREVER);
    data->batt_data.voltage = telem->vbatt_mV / 100;
    data->batt_data.current = telem->current_mA / 100;
    data->batt_data.capacity = telem->batt_capacity_mAh;
    data->batt_data.remaining = telem->batt_pct;
    (void)k_mutex_unlock(&data->lock);

    return 0;
}
#endif // CONFIG_RC_LINK_TELEMETRY

static struct rc_link_driver_api crsf_api = {
    .get_config = rc_link_crsf_get_config,
    .set_channels_callback = rc_link_crsf_set_channels_callback,
    .set_enabled = rc_link_crsf_set_enabled,
    .get_enabled = rc_link_crsf_get_enabled,
    .get_status = rc_link_crsf_get_status,
    .get_channels = rc_link_crsf_get_channels,
    .set_channels = rc_link_crsf_set_channels,
#ifdef CONFIG_RC_LINK_TELEMETRY
    .set_telem_callback = rc_link_crsf_set_telem_callback,
    .set_telem = rc_link_crsf_set_telem,
    .get_telem = rc_link_crsf_get_telem,
#endif // CONFIG_RC_LINK_TELEMETRY
};

static void process_tx(struct k_work *work)
{
    struct k_work_delayable *work_delayable = k_work_delayable_from_work(work);
    struct rc_link_crsf_data *data =
        CONTAINER_OF(work_delayable, struct rc_link_crsf_data, work_process_tx);
    const struct rc_link_crsf_config *config = data->dev->config;
    int data_type;
    int err = 0;

    (void)k_mutex_lock(&data->lock, K_FOREVER);

#ifdef CONFIG_RC_LINK_TELEMETRY
    k_work_reschedule(work_delayable, K_MSEC(config->tx_period));
#endif

    data_type = data->tx_message_type ? data->tx_message_type :
#ifdef CONFIG_RC_LINK_TELEMETRY
        update_telem_schedule(data->dev);
#else
        -1;
#endif // CONFIG_RC_LINK_TELEMETRY

    switch (data_type)
    {
#ifdef CONFIG_RC_LINK_TELEMETRY
    case CRSF_FRAMETYPE_GPS: // TODO
        err = -ENOTSUP;
        break;
    case CRSF_FRAMETYPE_VARIO: // TODO
        pack_battery_sensor_frame(data->tx_buf_ptr, data);
        break;
    case CRSF_FRAMETYPE_BATTERY_SENSOR: // TODO
        err = -ENOTSUP;
        break;
    case CRSF_FRAMETYPE_BARO_ALTITUDE: // TODO
        err = -ENOTSUP;
        break;
#endif // CONFIG_RC_LINK_TELEMETRY
    case CRSF_FRAMETYPE_RADIO_ID:
        err = -ENOTSUP;
        break;
    case CRSF_FRAMETYPE_RC_CHANNELS_PACKED: {
        pack_channels_frame(data->tx_buf_ptr, data);
        break;
    }
#ifdef CONFIG_RC_LINK_TELEMETRY
    case CRSF_FRAMETYPE_ATTITUDE:
        err = -ENOTSUP;
        break;
    case CRSF_FRAMETYPE_FLIGHT_MODE: // TODO
        err = -ENOTSUP;
        break;
#endif // CONFIG_RC_LINK_TELEMETRY
    case CRSF_FRAMETYPE_DEVICE_INFO: {
        pack_device_info_frame(data->tx_buf_ptr, config, data);
        break;
    }
#ifdef CONFIG_RC_LINK_TELEMETRY
    case CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY:
        err = -ENOTSUP;
        break;
    case CRSF_FRAMETYPE_PARAMETER_READ:
        err = -ENOTSUP;
        break;
    case CRSF_FRAMETYPE_PARAMETER_WRITE:
        err = -ENOTSUP;
        break;
    case CRSF_FRAMETYPE_COMMAND:
        err = -ENOTSUP;
        break;
    case CRSF_FRAMETYPE_KISS_REQ:
        err = -ENOTSUP;
        break;
    case CRSF_FRAMETYPE_KISS_RESP:
        err = -ENOTSUP;
        break;
    case CRSF_FRAMETYPE_MSP_REQ:
        err = -ENOTSUP;
        break;
    case CRSF_FRAMETYPE_MSP_RESP:
        err = -ENOTSUP;
        break;
    case CRSF_FRAMETYPE_MSP_WRITE:
        err = -ENOTSUP;
        break;
    case CRSF_FRAMETYPE_ARDUPILOT_RESP:
        err = -ENOTSUP;
        break;
#endif // CONFIG_RC_LINK_TELEMETRY
    default:
        err = -EINVAL;
        break;
    }
    
    if (!err) {
        LOG_DBG("Tx frame: %02X", data_type);
#if RC_LINK_CRSF_MANUAL_CACHE_COHERENCY_REQUIRED
        arch_dcache_flush_range(data->tx_buf_ptr, RC_LINK_CRSF_BUF_SIZE);
#endif
        transmit_bytes(config->uart, data->tx_buf_ptr->bytes,
            crsf_frame_get_packet_size(data->tx_buf_ptr));
    } else {
        LOG_ERR("Tx frame: %02X, error: %d", data_type, err);
    }

    data->tx_address = 0;
    data->tx_message_type = 0;
    (void)k_mutex_unlock(&data->lock);
}

static void process_rx(struct k_work* work)
{
    struct rc_link_crsf_data *data =
        CONTAINER_OF(work, struct rc_link_crsf_data, work_process_rx);
    const struct rc_link_crsf_config *config = data->dev->config;
    enum crsf_frame_type frame_type;
    int packet_addr;
    bool telem_frame = false;
    int err;

    if (data->num_rx_bytes == 0) {
        reset_rx(data->dev);
        return;
    }

#if RC_LINK_CRSF_MANUAL_CACHE_COHERENCY_REQUIRED
    arch_dcache_invd_range(data->rx_buf_ptr, RC_LINK_CRSF_BUF_SIZE);
#endif

    err = check_packet_integrity(data->rx_buf_ptr, data->num_rx_bytes);
    if (err) {
        // const uint8_t packed_len = MIN(
        //     CRSF_MAX_PACKET_LEN,
        //     crsf_frame_get_packet_size(data->rx_buf_ptr)
        // );
        // LOG_HEXDUMP_WRN(data->rx_buf_ptr->bytes, packed_len, "Corrupt packet received:");
        reset_rx(data->dev);
        return;
    }
    
    packet_addr = check_packet_address(data->rx_buf_ptr, config->dev_addr);
    if (packet_addr < 0) {
        LOG_INF("Ignoring packet for %02X", -packet_addr);
        reset_rx(data->dev);
        return;
    }

    frame_type = crsf_frame_get_frame_type(data->rx_buf_ptr);
    (void)k_mutex_lock(&data->lock, K_FOREVER);
    switch (frame_type) {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED: {
            err = unpack_channels_frame(data->rx_buf_ptr, data);
            break;
        }
        case CRSF_FRAMETYPE_DEVICE_PING: {
            err = handle_ping_frame(data->rx_buf_ptr, data);
            break;
        }
        case CRSF_FRAMETYPE_DEVICE_INFO: {
            err = unpack_device_info_frame(data->rx_buf_ptr, data);
            break;
        }
        case CRSF_FRAMETYPE_LINK_STATISTICS: {
            err = unpack_link_statistics_frame(data->rx_buf_ptr, data);
            break;
        }
#ifdef CONFIG_RC_LINK_TELEMETRY
        case CRSF_FRAMETYPE_GPS: {
            err = -ENOTSUP;
            telem_frame = true;
            break;
        }
        case CRSF_FRAMETYPE_VARIO: {
            err = -ENOTSUP;
            telem_frame = true;
            break;
        }
        case CRSF_FRAMETYPE_BATTERY_SENSOR: {
            err = unpack_battery_sensor_frame(data->rx_buf_ptr, data);
            telem_frame = true;
            break;
        }
        case CRSF_FRAMETYPE_BARO_ALTITUDE: {
            err = -ENOTSUP;
            telem_frame = true;
            break;
        }
        case CRSF_FRAMETYPE_ATTITUDE: {
            err = -ENOTSUP;
            telem_frame = true;
            break;
        }
        case CRSF_FRAMETYPE_FLIGHT_MODE: {
            err = -ENOTSUP;
            telem_frame = true;
            break;
        }
#endif // CONFIG_RC_LINK_TELEMETRY
        default: {
            err = -ENOTSUP;
            break;
        }
    }

    reset_rx(data->dev);

    if (!err) {
        LOG_DBG("Rx frame: %02X", frame_type);
        if (frame_type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED &&
            data->channels_cb)
        {
            uint16_t channels[CRSF_MAX_RC_CHANNELS];
            rc_link_crsf_get_channels(data->dev, &channels[0], CRSF_MAX_RC_CHANNELS);
            data->channels_cb(data->dev, &channels[0],
                CRSF_MAX_RC_CHANNELS, data->channels_user_data);
        } 
#if CONFIG_RC_LINK_TELEMETRY        
        else if (telem_frame && data->telem_cb)
        {
            struct rc_link_telemetry telem;
            err = rc_link_crsf_get_telem(data->dev, &telem);
            if (!err) {
                data->telem_cb(data->dev, &telem, data->channels_user_data);
            }
        }
#endif 
    } else {
        LOG_ERR("Rx frame: %02X, error: %d", frame_type, err);
    }

    (void)k_mutex_unlock(&data->lock);
}

static void crsf_irq_cb(const struct device *uart, void *user_data)
{
    struct rc_link_crsf_data *data = user_data;

    uart_irq_update(uart);
    if (!uart_irq_rx_ready(uart)) {
        return;
    }

    // check if the delta since that last received byte is greater than the tolerance
    // if so, start a new message rx
    uint32_t current_us = k_cyc_to_us_floor32(k_cycle_get_32());
    if (data->num_rx_bytes > CRSF_FRAME_HEADER_SIZE &&
        current_us - data->last_rx_us > data->rx_tol_us)
    {
        data->num_rx_bytes = 0;
        data->rx_tol_us = 0;
    }

    int recv = uart_fifo_read(uart, &data->rx_buf_ptr->bytes[data->num_rx_bytes],
        CRSF_MAX_PACKET_LEN - data->num_rx_bytes);
    if (recv <= 0) {
        // this should not happen
        return;
    }
    data->last_rx_us = current_us;
    data->num_rx_bytes += recv;

    if (data->num_rx_bytes <= CRSF_FRAME_NOT_COUNTED_BYTES) {
        // not enough bytes yet
        return;
    }

    const uint8_t packet_size = crsf_frame_get_packet_size(data->rx_buf_ptr);
    if (packet_size > CRSF_MAX_PACKET_LEN ||
        packet_size < CRSF_FRAME_HEADER_SIZE)
    {
        data->num_rx_bytes = 0;
        data->rx_tol_us = 0;
        return;
    }

    const uint8_t bytes_remaining = packet_size - data->num_rx_bytes;
    if (!data->rx_tol_us) {
        data->rx_tol_us = (bytes_remaining + 1) * data->rx_byte_tol_us;
    }

#if CONFIG_UART_ASYNC_API
    // first try using DMA rx
    if (bytes_remaining >= 4) {
        int err = uart_rx_enable(uart,
            &data->rx_buf_ptr->bytes[data->num_rx_bytes],
            packet_size - data->num_rx_bytes,
            data->rx_tol_us);
        if (!err) {
            uart_irq_rx_disable(uart);
            return;
        }
    }
    // otherwise continue using interrupts
#endif // CONFIG_UART_ASYNC_API

    if (data->num_rx_bytes >= packet_size) {
        (void)k_work_submit(&data->work_process_rx);
        uart_irq_rx_disable(uart);
    }
}

static void crsf_async_cb(const struct device *uart,
                                  struct uart_event *evt,
                                  void *user_data)
{
    struct rc_link_crsf_data *data = user_data;

    if (!evt) {
        return;
    }

    switch (evt->type) {
        case UART_RX_RDY:{
            struct uart_event_rx *evt_rx = (struct uart_event_rx *)evt;
            data->num_rx_bytes += evt_rx->len;
            k_work_submit(&data->work_process_rx);
            break;
        }
        case UART_RX_STOPPED: {
            rc_link_crsf_set_enabled(data->dev, false);
            struct uart_event_rx_stop *evt_stop = (struct uart_event_rx_stop *)evt;
            LOG_ERR("uart stopped: %d", evt_stop->reason);
            break;
        }
        default: break;
    }
}

static int rc_link_crsf_init(const struct device *dev)
{
    const struct rc_link_crsf_config *config = dev->config;
    struct rc_link_crsf_data *data = dev->data;
    int err;

    data->dev = dev;

    if (!device_is_ready(config->uart)) {
        LOG_ERR("uart not ready");
        return -ENODEV;
    }

    struct uart_config uart_dev_config = {
        .baudrate = data->is_ELRS ?
            ELRS_DEFAULT_BAUDRATE : CRSF_DEFAULT_BAUDRATE,
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE
    };
    data->rx_byte_tol_us = ((CRSF_BITS_PER_BYTE * USEC_PER_SEC) /
        uart_dev_config.baudrate) + 1;
    err = uart_configure(config->uart, &uart_dev_config);
    if (err) {
        LOG_ERR("uart configure error: %d", err);
        return err;
    }

#ifdef CONFIG_UART_ASYNC_API
    err = uart_callback_set(config->uart, crsf_async_cb, data);
    if (err) {
        LOG_ERR("uart set async callback error: %d", err);
        return err;
    }
#endif

    err = uart_irq_callback_user_data_set(config->uart, crsf_irq_cb, data);
    if (err) {
        LOG_ERR("uart set irq callback error: %d", err);
        return err;
    }

    err = k_mutex_init(&data->lock);
    if (err) {
        LOG_ERR("mutex init error: %d", err);
        return err;
    }

    k_work_init(&data->work_process_rx, process_rx);
    k_work_init_delayable(&data->work_process_tx, process_tx);

    return 0;
}

#ifdef CONFIG_RC_LINK_DOWNLINK_STATUS
#define INIT_BASE_CONFIG_DOWNLINK   \
    .supported_downlink_status = SUPPORTED_DOWNLINK,
#else
#define INIT_BASE_CONFIG_DOWNLINK
#endif // CONFIG_RC_LINK_DOWNLINK_STATUS

#ifdef CONFIG_RC_LINK_TELEMETRY
#define INIT_BASE_CONFIG_TELEM  \
    .supported_telemetry = SUPPORTED_TELEM,
#else
#define INIT_BASE_CONFIG_TELEM
#endif // CONFIG_RC_LINK_TELEMETRY

#define TBS_CRSF_INIT(inst) \
    static __aligned(CONFIG_DCACHE_LINE_SIZE) uint8_t                           \
        rc_link_crsf_rx_buf_##index[RC_LINK_CRSF_BUF_SIZE]                      \
        __nocache = { 0 };                                                      \
    \
    static __aligned(CONFIG_DCACHE_LINE_SIZE) uint8_t                           \
        rc_link_crsf_tx_buf_##index[RC_LINK_CRSF_BUF_SIZE]                      \
        __nocache = { 0 };                                                      \
    \
    static struct rc_link_crsf_config crsf_config_##inst = {                    \
        .base_config = {                                                        \
            .type = RC_LINK_TYPE_CRSF,                                          \
            .num_channels = CRSF_MAX_RC_CHANNELS,                               \
            .supported_uplink_status = SUPPORTED_UPLINK,                        \
            INIT_BASE_CONFIG_DOWNLINK                                           \
            INIT_BASE_CONFIG_TELEM                                              \
        },                                                                      \
        .uart = DEVICE_DT_GET(DT_INST_BUS(inst)),                               \
        .dev_addr = DT_PROP(DT_DRV_INST(inst), addr),                           \
        .device_info = {                                                        \
            .name_payload = DT_PROP(DT_DRV_INST(inst), dev_name),               \
            .raw_payload = {                                                    \
                .serialNo = DT_PROP(DT_DRV_INST(inst), serial_num),             \
                .hardwareVer = DT_PROP(DT_DRV_INST(inst), hw_ver),              \
                .softwareVer = DT_PROP(DT_DRV_INST(inst), sw_ver),              \
                .fieldCnt = DT_PROP(DT_DRV_INST(inst), field_cnt),              \
                .parameterVersion = DT_PROP(DT_DRV_INST(inst), param_version),  \
            },                                                                  \
        },                                                                      \
        .tx_period = CRSF_TELEM_TX_PERIOD_MS,                                   \
    };											                                \
    \
    static struct rc_link_crsf_data crsf_data_##inst = {                            \
        .enabled = false,                                                           \
        .is_ELRS = DT_PROP(DT_DRV_INST(inst), elrs),                                \
        .rx_buf_ptr = (union crsf_frame_buffer *)&rc_link_crsf_rx_buf_##index[0],   \
        .tx_buf_ptr = (union crsf_frame_buffer *)&rc_link_crsf_tx_buf_##index[0],   \
        .tx_address = CRSF_ADDRESS_BROADCAST,                                       \
    };											                                    \
    \
    DEVICE_DT_INST_DEFINE(inst,				    \
                  rc_link_crsf_init,		    \
                  NULL,						    \
                  &crsf_data_##inst,    \
                  &crsf_config_##inst,     \
                  POST_KERNEL,                  \
                  CONFIG_RC_LINK_INIT_PRIORITY, \
                  &crsf_api);

DT_INST_FOREACH_STATUS_OKAY(TBS_CRSF_INIT)
