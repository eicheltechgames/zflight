/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Common API for ESC DShot drivers
 */

#ifndef ZFLIGHT_DRIVERS_ESC_DSHOT_DSHOT_COMMON_H_
#define ZFLIGHT_DRIVERS_ESC_DSHOT_DSHOT_COMMON_H_

#include <zflight/drivers/esc/dshot.h>
#include <zephyr/sys/crc.h>

#ifdef __cplusplus
extern "C" {
#endif

/** DShot packet structure */
#define DSHOT_PACKET_DATA_BITS          11
#define DSHOT_PACKET_TELEM_REQ_BITS     1
#define DSHOT_PACKET_CRC_BITS           4
#define DSHOT_PACKET_BITS               16
#define DSHOT_BIT_RATE(dshot_type)      KHZ(dshot_type)

#define DSHOT_PACKET_MAX_DATA           0x7FF
#define DSHOT_PACKET_MAX_CRC            0xF

#define DSHOT_PACKET_DATA_OFFSET        5
#define DSHOT_PACKET_TELEM_REQ_OFFSET   4

#define DSHOT_PACKET_DATA_MASK \
    (DSHOT_PACKET_MAX_DATA << DSHOT_PACKET_DATA_OFFSET)
#define DSHOT_PACKET_TELEM_REQ_MASK \
    (DSHOT_PACKET_TELEM_REQ_BITS << DSHOT_PACKET_TELEM_REQ_OFFSET)
#define DSHOT_PACKET_CRC_MASK   DSHOT_PACKET_MAX_CRC

/** Number of timer counts per bit */
#define DSHOT_TIM_CNTS_PER_BIT  20
#define DSHOT_TIM_CNTS_0_BIT    7
#define DSHOT_TIM_CNTS_1_BIT    14

#define TELEM_PACKET_BITS           21
#define TELEM_BIT_RATE(dshot_type)  (5 * DSHOT_BIT_RATE(dshot_type) / 4)

#ifdef CONFIG_DSHOT_BIDIR
#define DSHOT_TIM_BUF_LEN       TELEM_PACKET_BITS
#else
#define DSHOT_TIM_BUF_LEN       (DSHOT_PACKET_BITS + 2)
#endif

#define DSHOT_CMD_MAX   47

struct dshot_command_settings {
    uint16_t repeat;
    uint16_t delay_ms;
};

static const struct dshot_command_settings command_settings[DSHOT_CMD_MAX] = {
    [DSHOT_CMD_MOTOR_STOP] = { 1, 0 },
    [DSHOT_CMD_BEEP1] = { 1, 260 },
    [DSHOT_CMD_BEEP2] = { 1, 260 },
    [DSHOT_CMD_BEEP3] = { 1, 260 },
    [DSHOT_CMD_BEEP4] = { 1, 260 },
    [DSHOT_CMD_BEEP5] = { 1, 260 },
    [DSHOT_CMD_ESC_INFO] = { 1, 12 },
    [DSHOT_CMD_SPIN_DIRECTION_1] = { 6, 0 },
    [DSHOT_CMD_SPIN_DIRECTION_2] = { 6, 0 },
    [DSHOT_CMD_3D_MODE_OFF] = { 6, 0 },
    [DSHOT_CMD_3D_MODE_ON] = { 6, 0 },
    [DSHOT_CMD_SETTINGS_REQUEST] = { 1, 0 },
    [DSHOT_CMD_SAVE_SETTINGS] = { 6, 35 },
#ifdef CONFIG_DSHOT_EDT
    [DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE] = { 6, 0 },
    [DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE] = { 6, 0 },
#endif
    [DSHOT_CMD_SPIN_DIRECTION_NORMAL] = { 6, 0 },
    [DSHOT_CMD_SPIN_DIRECTION_REVERSED] = { 6, 0 },
    [DSHOT_CMD_LED0_ON] = { 1, 0 },
    [DSHOT_CMD_LED1_ON] = { 1, 0 },
    [DSHOT_CMD_LED2_ON] = { 1, 0 },
    [DSHOT_CMD_LED3_ON] = { 1, 0 },
    [DSHOT_CMD_LED0_OFF] = { 1, 0 },
    [DSHOT_CMD_LED1_OFF] = { 1, 0 },
    [DSHOT_CMD_LED2_OFF] = { 1, 0 },
    [DSHOT_CMD_LED3_OFF] = { 1, 0 },
    [DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF] = { 1, 0 },
    [DSHOT_CMD_SILENT_MODE_ON_OFF] = { 1, 0 },
#ifdef CONFIG_DSHOT_BIDIR
    [DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE] = { 6, 0 },
    [DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE] = { 6, 0 },
    [DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY] = { 6, 0 },
    [DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY] = { 6, 0 },
#ifdef CONFIG_DSHOT_EDT
    [DSHOT_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY] = { 1, 0 },
    [DSHOT_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY] = { 1, 0 },
    [DSHOT_CMD_SIGNAL_LINE_CURRENT_TELEMETRY] = { 1, 0 },
    [DSHOT_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY] = { 1, 0 },
    [DSHOT_CMD_SIGNAL_LINE_ERPM_TELEMETRY] = { 1, 0 },
    [DSHOT_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY] = { 1, 0 },
#endif /* CONFIG_DSHOT_EDT */
#endif /* CONFIG_DSHOT_BIDIR */
 };



static inline uint16_t dshot_common_quantize_throttle(uint16_t raw_throttle)
{
    return (((uint32_t)raw_throttle * DSHOT_PACKET_MAX_DATA)
            / (uint32_t)UINT16_MAX);
}

static inline uint16_t dshot_common_make_packet(uint16_t payload,
                                                bool telem_req,
                                                bool bidir)
{
#ifndef CONFIG_DSHOT_BIDIR
    ARG_UNUSED(bidir);
#endif

    uint16_t packet = (payload << DSHOT_PACKET_TELEM_REQ_BITS) |
                        (telem_req ? 1 : 0);

    // compute checksum
    unsigned csum = 0;
    unsigned csum_data = packet;
    for (int i = 0; i < ((DSHOT_PACKET_BITS / 4) - 1); ++i) {
        csum ^=  csum_data;   // xor data by nibbles
        csum_data >>= 4;
    }
    // append checksum
#ifdef CONFIG_DSHOT_BIDIR
    if (bidir) {
        csum = ~csum;
    }
#endif
    csum &= DSHOT_PACKET_CRC_MASK;
    packet = (packet << DSHOT_PACKET_CRC_BITS) | csum;

    return packet;
}

static inline void dshot_common_load_dshot_buffer(uint16_t *buf, unsigned stride, uint16_t packet)
{
    int i;
    for (i = 0; i < DSHOT_PACKET_BITS; i++) {
        buf[i * stride] = (packet & 0x8000) ? DSHOT_TIM_CNTS_1_BIT : DSHOT_TIM_CNTS_0_BIT;
        packet <<= 1;
    }
    buf[i++ * stride] = 0;
    buf[i++ * stride] = 0;
}

static inline int dshot_common_unpack_telem_buffer(uint16_t *buf, uint32_t *telem_raw_out)
{
    return -ENOSYS;
}

static inline int dshot_common_decode_telem(uint32_t telem_raw,
                                            enum dshot_telem_type *out_type,
                                            uint16_t *out_value)
{
    return -ENOSYS;
}

#ifdef __cplusplus
}
#endif

#endif /* ZFLIGHT_DRIVERS_ESC_DSHOT_DSHOT_COMMON_H_ */