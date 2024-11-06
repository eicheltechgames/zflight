/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dshot_common.h"
#include <zephyr/sys/crc.h>

const struct dshot_command_settings dshot_cmd_settings[DSHOT_CMD_MAX + 1] = {
    [DSHOT_CMD_MOTOR_STOP] = { 1, 1, 0 },
    [DSHOT_CMD_BEEP1] = { 1, 1, 260 },
    [DSHOT_CMD_BEEP2] = { 1, 1, 260 },
    [DSHOT_CMD_BEEP3] = { 1, 1, 260 },
    [DSHOT_CMD_BEEP4] = { 1, 1, 260 },
    [DSHOT_CMD_BEEP5] = { 1, 1, 260 },
    [DSHOT_CMD_ESC_INFO] = { 1, 1, 12 },
    [DSHOT_CMD_SPIN_DIRECTION_1] = { 1, 6, 0 },
    [DSHOT_CMD_SPIN_DIRECTION_2] = { 1, 6, 0 },
    [DSHOT_CMD_3D_MODE_OFF] = { 1, 6, 0 },
    [DSHOT_CMD_3D_MODE_ON] = { 1, 6, 0 },
    [DSHOT_CMD_SETTINGS_REQUEST] = { 1, 1, 0 },
    [DSHOT_CMD_SAVE_SETTINGS] = { 1, 6, 35 },
#ifdef CONFIG_DSHOT_EDT
    [DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE] = { 1, 6, 0 },
    [DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE] = { 1, 6, 0 },
#endif
    [DSHOT_CMD_SPIN_DIRECTION_NORMAL] = { 1, 6, 0 },
    [DSHOT_CMD_SPIN_DIRECTION_REVERSED] = { 1, 6, 0 },
    [DSHOT_CMD_LED0_ON] = { 1, 1, 0 },
    [DSHOT_CMD_LED1_ON] = { 1, 1, 0 },
    [DSHOT_CMD_LED2_ON] = { 1, 1, 0 },
    [DSHOT_CMD_LED3_ON] = { 1, 1, 0 },
    [DSHOT_CMD_LED0_OFF] = { 1, 1, 0 },
    [DSHOT_CMD_LED1_OFF] = { 1, 1, 0 },
    [DSHOT_CMD_LED2_OFF] = { 1, 1, 0 },
    [DSHOT_CMD_LED3_OFF] = { 1, 1, 0 },
    [DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF] = { 1, 1, 0 },
    [DSHOT_CMD_SILENT_MODE_ON_OFF] = { 1, 1, 0 },
#ifdef CONFIG_DSHOT_BIDIR
    [DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE] = { 1, 6, 0 },
    [DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE] = { 1, 6, 0 },
    [DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY] = { 1, 6, 0 },
    [DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY] = { 1, 6, 0 },
#ifdef CONFIG_DSHOT_EDT
    [DSHOT_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY] = { 1, 1, 0 },
    [DSHOT_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY] = { 1, 1, 0 },
    [DSHOT_CMD_SIGNAL_LINE_CURRENT_TELEMETRY] = { 1, 1, 0 },
    [DSHOT_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY] = { 1, 1, 0 },
    [DSHOT_CMD_SIGNAL_LINE_ERPM_TELEMETRY] = { 1, 1, 0 },
    [DSHOT_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY] = { 1, 1, 0 },
#endif /* CONFIG_DSHOT_EDT */
#endif /* CONFIG_DSHOT_BIDIR */
    { 0, 0, 0 }
};

uint16_t dshot_common_make_packet(uint16_t payload, bool telem_req, bool bidir)
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

void dshot_common_load_dshot_buffer(uint32_t *buf, unsigned stride, uint16_t packet)
{
    int i;
    for (i = 0; i < DSHOT_PACKET_BITS; i++) {
        buf[i * stride] = (packet & 0x8000) ? DSHOT_TIM_CNTS_1_BIT : DSHOT_TIM_CNTS_0_BIT;
        packet <<= 1;
    }
    buf[i++ * stride] = 0;
    buf[i++ * stride] = 0;
}

int dshot_common_unpack_telem_buffer(uint32_t *buf, uint32_t *telem_raw_out)
{
    return -ENOSYS;
}

int dshot_common_decode_telem(uint32_t telem_raw, enum dshot_telem_type *out_type,
                                uint16_t *out_value)
{
    return -ENOSYS;
}