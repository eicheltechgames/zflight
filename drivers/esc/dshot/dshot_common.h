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

#define DSHOT_PACKET_DATA_BITS          11
#define DSHOT_PACKET_TELEM_REQ_BITS     1
#define DSHOT_PACKET_CRC_BITS           4
#define DSHOT_PACKET_BITS               16

#define DSHOT_PACKET_MAX_DATA           0x7FF
#define DSHOT_PACKET_MAX_CRC            0xF

#define DSHOT_PACKET_DATA_OFFSET        5
#define DSHOT_PACKET_TELEM_REQ_OFFSET   4

#define DSHOT_PACKET_DATA_MASK \
    (DSHOT_PACKET_MAX_DATA << DSHOT_PACKET_DATA_OFFSET)
#define DSHOT_PACKET_TELEM_REQ_MASK \
    (DSHOT_PACKET_TELEM_REQ_BITS << DSHOT_PACKET_TELEM_REQ_OFFSET)
#define DSHOT_PACKET_CRC_MASK   DSHOT_PACKET_MAX_CRC

static inline uint16_t dshot_common_quantize_throttle(uint16_t raw_throttle)
{
    return (((uint32_t)raw_throttle * DSHOT_PACKET_MAX_DATA)
            / (uint32_t)UINT16_MAX);
}

static inline uint16_t dshot_common_make_packet(uint16_t payload,
                                                bool telem_req,
                                                bool bidir)
{
#ifndef CONFIG_ESC_DSHOT_BIDIR
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
#ifdef CONFIG_ESC_DSHOT_BIDIR
    if (bidir) {
        csum = ~csum;
    }
#endif
    csum &= DSHOT_PACKET_CRC_MASK;
    packet = (packet << DSHOT_PACKET_CRC_BITS) | csum;

    return packet;
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