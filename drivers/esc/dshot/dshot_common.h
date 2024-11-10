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

static inline uint16_t dshot_common_quantize_throttle(uint16_t raw_throttle)
{
    return (((uint32_t)raw_throttle * DSHOT_PACKET_MAX_DATA)
            / (uint32_t)UINT16_MAX);
}

uint16_t dshot_common_make_packet(uint16_t payload,
                                  bool telem_req, bool bidir);

void dshot_common_load_dshot_buffer(uint32_t *buf,
                                    unsigned stride, uint16_t packet);

int dshot_common_unpack_telem_buffer(uint32_t *buf, uint32_t *telem_raw_out);

int dshot_common_decode_telem(uint32_t telem_raw,
                              enum dshot_telem_type *out_type, uint16_t *out_value);

#ifdef __cplusplus
}
#endif

#endif /* ZFLIGHT_DRIVERS_ESC_DSHOT_DSHOT_COMMON_H_ */