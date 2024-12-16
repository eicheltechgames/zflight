/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Definition of the TBS Crossfire Protocol
 */

#ifndef ZEPHYR_DRIVERS_RADIO_RX_CRSF_CRSF_PROTOCOL_H_
#define ZEPHYR_DRIVERS_RADIO_RX_CRSF_CRSF_PROTOCOL_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CRSF_SYNC_BYTE          0xC8
#define CRSF_CRC_POLY           0xD5
#define CRSF_MAX_PACKET_LEN     64
#define CRSF_MAX_RC_CHANNELS    16
#define CRSF_CHANNEL_VAL_MIN    0
#define CRSF_CHANNEL_VAL_MAX    2047

#define CRSF_FRAME_ADDR_SIZE            1
#define CRSF_FRAME_LEN_SIZE             1
#define CRSF_FRAME_NOT_COUNTED_BYTES    (CRSF_FRAME_ADDR_SIZE + CRSF_FRAME_LEN_SIZE)
#define CRSF_FRAME_TYPE_SIZE            1
#define CRSF_FRAME_CRC_SIZE             1
#define CRSF_FRAME_HEADER_SIZE          (CRSF_FRAME_ADDR_SIZE + CRSF_FRAME_LEN_SIZE + CRSF_FRAME_TYPE_SIZE)
#define CRSF_FRAME_EXT_DST_SIZE         1
#define CRSF_FRAME_EXT_SRC_SIZE         1
#define CRSF_FRAME_EXT_HEADER_SIZE      (CRSF_FRAME_HEADER_SIZE + CRSF_FRAME_EXT_DST_SIZE + CRSF_FRAME_EXT_SRC_SIZE)
#define CRSF_PAYLOAD_SIZE_MAX           (CRSF_MAX_PACKET_LEN - CRSF_FRAME_HEADER_SIZE - CRSF_FRAME_CRC_SIZE)
#define CRSF_EXT_PAYLOAD_SIZE_MAX       (CRSF_MAX_PACKET_LEN - CRSF_FRAME_EXT_HEADER_SIZE - CRSF_FRAME_CRC_SIZE)

#define CRSF_HEADER_ADDR_INDEX      0
#define CRSF_HEADER_LEN_INDEX       1
#define CRSF_HEADER_TYPE_INDEX      2
#define CRSF_EXT_HEADER_DST_INDEX   3
#define CRSF_EXT_HEADER_SRC_INDEX   4

//////////////////////////////////////////////////////////////

#define CRSF_MSP_REQ_PAYLOAD_SIZE 8
#define CRSF_MSP_RESP_PAYLOAD_SIZE 58
#define CRSF_MSP_MAX_PAYLOAD_SIZE (CRSF_MSP_REQ_PAYLOAD_SIZE > CRSF_MSP_RESP_PAYLOAD_SIZE ? CRSF_MSP_REQ_PAYLOAD_SIZE : CRSF_MSP_RESP_PAYLOAD_SIZE)

enum crsf_frame_type
{
    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_VARIO = 0x07,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_BARO_ALTITUDE = 0x09,
    CRSF_FRAMETYPE_OPENTX_SYNC = 0x10,
    CRSF_FRAMETYPE_HEARTBEAT = 0x0B,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
    CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED = 0x17,
    CRSF_FRAMETYPE_RX_ID = 0x1C,
    CRSF_FRAMETYPE_TX_ID = 0x1D,
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_DEVICE_PING = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
    // ELRS frames
    CRSF_FRAMETYPE_ELRS_STATUS = 0x2E,
    // Extended Header Frames continued
    CRSF_FRAMETYPE_COMMAND = 0x32,
    CRSF_FRAMETYPE_RADIO_ID = 0x3A,
    // KISS frames
    CRSF_FRAMETYPE_KISS_REQ  = 0x78,
    CRSF_FRAMETYPE_KISS_RESP = 0x79,
    // MSP commands
    CRSF_FRAMETYPE_MSP_REQ = 0x7A,
    CRSF_FRAMETYPE_MSP_RESP = 0x7B,
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C,
    // Displayport commands
    CRSF_FRAMETYPE_DISPLAYPORT_CMD = 0x7D,
    // Ardupilot frames
    CRSF_FRAMETYPE_ARDUPILOT_RESP = 0x80,
};

enum crsf_command {
    CRSF_COMMAND_SUBCMD_RX = 0x10
};

enum crsf_subcommand {
    CRSF_COMMAND_SUBCMD_RX_BIND = 0x01,
    CRSF_COMMAND_MODEL_SELECT_ID = 0x05
};

enum {
    CRSF_FRAME_TX_MSP_FRAME_SIZE = 58,
    CRSF_FRAME_RX_MSP_FRAME_SIZE = 8,
    CRSF_FRAME_ORIGIN_DEST_SIZE = 2,
};

enum {
    CRSF_FRAME_GPS_PAYLOAD_SIZE = 15,
    CRSF_FRAME_VARIO_PAYLOAD_SIZE = 2,
    CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE = 8,
    CRSF_FRAME_BARO_ALTITUDE_PAYLOAD_SIZE = 2,
    CRSF_FRAME_BARO_VARIO_ALTITUDE_PAYLOAD_SIZE = 4, // for ELRS frame type CRSF_FRAMETYPE_BARO_ALTITUDE
    // CRSF_FRAME_OPENTX_SYNC_PAYLOAD_SIZE = ?,
    CRSF_FRAME_HEARTBEAT_PAYLOAD_SIZE = 2,
    CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE = 10,
    CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE = 22,
    CRSF_FRAME_SUBSET_RC_CHANNELS_PACKED_PAYLOAD_SIZE = 22,
    // CRSF_FRAME_RX_ID_PAYLOAD_SIZE = ?,
    // CRSF_FRAME_TX_ID_PAYLOAD_SIZE = ?,
    CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE = 6,
    CRSF_FRAME_FLIGHT_MODE_PAYLOAD_SIZE = 16,
    CRSF_FRAME_DEVICE_PING_PAYLOAD_SIZE = 0,
    CRSF_FRAME_DEVICE_INFO_PAYLOAD_SIZE = 48,
    // CRSF_FRAME_PARAMETER_SETTINGS_ENTRY_PAYLOAD_SIZE = ?,
    // CRSF_FRAME_PARAMETER_READ_PAYLOAD_SIZE = ?,
    // CRSF_FRAME_PARAMETER_WRITE_PAYLOAD_SIZE = ?,
    // CRSF_FRAME_ELRS_STATUS_PAYLOAD_SIZE = ?,
    // CRSF_FRAME_COMMAND_PAYLOAD_SIZE = ?,
    // CRSF_FRAME_RADIO_ID_PAYLOAD_SIZE = ?,
    // CRSF_FRAME_KISS_REQ_PAYLOAD_SIZE  = ?,
    // CRSF_FRAME_KISS_RESP_PAYLOAD_SIZE = ?,
    // CRSF_FRAME_MSP_REQ_PAYLOAD_SIZE = ?,
    // CRSF_FRAME_MSP_RESP_PAYLOAD_SIZE = ?,
    // CRSF_FRAME_MSP_WRITE_PAYLOAD_SIZE = ?,
    // CRSF_FRAME_DISPLAYPORT_CMD_PAYLOAD_SIZE = ?,
    // CRSF_FRAME_ARDUPILOT_RESP_PAYLOAD_SIZE = ?,
    CRSF_FRAME_GENERAL_RESP_PAYLOAD_SIZE = CRSF_PAYLOAD_SIZE_MAX,
};

enum crsf_addr
{
    CRSF_ADDRESS_BROADCAST = 0x00,
    CRSF_ADDRESS_USB = 0x10,
    CRSF_ADDRESS_BLUETOOTH = 0x12,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
    CRSF_ADDRESS_RESERVED1 = 0x8A,
    CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
    CRSF_ADDRESS_GPS = 0xC2,
    CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
    CRSF_ADDRESS_RESERVED2 = 0xCA,
    CRSF_ADDRESS_RACE_TAG = 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE,
    CRSF_ADDRESS_ELRS_LUA = 0xEF,
};

enum crsf_value_type
{
    CRSF_UINT8 = 0,
    CRSF_INT8 = 1,
    CRSF_UINT16 = 2,
    CRSF_INT16 = 3,
    CRSF_UINT32 = 4,
    CRSF_INT32 = 5,
    CRSF_UINT64 = 6,
    CRSF_INT64 = 7,
    CRSF_FLOAT = 8,
    CRSF_TEXT_SELECTION = 9,
    CRSF_STRING = 10,
    CRSF_FOLDER = 11,
    CRSF_INFO = 12,
    CRSF_COMMAND = 13,
    CRSF_VTX = 15,
    CRSF_OUT_OF_RANGE = 127,
};

// These flags are or'ed with the field type above to hide the field from the normal LUA view
// #define CRSF_FIELD_HIDDEN       0x80     // marked as hidden in all LUA responses
// #define CRSF_FIELD_ELRS_HIDDEN  0x40     // marked as hidden when talking to ELRS specific LUA
// #define CRSF_FIELD_TYPE_MASK    ~(CRSF_FIELD_HIDDEN|CRSF_FIELD_ELRS_HIDDEN)

/**
 * Crossfire packet standard header
 */
struct crsf_header
{
    uint8_t device_addr; // crsf_addr
    uint8_t frame_size;  // total packet size - 2
    uint8_t type;        // crsf_frame_type
} __attribute__((__packed__));

/**
 * Crossfire packet standard frame
 */
struct crsf_frame
{
    struct crsf_header header;
    // crc included in payload since payload will typically be less than the max size
    uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + CRSF_FRAME_CRC_SIZE];
} __attribute__((__packed__));

/**
 * Crossfire packet extended header (crsf_frame_type in range 0x28 to 0x96)
 */
struct crsf_ext_header
{
    uint8_t device_addr; // crsf_addr
    uint8_t frame_size;  // total packet size - 2
    uint8_t type;        // crsf_frame_type
    uint8_t dest_addr;   // crsf_addr
    uint8_t orig_addr;   // crsf_addr
} __attribute__((__packed__));

/**
 * Crossfire packet standard frame
 */
struct crsf_ext_frame
{
    struct crsf_ext_header header;
    // crc included in payload since payload will typically be less than the max size
    uint8_t payload[CRSF_EXT_PAYLOAD_SIZE_MAX + CRSF_FRAME_CRC_SIZE];
} __attribute__((__packed__));

/**
 * Crossfire packed channel structure, each channel is 11 bits
 */
struct crsf_rc_channels_payload
{
    unsigned ch0 : 11;
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
} __attribute__((__packed__));

#define CRSF_DEVICE_INFO_NAME_LEN_MAX   15

struct crsf_device_info_raw_payload
{
    // char device_name[CRSF_DEVICE_INFO_NAME_LEN_MAX + 1]; // omitted because length is unknown
                                                            // see crsf_get_device_info_payload()
    uint32_t serialNo;      // device serial number
    uint32_t hardwareVer;
    uint32_t softwareVer;
    uint8_t  fieldCnt;
    uint8_t  parameterVersion;
} __attribute__((__packed__));

struct crsf_device_info_frame {
    char name_payload[CRSF_DEVICE_INFO_NAME_LEN_MAX + 1]; // +1 for null char
    struct crsf_device_info_raw_payload raw_payload;
};

struct crsf_msp_vtx_config_payload
{
    uint8_t  vtxType;
    uint8_t  band;
    uint8_t  channel;
    uint8_t  power;
    uint8_t  pitmode;
    uint16_t freq;
    uint8_t  deviceIsReady;
    uint8_t  lowPowerDisarm;
    uint16_t pitModeFreq;
    uint8_t  vtxTableAvailable;
    uint8_t  bands;
    uint8_t  channels;
    uint8_t  powerLevels;
} __attribute__((__packed__));

struct crsf_msp_vtx_power_level_payload
{
    uint8_t  powerLevel;
    uint16_t powerValue;
    uint8_t  powerLabelLength;
    uint8_t  label[3];
} __attribute__((__packed__));

struct crsf_msp_vtx_band_payload
{
    uint8_t  band;
    uint8_t  bandNameLength;
    uint8_t  bandName[8];
    uint8_t  bandLetter;
    uint8_t  isFactoryBand;
    uint8_t  channels;
    uint16_t channel[8];
} __attribute__((__packed__));

// #define MSP_REQUEST_PAYLOAD_LENGTH(len) 7 + len // status + flags + 2 function + 2 length + crc + payload
// #define MSP_REQUEST_LENGTH(len) (sizeof(struct crsf_ext_header) + MSP_REQUEST_PAYLOAD_LENGTH(len) + CRSF_FRAME_CRC_SIZE)
// #define MSP_REQUEST_FRAME_SIZE(len) (MSP_REQUEST_PAYLOAD_LENGTH(len) + CRSF_FRAME_LENGTH_EXT_TYPE_CRC)

// #define MSP_SET_VTX_CONFIG_PAYLOAD_LENGTH 15
// #define MSP_SET_VTXTABLE_BAND_PAYLOAD_LENGTH 29
// #define MSP_SET_VTXTABLE_POWERLEVEL_PAYLOAD_LENGTH 7

//CRSF_FRAMETYPE_BATTERY_SENSOR
struct crsf_sensor_battery_payload
{
    uint16_t voltage;        // mv * 100 BigEndian
    uint16_t current;        // ma * 100
    uint32_t capacity  : 24; // mah
    uint32_t remaining :  8; // %
} __attribute__((__packed__));

// CRSF_FRAMETYPE_BARO_ALTITUDE

struct crsf_sensor_baro_vario_payload
{
    uint16_t altitude; // Altitude in decimeters + 10000dm, or Altitude in meters if high bit is set, BigEndian
    int16_t verticalspd;  // Vertical speed in cm/s, BigEndian
} __attribute__((__packed__));

struct crsf_sensor_baro_payload
{
    uint16_t altitude; // Altitude in decimeters + 10000dm, or Altitude in meters if high bit is set, BigEndian
} __attribute__((__packed__));

struct crsf_sensor_vario_payload
{
    int16_t verticalspd;  // Vertical speed in cm/s, BigEndian
} __attribute__((__packed__));

/*
 * 0x14 Link statistics
 * Payload:
 *
 * uint8_t Uplink RSSI Ant. 1 ( dBm * -1 )
 * uint8_t Uplink RSSI Ant. 2 ( dBm * -1 )
 * uint8_t Uplink Package success rate / Link quality ( % )
 * int8_t Uplink SNR ( db )
 * uint8_t Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
 * uint8_t RF Mode ( enum 4fps = 0 , 50fps, 150hz)
 * uint8_t Uplink TX Power ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW )
 * uint8_t Downlink RSSI ( dBm * -1 )
 * uint8_t Downlink package success rate / Link quality ( % )
 * int8_t Downlink SNR ( db )
 * Uplink is the connection from the ground to the UAV and downlink the opposite direction.
 */
struct crsf_link_statistics_payload
{
    uint8_t uplink_RSSI_1;
    uint8_t uplink_RSSI_2;
    uint8_t uplink_Link_quality;
    int8_t uplink_SNR;
    uint8_t active_antenna;
    uint8_t rf_Mode;
    uint8_t uplink_TX_Power;
    uint8_t downlink_RSSI;
    uint8_t downlink_Link_quality;
    int8_t downlink_SNR;
} __attribute__((__packed__));

// TODO: Compile time check the payload sizes with the hard coded values

union crsf_frame_buffer
{
    uint8_t bytes[CRSF_MAX_PACKET_LEN]; // max 64 bytes for CRSF packet serial buffer
    struct crsf_frame frame;
    struct crsf_ext_frame ext_frame;
};

static inline enum crsf_addr crsf_frame_get_frame_device_addr(const union crsf_frame_buffer *frame)
{
    return (enum crsf_addr)frame->bytes[CRSF_HEADER_ADDR_INDEX];
}

static inline enum crsf_frame_type crsf_frame_get_frame_type(const union crsf_frame_buffer *frame)
{
    return (enum crsf_frame_type)frame->bytes[CRSF_HEADER_TYPE_INDEX];
}

static inline bool crsf_frame_is_extended_frame(const union crsf_frame_buffer *frame)
{
    return frame->bytes[CRSF_HEADER_TYPE_INDEX] >= CRSF_FRAMETYPE_DEVICE_PING;
}

static inline uint8_t crsf_frame_get_frame_len(const union crsf_frame_buffer *frame)
{
    return frame->bytes[CRSF_HEADER_LEN_INDEX];
}

static inline void crsf_frame_set_frame_len(union crsf_frame_buffer *frame, uint8_t payload_len)
{
    frame->bytes[CRSF_HEADER_LEN_INDEX] = payload_len + (CRSF_FRAME_TYPE_SIZE + CRSF_FRAME_CRC_SIZE);
    if (crsf_frame_is_extended_frame(frame)) {
        frame->bytes[CRSF_HEADER_LEN_INDEX] += (CRSF_FRAME_EXT_DST_SIZE + CRSF_FRAME_EXT_SRC_SIZE);
    }
}

static inline enum crsf_addr crsf_frame_get_frame_dst_addr(const union crsf_frame_buffer *frame)
{
    return (enum crsf_addr)frame->bytes[CRSF_EXT_HEADER_DST_INDEX];
}

static inline enum crsf_addr crsf_frame_get_frame_src_addr(const union crsf_frame_buffer *frame)
{
    return (enum crsf_addr)frame->bytes[CRSF_EXT_HEADER_SRC_INDEX];
}

static inline uint8_t crsf_frame_get_packet_size(const union crsf_frame_buffer *frame)
{
    return crsf_frame_get_frame_len(frame) + CRSF_FRAME_NOT_COUNTED_BYTES;
}

static inline uint8_t crsf_frame_get_payload_size(const union crsf_frame_buffer *frame)
{
    uint8_t size = crsf_frame_get_frame_len(frame) - CRSF_FRAME_TYPE_SIZE - CRSF_FRAME_CRC_SIZE;
    if (crsf_frame_is_extended_frame(frame)) {
        size -= CRSF_FRAME_EXT_DST_SIZE + CRSF_FRAME_EXT_SRC_SIZE;
    }
    return size;
}

static inline uint8_t crsf_frame_get_crc(const union crsf_frame_buffer *frame)
{
    return frame->bytes[crsf_frame_get_packet_size(frame) - 1];
}

static inline void crsf_frame_set_crc(union crsf_frame_buffer *frame, uint8_t crc)
{
    frame->bytes[crsf_frame_get_packet_size(frame) - 1] = crc;
}

static inline void crsf_get_payload(const union crsf_frame_buffer *frame, uint8_t const **payload_ptr, uint8_t *size)
{
    *payload_ptr = crsf_frame_is_extended_frame(frame) ? &frame->ext_frame.payload[0] : &frame->frame.payload[0];
    *size = crsf_frame_get_payload_size(frame);
}

static inline void crsf_set_payload(union crsf_frame_buffer *frame, const uint8_t *payload, uint8_t payload_len)
{
    crsf_frame_set_frame_len(frame, payload_len);
    memcpy(crsf_frame_is_extended_frame(frame) ? &frame->ext_frame.payload[0] : &frame->frame.payload[0], payload, payload_len);
}

static inline bool crsf_get_device_info_payload(const union crsf_frame_buffer *frame,
                                              struct crsf_device_info_frame *device_info)
{
    char const *device_name_end = memchr(frame->ext_frame.payload, '\0', CRSF_DEVICE_INFO_NAME_LEN_MAX + 1);
    if (device_name_end == NULL) {
        return false;
    }
    strcpy(&device_info->name_payload[0], &frame->ext_frame.payload[0]);
    memcpy(&device_info->raw_payload, device_name_end + 1, sizeof(struct crsf_device_info_raw_payload));
    return true;
}

static void crsf_set_device_info_payload(union crsf_frame_buffer *frame,
                                                   const struct crsf_device_info_frame *device_info)
{
    const uint8_t name_len = strlen(device_info->name_payload) + 1; // +1 for null char
    const uint8_t payload_len = name_len + sizeof(struct crsf_device_info_raw_payload);
    crsf_frame_set_frame_len(frame, payload_len);
    memcpy(&frame->ext_frame.payload, &device_info->name_payload[0], name_len);
    memcpy(&frame->ext_frame.payload + name_len, &device_info->raw_payload, sizeof(struct crsf_device_info_raw_payload));
}

#ifdef __cplusplus
}
#endif

#endif // ZEPHYR_DRIVERS_RADIO_RX_CRSF_CRSF_PROTOCOL_H_
