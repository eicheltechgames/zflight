/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZFLIGHT_INCLUDE_DRIVERS_ESC_DSHOT_H_
#define ZFLIGHT_INCLUDE_DRIVERS_ESC_DSHOT_H_

#include <zflight/drivers/esc.h>

#ifdef __cplusplus
extern "C" {
#endif

enum dshot_type {
    DSHOT_150 = 150,
    DSHOT_300 = 300,
    DSHOT_600 = 600
};

enum dshot_mode {
    DSHOT_MODE_NORMAL,
    DSHOT_MODE_BIDIRECTIONAL
};

enum dshot_command {
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEEP1,
    DSHOT_CMD_BEEP2,
    DSHOT_CMD_BEEP3,
    DSHOT_CMD_BEEP4,
    DSHOT_CMD_BEEP5,
    DSHOT_CMD_ESC_INFO,
    DSHOT_CMD_SPIN_DIRECTION_1,
    DSHOT_CMD_SPIN_DIRECTION_2,
    DSHOT_CMD_3D_MODE_OFF,
    DSHOT_CMD_3D_MODE_ON,
    DSHOT_CMD_SETTINGS_REQUEST,
    DSHOT_CMD_SAVE_SETTINGS,
    DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE,
    DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED,
    DSHOT_CMD_LED0_ON, // BLHeli32 only
    DSHOT_CMD_LED1_ON, // BLHeli32 only
    DSHOT_CMD_LED2_ON, // BLHeli32 only
    DSHOT_CMD_LED3_ON, // BLHeli32 only
    DSHOT_CMD_LED0_OFF, // BLHeli32 only
    DSHOT_CMD_LED1_OFF, // BLHeli32 only
    DSHOT_CMD_LED2_OFF, // BLHeli32 only
    DSHOT_CMD_LED3_OFF, // BLHeli32 only
    DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF, // KISS audio Stream mode on/Off
    DSHOT_CMD_SILENT_MODE_ON_OFF, // KISS silent Mode on/Off
#ifdef CONFIG_DSHOT_BIDIR
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE,
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE,
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY,
#ifdef CONFIG_DSHOT_EDT
    DSHOT_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY = 42,
    DSHOT_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_CURRENT_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_ERPM_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY,
#endif /* CONFIG_DSHOT_EDT */
#endif /* CONFIG_DSHOT_BIDIR */
    DSHOT_CMD_MAX = 47
};

enum dshot_telem_type {
    DSHOT_TELEM_FULL = 1,
#ifdef CONFIG_DSHOT_BIDIR
    DSHOT_TELEM_ERPM = 0x0,
#ifdef CONFIG_DSHOT_EDT
    DSHOT_TELEM_TEMP = 0x2,
    DSHOT_TELEM_CURR = 0x4,
    DSHOT_TELEM_VOLT = 0x6,
    DSHOT_TELEM_D1 = 0x8,
    DSHOT_TELEM_D2 = 0xA,
    DSHOT_TELEM_D3 = 0xC,
    DSHOT_TELEM_ES = 0xE
#endif /* CONFIG_DSHOT_EDT */
#endif /* CONFIG_DSHOT_BIDIR */
};

struct dshot_bidir_telem {
    uint16_t erpm;
#ifdef CONFIG_DSHOT_EDT
    uint16_t temperature;
    uint16_t current;
    uint16_t voltage;
    uint16_t debug_1;
    uint16_t debug_2;
    uint16_t debug_3;
    uint16_t state_event;
#endif
};

enum dshot_type dshot_get_type(struct device *dev) const;

int dshot_set_type(struct device *dev, enum dshot_type type);

enum dshot_mode dshot_get_mode(struct device *dev) const;

#ifdef CONFIG_DSHOT_BIDIR
int dshot_set_mode(struct device *dev, enum dshot_mode mode);

void dshot_get_bidir_telem(struct device *dev, struct dshot_bidir_telem *out_telem);
#endif

void dshot_request_telem(struct device *dev, uint32_t channel, enum dshot_telem_type type);

int dshot_set_command(struct device *dev, uint32_t channel, enum dshot_command command);

bool dshot_command_in_progress(struct device *dev, uint32_t channel) const;

#ifdef __cplusplus
}
#endif

#endif /* ZFLIGHT_INCLUDE_DRIVERS_ESC_DSHOT_H_ */