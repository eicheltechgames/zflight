/**
 * @file
 *
 * @brief Public APIs for the ESC Manager.
 */

/*
 * Copyright (c) 2024 Arthur Eichelberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZFLIGHT_INCLUDE_FLIGHT_H_
#define ZFLIGHT_INCLUDE_FLIGHT_H_

#include <errno.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct flight_control_inputs {
    float pitch_rate;
    float roll_rate;
    float yaw_rate;
    float z_accel;
};

struct flight_control_IMU_inputs {
    float pitch_rate;
    float roll_rate;
    float yaw_rate;
    float x_accel;
    float y_accel;
    float z_accel;
};

void flight_control(const struct flight_control_inputs *set_points,
                    const struct flight_control_IMU_inputs *imu_inputs,
                    float out_throttles[CONFIG_ZFLIGHT_NUM_MOTORS]);

#ifdef __cplusplus
}
#endif

#endif /* ZFLIGHT_INCLUDE_FLIGHT_H_ */