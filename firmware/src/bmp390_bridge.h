/*
 * Copyright (c) 2026 ZephyrAir Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * I2C hardware abstraction layer for the Bosch BMP3 Sensor API.
 */

#ifndef ZEPHYRAIR_BMP390_BRIDGE_H_
#define ZEPHYRAIR_BMP390_BRIDGE_H_

#include <zephyr/drivers/i2c.h>
#include <stdbool.h>
#include <stdint.h>
#include "bmp3.h"

#define BMP390_I2C_MAX_RETRIES 3

struct bmp390_diag_stats {
    uint32_t total_samples;
    uint32_t i2c_retries;
    uint32_t i2c_failures;
    bool     in_error_state;
};

int bmp390_bridge_init(struct bmp3_dev *dev, const struct i2c_dt_spec *i2c_spec);
int bmp390_bridge_sample_fetch(struct bmp3_dev *dev, float *pressure_pa, float *temp_c);
void bmp390_bridge_get_stats(struct bmp390_diag_stats *stats);
void bmp390_emul_inject_faults(uint32_t count);

#endif /* ZEPHYRAIR_BMP390_BRIDGE_H_ */
