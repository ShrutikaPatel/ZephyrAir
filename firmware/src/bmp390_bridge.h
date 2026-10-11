/*
 * Copyright (c) 2026 ZephyrAir Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform Bridge between Bosch BMP3-Sensor-API and Zephyr RTOS I2C
 */

#ifndef ZEPHYRAIR_BMP390_BRIDGE_H_
#define ZEPHYRAIR_BMP390_BRIDGE_H_

#include <zephyr/drivers/i2c.h>
#include "bmp3.h"

/**
 * @brief Initializes the Bosch BMP390 sensor using Zephyr I2C
 *
 * Connects the Bosch function pointers (.read, .write, .delay_us),
 * calls bmp3_init(), configures oversampling & ODR, and enters Normal Mode.
 *
 * @param dev      Pointer to Bosch BMP3 device structure
 * @param i2c_spec Pointer to Zephyr Devicetree I2C specification
 * @return 0 on success, negative error code on failure
 */
int bmp390_bridge_init(struct bmp3_dev *dev, const struct i2c_dt_spec *i2c_spec);

/**
 * @brief Reads compensated pressure (Pa) and temperature (deg C) from BMP390
 *
 * @param dev         Pointer to initialized Bosch BMP3 device structure
 * @param pressure_pa Output pointer for compensated pressure in Pascals
 * @param temp_c      Output pointer for compensated temperature in Celsius
 * @return 0 on success, negative error code on failure
 */
int bmp390_bridge_sample_fetch(struct bmp3_dev *dev, float *pressure_pa, float *temp_c);

#endif /* ZEPHYRAIR_BMP390_BRIDGE_H_ */
