/*
 * Copyright (c) 2026 ZephyrAir Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * I2C hardware abstraction layer for the Bosch BMP3 Sensor API.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include "bmp390_bridge.h"

static struct bmp390_diag_stats g_diag_stats = { 0 };

static BMP3_INTF_RET_TYPE bmp390_i2c_read(uint8_t reg_addr, uint8_t *read_data,
                                          uint32_t len, void *intf_ptr)
{
    const struct i2c_dt_spec *i2c_spec = (const struct i2c_dt_spec *)intf_ptr;

    for (int attempt = 1; attempt <= BMP390_I2C_MAX_RETRIES; attempt++) {
        int ret = i2c_write_read_dt(i2c_spec, &reg_addr, 1, read_data, len);
        if (ret == 0) {
            return BMP3_INTF_RET_SUCCESS;
        }

        g_diag_stats.i2c_retries++;
        printk("[BMP390 I2C] Read NACK on reg 0x%02X (attempt %d/%d)\n",
               reg_addr, attempt, BMP390_I2C_MAX_RETRIES);
        k_msleep(2);
    }

    g_diag_stats.i2c_failures++;
    g_diag_stats.in_error_state = true;
    return BMP3_E_COMM_FAIL;
}

static BMP3_INTF_RET_TYPE bmp390_i2c_write(uint8_t reg_addr, const uint8_t *write_data,
                                           uint32_t len, void *intf_ptr)
{
    const struct i2c_dt_spec *i2c_spec = (const struct i2c_dt_spec *)intf_ptr;
    uint8_t buf[64];

    if (len + 1 > sizeof(buf)) {
        return BMP3_E_COMM_FAIL;
    }

    buf[0] = reg_addr;
    memcpy(&buf[1], write_data, len);

    for (int attempt = 1; attempt <= BMP390_I2C_MAX_RETRIES; attempt++) {
        int ret = i2c_write_dt(i2c_spec, buf, len + 1);
        if (ret == 0) {
            return BMP3_INTF_RET_SUCCESS;
        }

        g_diag_stats.i2c_retries++;
        printk("[BMP390 I2C] Write NACK on reg 0x%02X (attempt %d/%d)\n",
               reg_addr, attempt, BMP390_I2C_MAX_RETRIES);
        k_msleep(2);
    }

    g_diag_stats.i2c_failures++;
    g_diag_stats.in_error_state = true;
    return BMP3_E_COMM_FAIL;
}

static void bmp390_delay_us(uint32_t period, void *intf_ptr)
{
    ARG_UNUSED(intf_ptr);
    k_usleep(period);
}

int bmp390_bridge_init(struct bmp3_dev *dev, const struct i2c_dt_spec *i2c_spec)
{
    if (!dev || !i2c_spec) {
        return -EINVAL;
    }

    if (!i2c_is_ready_dt(i2c_spec)) {
        printk("[BMP390 Bridge] ERROR: I2C bus is not ready!\n");
        return -ENODEV;
    }

    memset(&g_diag_stats, 0, sizeof(g_diag_stats));

    memset(dev, 0, sizeof(struct bmp3_dev));
    dev->intf     = BMP3_I2C_INTF;
    dev->intf_ptr = (void *)i2c_spec;
    dev->read     = bmp390_i2c_read;
    dev->write    = bmp390_i2c_write;
    dev->delay_us = bmp390_delay_us;

    /* Verify chip ID, perform soft reset, and load NVM calibration data */
    int8_t rslt = bmp3_init(dev);
    if (rslt != BMP3_OK) {
        printk("[BMP390 Bridge] ERROR: bmp3_init failed with code %d\n", rslt);
        return -EIO;
    }

    /* Enable pressure and temperature with 4x pressure oversampling at 50 Hz */
    struct bmp3_settings settings = { 0 };
    settings.press_en            = BMP3_ENABLE;
    settings.temp_en             = BMP3_ENABLE;
    settings.odr_filter.press_os = BMP3_OVERSAMPLING_4X;
    settings.odr_filter.temp_os  = BMP3_NO_OVERSAMPLING;
    settings.odr_filter.odr      = BMP3_ODR_50_HZ;

    uint16_t settings_sel = BMP3_SEL_PRESS_EN |
                            BMP3_SEL_TEMP_EN  |
                            BMP3_SEL_PRESS_OS |
                            BMP3_SEL_TEMP_OS  |
                            BMP3_SEL_ODR;

    rslt = bmp3_set_sensor_settings(settings_sel, &settings, dev);
    if (rslt != BMP3_OK) {
        printk("[BMP390 Bridge] ERROR: bmp3_set_sensor_settings failed (%d)\n", rslt);
        return -EIO;
    }

    settings.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&settings, dev);
    if (rslt != BMP3_OK) {
        printk("[BMP390 Bridge] ERROR: bmp3_set_op_mode failed (%d)\n", rslt);
        return -EIO;
    }

    printk("[BMP390 Bridge] Initialized BMP390 (Chip ID: 0x%02X, I2C: 0x%02X)\n",
           dev->chip_id, i2c_spec->addr);

    return 0;
}

int bmp390_bridge_sample_fetch(struct bmp3_dev *dev, float *pressure_pa, float *temp_c)
{
    struct bmp3_data comp_data = { 0 };

    int8_t rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &comp_data, dev);
    if (rslt != BMP3_OK) {
        return -EIO;
    }

    g_diag_stats.total_samples++;
    g_diag_stats.in_error_state = false;

    *pressure_pa = (float)comp_data.pressure;
    *temp_c      = (float)comp_data.temperature;

    return 0;
}

void bmp390_bridge_get_stats(struct bmp390_diag_stats *stats)
{
    if (stats) {
        *stats = g_diag_stats;
    }
}
