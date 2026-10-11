/*
 * Copyright (c) 2026 ZephyrAir Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform Bridge between Bosch BMP3-Sensor-API and Zephyr RTOS I2C
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include "bmp390_bridge.h"

/* ====================================================================
 * 1. The 3 Platform Hardware Callbacks (Passed via Function Pointers)
 * ==================================================================== */

/**
 * @brief Zephyr I2C Read Adapter for Bosch BMP3 API
 */
static BMP3_INTF_RET_TYPE bmp390_i2c_read(uint8_t reg_addr, uint8_t *read_data,
                                          uint32_t len, void *intf_ptr)
{
    const struct i2c_dt_spec *i2c_spec = (const struct i2c_dt_spec *)intf_ptr;

    int ret = i2c_write_read_dt(i2c_spec, &reg_addr, 1, read_data, len);
    return (ret == 0) ? BMP3_INTF_RET_SUCCESS : BMP3_E_COMM_FAIL;
}

/**
 * @brief Zephyr I2C Write Adapter for Bosch BMP3 API
 */
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

    int ret = i2c_write_dt(i2c_spec, buf, len + 1);
    return (ret == 0) ? BMP3_INTF_RET_SUCCESS : BMP3_E_COMM_FAIL;
}

/**
 * @brief Zephyr Microsecond Delay Adapter for Bosch BMP3 API
 */
static void bmp390_delay_us(uint32_t period, void *intf_ptr)
{
    ARG_UNUSED(intf_ptr);
    k_usleep(period);
}

/* ====================================================================
 * 2. High-Level Initialization & Sampling Functions
 * ==================================================================== */

int bmp390_bridge_init(struct bmp3_dev *dev, const struct i2c_dt_spec *i2c_spec)
{
    if (!dev || !i2c_spec) {
        return -EINVAL;
    }

    if (!i2c_is_ready_dt(i2c_spec)) {
        printk("[BMP390 Bridge] ERROR: I2C bus is not ready!\n");
        return -ENODEV;
    }

    /* Wire Bosch device struct to our Zephyr I2C callbacks */
    memset(dev, 0, sizeof(struct bmp3_dev));
    dev->intf     = BMP3_I2C_INTF;
    dev->intf_ptr = (void *)i2c_spec;
    dev->read     = bmp390_i2c_read;
    dev->write    = bmp390_i2c_write;
    dev->delay_us = bmp390_delay_us;

    /*
     * Call official Bosch initialization:
     * 1. Reads CHIP_ID (0x00) and verifies 0x60
     * 2. Sends soft-reset (0xB6 to 0x7E)
     * 3. Reads & quantizes all 21 factory calibration bytes (0x31..0x45)
     */
    int8_t rslt = bmp3_init(dev);
    if (rslt != BMP3_OK) {
        printk("[BMP390 Bridge] ERROR: bmp3_init failed with code %d\n", rslt);
        return -EIO;
    }

    /* Configure sensor: Enable Pressure & Temp, 4x Pressure Oversampling, 50 Hz ODR */
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

    /* Set power mode to Normal (continuous measurement) */
    settings.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&settings, dev);
    if (rslt != BMP3_OK) {
        printk("[BMP390 Bridge] ERROR: bmp3_set_op_mode failed (%d)\n", rslt);
        return -EIO;
    }

    printk("\n*********************************************************************\n");
    printk(">>> [Bosch BMP3 API] INITIALIZED SUCCESSFULLY!                    <<<\n");
    printk(">>> Chip ID: 0x%02X | Bus: I2C (0x%02X) | Mode: Normal (OSR 4x, 50Hz) <<<\n",
           dev->chip_id, i2c_spec->addr);
    printk("*********************************************************************\n\n");

    return 0;
}

int bmp390_bridge_sample_fetch(struct bmp3_dev *dev, float *pressure_pa, float *temp_c)
{
    struct bmp3_data comp_data = { 0 };

    /*
     * Calls official Bosch driver:
     * 1. Reads 6 raw ADC bytes (0x04..0x09) over I2C
     * 2. Runs 64-bit / double polynomial compensation using NVM trim data
     */
    int8_t rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &comp_data, dev);
    if (rslt != BMP3_OK) {
        return -EIO;
    }

    *pressure_pa = (float)comp_data.pressure;
    *temp_c      = (float)comp_data.temperature;

    return 0;
}
