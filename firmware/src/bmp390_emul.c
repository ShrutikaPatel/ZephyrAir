/*
 * Copyright (c) 2026 ZephyrAir Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * I2C bus emulator for the Bosch BMP390 barometric pressure sensor.
 */

#define DT_DRV_COMPAT bosch_bmp390

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(bmp390_emul, CONFIG_LOG_DEFAULT_LEVEL);

/* BMP390 register addresses */
#define BMP390_REG_CHIP_ID   0x00
#define BMP390_REG_ERR       0x02
#define BMP390_REG_STATUS    0x03
#define BMP390_REG_DATA_0    0x04 /* Pressure XLSB [7:0]   */
#define BMP390_REG_DATA_1    0x05 /* Pressure LSB  [15:8]  */
#define BMP390_REG_DATA_2    0x06 /* Pressure MSB  [23:16] */
#define BMP390_REG_DATA_3    0x07 /* Temp XLSB     [7:0]   */
#define BMP390_REG_DATA_4    0x08 /* Temp LSB      [15:8]  */
#define BMP390_REG_DATA_5    0x09 /* Temp MSB      [23:16] */
#define BMP390_REG_EVENT     0x10
#define BMP390_REG_INT_STAT  0x11
#define BMP390_REG_PWR_CTRL  0x1B
#define BMP390_REG_OSR       0x1C
#define BMP390_REG_ODR       0x1D
#define BMP390_REG_CONFIG    0x1F
#define BMP390_REG_CALIB0    0x31 /* NVM trimming coefficients (21 bytes) */
#define BMP390_REG_CMD       0x7E

#define BMP390_CHIP_ID_VAL   0x60
#define BMP390_NUM_REGS      0x80

/* Status register bit definitions */
#define BMP390_STATUS_CMD_RDY    BIT(4)
#define BMP390_STATUS_DRDY_PRESS BIT(5)
#define BMP390_STATUS_DRDY_TEMP  BIT(6)

struct bmp390_emul_data {
    uint8_t cur_reg;
    uint8_t regs[BMP390_NUM_REGS];
    uint32_t step_count;
};

/* Remaining consecutive I2C transfers to fail with -EIO for fault testing */
static volatile uint32_t g_injected_faults = 0;

void bmp390_emul_inject_faults(uint32_t count)
{
    g_injected_faults = count;
}

struct bmp390_emul_cfg {
    uint16_t addr;
};

/*
 * Default NVM calibration coefficients (registers 0x31..0x45) used by
 * the Bosch BMP3 compensation formulas.
 */
static const uint8_t bmp390_default_calib[21] = {
    0xB6, 0x6C, /* par_t1 */
    0x00, 0x4C, /* par_t2 */
    0xF6,       /* par_t3 */
    0x00, 0x4C, /* par_p1 */
    0x00, 0x40, /* par_p2 */
    0x00,       /* par_p3 */
    0x00,       /* par_p4 */
    0x10, 0x27, /* par_p5 */
    0x80, 0x02, /* par_p6 */
    0x00,       /* par_p7 */
    0x00,       /* par_p8 */
    0x00, 0x00, /* par_p9 */
    0x00,       /* par_p10 */
    0x00        /* par_p11 */
};

/* Synthesizes 24-bit uncompensated ADC readings prior to a data register read */
static void bmp390_emul_update_adc(struct bmp390_emul_data *data)
{
    data->step_count++;

    uint32_t raw_temp = 0x815F9D + (int32_t)(50000.0f * sinf(data->step_count * 0.03f));
    uint32_t raw_press = 0x6DDD00 + (int32_t)(30000.0f * cosf(data->step_count * 0.05f));

    data->regs[BMP390_REG_STATUS] = BMP390_STATUS_CMD_RDY |
                                    BMP390_STATUS_DRDY_PRESS |
                                    BMP390_STATUS_DRDY_TEMP;
    data->regs[BMP390_REG_INT_STAT] = BIT(3);
    data->regs[BMP390_REG_EVENT]    = 0x01;

    /* 24-bit little-endian pressure ADC count */
    data->regs[BMP390_REG_DATA_0] = (uint8_t)(raw_press & 0xFF);
    data->regs[BMP390_REG_DATA_1] = (uint8_t)((raw_press >> 8) & 0xFF);
    data->regs[BMP390_REG_DATA_2] = (uint8_t)((raw_press >> 16) & 0xFF);

    /* 24-bit little-endian temperature ADC count */
    data->regs[BMP390_REG_DATA_3] = (uint8_t)(raw_temp & 0xFF);
    data->regs[BMP390_REG_DATA_4] = (uint8_t)((raw_temp >> 8) & 0xFF);
    data->regs[BMP390_REG_DATA_5] = (uint8_t)((raw_temp >> 16) & 0xFF);
}

static int bmp390_emul_transfer_i2c(const struct emul *target, struct i2c_msg *msgs, int num_msgs, int addr)
{
    ARG_UNUSED(addr);
    struct bmp390_emul_data *data = target->data;

    if (g_injected_faults > 0) {
        g_injected_faults--;
        return -EIO;
    }

    if (num_msgs == 2) {
        if (msgs[0].flags & I2C_MSG_READ || msgs[0].len < 1) {
            return -EIO;
        }
        data->cur_reg = msgs[0].buf[0];

        if (data->cur_reg <= BMP390_REG_INT_STAT) {
            bmp390_emul_update_adc(data);
        }

        if (msgs[1].flags & I2C_MSG_READ) {
            for (int i = 0; i < msgs[1].len; i++) {
                uint8_t r = data->cur_reg + i;
                msgs[1].buf[i] = (r < BMP390_NUM_REGS) ? data->regs[r] : 0x00;
            }
        } else {
            for (int i = 0; i < msgs[1].len; i++) {
                uint8_t r = data->cur_reg + i;
                if (r < BMP390_NUM_REGS) {
                    data->regs[r] = msgs[1].buf[i];
                }
            }
        }
    } else if (num_msgs == 1) {
        if (!(msgs[0].flags & I2C_MSG_READ)) {
            /* BMP390 burst writes use interleaved [reg_addr, reg_data] pairs */
            for (int i = 0; i + 1 < msgs[0].len; i += 2) {
                uint8_t r = msgs[0].buf[i];
                uint8_t v = msgs[0].buf[i + 1];
                if (r == BMP390_REG_CMD && v == 0xB6) {
                    data->regs[BMP390_REG_ERR] = 0x00;
                    data->regs[BMP390_REG_STATUS] = BMP390_STATUS_CMD_RDY;
                } else if (r < BMP390_NUM_REGS) {
                    data->regs[r] = v;
                }
            }
        } else {
            for (int i = 0; i < msgs[0].len; i++) {
                uint8_t r = data->cur_reg + i;
                msgs[0].buf[i] = (r < BMP390_NUM_REGS) ? data->regs[r] : 0x00;
            }
        }
    } else {
        return -EIO;
    }

    return 0;
}

static struct i2c_emul_api bmp390_emul_api_i2c = {
    .transfer = bmp390_emul_transfer_i2c,
};

static int bmp390_emul_init(const struct emul *target, const struct device *parent)
{
    ARG_UNUSED(parent);
    struct bmp390_emul_data *data = target->data;

    memset(data->regs, 0, sizeof(data->regs));

    data->regs[BMP390_REG_CHIP_ID] = BMP390_CHIP_ID_VAL;
    data->regs[BMP390_REG_STATUS]  = BMP390_STATUS_CMD_RDY;
    memcpy(&data->regs[BMP390_REG_CALIB0], bmp390_default_calib, sizeof(bmp390_default_calib));

    data->step_count = 0;
    bmp390_emul_update_adc(data);

    LOG_INF("BMP390 I2C emulator initialized at 0x%02X",
            ((const struct bmp390_emul_cfg *)target->cfg)->addr);

    return 0;
}

static int bmp390_dummy_init(const struct device *dev)
{
    ARG_UNUSED(dev);
    return 0;
}

#define INIT_BMP390(n)                                                                             \
    static struct bmp390_emul_data bmp390_emul_data_##n;                                          \
    static const struct bmp390_emul_cfg bmp390_emul_cfg_##n = {                                    \
        .addr = DT_INST_REG_ADDR(n),                                                               \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, bmp390_dummy_init, NULL, NULL, NULL, POST_KERNEL,                     \
                          CONFIG_APPLICATION_INIT_PRIORITY, NULL);                                 \
    EMUL_DT_INST_DEFINE(n, bmp390_emul_init, &bmp390_emul_data_##n, &bmp390_emul_cfg_##n,          \
                        &bmp390_emul_api_i2c, NULL)

DT_INST_FOREACH_STATUS_OKAY(INIT_BMP390)