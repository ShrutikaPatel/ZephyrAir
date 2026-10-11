/*
 * Copyright (c) 2026 ZephyrAir Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Bosch BMP390 Digital Barometric Pressure Sensor Emulator (Full Calibrated Model)
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

/* ====================================================================
 * BMP390 Register Map Offsets
 * ==================================================================== */
#define BMP390_REG_CHIP_ID   0x00
#define BMP390_REG_ERR       0x02
#define BMP390_REG_STATUS    0x03
#define BMP390_REG_DATA_0    0x04 /* Press [7:0]   */
#define BMP390_REG_DATA_1    0x05 /* Press [15:8]  */
#define BMP390_REG_DATA_2    0x06 /* Press [23:16] */
#define BMP390_REG_DATA_3    0x07 /* Temp  [7:0]   */
#define BMP390_REG_DATA_4    0x08 /* Temp  [15:8]  */
#define BMP390_REG_DATA_5    0x09 /* Temp  [23:16] */
#define BMP390_REG_EVENT     0x10
#define BMP390_REG_INT_STAT  0x11
#define BMP390_REG_PWR_CTRL  0x1B
#define BMP390_REG_OSR       0x1C
#define BMP390_REG_ODR       0x1D
#define BMP390_REG_CONFIG    0x1F
#define BMP390_REG_CALIB0    0x31 /* Calibration parameters (21 bytes) */
#define BMP390_REG_CMD       0x7E

#define BMP390_CHIP_ID_VAL   0x60
#define BMP390_NUM_REGS      0x80

/* Status flags */
#define BMP390_STATUS_CMD_RDY    BIT(4)
#define BMP390_STATUS_DRDY_PRESS BIT(5)
#define BMP390_STATUS_DRDY_TEMP  BIT(6)

/* Emulator runtime state */
struct bmp390_emul_data {
    uint8_t cur_reg;
    uint8_t regs[BMP390_NUM_REGS];
    uint32_t step_count;
};

struct bmp390_emul_cfg {
    uint16_t addr;
};

/*
 * Exact Factory Calibration Coefficients for Bosch BMP3 Polynomial Compensation
 * (21 bytes stored from 0x31 to 0x45)
 */
static const uint8_t bmp390_default_calib[21] = {
    /* T1 (uint16_t = 27838 = 0x6CB6) */
    0xB6, 0x6C,
    /* T2 (uint16_t = 19456 = 0x4C00) */
    0x00, 0x4C,
    /* T3 (int8_t = -10 = 0xF6) */
    0xF6,
    /* P1 (int16_t = 19456 = 0x4C00) */
    0x00, 0x4C,
    /* P2 (int16_t = 16384 = 0x4000) */
    0x00, 0x40,
    /* P3 (int8_t = 0) */
    0x00,
    /* P4 (int8_t = 0) */
    0x00,
    /* P5 (uint16_t = 10000 = 0x2710) */
    0x10, 0x27,
    /* P6 (uint16_t = 640 = 0x0280) */
    0x80, 0x02,
    /* P7 (int8_t = 0) */
    0x00,
    /* P8 (int8_t = 0) */
    0x00,
    /* P9 (int16_t = 0) */
    0x00, 0x00,
    /* P10 (int8_t = 0) */
    0x00,
    /* P11 (int8_t = 0) */
    0x00
};

/* Helper: updates simulated 24-bit raw ADC registers before a read */
static void bmp390_emul_update_adc(struct bmp390_emul_data *data)
{
    data->step_count++;

    /*
     * Generate 24-bit ADC counts calibrated against bmp390_default_calib:
     * Raw temp 0x815F9D -> ~24.43 °C
     * Raw press 0x6DDD00 -> ~101338 Pa
     */
    uint32_t raw_temp = 0x815F9D + (int32_t)(50000.0f * sinf(data->step_count * 0.03f));
    uint32_t raw_press = 0x6DDD00 + (int32_t)(30000.0f * cosf(data->step_count * 0.05f));

    /* Set Data Ready status and Interrupt Status */
    data->regs[BMP390_REG_STATUS] = BMP390_STATUS_CMD_RDY |
                                    BMP390_STATUS_DRDY_PRESS |
                                    BMP390_STATUS_DRDY_TEMP;
    data->regs[BMP390_REG_INT_STAT] = BIT(3); /* drdy interrupt bit */
    data->regs[BMP390_REG_EVENT]    = 0x01;   /* por_detected */

    /* Pack 24-bit raw pressure into DATA_0, DATA_1, DATA_2 */
    data->regs[BMP390_REG_DATA_0] = (uint8_t)(raw_press & 0xFF);
    data->regs[BMP390_REG_DATA_1] = (uint8_t)((raw_press >> 8) & 0xFF);
    data->regs[BMP390_REG_DATA_2] = (uint8_t)((raw_press >> 16) & 0xFF);

    /* Pack 24-bit raw temperature into DATA_3, DATA_4, DATA_5 */
    data->regs[BMP390_REG_DATA_3] = (uint8_t)(raw_temp & 0xFF);
    data->regs[BMP390_REG_DATA_4] = (uint8_t)((raw_temp >> 8) & 0xFF);
    data->regs[BMP390_REG_DATA_5] = (uint8_t)((raw_temp >> 16) & 0xFF);
}

/* I2C Transfer Callback */
static int bmp390_emul_transfer_i2c(const struct emul *target, struct i2c_msg *msgs, int num_msgs, int addr)
{
    ARG_UNUSED(addr);
    struct bmp390_emul_data *data = target->data;

    if (num_msgs == 2) {
        /* Standard write-then-read */
        if (msgs[0].flags & I2C_MSG_READ || msgs[0].len < 1) {
            return -EIO;
        }
        data->cur_reg = msgs[0].buf[0];

        /* Refresh ADC values when status or data registers are queried */
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
            /*
             * BMP390 Burst Write Format:
             * Interleaved [reg_addr_0, reg_data_0, reg_addr_1, reg_data_1, ...]
             */
            for (int i = 0; i + 1 < msgs[0].len; i += 2) {
                uint8_t r = msgs[0].buf[i];
                uint8_t v = msgs[0].buf[i + 1];
                if (r == BMP390_REG_CMD && v == 0xB6) {
                    /* Soft-reset command: keep calibration and chip_id intact */
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

    LOG_INF("BMP390 emulator ready for Bosch API at 0x%02X",
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