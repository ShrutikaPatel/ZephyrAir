/*
 * Copyright (c) 2026 ZephyrAir Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Bosch BMP390 Digital Barometric Pressure Sensor Emulator
 */

#define DT_DRV_COMPAT bosch_bmp390

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bmp390_emul, CONFIG_LOG_DEFAULT_LEVEL);

/* BMP390 Register Map constants */
#define BMP390_REG_CHIP_ID   0x00
#define BMP390_CHIP_ID_VAL   0x60
#define BMP390_NUM_REGS      0x80

/* Emulator runtime state */
struct bmp390_emul_data {
    uint8_t cur_reg;
    uint8_t regs[BMP390_NUM_REGS];
};

struct bmp390_emul_cfg {
    uint16_t addr;
};

/* I2C Transfer Callback: handles master read/write transactions */
static int bmp390_emul_transfer_i2c(const struct emul *target, struct i2c_msg *msgs, int num_msgs, int addr)
{
    ARG_UNUSED(addr);
    struct bmp390_emul_data *data = target->data;

    if (num_msgs == 2) {
        /*
         * Standard write-then-read transaction:
         * msgs[0]: writes the register pointer
         * msgs[1]: reads data starting from that register pointer
         */
        if (msgs[0].flags & I2C_MSG_READ) {
            return -EIO;
        }
        if (msgs[0].len < 1) {
            return -EIO;
        }
        data->cur_reg = msgs[0].buf[0];

        if (msgs[1].flags & I2C_MSG_READ) {
            for (int i = 0; i < msgs[1].len; i++) {
                uint8_t r = data->cur_reg + i;
                if (r < BMP390_NUM_REGS) {
                    msgs[1].buf[i] = data->regs[r];
                } else {
                    msgs[1].buf[i] = 0x00;
                }
            }
        } else {
            /* Repeated write */
            for (int i = 0; i < msgs[1].len; i++) {
                uint8_t r = data->cur_reg + i;
                if (r < BMP390_NUM_REGS) {
                    data->regs[r] = msgs[1].buf[i];
                }
            }
        }
    } else if (num_msgs == 1) {
        if (!(msgs[0].flags & I2C_MSG_READ)) {
            /* Single write: [reg_addr, val1, val2, ...] */
            if (msgs[0].len >= 1) {
                data->cur_reg = msgs[0].buf[0];
                for (int i = 1; i < msgs[0].len; i++) {
                    uint8_t r = data->cur_reg + (i - 1);
                    if (r < BMP390_NUM_REGS) {
                        data->regs[r] = msgs[0].buf[i];
                    }
                }
            }
        } else {
            /* Single read from current register */
            for (int i = 0; i < msgs[0].len; i++) {
                uint8_t r = data->cur_reg + i;
                if (r < BMP390_NUM_REGS) {
                    msgs[0].buf[i] = data->regs[r];
                }
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

    /* Initialize register 0x00 with official Bosch BMP390 Chip ID: 0x60 */
    data->regs[BMP390_REG_CHIP_ID] = BMP390_CHIP_ID_VAL;

    LOG_INF("BMP390 emulator ready: addr 0x%02X, CHIP_ID=0x%02X",
            ((const struct bmp390_emul_cfg *)target->cfg)->addr,
            BMP390_CHIP_ID_VAL);

    return 0;
}

static int bmp390_dummy_init(const struct device *dev)
{
    ARG_UNUSED(dev);
    return 0;
}

#define INIT_BMP390(n)                                                                             \
    static struct bmp390_emul_data bmp390_emul_data_##n;                                           \
    static const struct bmp390_emul_cfg bmp390_emul_cfg_##n = {                                    \
        .addr = DT_INST_REG_ADDR(n),                                                               \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, bmp390_dummy_init, NULL, NULL, NULL, POST_KERNEL,                     \
                          CONFIG_APPLICATION_INIT_PRIORITY, NULL);                                 \
    EMUL_DT_INST_DEFINE(n, bmp390_emul_init, &bmp390_emul_data_##n, &bmp390_emul_cfg_##n,          \
                        &bmp390_emul_api_i2c, NULL)

DT_INST_FOREACH_STATUS_OKAY(INIT_BMP390)
