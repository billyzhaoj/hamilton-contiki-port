/*
 * Copyright (C) 2014 PHYTEC Messtechnik GmbH
 *               2017 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/**
 * @ingroup     drivers_hdc1000
 * @{
 *
 * @file
 * @brief       Driver for the TI HDC1000 Humidity and Temperature Sensor.
 *
 * @author      Johann Fischer <j.fischer@phytec.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <string.h>

#include "dev/i2c.h"
#include "dev/hdc1000.h"

#define I2C_SPEED                  I2C_SPEED_FAST
#define HDC1000_I2C                I2C_0
#define HDC1000_ADDR               0x40
#define HDC1000_RES                HDC1000_14BIT

int hdc1000_init(void)
{
    uint8_t reg[2];
    uint16_t tmp;

    /* initialize the I2C bus */
    i2c_acquire(HDC1000_I2C);
    if (i2c_init_master(HDC1000_I2C, I2C_SPEED) < 0) {
        i2c_release(HDC1000_I2C);
        return HDC1000_NOBUS;
    }

    /* try if we can interact with the device by reading its manufacturer ID */
    if (i2c_read_regs(HDC1000_I2C, HDC1000_ADDR,
                      HDC1000_MANUFACTURER_ID, reg, 2) != 2) {
        i2c_release(HDC1000_I2C);
        return HDC1000_NOBUS;
    }

    tmp = ((uint16_t)reg[0] << 8) | reg[1];
    if (tmp != HDC1000_MID_VALUE) {
        i2c_release(HDC1000_I2C);
        return HDC1000_NODEV;
    }

    /* set resolution for both sensors and sequence mode */
    tmp = (HDC1000_SEQ_MOD | HDC1000_RES);
    reg[0] = (tmp >> 8);
    reg[1] = tmp;

    if (i2c_write_regs(HDC1000_I2C, HDC1000_ADDR, HDC1000_CONFIG, reg, 2) != 2) {
        i2c_release(HDC1000_I2C);
        return HDC1000_NOBUS;
    }
    i2c_release(HDC1000_I2C);

    /* all set */
    return HDC1000_OK;
}
