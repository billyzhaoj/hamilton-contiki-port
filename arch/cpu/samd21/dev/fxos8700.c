/*
 * Copyright (C) Daniel Krebs
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @file
 * @brief       Driver for the FXOS8700 temperature sensor with serial EEPROM
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 */


#include "dev/fxos8700.h"

#define I2C_SPEED            I2C_SPEED_FAST

#define ACCEL_ONLY_MODE      0x00
#define MAG_ONLY_MODE        0x01
#define HYBRID_MODE          0x03

#define FXOS8700_I2C         I2C_0
#define FXOS8700_ADDR        0x1E

static int fxos8700_write_regs(uint8_t reg, uint8_t *data, size_t len) {
  i2c_acquire(FXOS8700_I2C);
  if (i2c_write_regs(FXOS8700_I2C, FXOS8700_ADDR, reg, (char *) data, len) <= 0) {
    //DEBUG("[fxos8700] Can't write to register 0x%x\n", reg);
    i2c_release(FXOS8700_I2C);
    return -1;
  }
  i2c_release(FXOS8700_I2C);

  return 0;
}

int fxos8700_init(void) {
  int rv;
  uint8_t config;

  i2c_acquire(FXOS8700_I2C);
  if (i2c_init_master(FXOS8700_I2C, I2C_SPEED) != 0) {
    //DEBUG("[fxos8700] Can't initialize I2C master\n");
    i2c_release(FXOS8700_I2C);
    return -3;
  }

  rv = i2c_read_regs(FXOS8700_I2C, FXOS8700_ADDR, 0x0D, &config, 1);
  if (rv != 1) {
    i2c_release(FXOS8700_I2C);
    //DEBUG("[fxos8700] Could not read WHOAMI (%d)\n", rv);
    return -4;
  }

  i2c_release(FXOS8700_I2C);

  /* Configure the ODR to maximum (400Hz in hybrid mode) */
  config = 0x00;
  if (fxos8700_write_regs(FXOS8700_REG_CTRL_REG1, &config, 1) != 0) {
    return -6;
  }
  /* Activate hybrid mode */
  config = HYBRID_MODE;
  if (fxos8700_write_regs(FXOS8700_REG_M_CTRL_REG1, &config, 1) != 0) {
    return -7;
  }
  /* Set burst read mode (accel + magnet together) */
  config = 0x20;
  if (fxos8700_write_regs(FXOS8700_REG_M_CTRL_REG2, &config, 1) != 0) {
    return -8;
  }

  return 0;
}

int fxos8700_set_active(void) {
  uint8_t config = 0x01;
  if (fxos8700_write_regs(FXOS8700_REG_CTRL_REG1, &config, 1) != 0) {
    return -1;
  }
  return 0;
}

int fxos8700_set_idle(void) {
  uint8_t config = 0x00;
  if (fxos8700_write_regs(FXOS8700_REG_CTRL_REG1, &config, 1) != 0) {
    return -1;
  }
  return 0;
}
