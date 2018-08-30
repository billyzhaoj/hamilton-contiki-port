/*
 * Copyright (C) 2014 PHYTEC Messtechnik GmbH
 *               2017 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/**
 * @ingroup     drivers_tmp006
 * @{
 *
 * @file
 * @brief       Driver for the TI TMP006 Infrared Thermopile Sensor.
 *
 * @author      Johann Fischer <j.fischer@phytec.de>
 * @author      Peter Kietzmann <peter.kietzmann@haw-hamburg.de>
 * @author      Sebastian Meiling <s@mlng.net>
 *
 * @}
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "dev/tmp006.h"

#define TMP006_CONFIG_RST           (1 << 15)

#define TMP006_CONFIG_MOD_SHIFT     (12U)
#define TMP006_CONFIG_MOD_MASK      (0x7000)
#define TMP006_CONFIG_MOD(x)        (((uint16_t)(((uint16_t)(x)) << TMP006_CONFIG_MOD_SHIFT))\
                                     & TMP006_CONFIG_MOD_MASK)
#define TMP006_CONFIG_MOD_CC        (0x07)
#define TMP006_CONFIG_MOD_OFF       (0x00)

#define TMP006_CONFIG_CR_SHIFT      (9U)
#define TMP006_CONFIG_CR_MASK       (0x0E00)
#define TMP006_CONFIG_CR(x)         (((uint16_t)(((uint16_t)(x)) << TMP006_CONFIG_CR_SHIFT))\
                                     & TMP006_CONFIG_CR_MASK)

#define TMP006_CONFIG_DRDY_PIN_EN   (1 << 8)
#define TMP006_CONFIG_DRDY          (1 << 7)

#define TMP006_MID_VALUE            (0x5449) /**< Manufacturer ID */
#define TMP006_DID_VALUE            (0x0067) /**< Device ID */

#define I2C_SPEED                   I2C_SPEED_FAST
#define TMP006_I2C  I2C_0
#define TMP006_ADDR 0x44
#define TMP006_RATE TMP006_CONFIG_CR_AS2
#define TMP006_CONVERSION_TIME  550000UL


int tmp006_init(void) {
  uint8_t reg[2];
  uint16_t tmp;

  if (TMP006_RATE > TMP006_CONFIG_CR_AS16) {
    //LOG_ERROR("tmp006_init: invalid conversion rate!\n");
    return -TMP006_ERROR_CONF;
  }

  /* setup the I2C bus */
  i2c_acquire(TMP006_I2C);
  if (i2c_init_master(TMP006_I2C, I2C_SPEED) < 0) {
    i2c_release(TMP006_I2C);
    //LOG_ERROR("tmp006_init: error initializing I2C bus\n");
    return -TMP006_ERROR_BUS;
  }
  /* test device id */
  if (i2c_read_regs(TMP006_I2C, TMP006_ADDR, TMP006_REGS_DEVICE_ID, reg, 2) != 2) {
    i2c_release(TMP006_I2C);
    //LOG_ERROR("tmp006_init: error reading device ID!\n");
    return -TMP006_ERROR_BUS;
  }
  tmp = ((uint16_t) reg[0] << 8) | reg[1];
  if (tmp != TMP006_DID_VALUE) {
    return -TMP006_ERROR_DEV;
  }
  /* set conversion rate */
  tmp = TMP006_CONFIG_CR(TMP006_RATE);
  reg[0] = (tmp >> 8);
  reg[1] = tmp;
  if (i2c_write_regs(TMP006_I2C, TMP006_ADDR, TMP006_REGS_CONFIG, reg, 2) != 2) {
    i2c_release(TMP006_I2C);
    //LOG_ERROR("tmp006_init: error setting conversion rate!\n");
    return -TMP006_ERROR_BUS;
  }
  i2c_release(TMP006_I2C);

  return TMP006_OK;
}

int tmp006_reset(void) {
  uint8_t reg[2];
  uint16_t tmp = TMP006_CONFIG_RST;
  reg[0] = (tmp >> 8);
  reg[1] = tmp;

  /* Acquire exclusive access to the bus. */
  i2c_acquire(TMP006_I2C);
  if (i2c_write_regs(TMP006_I2C, TMP006_ADDR, TMP006_REGS_CONFIG, reg, 2) != 2) {
    i2c_release(TMP006_I2C);
    return -TMP006_ERROR_BUS;
  }
  i2c_release(TMP006_I2C);
  return TMP006_OK;
}

int tmp006_set_active(void) {
  uint8_t reg[2];

  i2c_acquire(TMP006_I2C);
  if (i2c_read_regs(TMP006_I2C, TMP006_ADDR, TMP006_REGS_CONFIG, reg, 2) != 2) {
    i2c_release(TMP006_I2C);
    return -TMP006_ERROR_BUS;
  }

  reg[0] |= (TMP006_CONFIG_MOD(TMP006_CONFIG_MOD_CC) >> 8);
  if (i2c_write_regs(TMP006_I2C, TMP006_ADDR, TMP006_REGS_CONFIG, reg, 2) != 2) {
    i2c_release(TMP006_I2C);
    return -TMP006_ERROR_BUS;
  }
  i2c_release(TMP006_I2C);
  return TMP006_OK;
}

int tmp006_set_standby(void) {
  uint8_t reg[2];

  i2c_acquire(TMP006_I2C);
  if (i2c_read_regs(TMP006_I2C, TMP006_ADDR, TMP006_REGS_CONFIG, reg, 2) != 2) {
    i2c_release(TMP006_I2C);
    return -TMP006_ERROR_BUS;
  }

  reg[0] &= ~(TMP006_CONFIG_MOD(TMP006_CONFIG_MOD_CC) >> 8);
  if (i2c_write_regs(TMP006_I2C, TMP006_ADDR, TMP006_REGS_CONFIG, reg, 2) != 2) {
    i2c_release(TMP006_I2C);
    return -TMP006_ERROR_BUS;
  }
  i2c_release(TMP006_I2C);
  return TMP006_OK;
}
