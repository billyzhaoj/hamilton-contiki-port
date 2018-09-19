/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_at86rf2xx
 * @{
 *
 * @file
 * @brief       Netdev adaption for the AT86RF2xx drivers
 *
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Kévin Roussel <Kevin.Roussel@inria.fr>
 * @author      Martine Lenders <mlenders@inf.fu-berlin.de>
 *
 * @}
 */

#include <string.h>
#include <errno.h>


#include "at86rf233.h"
#include "at86rf233_registers.h"
#include "at86rf233_internal.h"
#include "dev/leds.h"
#include "asf_headers/samr21e18a.h"

#define _MAX_MHR_OVERHEAD   (25)

static at86rf2xx_t radio_dev;
at86rf2xx_t *dev = &radio_dev;

static const at86rf2xx_params_t at86rf2xx_params =
        {
                .spi = SPI_DEV(0),
                .spi_clk = SPI_CLK_8MHZ,
                .cs_pin = GPIO_PIN(PB, 31),
                .int_pin = GPIO_PIN(PB, 0),
                .sleep_pin = GPIO_PIN(PA, 20),
                .reset_pin = GPIO_PIN(PB, 15)
        };

static void _irq_handler(void *arg) {
  at86rf2xx_reg_read(dev, AT86RF2XX_REG__IRQ_STATUS);
}

void at86rf2xx_setup(at86rf2xx_t *dev, const at86rf2xx_params_t *params) {
  /* initialize device descriptor */
  memcpy(&dev->params, params, sizeof(at86rf2xx_params_t));
  dev->idle_state = AT86RF2XX_STATE_TRX_OFF;
  /* radio state is P_ON when first powered-on */
  dev->state = AT86RF2XX_STATE_P_ON;
  dev->pending_tx = 0;
}

void at86rf2xx_reset(at86rf2xx_t *dev) {
  /** Hardware reset */
  at86rf2xx_hardware_reset(dev);

  /* set short and long address */
  at86rf2xx_set_addr_short(dev, AT86RF2XX_DEFAULT_ADDR_SHORT);
  at86rf2xx_set_addr_long(dev, AT86RF2XX_DEFAULT_ADDR_LONG);

  /* set default PAN id */
  at86rf2xx_set_pan(dev, AT86RF2XX_DEFAULT_PANID);

  /* set default Channel ID */
  at86rf2xx_set_chan(dev, AT86RF2XX_DEFAULT_CHANNEL);

  /* set default TX Power */
  at86rf2xx_set_txpower(dev, AT86RF2XX_DEFAULT_TXPOWER);

  /* Enable safe mode*/
  at86rf2xx_reg_write(dev, AT86RF2XX_REG__TRX_CTRL_2, AT86RF2XX_TRX_CTRL_2_MASK__RX_SAFE_MODE);

  /* Disable clock output to save power*/
  uint8_t tmp = at86rf2xx_reg_read(dev, AT86RF2XX_REG__TRX_CTRL_0);
  tmp &= ~(AT86RF2XX_TRX_CTRL_0_MASK__CLKM_CTRL);
  tmp &= ~(AT86RF2XX_TRX_CTRL_0_MASK__CLKM_SHA_SEL);
  tmp |= (AT86RF2XX_TRX_CTRL_0_CLKM_CTRL__OFF);
  at86rf2xx_reg_write(dev, AT86RF2XX_REG__TRX_CTRL_0, tmp);

  /* Enable interrupts*/
  //at86rf2xx_reg_write(dev, AT86RF2XX_REG__IRQ_MASK, AT86RF2XX_IRQ_STATUS_MASK__TRX_END);

  /* Clear interrupt flags */
  at86rf2xx_reg_read(dev, AT86RF2XX_REG__IRQ_STATUS);

  /* Turn into RX state*/
  at86rf2xx_set_state(dev, AT86RF2XX_STATE_RX_AACK_ON);

  for (int i = 0; i < 10000; i++);
  uint8_t state = at86rf2xx_reg_read(dev, AT86RF2XX_REG__TRX_STATUS);
  if (state != AT86RF2XX_STATE_RX_AACK_ON) {
    leds_on(LEDS_ALL);
  }
}

int radio_init(void) {
  at86rf2xx_setup(dev, &at86rf2xx_params);

  /* initialize GPIOs */
  gpio_init(dev->params.sleep_pin, GPIO_OUT);
  gpio_clear(dev->params.sleep_pin);
  gpio_init(dev->params.reset_pin, GPIO_OUT);
  gpio_set(dev->params.reset_pin);
  spi_init_cs(dev->params.spi, dev->params.cs_pin);
  gpio_init_int(dev->params.int_pin, GPIO_IN, GPIO_RISING, _irq_handler, dev);


  /* Initialize SPI */
  spi_init(0);

  /* test if the SPI is set up correctly and the device is responding */
  if (at86rf2xx_reg_read(dev, AT86RF2XX_REG__PART_NUM) !=
      AT86RF2XX_PARTNUM) {
    // Error, unable to read correct part number.
    return -1;
  }


  /* Reset the device to default and put it into RX state */
  at86rf2xx_reset(dev);
  return 0;
}

size_t send(uint8_t *data, size_t len) {
  return at86rf2xx_send(dev, data, len);
}

void reset(void) {
  return at86rf2xx_reset(dev);
}

void at86rf2xx_tx_prepare(at86rf2xx_t *dev) {
  uint8_t state;
  do {
    state = at86rf2xx_get_status(dev);
  } while (state == AT86RF2XX_STATE_BUSY_TX_ARET);

  /* if receiving cancel */
  if (state == AT86RF2XX_STATE_BUSY_RX_AACK) {
    at86rf2xx_force_trx_off(dev);
    dev->idle_state = AT86RF2XX_STATE_RX_AACK_ON;
  } else if (state != AT86RF2XX_STATE_TX_ARET_ON) {
    dev->idle_state = state;
  } else {

  }
  at86rf2xx_set_state(dev, AT86RF2XX_STATE_TX_ARET_ON);
  dev->tx_frame_len = IEEE802154_FCS_LEN;
}

size_t at86rf2xx_tx_load(at86rf2xx_t *dev, uint8_t *data, size_t len,
                         size_t offset) {
  dev->tx_frame_len = (uint8_t) len;
  at86rf2xx_sram_write(dev, offset + 1, data, len);
  return offset + len;
}

void at86rf2xx_tx_exec(at86rf2xx_t *dev) {
  at86rf2xx_sram_write(dev, 0, &(dev->tx_frame_len), 1);
  at86rf2xx_reg_write(dev, AT86RF2XX_REG__TRX_STATE, AT86RF2XX_TRX_STATE__TX_START);
}

size_t at86rf2xx_send(at86rf2xx_t *dev, uint8_t *data, size_t len) {
  if (len > AT86RF2XX_MAX_PKT_LENGTH) {
    return 0; //0 is error in this case
  }
  uint8_t state = at86rf2xx_reg_read(dev, AT86RF2XX_REG__TRX_STATUS);
  at86rf2xx_tx_prepare(dev);
  state = at86rf2xx_reg_read(dev, AT86RF2XX_REG__TRX_STATUS);
  at86rf2xx_tx_load(dev, data, len, 0);
  state = at86rf2xx_reg_read(dev, AT86RF2XX_REG__TRX_STATUS);
  at86rf2xx_tx_exec(dev);
  state = at86rf2xx_reg_read(dev, AT86RF2XX_REG__TRX_STATUS);
  state = -1;
  for (state = 0; state < -1; state++);


  return len;
}

void EIC_Handler(void) {
  _irq_handler((void *) 1);
}