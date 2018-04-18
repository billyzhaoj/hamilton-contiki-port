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

#define _MAX_MHR_OVERHEAD   (25)

static at86rf2xx_t radio_dev;
at86rf2xx_t* dev = &radio_dev;

static const at86rf2xx_params_t at86rf2xx_params =
{
    .spi = SPI_DEV(0),
    .spi_clk = SPI_CLK_8MHZ,
    .cs_pin = GPIO_PIN(PB, 31),
    .int_pin = GPIO_PIN(PB, 0),
    .sleep_pin = GPIO_PIN(PA, 20),
    .reset_pin = GPIO_PIN(PB, 15)
};

static void _irq_handler(void *arg)
{
//    if (dev->event_callback) {
//        dev->event_callback(dev, NETDEV_EVENT_ISR);
//    }
}

void at86rf2xx_setup(at86rf2xx_t *dev, const at86rf2xx_params_t *params)
{
    /* initialize device descriptor */
    memcpy(&dev->params, params, sizeof(at86rf2xx_params_t));
    dev->idle_state = AT86RF2XX_STATE_TRX_OFF;
    /* radio state is P_ON when first powered-on */
    dev->state = AT86RF2XX_STATE_P_ON;
    dev->pending_tx = 0;
}

int radio_init(void)
{
    at86rf2xx_setup(dev, &at86rf2xx_params);

    /* initialize GPIOs */
    spi_init_cs(dev->params.spi, dev->params.cs_pin);
    gpio_init(dev->params.sleep_pin, GPIO_OUT);
    gpio_clear(dev->params.sleep_pin);
    gpio_init(dev->params.reset_pin, GPIO_OUT);
    gpio_set(dev->params.reset_pin);
    gpio_init_int(dev->params.int_pin, GPIO_IN, GPIO_RISING, _irq_handler, dev);

    at86rf2xx_hardware_reset(dev);

    /* Reset state machine to ensure a known state */
    if (dev->state == AT86RF2XX_STATE_P_ON) {
        at86rf2xx_set_state(dev, AT86RF2XX_STATE_FORCE_TRX_OFF);
    }

    at86rf2xx_set_state(dev, AT86RF2XX_STATE_SLEEP);

    /* test if the SPI is set up correctly and the device is responding */
    if (at86rf2xx_reg_read(dev, AT86RF2XX_REG__PART_NUM) !=
        AT86RF2XX_PARTNUM) {
        return -1;
    }

    //leds_on(LEDS_ALL);
        
    return 0;
}
