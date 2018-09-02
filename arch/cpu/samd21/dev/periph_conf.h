/*
 * Copyright (C) 2016 University of California, Berkeley
 * Copyright (C) 2014-2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_hamilton
 * @{
 *
 * @file
 * @brief       Configuration of CPU peripherals for the Hamilton mote
 *
 * @author      Michael Andersen <m.andersen@berkeley.edu>
 * @author      Hyung-Sin Kim <hs.kim@berkeley.edu>
 */

#ifndef PERIPH_CONF_H_
#define PERIPH_CONF_H_

#include <stdint.h>

#include "periph_cpu.h"
#include "asf_headers/samr21e18a.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CLOCK_CORECLOCK     (48000000U)
#define WAITSTATES          ((CLOCK_CORECLOCK - 1) / 24000000)
#define CLOCK_OSCULP32K      32768U
#define SYSTICK_PERIOD      CLOCK_CORECLOCK / CLOCK_CONF_SECOND

/**
 * @name ADC configuration
 * @{
 */
#define ADC_NUMOF          (1U)
#define ADC_DEV             ADC

#define ADC_PIN(portg, pin, chan) (((portg)<<16) | ((pin) << 8) | (chan))
#define ADC_GET_PIN(channel) (((channel) >> 8) & 0xff)
#define ADC_GET_PORT_GROUP(channel) (&(PORT->Group[((channel)>>16) & 0xFF]))
#define ADC_GET_CHANNEL(channel) ((channel) & 0xff)

#define ADC_PIN_PA06        ADC_PIN(0, 6, 6)
#define ADC_PIN_PA07        ADC_PIN(0, 7, 7)
#define ADC_PIN_PA08        ADC_PIN(0, 8, 16)
/** @} */

/**
 * @name PWM configuration
 * @{
 */
#define PWM_NUMOF           (PWM_0_EN + PWM_1_EN)
#define PWM_0_EN            1
#define PWM_1_EN            1
#define PWM_MAX_CHANNELS    2
/* for compatibility with test application */
#define PWM_0_CHANNELS      PWM_MAX_CHANNELS
#define PWM_1_CHANNELS      PWM_MAX_CHANNELS

/* PWM device configuration */
#if PWM_NUMOF
static const pwm_conf_t pwm_config[] = {
#if PWM_0_EN
        {TCC1, {
                /* GPIO pin, MUX value, TCC channel */
                {GPIO_PIN(PA, 6), GPIO_MUX_E, 0},
                {GPIO_PIN(PA, 7), GPIO_MUX_E, 1}
        }},
#endif
#if PWM_1_EN
        {TCC0, {
                /* GPIO pin, MUX value, TCC channel */
                {GPIO_PIN(PA, 18), GPIO_MUX_F, 2},
                {GPIO_PIN(PA, 19), GPIO_MUX_F, 3}
        }},
#endif
};
#endif
/** @} */

/**
 * @name SPI configuration
 * @{
 */
static const spi_conf_t spi_config[] = {
        {
                .dev      = &SERCOM4->SPI,
                .miso_pin = GPIO_PIN(PC, 19),
                .mosi_pin = GPIO_PIN(PB, 30),
                .clk_pin  = GPIO_PIN(PC, 18),
                .miso_mux = GPIO_MUX_F,
                .mosi_mux = GPIO_MUX_F,
                .clk_mux  = GPIO_MUX_F,
                .miso_pad = SPI_PAD_MISO_0,
                .mosi_pad = SPI_PAD_MOSI_2_SCK_3
        }
};

#define SPI_NUMOF           (sizeof(spi_config) / sizeof(spi_config[0]))
/** @} */

/**
 * @name I2C configuration
 * @{
 */
#define I2C_NUMOF          (1U)
#define I2C_0_EN            1
#define I2C_1_EN            0
#define I2C_2_EN            0
#define I2C_3_EN            0
#define I2C_IRQ_PRIO        1

#define I2C_0_DEV           SERCOM3->I2CM
#define I2C_0_IRQ           SERCOM3_IRQn
#define I2C_0_ISR           isr_sercom3
/* I2C 0 GCLK */
#define I2C_0_GCLK_ID       SERCOM3_GCLK_ID_CORE
#define I2C_0_GCLK_ID_SLOW  SERCOM3_GCLK_ID_SLOW
/* I2C 0 pin configuration */
#define I2C_0_SDA           GPIO_PIN(PA, 16)
#define I2C_0_SCL           GPIO_PIN(PA, 17)
#define I2C_0_MUX           GPIO_MUX_D
/** @} */


/**
 * @name Sensor configuration
 * @{
 */
#define EKMB_PARAMS_BOARD    { .gpio = GPIO_PIN(PA,6 ) }
/** @} */

/**
 * @name Random Number Generator configuration
 * @{
 */
#define RANDOM_NUMOF       (0U)
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H_ */
/** @} */
