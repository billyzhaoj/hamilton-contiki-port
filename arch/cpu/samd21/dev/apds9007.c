/*
 * Copyright (C) 2017 Hyung-Sin Kim
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/**
 * @ingroup     drivers_apds9007
 * @{
 *
 * @file
 * @brief       Driver for the APDS9007 Light Sensor.
 *
 * @author      Hyung-Sin Kim <hs.kim@berkeley.edu>
 *
 * @}
 */

#include <string.h>

#include "dev/apds9007.h"

#define APDS9007_PIN    GPIO_PIN(PA,28)
#define APDS9007_ADC    ADC_PIN_PA08
#define APDS9007_RES    ADC_RES_16BIT
#define APDS9007_STABILIZATION_TIME 20000UL

int apds9007_set_active(void) {
    gpio_write(APDS9007_PIN, 0);
    return 0;
} 

int apds9007_set_idle(void) {
    gpio_write(APDS9007_PIN, 1);
    return 0;
}

int apds9007_init(void) {
    gpio_init(APDS9007_PIN, GPIO_OUT);
    adc_init(APDS9007_ADC);
    apds9007_set_idle();
    return 0;
}
