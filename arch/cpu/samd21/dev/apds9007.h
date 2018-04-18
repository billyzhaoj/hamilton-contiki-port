/*
 * Copyright (C) 2017 Hyung-Sin Kim
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_apds9007 light sensor
 * @ingroup     drivers_sensors
 *
 * The connection between the MCU and the APDS9007 is based on the
 * GPIO-interface.
 *
 * @{
 *
 * @file
 * @brief       Driver for the APDS9007 light sensor
 *
 * @author      Hyung-Sin <hs.kim@berkeley.edu>
 */

#ifndef APDS9007_H_
#define APDS9007_H_

#include <stdint.h>
#include "dev/gpio.h"
#include "dev/adc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Initialize an APDS9007 device
 *
 * @param[out] dev          device descriptor
 * @param[in] params        I2C bus the device is connected to
 *
 * The valid address range is 0x1E - 0x1F depending on the configuration of the
 * address pins SA0 and SA1.
 *
 * @return                   0 on success
 * @return                  -1 on error
 * @return                  -2 on invalid address
 */
int apds9007_init(void);

int apds9007_set_active(void);

int apds9007_set_idle(void);

#ifdef __cplusplus
}
#endif

/** @} */
#endif /* APDS9007_H_ */
