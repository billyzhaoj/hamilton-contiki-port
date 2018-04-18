/*
 * Copyright (C) 2017 UC Berkeley
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_pulse_counter sensor
 * @ingroup     drivers_sensors
 *
 * The connection between the MCU and the PULSE_COUNTER is based on the
 * GPIO-interface.
 *
 * @{
 *
 * @file
 * @brief       Driver for the PULSE_COUNTER sensor
 *
 * @author      Hyung-Sin <hs.kim@cs.berkeley.edu>
 */

#ifndef PULSE_COUNTER_H_
#define PULSE_COUNTER_H_

#include <stdint.h>
#include "dev/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Initialize an PULSE_COUNTER device
 *
 * @param[out] dev          device descriptor
 * @param[in] params        GPIO bus the device is connected to
 *
 * The valid address range is 0x1E - 0x1F depending on the configuration of the
 * address pins SA0 and SA1.
 *
 * @return                   0 on success
 * @return                  -1 on error
 * @return                  -2 on invalid address
 */
int button_init(void);

/**
 * @brief   Read PULSE_COUNTER value
 *
 * @return  Accumunlated pulse counts
 */
int16_t button_read(void);
#ifdef __cplusplus
}
#endif

/** @} */
#endif /* PULSE_COUNTER_ */
