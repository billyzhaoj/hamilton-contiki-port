/*
 * Copyright (C) 2017 UC Berkeley
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/**
 * @ingroup     drivers_pulse_counter
 * @{
 *
 * @file
 * @brief       Driver for the PULSE COUNTER.
 *
 * @author      Hyung-Sin Kim <hs.kim@cs.berkeley.edu>
 *
 * @}
 */

#include <string.h>
#include "dev/button.h"

#define BUTTON_PIN    GPIO_PIN(PA, 18)

uint16_t pulse_count = 0;

/* Accumulate pulse count */
void pulse_counter_trigger(void *arg) {
  pulse_count++;
}

/* Initialize pulse counter */
int button_init(void) {
  pulse_count = 0;
  if (gpio_init_int(BUTTON_PIN, GPIO_IN_PU, GPIO_FALLING, pulse_counter_trigger, NULL)) {
    return -1;
  }
  return 0;
}

/* Return the accumulated pulse counts and reset the count to zero */
int16_t button_read(void) {
  int16_t pulse_count_output = pulse_count;
  pulse_count = 0;
  return pulse_count_output;
}

