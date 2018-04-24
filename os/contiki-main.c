/*
 * Copyright (c) 2017, George Oikonomou - http://www.spd.gr
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup main
 * @{
 */
/*---------------------------------------------------------------------------*/
/**
 * \file
 *
 * Implementation of \os's main routine
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "contiki-net.h"
#include "sys/platform.h"
#include "sys/energest.h"
#include "dev/watchdog.h"
#include "dev/leds.h"
#include "dev/gpio.h"

/**
 * @name LED pin definitions
 * @{
 */
#define LED_PORT            PORT->Group[0]
#define LED_PIN             (19)

#define LED_ON       (LED_PORT.OUTSET.reg = (1 << LED_PIN))
#define LED_OFF      (LED_PORT.OUTCLR.reg = (1 << LED_PIN))

#if BUILD_WITH_ORCHESTRA
#include "os/services/orchestra/orchestra.h"
#endif /* BUILD_WITH_ORCHESTRA */
#if BUILD_WITH_SHELL
#include "os/services/shell/serial-shell.h"
#endif /* BUILD_WITH_SHELL */

#include <stdio.h>
#include <stdint.h>
/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "Main"
#define LOG_LEVEL LOG_LEVEL_MAIN

/* hskim: for measurement of context switch time */
/*PROCESS(temp_process, "temp process");
AUTOSTART_PROCESSES(&temp_process);

PROCESS(temp2_process, "temp2 process");
AUTOSTART_PROCESSES(&temp2_process);

PROCESS_THREAD(temp_process, ev, data)
{
  PROCESS_BEGIN();
  while(1) {
    LED_OFF;
    while(process_post(&temp2_process, PROCESS_EVENT_TIMER, NULL) != PROCESS_ERR_OK);    
    PROCESS_YIELD();
    
  }

  PROCESS_END();
}

PROCESS_THREAD(temp2_process, ev, data)
{
  PROCESS_BEGIN();
  process_start(&temp_process, NULL);
  PROCESS_YIELD();

  while(1) {
    LED_ON;
    while(process_post(&temp_process, PROCESS_EVENT_TIMER, NULL) != PROCESS_ERR_OK);
    PROCESS_YIELD();
    
  }

  PROCESS_END();
}*/

/*---------------------------------------------------------------------------*/
int
#if PLATFORM_MAIN_ACCEPTS_ARGS
main(int argc, char **argv)
{
  platform_process_args(argc, argv);
#else
main(void)
{
#endif
  platform_init_stage_one();

  clock_init();
  rtimer_init();
  process_init();
  process_start(&etimer_process, NULL);
  ctimer_init();
  watchdog_init();

  energest_init();

  platform_init_stage_two();

  platform_init_stage_three();

  autostart_start(autostart_processes);
  /* hskim: for measurement of context switch time */
  //process_start(&temp2_process, NULL);
  watchdog_start();

#if PLATFORM_PROVIDES_MAIN_LOOP
  platform_main_loop();
#else
  while(1) {
    uint8_t r;
    do {
      r = process_run();
      watchdog_periodic();
    } while(r > 0);
    platform_idle();
  }
#endif

  return 0;
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 */
