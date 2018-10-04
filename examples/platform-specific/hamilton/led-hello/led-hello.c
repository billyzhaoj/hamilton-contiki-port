/* This is a very simple hello_world program.
 * It aims to demonstrate the co-existence of two processes:
 * One of them prints a hello world message and the other blinks the LEDs
 *
 * It is largely based on hello_world of the original sensinode port
 *
 * Author: George Oikonomou - <oikonomou@users.sourceforge.net>
 */

#include "contiki.h"
#include "dev/leds.h"

#include <stdio.h> /* For printf() */
/*---------------------------------------------------------------------------*/
//static struct etimer et_hello;
//static struct etimer et_blink;
//static uint16_t count;
// static uint8_t blinks;
/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process,
"Hello world process");
// PROCESS(blink_process, "LED blink process");
// AUTOSTART_PROCESSES(&hello_world_process, &blink_process);
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/
//static struct rtimer rt;
rtimer_clock_t rt_for, rt_now;

void
rt_callback(struct rtimer *t, void *ptr) {
  rt_now = RTIMER_NOW();

  leds_on(LEDS_ALL);
}


PROCESS_THREAD(hello_world_process, ev, data
)
{
PROCESS_BEGIN();
//  count = 0;

//int i = 0;
while (1) {
}

PROCESS_END();
}