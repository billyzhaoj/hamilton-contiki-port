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
#include "radio/at86rf233.h"
#include <stdio.h> /* For printf() */
/*---------------------------------------------------------------------------*/
//static struct etimer et_hello;
static struct etimer et_blink;
//static uint16_t count;
static uint8_t blinks;
/*---------------------------------------------------------------------------*/
//PROCESS(hello_world_process,
//"Hello world process");
PROCESS(blink_process,
"LED blink process");
AUTOSTART_PROCESSES(&blink_process);
//AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/
//static struct rtimer rt;
//rtimer_clock_t rt_for, rt_now;
//
//void
//rt_callback(struct rtimer *t, void *ptr) {
//  rt_now = RTIMER_NOW();
//
//  leds_on(LEDS_ALL);
//}
//
//
//PROCESS_THREAD(hello_world_process, ev, data
//)
//{
//PROCESS_BEGIN();
//uint8_t xd[10] = {255, 255, 0, 0 ,255, 255, 0, 0, 128, 128};
//
//while(1) {
//
//  send(xd,10);
//  leds_on(LEDS_ALL);
//  for(int i =0; i < 100000; i++);
//  leds_off(LEDS_ALL);
//  for(int i =0; i < 100000; i++);
//}
//
//PROCESS_END();
//}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(blink_process, ev, data
)
{
PROCESS_BEGIN();

blinks = 0;

while (1) {
etimer_set(&et_blink, CLOCK_SECOND);

PROCESS_WAIT_EVENT_UNTIL(ev
== PROCESS_EVENT_TIMER);
uint8_t xd[1] = {1};
if (blinks %10 == 0) {
send(xd,
1);
}
leds_off(LEDS_ALL);
leds_on(blinks
& LEDS_ALL);
blinks++;
//printf("Blink... (state %2d)\n", (int) leds_get());
}

PROCESS_END();
}
/*---------------------------------------------------------------------------*/
