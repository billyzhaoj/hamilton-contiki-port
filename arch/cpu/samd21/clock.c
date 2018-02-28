#include <stdio.h>
#include "contiki.h"
#include "clock.h"
#include "headers/samr21e18a.h"

static volatile unsigned long seconds;
static volatile clock_time_t ticks;


#define RELOAD_VALUE		((F_CPU / CLOCK_CONF_SECOND) - 1)

void
clock_init(void)
{
  SysTick_Config(.1);
}

clock_time_t
clock_time(void)
{
  return ticks;
}

unsigned long
clock_seconds(void)
{
  return seconds;
}

void
clock_delay(unsigned int delay)
{
  for(; delay>0; delay--)
  {
    unsigned int j;
    for(j = 50; j>0; j--)
    {
      //__NOP();
    }
  }
}

void
clock_wait(clock_time_t delay)
{
  clock_time_t start;
  start = clock_time();
  while(clock_time() - start < delay);
}
