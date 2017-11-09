#include <stdio.h>
#include "contiki.h"

static volatile unsigned long seconds;
static volatile clock_time_t ticks;

#define RELOAD_VALUE		((F_CPU / CLOCK_CONF_SECOND) - 1)

void
SysTick_Handler(void)
{
  ticks++;
  if((ticks % CLOCK_SECOND) == 0)
  {
    seconds++;
  }
}

void
clock_init(void)
{
  ticks = 0;
  seconds = 0;

/* TBH, this may not be the right config */
// SysTick_Config(RELOAD_VALUE); /* may work better */

  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

  SysTick_SetReload(RELOAD_VALUE);

  SysTick_ITConfig(ENABLE);

  SysTick_CounterCmd(SysTick_Counter_Enable);
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
      __NOP();
    }
  }
}

void
clock_wait(int delay)
{
  clock_time_t start;
  start = clock_time();
  while(clock_time() - start < delay);
}



