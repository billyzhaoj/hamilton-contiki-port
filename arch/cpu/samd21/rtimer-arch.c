#include "contiki.h"

#include <stdint.h>

static volatile rtimer_clock_t next_trigger;


void
rtimer_arch_init(void)
{
	return;
}

void
rtimer_arch_schedule(rtimer_clock_t t)
{
	rtimer_clock_t now;
	now = RTIMER_NOW();

	if((int32_t)(t-now) < 7)
	{
		t = now + 7;
	}

	next_trigger = t;
}

rtimer_clock_t
rtimer_arch_next_trigger(void)
{
	return next_trigger;
}

rtimer_clock_t
rtimer_clock_now(void)
{
	return next_triggers;
}

void
rtimer_isr(void)
{
	return;
}