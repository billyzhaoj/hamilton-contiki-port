#include "contiki.h"
#include "reg.h"
#include "dev/leds.h"
#include "dev/gpio.h"

#define LED_PIN		(19)

// unsigned char leds_get(void);
// void leds_set(unsigned char leds);
// void leds_on(unsigned char leds);
// void leds_off(unsigned char leds);
// void leds_toggle(unsigned char leds);

/**
 * Leds implementation
 */
void leds_arch_init(void)
{
	REG(GPIO_BASE + GPIO_DIRSET+GPIO_DIRCLR_DIRCLR);
	REG((GPIO_BASE + GPIO_DIRSET+GPIO_DIRSET_DIRSET)) |= GPIO_PIN_MASK(LED_PIN);

	REG((GPIO_BASE+GPIO_OUTTGL+GPIO_OUT_OUT)) |= GPIO_PIN_MASK(LED_PIN);

	REG((GPIO_BASE+GPIO_OUTCLR+GPIO_OUT_OUT)) |= GPIO_PIN_MASK(LED_PIN);
}
unsigned char leds_arch_get(void)
{
	return REG_H(GPIO_OUT+GPIO_OUT_OUT);
}

void leds_arch_set(unsigned char leds)
{ 

    //leds_arch_init();
	if(leds > 0)
	{ 
	    REG((GPIO_BASE+GPIO_OUTSET+GPIO_OUT_OUT)) |= GPIO_PIN_MASK(LED_PIN);
	}
	else
	{
	}
}
