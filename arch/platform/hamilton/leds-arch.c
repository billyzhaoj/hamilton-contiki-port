#include "contiki.h"
#include "dev/leds.h"
#include "dev/gpio.h"

/**
 * @name LED pin definitions
 * @{
 */
#define LED_PORT            PORT->Group[0]
#define LED_PIN             (19)
/** @} */

/**
 * Leds implementation
 */
void leds_arch_init(void)
{
    LED_PORT.DIRSET.reg = (1 << LED_PIN);
    LED_PORT.OUTCLR.reg = (1 << LED_PIN);
    LED_PORT.PINCFG[LED_PIN].bit.PULLEN = 0;
}

unsigned char leds_arch_get(void)
{
	return 0;//REG_H(GPIO_OUT+GPIO_OUT_OUT);
}

void leds_arch_set(unsigned char leds)
{ 
	if(leds > 0)
	{ 
        LED_PORT.OUTSET.reg = (1 << LED_PIN);
	}
	else
	{
        LED_PORT.OUTCLR.reg = (1 << LED_PIN);
	}
}
