/**
 * @{
 * @file
 * @brief       Contiki configuration for Hamilton
 * @author      Bradley Cage <cage.bradley@berkeley.edu>
 * @}
 */

#ifndef __PLATFORM_CONF_H__
#define __PLATFORM_CONF_H__

//typedef unsigned long clock_time_t;
//typedef unsigned long long rtimer_clock_t;

/**
 * Using the internal oscillator at a frequency of 8MHz, core frequency
 * is adjusted according as below:
 *
 * F_CPU = 8MHz / DIV
 */

/* Adjust value as needed for clock division */
#define CLOCK_DIV		(8U)
/* Generate core clock frequency */
#define F_CPU			(8000000 / CLOCK_DIV)

#define PLATFORM_HAS_LEDS	1
#define PLATFORM_HAS_BUTTON	1


#endif /* __PLATFORM_CONF_H__ */
