#include "contiki.h"
#include "dev/leds.h"
#include "sys/platform.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/select.h>
#include <errno.h>
#include "headers/samr21e18a.h"

#define SELECT_MAX 8 
void
platform_init_stage_one(void)
{

}

void
platform_init_stage_two(void)
{

}

void
platform_init_stage_three(void)
{
    leds_arch_init();
}

enum system_sleepmode {
    /**
     *  Idle 0 mode.
     *  Potential Wake Up sources: Synchronous(APB, AHB), asynchronous.
     */
    SYSTEM_SLEEPMODE_IDLE_0,
    /**
     *  Idle 1 mode.
     *  Potential Wake Up sources: Synchronous (APB), asynchronous
     */
    SYSTEM_SLEEPMODE_IDLE_1,
    /**
     *  Idle 2 mode.
     *  Potential Wake Up sources: Asynchronous
     */
    SYSTEM_SLEEPMODE_IDLE_2,
    /**
     * Standby mode.
     * Potential Wake Up sources: Asynchronous
     */
    SYSTEM_SLEEPMODE_STANDBY,
};


static uint8_t
irq_disable(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    return mask;
}
void irq_restore(unsigned int state)
{
    __set_PRIMASK(state);
}
void
platform_idle(void)
{
    PM->SLEEP.reg = SYSTEM_SLEEPMODE_IDLE_0;
    /* Sleeping */
    SCB->SCR &= ~(SCB_SCR_SLEEPDEEP_Msk);
    /* IRQ Disable */
    unsigned state = irq_disable() ;
    __DSB();
    __WFI();
    irq_restore(state);
}

void
platform_process_args(int argc, char** argv)
{
}

void
platform_main_loop(void)
{
}

