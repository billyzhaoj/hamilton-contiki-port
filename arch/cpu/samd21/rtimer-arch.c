/*
 * Copyright (c) 2017, Bradley Cage, cage.bradley(at)berkeley.edu
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

#include "contiki.h"
#include "asf_headers/samr21e18a.h"
#include "os/sys/rtimer.h"


static volatile rtimer_clock_t curtime;
static volatile rtimer_clock_t next_trigger;
static volatile rtimer_clock_t next_tick;


void
rtimer_arch_init(void)
{
     /* configure GCLK1 (configured to 1MHz) to feed TC3, TC4 and TC5 */
    //GCLK->CLKCTRL.reg = (uint16_t)((GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK1 | (TC4_GCLK_ID << GCLK_CLKCTRL_ID_Pos)));
     /* configure GCLK2 as the source (32kHz)*/
    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK2 | GCLK_CLKCTRL_ID(RTC_GCLK_ID));
    while (GCLK->STATUS.bit.SYNCBUSY) {}
    //switch(dev)
    /*Enable Timer2*/
    PM->APBAMASK.reg |= PM_APBAMASK_RTC;
    /* reset timer */
    TIMER_2_DEV.CTRL.bit.SWRST = 1;
    while (TIMER_2_DEV.CTRL.bit.SWRST) {}
    /* Configure RTT as 32bit counter with no prescaler (32.768kHz) no clear on match compare */
    TIMER_2_DEV.CTRL.reg = (RTC_MODE0_CTRL_MODE_COUNT32 | RTC_MODE0_CTRL_PRESCALER_DIV1);
    while (GCLK->STATUS.bit.SYNCBUSY) {}
    /* Timer2 init */
    NVIC_EnableIRQ(RTC_IRQn);
    /* Timer2 Start */
    TIMER_2_DEV.CTRL.bit.ENABLE = 1;
    while (GCLK->STATUS.bit.SYNCBUSY) {}
}

void
rtimer_arch_pseudo_schedule(void)
{
    TIMER_2_DEV.INTFLAG.reg |= RTC_MODE0_INTFLAG_CMP0;
    TIMER_2_DEV.COMP[0].reg = ( curtime < next_trigger 
            && next_trigger < next_tick) ? next_trigger : next_tick;
    TIMER_2_DEV.INTENSET.bit.CMP0 = 1;
}

void
rtimer_arch_schedule(rtimer_clock_t t)
{
    next_trigger = t;
    rtimer_arch_pseudo_schedule();
}

rtimer_clock_t
rtimer_arch_next_trigger(void)
{
	return next_trigger;
}

rtimer_clock_t
rtimer_arch_now(void)
{
    TIMER_2_DEV.READREQ.reg = RTC_READREQ_RREQ | RTC_READREQ_ADDR(0x10);
    while (TIMER_2_DEV.STATUS.bit.SYNCBUSY) {}
    return TIMER_2_DEV.COUNT.reg; 
}

void
RTC_Handler(void)
{
    if ( TIMER_2_DEV.INTFLAG.bit.CMP0 && TIMER_2_DEV.INTENSET.bit.CMP0 ) {
        TIMER_2_DEV.INTFLAG.reg |= RTC_MODE0_INTFLAG_CMP0;
        TIMER_2_DEV.INTENCLR.reg = RTC_MODE0_INTENCLR_CMP0;
    }
    if ( TIMER_2_DEV.INTFLAG.bit.OVF && TIMER_2_DEV.INTENSET.bit.OVF ) {
        TIMER_2_DEV.INTFLAG.reg |= RTC_MODE0_INTFLAG_OVF;
        TIMER_2_DEV.INTENCLR.reg = RTC_MODE0_INTENCLR_OVF;
    }

    update_ticks();
    curtime = RTIMER_NOW();
    next_tick= curtime + 32768/CLOCK_CONF_SECOND;
    rtimer_arch_pseudo_schedule();
    rtimer_run_next();
}
