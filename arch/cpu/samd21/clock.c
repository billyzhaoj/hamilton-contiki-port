#include <stdio.h>
#include "contiki.h"
#include "clock.h"
#include "headers/samr21e18a.h"
#include "component/pm.h"
#include "sys/rtimer.h"
#include "sys/etimer.h"


#include "dev/leds.h"
static volatile unsigned long seconds;
static volatile clock_time_t ticks;

#define CLOCK_CORECLOCK     (48000000U)
#define WAITSTATES          ((CLOCK_CORECLOCK - 1) / 24000000)
#define CLOCK_OSCULP32K      32768U
#define SYSTICK_PERIOD      CLOCK_CORECLOCK / CLOCK_CONF_SECOND

static volatile uint64_t rt_ticks_startup = 0, rt_ticks_epoch = 0;
void
clock_init(void)
{
  //Configure clock for systick period	
  SysTick_Config(SYSTICK_PERIOD);
  // enable clocks for the power, sysctrl and gclk modules 
  PM->APBAMASK.reg = (PM_APBAMASK_PM | PM_APBAMASK_SYSCTRL | PM_APBAMASK_GCLK); 
  PM->APBBMASK.reg |= PM_APBBMASK_NVMCTRL; 
  NVMCTRL->CTRLB.reg |= NVMCTRL_CTRLB_RWS(WAITSTATES);
  PM->APBBMASK.reg &= ~PM_APBBMASK_NVMCTRL; 
  //ULP32_DFLL
      /* reset the GCLK module so it is in a known state */
  GCLK->CTRL.reg = GCLK_CTRL_SWRST;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}

  /* Setup clock GCLK3 with divider 1 */
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(3) | GCLK_GENDIV_DIV(1);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}

  /* Enable GCLK3 with OSCULP32K as source */
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(3) |
                      GCLK_GENCTRL_GENEN |
                      GCLK_GENCTRL_SRC_OSCULP32K;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}

  /* set GCLK3 as source for DFLL */
  GCLK->CLKCTRL.reg = (GCLK_CLKCTRL_GEN_GCLK3 |
                       GCLK_CLKCTRL_ID_DFLL48 |
                       GCLK_CLKCTRL_CLKEN);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}

  /* Disable ONDEMAND mode while writing configurations */
  SYSCTRL->DFLLCTRL.bit.ONDEMAND = 0;
  while ((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0) {
      /* Wait for DFLL sync */
  }

  /* get the coarse and fine values stored in NVM (Section 9.3) */
  uint32_t coarse = (*(uint32_t *)(0x806024) >> 26);  /* Bits 63:58 */
  uint32_t fine = (*(uint32_t *)(0x806028) & 0x3FF);  /* Bits 73:64 */

  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_CSTEP(coarse >> 1) |
                         SYSCTRL_DFLLMUL_FSTEP(fine >> 1) |
                         SYSCTRL_DFLLMUL_MUL(CLOCK_CORECLOCK / CLOCK_OSCULP32K);
  SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE(coarse) |
                         SYSCTRL_DFLLVAL_FINE(fine);
  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_MODE;
  while ((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0) {
      /* Wait for DFLL sync */
  }

  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE;
  while ((SYSCTRL->PCLKSR.reg & (SYSCTRL_PCLKSR_DFLLRDY |
                                 SYSCTRL_PCLKSR_DFLLLCKF |
                                 SYSCTRL_PCLKSR_DFLLLCKC)) == 0) {
      /* Wait for DFLLLXXX sync */
  }

  /* select the DFLL as source for clock generator 0 (CPU core clock) */
  GCLK->GENDIV.reg =  (GCLK_GENDIV_DIV(1U) | GCLK_GENDIV_ID(0));
  GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(0));
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}


//  /* setup generic clock 1 as 8MHz for timer.c */
//  GCLK->GENDIV.reg = (GCLK_GENDIV_DIV(CLOCK_CORECLOCK / 8000000ul) |
//                      GCLK_GENDIV_ID(1));
//  GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN |
//                       GCLK_GENCTRL_SRC_DFLL48M |
//                       GCLK_GENCTRL_ID(1));
//


  /* OSC8M is turned on by default and feeds GCLK0.
   * OSC8M must be disabled after another oscillator is set 
   * to feed GCLK0 
   */
  SYSCTRL->OSC8M.bit.ENABLE = 0;
  while ((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_OSC8MRDY)) {}
  /* make sure we synchronize clock generator 0 before we go on */ 

  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}


  /* Setup Clock generator 2 with divider 1 (32.768kHz) */
  GCLK->GENDIV.reg  = (GCLK_GENDIV_ID(2)  | GCLK_GENDIV_DIV(0));
  GCLK->GENCTRL.reg = (GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_GENEN |
          GCLK_GENCTRL_RUNSTDBY |
          GCLK_GENCTRL_SRC_OSCULP32K);

  while (GCLK->STATUS.bit.SYNCBUSY) {}

  /* redirect all peripherals to a disabled clock generator (7) by default */
  for (int i = 0x3; i <= 0x22; i++) {
      GCLK->CLKCTRL.reg = ( GCLK_CLKCTRL_ID(i) | GCLK_CLKCTRL_GEN_GCLK7 );
      while (GCLK->STATUS.bit.SYNCBUSY) {}
  }
}


void update_ticks(void)
{
  rtimer_clock_t now;
  uint64_t prev_rt_ticks_startup, cur_rt_ticks_startup;
  uint32_t cur_rt_ticks_startup_hi;

  now = RTIMER_NOW();
  prev_rt_ticks_startup = rt_ticks_startup;

  cur_rt_ticks_startup_hi = prev_rt_ticks_startup >> 32;
  if(now < (rtimer_clock_t)prev_rt_ticks_startup) {
    cur_rt_ticks_startup_hi++;
  }
  cur_rt_ticks_startup = (uint64_t)cur_rt_ticks_startup_hi << 32 | now;
  rt_ticks_startup = cur_rt_ticks_startup;

  rt_ticks_epoch += cur_rt_ticks_startup - prev_rt_ticks_startup;

  /*
   * Inform the etimer library that the system clock has changed and that an
   * etimer might have expired.
   */
  if(etimer_pending()) {
    etimer_request_poll();
  }



}


clock_time_t
clock_time(void)
{
  return rt_ticks_startup / (CLOCK_OSCULP32K / CLOCK_CONF_SECOND ); 
}

unsigned long
clock_seconds(void)
{
  return rt_ticks_epoch /( CLOCK_OSCULP32K/CLOCK_CONF_SECOND);
}
//TODO: Change
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

void
SysTick_Handler(void) {
    update_ticks();
}
