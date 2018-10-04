#include "contiki.h"
#include "asf_headers/samr21e18a.h"
#include "dev/leds.h"
#include "dev/spi.h"
#include "dev/hdc1000.h"
#include "dev/tmp006.h"
#include "dev/fxos8700.h"
#include "dev/apds9007.h"
#include "dev/button.h"
#include "radio/at86rf233.h"
#include "sys/platform.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/select.h>
#include <errno.h>

#define SELECT_MAX 8

void
platform_init_stage_one(void) {
}

void
platform_init_stage_two(void) {
}

void
platform_init_stage_three(void) {
  leds_arch_init();
  apds9007_init();
  button_init();

  hdc1000_init();
  fxos8700_init();
  tmp006_init();
#ifdef SPI_NUMOF
  for (unsigned i = 0; i < SPI_NUMOF; i++) {
      spi_init(SPI_DEV(i));
  }
#endif
  //radio_init();
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
irq_disable(void) {
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  return mask;
}

void irq_restore(unsigned int state) {
  __set_PRIMASK(state);
}

void
platform_idle(void) {

  //PM->SLEEP.reg = SYSTEM_SLEEPMODE_IDLE_0;
  /* Sleeping */

  SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk);
  //SCB->SCR &= ~(SCB_SCR_SLEEPDEEP_Msk);
  /* IRQ Disable */
  unsigned state = irq_disable();
  __DSB();
  __WFI();
  irq_restore(state);
}

void
platform_process_args(int argc, char **argv) {
}

void
platform_main_loop(void) {
}

