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
/**
 * \addtogroup samd21
 * @{
 *
 * \defgroup samd21-gclk samd21 Generic Clock Controller
 *
 * Driver for the samd21 Generic Clock Controller
 * @{
 *
 * \file
 * Header file for the samd21 Generic Clock Controller
 */
#ifndef GCLK_H_
#define GCLK_H_

/*---------------------------------------------------------------------------*/
/** \name Control bit masks
 * @{
 */

#define GCLK_CTRL_SWRST      0x01 /**< There is a reset operation ongoing */
#define GCLK_CTRL_RESET      0x00 /**< Default state, no reset in progress */
#define GCLK_CTRL_OFFSET    0x0 /**< Address offset */

/** @} */

/*---------------------------------------------------------------------------*/
/** \name Status bit masks
 * @{
 */

#define GCLK_STATUS_SYNCBUSY    0x80 /**< Set when synchronization of registers between clock domains is started, cleared when complete */
#define GCLK_STATUS_RESET      0x00 /**< Default state, no reset in progress */
#define GCLK_STATUS_OFFSET      0x1 /**< Address offset */

/** @} */

/*---------------------------------------------------------------------------*/
/** \name Generic clock control bit masks
 * @{
 */

#define GCLK_CLKCTRL_WRTLOCK          0x8000 /**< Lock generated clock, generator, and division factor*/
#define GCLK_CLKCTRL_CLKEN            0x4000 /**< Generic clock enabled*/
#define GCLK_CLKCTRL_GCLKGEN0          0x0000 /**< Generic clock generator 0 */
#define GCLK_CLKCTRL_GCLKGEN1          0x0100 /**< Generic clock generator 1 */
#define GCLK_CLKCTRL_GCLKGEN2          0x0200 /**< Generic clock generator 2 */
#define GCLK_CLKCTRL_GCLKGEN3          0x0300 /**< Generic clock generator 3 */
#define GCLK_CLKCTRL_GCLKGEN4          0x0400 /**< Generic clock generator 4 */
#define GCLK_CLKCTRL_GCLKGEN5          0x0500 /**< Generic clock generator 5 */
#define GCLK_CLKCTRL_GCLKGEN6          0x0600 /**< Generic clock generator 6 */
#define GCLK_CLKCTRL_GCLKGEN7          0x0700 /**< Generic clock generator 7 */
#define GCLK_CLKCTRL_GCLKGEN8          0x0800 /**< Generic clock generator 8 */
#define GCLK_CLKCTRL_GCLK_DFLL48M_REF      0x0000 /**< DFLL48M Reference*/
#define GCLK_CLKCTRL_GCLK_DPLL          0x0001 /**< FDPLL96M input clock source for reference*/
#define GCLK_CLKCTRL_GCLK_DPLL_32K        0x0002 /**< FDPLL96M 32kHz cloock for FDPLL96M internal lock timer*/
#define GCLK_CLKCTRL_GCLK_WDT          0x0003 /**< WDT */
#define GCLK_CLKCTRL_GCLK_RTC          0x0004 /**< RTC */
#define GCLK_CLKCTRL_GCLK_EIC          0x0005 /**< EIC */
#define GCLK_CLKCTRL_GCLK_USB          0x0006 /**< USB */
#define GCLK_CLKCTRL_GCLK_EVSYS_CHANNEL_0    0x0007 /**< EVSYS_CHANNEL_0 */
#define GCLK_CLKCTRL_GCLK_EVSYS_CHANNEL_1    0x0008 /**< EVSYS_CHANNEL_1 */
#define GCLK_CLKCTRL_GCLK_EVSYS_CHANNEL_2    0x0009 /**< EVSYS_CHANNEL_2 */
#define GCLK_CLKCTRL_GCLK_EVSYS_CHANNEL_3    0x000A /**< EVSYS_CHANNEL_3 */
#define GCLK_CLKCTRL_GCLK_EVSYS_CHANNEL_4    0x000B /**< EVSYS_CHANNEL_4 */
#define GCLK_CLKCTRL_GCLK_EVSYS_CHANNEL_5    0x000C /**< EVSYS_CHANNEL_5 */
#define GCLK_CLKCTRL_GCLK_EVSYS_CHANNEL_6    0x000D /**< EVSYS_CHANNEL_6 */
#define GCLK_CLKCTRL_GCLK_EVSYS_CHANNEL_7    0x000E /**< EVSYS_CHANNEL_7 */
#define GCLK_CLKCTRL_GCLK_EVSYS_CHANNEL_8    0x000F /**< EVSYS_CHANNEL_8 */
#define GCLK_CLKCTRL_GCLK_EVSYS_CHANNEL_9    0x0010 /**< EVSYS_CHANNEL_9 */
#define GCLK_CLKCTRL_GCLK_EVSYS_CHANNEL_10    0x0011 /**< EVSYS_CHANNEL_10 */
#define GCLK_CLKCTRL_GCLK_EVSYS_CHANNEL_11    0x0012 /**< EVSYS_CHANNEL_11 */
#define GCLK_CLKCTRL_GCLK_SERCOMx_SLOW      0x0013 /**< SERCOMx_SLOW */
#define GCLK_CLKCTRL_GCLK_SERCOM0_CORE      0x0014 /**< SERCOM0_CORE */
#define GCLK_CLKCTRL_GCLK_SERCOM1_CORE      0x0015 /**< SERCOM1_CORE */
#define GCLK_CLKCTRL_GCLK_SERCOM2_CORE      0x0016 /**< SERCOM2_CORE */
#define GCLK_CLKCTRL_GCLK_SERCOM3_CORE      0x0017 /**< SERCOM3_CORE */
#define GCLK_CLKCTRL_GCLK_SERCOM4_CORE      0x0018 /**< SERCOM4_CORE */
#define GCLK_CLKCTRL_GCLK_SERCOM5_CORE      0x0019 /**< SERCOM5_CORE */
#define GCLK_CLKCTRL_GCLK_TCC0          0x001A /**< TCC0 */
#define GCLK_CLKCTRL_GCLK_TCC1          0x001A /**< TCC1 */
#define GCLK_CLKCTRL_GCLK_TCC2          0x001B /**< TCC2 */
#define GCLK_CLKCTRL_GCLK_TC3          0x001B /**< TC3 */
#define GCLK_CLKCTRL_GCLK_TC4          0x001C /**< TC4 */
#define GCLK_CLKCTRL_GCLK_TC5          0x001C /**< TC5 */
#define GCLK_CLKCTRL_GCLK_ADC          0x001E /**< ADC */
#define GCLK_CLKCTRL_GCLK_AC_DIG        0x001F /**< AC_DIG */
#define GCLK_CLKCTRL_GCLK_AC_ANA        0x0020 /**< AC_ANA */
#define GCLK_CLKCTRL_GCLK_PTC          0x0022 /**< PTCReserved */
#define GCLK_CLKCTRL_RESET            0x0000 /**< Default state after reset */
#define GCLK_CLKCTRL_OFFSET            0x01 /**< Address offset */

/** @} */

/*---------------------------------------------------------------------------*/
/** \name Generic clock generator control bit masks
 * @{
 */

#define GCLK_GENCTRL_RUNSTDBY        0x00200000 /**< Generic clock generator is kept running and output dedicated to GCLK_IO pin during standby mode */
#define GCLK_GENCTRL_DIVSEL          0x00100000 /**< Generic clock generator = clock source / 2^(GENDIV.DIV+1), if not set: generator = clock source / GENDIV.DIV */
#define GCLK_GENCTRL_OE            0x00080000 /**< Generic clock generator is output to corresponding GCLK_IO, unless corresponding GCLK_IO is selected as a rouce in the GENCLK.SRC */
#define GCLK_GENCTRL_OOV          0x00040000 /**< GCLK_IO will be one when the generic clock generator is turned off or when OE bit is zero */
#define GCLK_GENCTRL_IDC          0x00020000 /**< Generic clock generator duty cylce is 50/50 */
#define GCLK_GENCTRL_GENGEN          0x00010000 /**< Generic clock generator is enabled */
#define GCLK_GENCTRL_XOSC          0x00000000 /**< XOSC oscillator output*/
#define GCLK_GENCTRL_GCLKIN          0x00000100 /**< Generator input pad*/
#define GCLK_GENCTRL_GCLKGEN1        0x00000200 /**< Generic clock generator 1 output */
#define GCLK_GENCTRL_OSCULP32K        0x00000300 /**< OSCULP32K oscillator output */
#define GCLK_GENCTRL_OSC32K          0x00000400 /**< OSC32K osciallator output */
#define GCLK_GENCTRL_XOSC32K        0x00000500 /**< XOSC32K oscillator output */
#define GCLK_GENCTRL_OSC8M          0x00000600 /**< OSC8M oscillator output */
#define GCLK_GENCTRL_DFLL48M        0x00000700 /**< DFLL48M output */
#define GCLK_GENCTRL_FDPLL96M        0x00000800 /**< FDPLL96M output */
#define GCLK_GENCTRL_GCLKGEN0        0x00000000 /**< Generic clock generator 0 */
#define GCLK_GENCTRL_GCLKGEN1        0x00000001 /**< Generic clock generator 1 */
#define GCLK_GENCTRL_GCLKGEN2        0x00000002 /**< Generic clock generator 2 */
#define GCLK_GENCTRL_GCLKGEN3        0x00000003 /**< Generic clock generator 3 */
#define GCLK_GENCTRL_GCLKGEN4        0x00000004 /**< Generic clock generator 4 */
#define GCLK_GENCTRL_GCLKGEN5        0x00000005 /**< Generic clock generator 5 */
#define GCLK_GENCTRL_GCLKGEN6        0x00000006 /**< Generic clock generator 6 */
#define GCLK_GENCTRL_GCLKGEN7        0x00000007 /**< Generic clock generator 7 */
#define GCLK_GENCTRL_GCLKGEN8        0x00000008 /**< Generic clock generator 8 */
#define GCLK_GENCTRL_RESET          0x00000000 /**< Default state after reset */
#define GCLK_GENCTRL_OFFSET          0x4 /**< Address offset */

/** @} */


/*---------------------------------------------------------------------------*/
/** \name Generic clock generator control bit masks
 * @{
 */

#define GCLK_GENDIV_GCLKGEN0        0x00000000 /**< Generic clock generator 0 */
#define GCLK_GENDIV_GCLKGEN1        0x00000000 /**< Generic clock generator 1 */
#define GCLK_GENDIV_GCLKGEN2        0x00000000 /**< Generic clock generator 2 */
#define GCLK_GENDIV_GCLKGEN3        0x00000000 /**< Generic clock generator 3 */
#define GCLK_GENDIV_GCLKGEN4        0x00000000 /**< Generic clock generator 4 */
#define GCLK_GENDIV_GCLKGEN5        0x00000000 /**< Generic clock generator 5 */
#define GCLK_GENDIV_GCLKGEN6        0x00000000 /**< Generic clock generator 6 */
#define GCLK_GENDIV_GCLKGEN7        0x00000000 /**< Generic clock generator 7 */
#define GCLK_GENDIV_GCLKGEN8        0x00000000 /**< Generic clock generator 8 */
#define GCLK_GENDIV_RESET          0x00000000 /**< Default state after reset */
#define GCLK_GENDIV_OFFSET          0x8 /**< Address offset */

/** @} */


#endif /* GCLK_H_ */

/**
 * @}
 * @}
 */
