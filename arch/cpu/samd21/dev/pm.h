/*
 * Copyright (c) 2017, Bradley Cage
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
 * \defgroup samd21-PM samd21 Power Manager 
 *
 * Driver for the samd21 Power Manager
 * @{
 *
 * \file
 * Header file for the samd21 Power Manager
 */
#ifndef PM_H_
#define PM_H_

/*---------------------------------------------------------------------------*/

/** \name PM Sleep bit values
 * @{
 */
#define PM_SLEEP_CPU			0x00 /**< CPU clock domain is stopped */
#define PM_SLEEP_AHB			0x01 /**< CPU and AHB clock domains are stopped */
#define PM_SLEEP_APB			0x02 /**< CPU, AHB, and APB clock domains are stopped */
#define PM_SLEEP_RESET			(PM_SLEEP_CPU) /**< Reset behaviour */
#define PM_SLEEP_OFFSET			0x02 /**< Address offset */

/** @} */
/*---------------------------------------------------------------------------*/

/** \name CPU Clock Select bit values
 * @{
 */
#define PM_CPUSEL_DIV1			0x00 /**< Divide by 1 */
#define PM_CPUSEL_DIV2			0x01 /**< Divide by 2 */
#define PM_CPUSEL_DIV4			0x02 /**< Divide by 4 */
#define PM_CPUSEL_DIV8			0x03 /**< Divide by 8 */
#define PM_CPUSEL_DIV16			0x04 /**< Divide by 16 */
#define PM_CPUSEL_DIV32			0x05 /**< Divide by 32 */
#define PM_CPUSEL_DIV64			0x06 /**< Divide by 64 */
#define PM_CPUSEL_DIV128		0x07 /**< Divide by 128*/
#define PM_CPUSEL_RESET			(PM_CPUSEL_DIV1) /**< Reset behaviour */
#define PM_CPUSEL_OFFSET		0x00 /**< Address offset */

/** @} */
/*---------------------------------------------------------------------------*/

/** \name APBA Clock Select bit values
 * @{
 */
#define PM_APBASEL_DIV1			0x00 /**< Divide by 1 */
#define PM_APBASEL_DIV2			0x01 /**< Divide by 2 */
#define PM_APBASEL_DIV4			0x02 /**< Divide by 4 */
#define PM_APBASEL_DIV8			0x03 /**< Divide by 8 */
#define PM_APBASEL_DIV16		0x04 /**< Divide by 16 */
#define PM_APBASEL_DIV32		0x05 /**< Divide by 32 */
#define PM_APBASEL_DIV64		0x06 /**< Divide by 64 */
#define PM_APBASEL_DIV128		0x07 /**< Divide by 128 */
#define PM_APBASEL_RESET		(PM_APBASEL_DIV1) /**< Reset behaviour */
#define PM_APBASEL_OFFSET		0x09 /**< Address offset */

/** @} */
/*---------------------------------------------------------------------------*/

/** \name APBB Clock Select bit values
 * @{
 */
#define PM_APBBSEL_DIV1			0x00 /**< Divide by 1 */
#define PM_APBBSEL_DIV2			0x01 /**< Divide by 2 */
#define PM_APBBSEL_DIV4			0x02 /**< Divide by 4 */
#define PM_APBBSEL_DIV8			0x03 /**< Divide by 8 */
#define PM_APBBSEL_DIV16		0x04 /**< Divide by 16 */
#define PM_APBBSEL_DIV32		0x05 /**< Divide by 32 */
#define PM_APBBSEL_DIV64		0x06 /**< Divide by 64 */
#define PM_APBBSEL_DIV128		0x07 /**< Divide by 128 */
#define PM_APBBSEL_RESET		(PM_APBBSEL_DIV1) /**< Reset behaviour */
#define PM_APBBSEL_OFFSET		0x0A /**< Address offset */

/** @} */
/*---------------------------------------------------------------------------*/

/** \name APBC Clock Select bit values
 * @{
 */
#define PM_APBCSEL_DIV1			0x00 /**< Divide by 1 */
#define PM_APBCSEL_DIV2			0x01 /**< Divide by 2 */
#define PM_APBCSEL_DIV4			0x02 /**< Divide by 4 */
#define PM_APBCSEL_DIV8			0x03 /**< Divide by 8 */
#define PM_APBCSEL_DIV16		0x04 /**< Divide by 16 */
#define PM_APBCSEL_DIV32		0x05 /**< Divide by 32 */
#define PM_APBCSEL_DIV64		0x06 /**< Divide by 64 */
#define PM_APBCSEL_DIV128		0x07 /**< Divide by 128 */
#define PM_APBCSEL_RESET		(PM_APBCSEL_DIV1) /**< Reset behaviour */
#define PM_APBCSEL_OFFSET		0x0B /**< Address offset */

/** @} */
/*---------------------------------------------------------------------------*/

/** \name AHB Mask bit masks
 * @{
 */

#define PM_AHB_USB 				0x00000040 /**< AHB clock for the USB is enabled */
#define PM_AHB_DMAC				0x00000020 /**< AHB clock for the DMAC is enabled */
#define PM_AHB_NVMCTRL			0x00000010 /**< AHB clock for the NVMCTRL is enabled */
#define PM_AHB_DSU				0x00000008 /**< AHB clock for the DSU is enabled */
#define PM_AHB_HPB2				0x00000004 /**< AHB clock for the HPB2 is enabled */
#define PM_AHB_HPB1				0x00000002 /**< AHB clock for the HPB1 is enabled */
#define PM_AHB_HPB0				0x00000001 /**< AHB clock for the HPB0 is enabled */
#define PM_AHB_RESET			0x0000007F /**< AHB mask reset state */
#define PM_AHB_OFFSET			0x14 /**< Address offset */

/** @} */
/*---------------------------------------------------------------------------*/

/** \name APBA Mask bit masks
 * @{
 */

#define PM_APBA_EIC 			0x00000040 /**< APBA clock for the EIC is enabled */
#define PM_APBA_RTC				0x00000020 /**< APBA clock for the RTC is enabled */
#define PM_APBA_WDT				0x00000010 /**< APBA clock for the WDT is enabled */
#define PM_APBA_GCLK			0x00000008 /**< APBA clock for the GCLK is enabled */
#define PM_APBA_SYSCTRL			0x00000004 /**< APBA clock for the SYSCTRL is enabled */
#define PM_APBA_PM				0x00000002 /**< APBA clock for the PM is enabled */
#define PM_APBA_PAC0			0x00000001 /**< APBA clock for the PAC0 is enabled */
#define PM_APBA_RESET			0x0000007F /**< APBA mask reset state */
#define PM_APBA_OFFSET 			0x18 /**< Address offset */

/** @} */
/*---------------------------------------------------------------------------*/

/** \name APBB Mask bit masks
 * @{
 */

#define PM_APBB_USB				0x00000020 /**< APBB clock for the USB is enabled */
#define PM_APBB_DMAC			0x00000010 /**< APBB clock for the DMAC is enabled */
#define PM_APBB_PORT			0x00000008 /**< APBB clock for the PORT is enabled */
#define PM_APBB_NVMCTRL			0x00000004 /**< APBB clock for the NVMCTRL is enabled */
#define PM_APBB_DSU				0x00000002 /**< APBB clock for the DSU is enabled */
#define PM_APBB_PAC1			0x00000001 /**< APBB clock for the PAC1 is enabled */
#define PM_APBB_RESET			0x0000007F /**< APBB mask reset state */
#define PM_APBA_OFFSET 			0x1C /**< Address offset */

/** @} */
/*---------------------------------------------------------------------------*/

/** \name APBC Mask bit masks
 * @{
 */

#define PM_APBC_RFCTRL			0x00200000 /**< APBC clock for the RFCTRL is enabled */
#define PM_APBC_PTC				0x00080000 /**< APBC clock for the PTC is enabled */
#define PM_APBC_AC 				0x00020000 /**< APBC clock for the AC is enabled */
#define PM_APBC_ADC				0x00010000 /**< APBC clock for the ADC is enabled */
#define PM_APBC_TC5				0x00002000 /**< APBC clock for the TC5 is enabled */
#define PM_APBC_TC4				0x00001000 /**< APBC clock for the TC4 is enabled */
#define PM_APBC_TC3				0x00000800 /**< APBC clock for the TC3 is enabled */
#define PM_APBC_TC2				0x00000400 /**< APBC clock for the TC2 is enabled */
#define PM_APBC_TC1				0x00000200 /**< APBC clock for the TC1 is enabled */
#define PM_APBC_TC0				0x00000100 /**< APBC clock for the TC0 is enabled */
#define PM_APBC_SERCOM5			0x00000080 /**< APBC clock for the SERCOM5 is enabled */
#define PM_APBC_SERCOM4			0x00000040 /**< APBC clock for the SERCOM4 is enabled */
#define PM_APBC_SERCOM3			0x00000020 /**< APBC clock for the SERCOM3 is enabled */
#define PM_APBC_SERCOM2			0x00000010 /**< APBC clock for the SERCOM2 is enabled */
#define PM_APBC_SERCOM1			0x00000008 /**< APBC clock for the SERCOM1 is enabled */
#define PM_APBC_SERCOM0			0x00000004 /**< APBC clock for the SERCOM0 is enabled */
#define PM_APBC_EVSYS			0x00000002 /**< APBC clock for the EVSYS is enabled */
#define PM_APBC_PAC2			0x00000001 /**< APBC clock for the PAC2 is enabled */
#define PM_APBB_RESET			0x00010000 /**< APBC mask reset state */
#define PM_APBA_OFFSET 			0x20 /**< Address offset */

/** @} */
/*---------------------------------------------------------------------------*/

/** \name Interrupt Enable Clear bit mask
 * @{
 */

#define PM_INTENCLR_CKRDY		0x01 /**< Clock ready interrupt is enabled and will generate an interrupt when set */
#define PM_INTENCLR_RESET		0x00 /**< Interrupt enable clear reset state*/
#define PM_INTENCLR_OFFSET		0x34 /**< Address offset*/

/** @} */
/*---------------------------------------------------------------------------*/

/** \name Interrupt Enable Set bit mask
 * @{
 */

#define PM_INTENSET_CKRDY		0x01 /**< Clock ready interrupt is enabled */
#define PM_INTENSET_RESET		0x00 /**< Interrupt set clear reset state*/
#define PM_INTENSET_OFFSET		0x35 /**< Address offset*/

/** @} */
/*---------------------------------------------------------------------------*/

/** \name Interrupt Flag satus and clear bit mask
 * @{
 */

#define PM_INTFLAG_CKRDY		0x01 /**< Writing 1 clears flag, flag set when CPU/APBx clocks have frequencies as indicated in CPU/APBxSEL registers and will generate interrupt if INTENCLR/SET.CKRDY is one*/
#define PM_INTFLAG_RESET		0x00 /**< Interrupt set clear reset state*/
#define PM_INTFLAG_OFFSET		0x36 /**< Address offset*/

/** @} */
/*---------------------------------------------------------------------------*/

/** \name Reset cause bit mask
 * @{
 */

#define PM_RCAUSE_SYST			0x40 /**< Set if system reset request has been performed */
#define PM_RCAUSE_WDT			0x20 /**< Set if watchdog timer reset occurs */
#define PM_RCAUSE_EXT			0x10 /**< Set if external reset occurs */
#define PM_RCAUSE_BOD33			0x04 /**< Set if a BOD33 reset occurs */
#define PM_RCAUSE_BOD12			0x02 /**< Set if a BOD12 reset occurs */
#define PM_RCAUSE_POR			0x01 /**< Set if a POR reset occurs */
#define PM_RCAUSE_RESET			0x01 /**< Reset cause default state */
#define PM_RCAUSE_OFFSET		0x38 /**< Address offset */

/** @} */

#endif /* PM_H_ */

/**
 * @}
 * @}
 */
