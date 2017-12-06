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
 * \defgroup samd21-reg samd21 Register Manipulation
 *
 * Macros for hardware direct hardware access
 * @{
 *
 * \file
 * Header file for the samd21 register manipulation macros
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "reg.h"

#include <stdint.h>

#define GPIO_BASE				0x41004400 /**< Base GPIO/PORT address */

#define GPIO_A					0x00 /**< GPIO group A offset */

/**
 * \brief Converts a pin number to a pin mask
 * \param PIN The pin number in the range [0..7]
 * \return A pin mask which can be used as the PIN_MASK argument of the macros
 * in this category
 */
#define GPIO_PIN_MASK(PIN) (1 << (PIN))

/*---------------------------------------------------------------------------*/
/** \name GPIO Register offset declarations
 * @{
 */
#define GPIO_DIR				0x00 /**< GPIO group A data direction */
#define GPIO_DIRCLR				0x04 /**< GPIO group A data direction clear */
#define GPIO_DIRSET				0x08 /**< GPIO group A data direction set */
#define GPIO_DIRTGL				0x0C /**< GPIO group A data direction toggle */
#define GPIO_OUT				0x10 /**< GPIO group A data output value */
#define GPIO_OUTCLR				0x14 /**< GPIO group A data output value clear */
#define GPIO_OUTSET				0x18 /**< GPIO group A data output value set */
#define GPIO_OUTTGL				0x1C /**< GPIO group A data output toggle */
#define GPIO_IN					0x20 /**< GPIO group A input value */
#define GPIO_CTRL				0x24 /**< GPIO group A control */
#define GPIO_WRCONFIG			0x28 /**< GPIO group A write configuration */
/** @} */

/*---------------------------------------------------------------------------*/
/** \name GPIO_DIR register bit masks
 * \brief Access and modify the GPIO pin directions performing a read-write-
 * modify operation
 * @{
 */
#define GPIO_DIR_DIR			0x00000000 /**< Pin Input (0) / Output (1) */
/** @} */

/*---------------------------------------------------------------------------*/
/** \name GPIO_DIRCLR register bit masks
 * \brief Clear one or more GPIO pin directions without performing a read-write-
 *	modify operation. Changes in this register will be reflectted in DIR, DIRTGL 
 *  and DIRSET registers. 
 * @{
 */
#define GPIO_DIRCLR_DIRCLR		0x00000000 /**< No Effect (0) / Clear bit, set to Input (1) */
/** @} */

/*---------------------------------------------------------------------------*/
/** \name GPIO_DIRSET register bit masks
 * \brief Set one or more GPIO pin directions without performing a read-write-
 *	modify operation. Changes in this register will be reflectted in DIR, DIRTGL 
 *  and DIRCLR registers. 
 * @{
 */
#define GPIO_DIRSET_DIRSET		0x00000000 /**< No Effect (0) / Set pin to Output (1) */

/** @} */

/*---------------------------------------------------------------------------*/
/** \name GPIO_DIRTGL register bit masks
 * \brief Toggle one or more GPIO pin directions without performing a read-write-
 *	modify operation. Changes in this register will be reflectted in DIR, DIRCLR 
 *  and DIRSET registers. 
 * @{
 */
#define GPIO_DIRTGL_DIRTGL		0x00000000 /**< No Effect (0) / Toggle pin direction (1) */
/** @} */

/*---------------------------------------------------------------------------*/
/** \name GPIO_OUT register bit masks
 * \brief Access and modify the GPIO output values performing a read-write-
 * modify operation. These bits set the logical output drive of the I/O pins 
 * configured as outputs via the DIR register. For pins configured as inputs
 * with pull enabled via the PULLEN register, these bits will set the input 
 * pull direction. 
 * @{
 */
#define GPIO_OUT_OUT			0x00000000 /**< Output low (0) / Output high (1) */
/** @} */

/*---------------------------------------------------------------------------*/
/** \name GPIO_OUTCLR register bit masks
 * \brief Clear one or more GPIO pin outputs without performing a read-write-
 *	modify operation. Changes in this register will be reflectted in OUT, OUTSET 
 *  and OUTTGL registers. For pins configured as inputs via DIR register with 
 *  pull enabled via PULLEN register, the bits will set pull direction to pull-down 
 * @{
 */
#define GPIO_OUTCLR_OUTCLR		0x00000000 /**< No Effect (0) / Clear bit (1) */
/** @} */

/*---------------------------------------------------------------------------*/
/** \name GPIO_OUTSET register bit masks
 * \brief Sets one or more GPIO pin outputs without performing a read-write-
 *	modify operation. Changes in this register will be reflectted in OUT, OUTSET 
 *  and OUTTGL registers. For pins configured as inputs via DIR register with 
 *  pull enabled via PULLEN register, the bits will set pull direction to pull-up 
 * @{
 */
#define GPIO_OUTSET_OUTSET 		0x00000000 /**< No effect (0) / Output high (1) */
/** @} */

/*---------------------------------------------------------------------------*/
/** \name GPIO_OUTTGL register bit masks
 * \brief Toggles one or more GPIO pin outputs without performing a read-write-
 *	modify operation. Changes in this register will be reflectted in OUT, OUTSET 
 *  and OUTTGL registers. For pins configured as inputs via DIR register with 
 *  pull enabled via PULLEN register, the bits will toggle the pull direction
 * @{
 */
#define GPIO_OUTTGL_OUTTGL		0x00000000 /**< No effect (0) / Toggle pin direction (1) */
/** @} */

/*---------------------------------------------------------------------------*/
/** \name GPIO_IN register bit masks
 * \brief These bits are cleared when the corresponding I/O pin input sampler 
 * detects a logical low level on the input pin. These bits are set when the 
 * corresponding I/O pin input sampler detecs a logical high level on the pin
 * @{
 */
#define GPIO_IN_IN				0x00000000 /**< Logical low (0) / Logical high (1) */
/** @} */

/*---------------------------------------------------------------------------*/
/** \name GPIO_CTRL register bit masks
 * \brief Configures the input sampling functionality of the I/O pin input 
 * samplers for pins configure as inputs via DIR. The input samplers are enabled
 * and disabled in sub-groups of eight. Thus, if any pins within a byte request 
 * continous sampling, all pins in that sub-group will be continuously sampled.
 * @{
 */
#define GPIO_CTRL_CTRL			0x00000000 /**< Input synchronizer disabled (0) / enabled (1) */
/** @} */

/*---------------------------------------------------------------------------*/
/** \name GPIO_WRCONFIG register bit masks
 * @{
 */
#define GPIO_WRCONFIG_WRCONFIG	0x00000000 
#define GPIO_WRCONFIG_HWSEL		0x80000000 /**< 0/1 <=> lower/upper 16 pins of GPIO group will be configured */
#define GPIO_WRCONFIG_WRPINCFG	0x40000000 /**< PINCFGy registers of selected pins will be updated */
#define GPIO_WRCONFIG_WRPMUX	0x10000000 /**< Determines atomic write operation, PMUXn registers of selected pins will be updated */
#define GPIO_WRCONFIG_PUMUX		0x00000000 /**< [27:24] Bits determine new value written ot PMUXn register for pins selected by PINMASK and HWSEL when WRPMUX is set*/
#define GPIO_WRCONFIG_DRVSTR	0x00400000 /**< Drive Strength - Determines new value written to PINCFGy.DRVSTR for all pins selected by the pinmask */
#define GPIO_WRCONFIG_PULLEN	0x00040000 /**< Pull enable - Determines new value written to PINCFGy.PULLEN for all pins selected by the pinmask */
#define GPIO_WRCONFIG_INEN		0x00020000 /**< Input enable */
#define GPIO_WRCONFIG_PMUXEN	0x00010000 /**< Peripheral mux enable - determines new value written to PINCFGy.PMUXEN for pins selected by mask */
#define GPIO_WRCONFIG_PINMASK	0x00000000 /**< [15:0] select pins to be conf. within half-word group selected by WRCONFIG.HWSEL */

/** @} */

/*---------------------------------------------------------------------------*/
/** \name PMUXO and PMUXE peripheral function definitions
 * @{
 */
#define GPIO_PMUX_A		0x0 /**< Peripheral function A selected */
#define GPIO_PMUX_B		0x1 /**< Peripheral function B selected */
#define GPIO_PMUX_c		0x2 /**< Peripheral function C selected */
#define GPIO_PMUX_D		0x3 /**< Peripheral function D selected */
#define GPIO_PMUX_E		0x4 /**< Peripheral function E selected */
#define GPIO_PMUX_F		0x5 /**< Peripheral function F selected */
#define GPIO_PMUX_G		0x6 /**< Peripheral function G selected */
#define GPIO_PMUX_H		0x7 /**< Peripheral function H selected */

/** @} */

/*---------------------------------------------------------------------------*/
/** \name PMUX register offset definitions
 * \brief Each Peripheral Mux Register is comprised of two half-byte halves, 
 * bits [7:4] are odd, and bits [3:0] are even. Peripheral functions can then 
 * be selected for each mux using the defined function definitions A-H above. 
 * Note that not all functions A-H may be valid. 
 * @{
 */

#define GPIO_PMUX0_OFFSET	0x30 /**< MUX 0 Offset */
#define GPIO_PMUX1_OFFSET	0x31 /**< MUX 1 Offset */
#define GPIO_PMUX2_OFFSET	0x32 /**< MUX 2 Offset */
#define GPIO_PMUX3_OFFSET	0x33 /**< MUX 3 Offset */
#define GPIO_PMUX4_OFFSET	0x34 /**< MUX 4 Offset */
#define GPIO_PMUX5_OFFSET	0x35 /**< MUX 5 Offset */
#define GPIO_PMUX6_OFFSET	0x36 /**< MUX 6 Offset */
#define GPIO_PMUX7_OFFSET	0x37 /**< MUX 7 Offset */
#define GPIO_PMUX8_OFFSET	0x38 /**< MUX 8 Offset */
#define GPIO_PMUX9_OFFSET	0x39 /**< MUX 9 Offset */
#define GPIO_PMUX10_OFFSET	0x3A /**< MUX 10 Offset */
#define GPIO_PMUX11_OFFSET	0x3B /**< MUX 11 Offset */
#define GPIO_PMUX12_OFFSET	0x3C /**< MUX 12 Offset */
#define GPIO_PMUX13_OFFSET	0x3D /**< MUX 13 Offset */
#define GPIO_PMUX14_OFFSET	0x3E /**< MUX 14 Offset */
#define GPIO_PMUX15_OFFSET	0x3F /**< MUX 15 Offset */

/** @} */

/*---------------------------------------------------------------------------*/
/** \name GPIO_PINCFG fields and values
 * @{
 */

#define GPIO_PINCFG_DRVSTR			0x40 /**< Stronger pin drive strength */
#define GPIO_PINCFG_PULLEN			0x04 /**< Internal pull resistor enabled, and input driven to defined logic level in abscence of external input. */
#define GPIO_PINCFG_INEN			0x02 /**< Input buffer for I/O pin is enabled and input will be sampled upon request */
#define GPIO_PINCFG_PMUXEN			0x01 /**< The peripheral multiplexer selection is enabled, and the selected peripheral controls the direction and output drive value */

/** @} */

/*---------------------------------------------------------------------------*/
/** \name GPIO_PINCFG register offsets, each takes PNCFG fields and values
 * @{
 */

#define GPIO_PINCFG0_OFFSET			0x40 /**< Pin configuration 0 offset */
#define GPIO_PINCFG1_OFFSET			0x41 /**< Pin configuration 1 offset */
#define GPIO_PINCFG2_OFFSET			0x42 /**< Pin configuration 2 offset */
#define GPIO_PINCFG3_OFFSET			0x43 /**< Pin configuration 3 offset */
#define GPIO_PINCFG4_OFFSET			0x44 /**< Pin configuration 4 offset */
#define GPIO_PINCFG5_OFFSET			0x45 /**< Pin configuration 5 offset */
#define GPIO_PINCFG6_OFFSET			0x46 /**< Pin configuration 6 offset */
#define GPIO_PINCFG7_OFFSET			0x47 /**< Pin configuration 7 offset */
#define GPIO_PINCFG8_OFFSET			0x48 /**< Pin configuration 8 offset */
#define GPIO_PINCFG9_OFFSET			0x49 /**< Pin configuration 9 offset */
#define GPIO_PINCFG10_OFFSET		0x4A /**< Pin configuration 10 offset */
#define GPIO_PINCFG11_OFFSET		0x4B /**< Pin configuration 11 offset */
#define GPIO_PINCFG12_OFFSET		0x4C /**< Pin configuration 12 offset */
#define GPIO_PINCFG13_OFFSET		0x4D /**< Pin configuration 13 offset */
#define GPIO_PINCFG14_OFFSET		0x4E /**< Pin configuration 14 offset */
#define GPIO_PINCFG15_OFFSET		0x4F /**< Pin configuration 15 offset */
#define GPIO_PINCFG16_OFFSET		0x50 /**< Pin configuration 16 offset */
#define GPIO_PINCFG17_OFFSET		0x51 /**< Pin configuration 17 offset */
#define GPIO_PINCFG18_OFFSET		0x52 /**< Pin configuration 18 offset */
#define GPIO_PINCFG19_OFFSET		0x53 /**< Pin configuration 19 offset */
#define GPIO_PINCFG20_OFFSET		0x54 /**< Pin configuration 20 offset */
#define GPIO_PINCFG21_OFFSET		0x55 /**< Pin configuration 21 offset */
#define GPIO_PINCFG22_OFFSET		0x56 /**< Pin configuration 22 offset */
#define GPIO_PINCFG23_OFFSET		0x57 /**< Pin configuration 23 offset */
#define GPIO_PINCFG24_OFFSET		0x58 /**< Pin configuration 24 offset */
#define GPIO_PINCFG25_OFFSET		0x59 /**< Pin configuration 25 offset */
#define GPIO_PINCFG26_OFFSET		0x5A /**< Pin configuration 26 offset */
#define GPIO_PINCFG27_OFFSET		0x5B /**< Pin configuration 27 offset */
#define GPIO_PINCFG28_OFFSET		0x5C /**< Pin configuration 28 offset */
#define GPIO_PINCFG29_OFFSET		0x5D /**< Pin configuration 29 offset */
#define GPIO_PINCFG30_OFFSET		0x5E /**< Pin configuration 30 offset */
#define GPIO_PINCFG31_OFFSET		0x5F /**< Pin configuration 31 offset */

/** @} */

#endif /* GPIO_H_ */

/**
 * @}
 * @}
 */
