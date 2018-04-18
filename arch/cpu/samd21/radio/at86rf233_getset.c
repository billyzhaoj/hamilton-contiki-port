/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_at86rf2xx
 * @{
 *
 * @file
 * @brief       Getter and setter functions for the AT86RF2xx drivers
 *
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Baptiste Clenet <bapclenet@gmail.com>
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @author      Kévin Roussel <Kevin.Roussel@inria.fr>
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 *
 * @}
 */

#include "at86rf233.h"
#include "at86rf233_internal.h"
#include "at86rf233_registers.h"
#include "dev/spi.h"

static const int16_t tx_pow_to_dbm[] = {4, 4, 3, 3, 2, 2, 1,
                                        0, -1, -2, -3, -4, -6, -8, -12, -17};
static const uint8_t dbm_to_tx_pow[] = {0x0f, 0x0f, 0x0f, 0x0e, 0x0e, 0x0e,
                                        0x0e, 0x0d, 0x0d, 0x0d, 0x0c, 0x0c,
                                        0x0b, 0x0b, 0x0a, 0x09, 0x08, 0x07,
                                        0x06, 0x05, 0x03,0x00};

#ifdef RADIO_DUTYCYCLE_MONITOR
static uint32_t prev = 0;
#endif

static uint8_t flags = 0;

uint16_t at86rf2xx_get_addr_short(at86rf2xx_t *dev)
{
    return 0;
}

void at86rf2xx_set_addr_short(at86rf2xx_t *dev, uint16_t addr)
{
}

uint64_t at86rf2xx_get_addr_long(at86rf2xx_t *dev)
{
    return 0;
}

void at86rf2xx_set_addr_long(at86rf2xx_t *dev, uint64_t addr)
{
    for (int i = 0; i < 8; i++) {
        //dev->netdev.long_addr[i] = (uint8_t)(addr >> (i * 8));
        at86rf2xx_reg_write(dev, (AT86RF2XX_REG__IEEE_ADDR_0 + i),
                            (addr >> ((7 - i) * 8)));
    }
}

uint8_t at86rf2xx_get_chan(at86rf2xx_t *dev)
{
    return 0;//dev->netdev.chan;
}

void at86rf2xx_set_chan(at86rf2xx_t *dev, uint8_t channel)
{
    if ((channel < AT86RF2XX_MIN_CHANNEL) ||
        (channel > AT86RF2XX_MAX_CHANNEL)) {
        return;
    }

    at86rf2xx_configure_phy(dev);
}

uint8_t at86rf2xx_get_page(at86rf2xx_t *dev)
{
    (void) dev;
    return 0;
}

void at86rf2xx_set_page(at86rf2xx_t *dev, uint8_t page)
{
    (void) dev;
    (void) page;
}

uint16_t at86rf2xx_get_pan(at86rf2xx_t *dev)
{
    return 0;//dev->netdev.pan;
}

void at86rf2xx_set_pan(at86rf2xx_t *dev, uint16_t pan)
{
    //le_uint16_t le_pan = byteorder_btols(byteorder_htons(pan));
    //at86rf2xx_reg_write(dev, AT86RF2XX_REG__PAN_ID_0, le_pan.u8[0]);
    //at86rf2xx_reg_write(dev, AT86RF2XX_REG__PAN_ID_1, le_pan.u8[1]);
}

int16_t at86rf2xx_get_txpower(at86rf2xx_t *dev)
{
    uint8_t txpower = at86rf2xx_reg_read(dev, AT86RF2XX_REG__PHY_TX_PWR)
                & AT86RF2XX_PHY_TX_PWR_MASK__TX_PWR;
    return tx_pow_to_dbm[txpower];
}

void at86rf2xx_set_txpower(at86rf2xx_t *dev, int16_t txpower)
{
    txpower += 17;

    if (txpower < 0) {
        txpower = 0;
    }
    else if (txpower > 21) {
        txpower = 21;
    }
    at86rf2xx_reg_write(dev, AT86RF2XX_REG__PHY_TX_PWR,
                        dbm_to_tx_pow[txpower]);
}

uint8_t at86rf2xx_get_max_retries(at86rf2xx_t *dev)
{
    return (at86rf2xx_reg_read(dev, AT86RF2XX_REG__XAH_CTRL_0) >> 4);
}

void at86rf2xx_set_max_retries(at86rf2xx_t *dev, uint8_t max)
{
    max = (max > 7) ? 7 : max;
    uint8_t tmp = at86rf2xx_reg_read(dev, AT86RF2XX_REG__XAH_CTRL_0);
    tmp &= ~(AT86RF2XX_XAH_CTRL_0__MAX_FRAME_RETRIES);
    tmp |= (max << 4);
    at86rf2xx_reg_write(dev, AT86RF2XX_REG__XAH_CTRL_0, tmp);
}

uint8_t at86rf2xx_get_csma_max_retries(at86rf2xx_t *dev)
{
    uint8_t tmp;
    tmp  = at86rf2xx_reg_read(dev, AT86RF2XX_REG__XAH_CTRL_0);
    tmp &= AT86RF2XX_XAH_CTRL_0__MAX_CSMA_RETRIES;
    tmp >>= 1;
    return tmp;
}

void at86rf2xx_set_csma_max_retries(at86rf2xx_t *dev, int8_t retries)
{
    retries = (retries > 5) ? 5 : retries; /* valid values: 0-5 */
    retries = (retries < 0) ? 7 : retries; /* max < 0 => disable CSMA (set to 7) */
    
    uint8_t tmp = at86rf2xx_reg_read(dev, AT86RF2XX_REG__XAH_CTRL_0);
    tmp &= ~(AT86RF2XX_XAH_CTRL_0__MAX_CSMA_RETRIES);
    tmp |= (retries << 1);
    at86rf2xx_reg_write(dev, AT86RF2XX_REG__XAH_CTRL_0, tmp);
}

void at86rf2xx_set_csma_backoff_exp(at86rf2xx_t *dev, uint8_t min, uint8_t max)
{
    max = (max > 8) ? 8 : max;
    min = (min > max) ? max : min;
    
    at86rf2xx_reg_write(dev,
            AT86RF2XX_REG__CSMA_BE,
            (max << 4) | (min));
}

void at86rf2xx_set_csma_seed(at86rf2xx_t *dev, uint8_t entropy[2])
{
    if(entropy == NULL) {
        return;
    }
    
    at86rf2xx_reg_write(dev,
                           AT86RF2XX_REG__CSMA_SEED_0,
                           entropy[0]);

    uint8_t tmp = at86rf2xx_reg_read(dev, AT86RF2XX_REG__CSMA_SEED_1);
    tmp &= ~(AT86RF2XX_CSMA_SEED_1__CSMA_SEED_1);
    tmp |= entropy[1] & AT86RF2XX_CSMA_SEED_1__CSMA_SEED_1;
    at86rf2xx_reg_write(dev, AT86RF2XX_REG__CSMA_SEED_1, tmp);
}

int8_t at86rf2xx_get_cca_threshold(at86rf2xx_t *dev)
{
    int8_t tmp = at86rf2xx_reg_read(dev, AT86RF2XX_REG__CCA_THRES);
    tmp &= AT86RF2XX_CCA_THRES_MASK__CCA_ED_THRES;
    tmp <<= 1;
    return (RSSI_BASE_VAL + tmp);
}

void at86rf2xx_set_cca_threshold(at86rf2xx_t *dev, int8_t value)
{
    /* ensure the given value is negative, since a CCA threshold > 0 is
       just impossible: thus, any positive value given is considered
       to be the absolute value of the actually wanted threshold */
    if (value > 0) {
        value = -value;
    }
    /* transform the dBm value in the form
       that will fit in the AT86RF2XX_REG__CCA_THRES register */
    value -= RSSI_BASE_VAL;
    value >>= 1;
    value &= AT86RF2XX_CCA_THRES_MASK__CCA_ED_THRES;
    value |= AT86RF2XX_CCA_THRES_MASK__RSVD_HI_NIBBLE;
    at86rf2xx_reg_write(dev, AT86RF2XX_REG__CCA_THRES, value);
}

int8_t at86rf2xx_get_ed_level(at86rf2xx_t *dev)
{
    uint8_t tmp = at86rf2xx_reg_read(dev, AT86RF2XX_REG__PHY_ED_LEVEL);
    int8_t ed = (int8_t)tmp + RSSI_BASE_VAL;
    return ed;
}

void at86rf2xx_set_option(at86rf2xx_t *dev, uint16_t option, bool state)
{
    uint8_t tmp;

    /* set option field */
    if (state) {
        flags |= option;
        /* trigger option specific actions */
        switch (option) {
            case AT86RF2XX_OPT_PROMISCUOUS:
                /* disable auto ACKs in promiscuous mode */
                tmp = at86rf2xx_reg_read(dev, AT86RF2XX_REG__CSMA_SEED_1);
                tmp |= AT86RF2XX_CSMA_SEED_1__AACK_DIS_ACK;
                at86rf2xx_reg_write(dev, AT86RF2XX_REG__CSMA_SEED_1, tmp);
                /* enable promiscuous mode */
                tmp = at86rf2xx_reg_read(dev, AT86RF2XX_REG__XAH_CTRL_1);
                tmp |= AT86RF2XX_XAH_CTRL_1__AACK_PROM_MODE;
                at86rf2xx_reg_write(dev, AT86RF2XX_REG__XAH_CTRL_1, tmp);
                break;
            /*case AT86RF2XX_OPT_AUTOACK:
                tmp = at86rf2xx_reg_read(dev, AT86RF2XX_REG__CSMA_SEED_1);
                tmp &= ~(AT86RF2XX_CSMA_SEED_1__AACK_DIS_ACK);
                at86rf2xx_reg_write(dev, AT86RF2XX_REG__CSMA_SEED_1, tmp);
                break;
            case AT86RF2XX_OPT_ACK_PENDING:
                tmp = at86rf2xx_reg_read(dev, AT86RF2XX_REG__CSMA_SEED_1);
                tmp |= AT86RF2XX_CSMA_SEED_1__AACK_SET_PD;
                at86rf2xx_reg_write(dev, AT86RF2XX_REG__CSMA_SEED_1, tmp);
                break;*/
            case AT86RF2XX_OPT_TELL_RX_START:
                tmp = at86rf2xx_reg_read(dev, AT86RF2XX_REG__IRQ_MASK);
                tmp |= AT86RF2XX_IRQ_STATUS_MASK__RX_START;
                at86rf2xx_reg_write(dev, AT86RF2XX_REG__IRQ_MASK, tmp);
                break;
            case AT86RF2XX_OPT_TELL_CCA_ED_DONE:
                tmp = at86rf2xx_reg_read(dev, AT86RF2XX_REG__IRQ_MASK);
                tmp |= AT86RF2XX_IRQ_STATUS_MASK__CCA_ED_DONE;
                at86rf2xx_reg_write(dev, AT86RF2XX_REG__IRQ_MASK, tmp);
                break;
            default:
                /* do nothing */
                break;
        }
    }
    else {
        flags &= ~(option);
        /* trigger option specific actions */
        switch (option) {
            case AT86RF2XX_OPT_PROMISCUOUS:
                /* disable promiscuous mode */
                tmp = at86rf2xx_reg_read(dev, AT86RF2XX_REG__XAH_CTRL_1);
                tmp &= ~(AT86RF2XX_XAH_CTRL_1__AACK_PROM_MODE);
                at86rf2xx_reg_write(dev, AT86RF2XX_REG__XAH_CTRL_1, tmp);
                /* re-enable AUTOACK only if the option is set */
                if (flags & AT86RF2XX_OPT_AUTOACK) {
                    tmp = at86rf2xx_reg_read(dev,
                                             AT86RF2XX_REG__CSMA_SEED_1);
                    tmp &= ~(AT86RF2XX_CSMA_SEED_1__AACK_DIS_ACK);
                    at86rf2xx_reg_write(dev, AT86RF2XX_REG__CSMA_SEED_1,
                                        tmp);
                }
                break;
            /*case AT86RF2XX_OPT_AUTOACK:
                tmp = at86rf2xx_reg_read(dev, AT86RF2XX_REG__CSMA_SEED_1);
                tmp |= AT86RF2XX_CSMA_SEED_1__AACK_DIS_ACK;
                at86rf2xx_reg_write(dev, AT86RF2XX_REG__CSMA_SEED_1, tmp);
                break;
            case AT86RF2XX_OPT_ACK_PENDING:
                tmp = at86rf2xx_reg_read(dev, AT86RF2XX_REG__CSMA_SEED_1);
                tmp &= ~(AT86RF2XX_CSMA_SEED_1__AACK_SET_PD);
                at86rf2xx_reg_write(dev, AT86RF2XX_REG__CSMA_SEED_1, tmp);
                break;*/
            case AT86RF2XX_OPT_TELL_RX_START:
                tmp = at86rf2xx_reg_read(dev, AT86RF2XX_REG__IRQ_MASK);
                tmp &= ~AT86RF2XX_IRQ_STATUS_MASK__RX_START;
                at86rf2xx_reg_write(dev, AT86RF2XX_REG__IRQ_MASK, tmp);
                break;
            case AT86RF2XX_OPT_TELL_CCA_ED_DONE:
                tmp = at86rf2xx_reg_read(dev, AT86RF2XX_REG__IRQ_MASK);
                tmp &= ~AT86RF2XX_IRQ_STATUS_MASK__CCA_ED_DONE;
                at86rf2xx_reg_write(dev, AT86RF2XX_REG__IRQ_MASK, tmp);
                break;
            default:
                /* do nothing */
                break;
        }
    }
}

/**
 * @brief Internal function to change state
 * @details For all cases but AT86RF2XX_STATE_FORCE_TRX_OFF state and
 *          cmd parameter are the same.
 *
 * @param dev       device to operate on
 * @param state     target state
 * @param cmd       command to initiate state transition
 */

static inline void _set_state(at86rf2xx_t *dev, uint8_t state, uint8_t cmd)
{
    at86rf2xx_reg_write(dev, AT86RF2XX_REG__TRX_STATE, cmd);

    /* To prevent a possible race condition when changing to
     * RX_AACK_ON state the state doesn't get read back in that
     * case. See discussion
     * in https://github.com/RIOT-OS/RIOT/pull/5244
     */
    if (state != AT86RF2XX_STATE_RX_AACK_ON) {
        while (at86rf2xx_get_status(dev) != state) {
            at86rf2xx_reg_write(dev, AT86RF2XX_REG__TRX_STATE, cmd);    
        }
    }
    /* Although RX_AACK_ON state doesn't get read back,
     * at least make sure if state transition is in progress or not
     */
    else {
        while (at86rf2xx_get_status(dev) == AT86RF2XX_STATE_IN_PROGRESS) {}
    }

    dev->state = state;
}

uint8_t at86rf2xx_set_state(at86rf2xx_t *dev, uint8_t state)
{
    uint8_t old_state;

    /* make sure there is no ongoing transmission, or state transition already
     * in progress */
    do {
        old_state = at86rf2xx_get_status(dev);
    } while (old_state == AT86RF2XX_STATE_BUSY_RX_AACK ||
             old_state == AT86RF2XX_STATE_BUSY_TX_ARET ||
             old_state == AT86RF2XX_STATE_IN_PROGRESS);

    if (state == AT86RF2XX_STATE_FORCE_TRX_OFF) {
        _set_state(dev, AT86RF2XX_STATE_TRX_OFF, state);
        return old_state;
    }

    if (state == old_state) {
        return old_state;
    }

    /* we need to go via PLL_ON if we are moving between RX_AACK_ON <-> TX_ARET_ON */
    if ((old_state == AT86RF2XX_STATE_RX_AACK_ON &&
             state == AT86RF2XX_STATE_TX_ARET_ON) ||
        (old_state == AT86RF2XX_STATE_TX_ARET_ON &&
             state == AT86RF2XX_STATE_RX_AACK_ON)) {
        _set_state(dev, AT86RF2XX_STATE_PLL_ON, AT86RF2XX_STATE_PLL_ON);
    }
    /* check if we need to wake up from sleep mode */
    else if (old_state == AT86RF2XX_STATE_SLEEP) {
#ifdef RADIO_DUTYCYCLE_MONITOR
        if (prev == 0) {
            prev = xtimer_now().ticks32;
        } else {
            uint32_t now = xtimer_now().ticks32;
            radioOffTime += (now - prev);
            prev = now;
        }
#endif
        at86rf2xx_assert_awake(dev);
    }

    if (state == AT86RF2XX_STATE_SLEEP) {
#ifdef RADIO_DUTYCYCLE_MONITOR
        if (prev == 0) {
            prev = xtimer_now().ticks32;
        } else {
            uint32_t now = xtimer_now().ticks32;
            radioOnTime += (now - prev);
            prev = now;
        }
#endif
        /* First go to TRX_OFF */
        at86rf2xx_set_state(dev, AT86RF2XX_STATE_FORCE_TRX_OFF);
        /* Discard all IRQ flags, framebuffer is lost anyway */
        at86rf2xx_reg_read(dev, AT86RF2XX_REG__IRQ_STATUS);
        /* Go to SLEEP mode from TRX_OFF */
        gpio_set(dev->params.sleep_pin);
        dev->state = state;
    } else {
        _set_state(dev, state, state);
    }

    return old_state;
}
