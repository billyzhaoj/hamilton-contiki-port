/*
 * Copyright (C) 2014 PHYTEC Messtechnik GmbH
 *               2017 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_hdc1000 HDC1000 Humidity and Temperature Sensor
 * @ingroup     drivers_sensors
 * @brief       Driver for the TI HDC1000 Humidity and Temperature Sensor
 *
 * The driver will initialize the sensor for best resolution (14 bit). Currently
 * the driver doesn't use the heater. Temperature and humidity are acquired in
 * sequence. The sensor is always in sleep mode.
 *
 * The temperature and humidity values can either be acquired using the
 * simplified `hdc1000_read()` function, or the conversion can be triggered
 * manually using the `hdc1000_trigger_conversion()` and `hdc1000_get_results()`
 * functions sequentially. If using the second method, on must wait at least
 * `HDC1000_CONVERSION_TIME` between triggering the conversion and reading the
 * results.
 *
 * @note        The driver does currently not support using the devices heating
 *              unit.
 *
 * @{
 *
 * @file
 * @brief       Interface definition for the HDC1000 sensor driver
 *
 * @author      Johann Fischer <j.fischer@phytec.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 */

#ifndef HDC1000_H
#define HDC1000_H

#include <stdint.h>

#include "dev/i2c.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @name    Manufacturer and Device IDs
 * @{
 */
#define HDC1000_MID_VALUE          0x5449
#define HDC1000_DID_VALUE          0x1000
/** @} */

/**
 * @name    Register Map
 * @{
 */
#define HDC1000_TEMPERATURE         (0x00)
#define HDC1000_HUMIDITY            (0x01)
#define HDC1000_CONFIG              (0x02)
#define HDC1000_SID1                (0xFB)
#define HDC1000_SID2                (0xFC)
#define HDC1000_SID3                (0xFD)
#define HDC1000_MANUFACTURER_ID     (0xFE)
#define HDC1000_DEVICE_ID           (0xFF)
/** @} */

/**
 * @name    Configuration register bitmap
 * @{
 */
#define HDC1000_RST                 (1 << 15)
#define HDC1000_HEAT                (1 << 13)
#define HDC1000_SEQ_MOD             (1 << 12)
#define HDC1000_BTST_LOW            (1 << 11)
#define HDC1000_TRES_MSK            (1 << 10)
#define HDC1000_TRES11              (1 << 10)
#define HDC1000_TRES14              (0)
#define HDC1000_HRES_MSK            (1 << 9 | 1 << 8)
#define HDC1000_HRES14              (0)
#define HDC1000_HRES11              (1 << 8)
#define HDC1000_HRES8               (1 << 9)

/**
 * @brief   HDC1000 specific return values
 */
enum {
    HDC1000_OK     = 0,     /**< everything went as expected */
    HDC1000_NODEV  = -1,    /**< no HDC1000 device found on the bus */
    HDC1000_NOBUS  = -2,    /**< errors while initializing the I2C bus */
    HDC1000_BUSERR = -3     /**< error during I2C communication */
};

/**
 * @brief   Possible resolution values
 */
typedef enum {
    HDC1000_11BIT = (HDC1000_TRES11 | HDC1000_HRES11),  /**< 11-bit conversion */
    HDC1000_14BIT = (HDC1000_TRES14 | HDC1000_HRES14)   /**< 14-bit conversion */
} hdc1000_res_t;

/**
 * @brief   Initialize the given HDC1000 device
 *
 * @param[out] dev          device descriptor of sensor to initialize
 * @param[in]  params       configuration parameters
 *
 * @return                  HDC1000_OK on success
 * @return                  HDC1000_NOBUS if initialization of I2C bus fails
 * @return                  HDC1000_NODEV if no HDC1000 device found on bus
 */
int hdc1000_init(void);

#ifdef __cplusplus
}
#endif

#endif /* HDC1000_H */
/** @} */
