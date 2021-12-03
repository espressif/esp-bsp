/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#define HTS221_CALIBRATION_DATA_SIZE                      16u
#define HTS221_CALIBRATION_DATA_START          ((uint8_t)0x30)

/**
* @brief Device Identification register.
* \code
* Read
* Default value: 0xBC
* 7:0 This read-only register contains the device identifier for HTS221.
* \endcode
*/
#define HTS221_WHO_AM_I_REG         ((uint8_t)0x0F)

/**
* @brief Humidity and temperature average mode register.
* \code
* Read/write
* Default value: 0x1B
* 7:6 Reserved.
* 5:3 AVGT2-AVGT1-AVGT0: Select the temperature internal average.
*
*      AVGT2 | AVGT1 | AVGT0 | Nr. Internal Average
*   ----------------------------------------------------
*       0    |   0   |   0   |    2
*       0    |   0   |   1   |    4
*       0    |   1   |   0   |    8
*       0    |   1   |   1   |    16
*       1    |   0   |   0   |    32
*       1    |   0   |   1   |    64
*       1    |   1   |   0   |    128
*       1    |   1   |   1   |    256
*
* 2:0 AVGH2-AVGH1-AVGH0: Select humidity internal average.
*      AVGH2 | AVGH1 |  AVGH0 | Nr. Internal Average
*   ------------------------------------------------------
*       0    |   0   |   0   |    4
*       0    |   0   |   1   |    8
*       0    |   1   |   0   |    16
*       0    |   1   |   1   |    32
*       1    |   0   |   0   |    64
*       1    |   0   |   1   |    128
*       1    |   1   |   0   |    256
*       1    |   1   |   1   |    512
*
* \endcode
*/
#define HTS221_AV_CONF_REG        ((uint8_t)0x10)
#define HTS221_AVGH_MASK          ((uint8_t)0x07)
#define HTS221_AVGT_MASK          ((uint8_t)0x38)

/**
* @brief Control register 1.
* \code
* Read/write
* Default value: 0x00
* 7 PD: power down control. 0 - power down mode; 1 - active mode.
* 6:3 Reserved.
* 2 BDU: block data update. 0 - continuous update; 1 - output registers not updated until MSB and LSB reading.
* 1:0 ODR1, ODR0: output data rate selection.
*
*   ODR1  | ODR0  | Humidity output data-rate(Hz)  | Pressure output data-rate(Hz)
*   ----------------------------------------------------------------------------------
*     0   |   0   |         one shot               |         one shot
*     0   |   1   |            1                   |            1
*     1   |   0   |            7                   |            7
*     1   |   1   |           12.5                 |           12.5
*
* \endcode
*/
#define HTS221_CTRL_REG1      ((uint8_t)0x20)
#define HTS221_PD_MASK        ((uint8_t)0x80)
#define HTS221_BDU_MASK       ((uint8_t)0x04)
#define HTS221_ODR_MASK       ((uint8_t)0x03)
#define HTS221_BDU_BIT        2u
#define HTS221_PD_BIT         7u

/**
* @brief Control register 2.
* \code
* Read/write
* Default value: 0x00
* 7 BOOT:  Reboot memory content. 0: normal mode; 1: reboot memory content. Self-cleared upon completation.
* 6:2 Reserved.
* 1 HEATHER: 0: heater enable; 1: heater disable.
* 0 ONE_SHOT: 0: waiting for start of conversion; 1: start for a new dataset. Self-cleared upon completation.
* \endcode
*/
#define HTS221_CTRL_REG2       ((uint8_t)0x21)
#define HTS221_BOOT_MASK       ((uint8_t)0x80)
#define HTS221_HEATER_MASK     ((uint8_t)0x02)
#define HTS221_ONE_SHOT_MASK   ((uint8_t)0x01)
#define HTS221_HEATER_BIT      1u

/**
* @brief Control register 3.
* \code
* Read/write
* Default value: 0x00
* 7 DRDY_H_L: Interrupt edge. 0: active high, 1: active low.
* 6 PP_OD: Push-Pull/OpenDrain selection on interrupt pads. 0: push-pull; 1: open drain.
* 5:3 Reserved.
* 2 DRDY: interrupt config. 0: disable, 1: enable.
* \endcode
*/
#define HTS221_CTRL_REG3       ((uint8_t)0x22)
#define HTS221_DRDY_H_L_MASK   ((uint8_t)0x80)
#define HTS221_PP_OD_MASK      ((uint8_t)0x40)
#define HTS221_DRDY_MASK       ((uint8_t)0x04)
#define HTS221_DRDY_BIT        2u

/**
* @brief  Humidity data (LSB).
* \code
* Read
* Default value: 0x00.
* HOUT7 - HOUT0: Humidity data LSB (2's complement).
* \endcode
*/
#define HTS221_HR_OUT_L_REG        ((uint8_t)0x28)

/**
* @brief  Temperature data (LSB).
* \code
* Read
* Default value: 0x00.
* TOUT7 - TOUT0: temperature data LSB.
* \endcode
*/
#define HTS221_TEMP_OUT_L_REG       ((uint8_t)0x2A)
