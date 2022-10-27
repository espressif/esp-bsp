/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

/*
 *   ES7210_REGISTER NAME_REG_REGISTER ADDRESS
 */
#define  ES7210_RESET_REG00                 0x00        /* Reset control */
#define  ES7210_CLOCK_OFF_REG01             0x01        /* Used to turn off the ADC clock */
#define  ES7210_MAINCLK_REG02               0x02        /* Set ADC clock frequency division */
#define  ES7210_MASTER_CLK_REG03            0x03        /* MCLK source $ SCLK division */
#define  ES7210_LRCK_DIVH_REG04             0x04        /* lrck_divh */
#define  ES7210_LRCK_DIVL_REG05             0x05        /* lrck_divl */
#define  ES7210_POWER_DOWN_REG06            0x06        /* power down */
#define  ES7210_OSR_REG07                   0x07
#define  ES7210_MODE_CONFIG_REG08           0x08        /* Set master/slave & channels */
#define  ES7210_TIME_CONTROL0_REG09         0x09        /* Set Chip intial state period*/
#define  ES7210_TIME_CONTROL1_REG0A         0x0A        /* Set Power up state period */
#define  ES7210_SDP_INTERFACE1_REG11        0x11        /* Set sample & fmt */
#define  ES7210_SDP_INTERFACE2_REG12        0x12        /* Pins state */

#define  ES7210_ADC_AUTOMUTE_REG13          0x13        /* Set mute */
#define  ES7210_ADC34_MUTERANGE_REG14       0x14        /* Set mute range */
#define  ES7210_ALC_SEL_REG16               0x16        /* Set ALC mode */
#define  ES7210_ADC1_DIRECT_DB_REG1B        0x1B
#define  ES7210_ADC2_DIRECT_DB_REG1C        0x1C
#define  ES7210_ADC3_DIRECT_DB_REG1D        0x1D
#define  ES7210_ADC4_DIRECT_DB_REG1E        0x1E        /* ADC direct dB when ALC close, ALC max gain when ALC open */
#define  ES7210_ADC34_HPF2_REG20            0x20        /* HPF */
#define  ES7210_ADC34_HPF1_REG21            0x21
#define  ES7210_ADC12_HPF2_REG22            0x22
#define  ES7210_ADC12_HPF1_REG23            0x23
#define  ES7210_ANALOG_REG40                0x40        /* ANALOG Power */

#define  ES7210_MIC12_BIAS_REG41            0x41
#define  ES7210_MIC34_BIAS_REG42            0x42
#define  ES7210_MIC1_GAIN_REG43             0x43
#define  ES7210_MIC2_GAIN_REG44             0x44
#define  ES7210_MIC3_GAIN_REG45             0x45
#define  ES7210_MIC4_GAIN_REG46             0x46
#define  ES7210_MIC1_POWER_REG47            0x47
#define  ES7210_MIC2_POWER_REG48            0x48
#define  ES7210_MIC3_POWER_REG49            0x49
#define  ES7210_MIC4_POWER_REG4A            0x4A
#define  ES7210_MIC12_POWER_REG4B           0x4B        /* MICBias & ADC & PGA Power */
#define  ES7210_MIC34_POWER_REG4C           0x4C
