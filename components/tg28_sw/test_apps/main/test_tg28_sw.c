/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "tg28_sw.h"
#include "tg28_sw_priv.h"

#define TEST_REG_MODE          0x17
#define TEST_REG_MODULE_ENABLE 0x18
#define TEST_REG_MODEL         0xA1
#define TEST_REG_GAUGE_CONTROL 0xA2
#define TEST_GAUGE_RESET_MASK  (1U << 2)
#define TEST_GAUGE_ENABLE_MASK (1U << 3)
#define TEST_CHARGER_MASK      (1U << 1)
#define TEST_WATCHDOG_MASK     (1U << 0)
#define TEST_SOURCE_MASK       (1U << 4)
#define TEST_BROM_MASK         (1U << 0)

typedef struct {
    uint8_t registers[256];
    uint8_t rom[TG28_SW_BATTERY_MODEL_SIZE];
    uint8_t sram[TG28_SW_BATTERY_MODEL_SIZE];
    size_t model_index;
    bool active_sram;
    bool brom_window_sram;
    unsigned reset_count;
    unsigned delay_count;
    uint8_t fail_read_reg;
    unsigned fail_read_on;
    unsigned read_count[256];
    uint8_t fail_write_reg;
    unsigned fail_write_on;
    unsigned fail_write_times;
    bool failed_write_applies;
    unsigned write_count[256];
} fake_tg28_t;

static void fake_apply_write(fake_tg28_t *fake, uint8_t reg, uint8_t value)
{
    if (reg == TEST_REG_MODEL) {
        if ((fake->registers[TEST_REG_GAUGE_CONTROL] & TEST_BROM_MASK) &&
                fake->model_index < TG28_SW_BATTERY_MODEL_SIZE) {
            fake->sram[fake->model_index++] = value;
            fake->brom_window_sram = true;
        }
        return;
    }

    const uint8_t previous = fake->registers[reg];
    fake->registers[reg] = value;
    if (reg == TEST_REG_GAUGE_CONTROL &&
            !(previous & TEST_BROM_MASK) && (value & TEST_BROM_MASK)) {
        fake->model_index = 0;
    }
    if (reg == TEST_REG_MODE &&
            (previous & TEST_GAUGE_RESET_MASK) &&
            !(value & TEST_GAUGE_RESET_MASK)) {
        ++fake->reset_count;
        if (fake->registers[TEST_REG_MODULE_ENABLE] & TEST_GAUGE_ENABLE_MASK) {
            fake->active_sram =
                !!(fake->registers[TEST_REG_GAUGE_CONTROL] & TEST_SOURCE_MASK);
        }
        fake->brom_window_sram = false;
    }
}

static esp_err_t fake_read(void *context, uint8_t reg, void *data, size_t size)
{
    fake_tg28_t *fake = context;
    assert(size == 1);
    ++fake->read_count[reg];
    if (reg == fake->fail_read_reg &&
            fake->read_count[reg] == fake->fail_read_on) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t value = fake->registers[reg];
    if (reg == TEST_REG_MODEL &&
            (fake->registers[TEST_REG_GAUGE_CONTROL] & TEST_BROM_MASK) &&
            fake->model_index < TG28_SW_BATTERY_MODEL_SIZE) {
        const uint8_t *window = fake->brom_window_sram ? fake->sram :
                                fake->rom;
        value = window[fake->model_index++];
    }
    *(uint8_t *)data = value;
    return ESP_OK;
}

static esp_err_t fake_write(void *context, uint8_t reg, const void *data,
                            size_t size)
{
    fake_tg28_t *fake = context;
    assert(size == 1);
    ++fake->write_count[reg];
    const uint8_t value = *(const uint8_t *)data;
    const unsigned fail_write_times = fake->fail_write_times == 0 ?
                                      1 : fake->fail_write_times;
    if (reg == fake->fail_write_reg && fake->fail_write_on != 0 &&
            fake->write_count[reg] >= fake->fail_write_on &&
            fake->write_count[reg] < fake->fail_write_on + fail_write_times) {
        if (fake->failed_write_applies) {
            fake_apply_write(fake, reg, value);
        }
        return ESP_ERR_TIMEOUT;
    }
    fake_apply_write(fake, reg, value);
    return ESP_OK;
}

static void fake_delay_ms(void *context, uint32_t milliseconds)
{
    fake_tg28_t *fake = context;
    assert(milliseconds == 1000);
    ++fake->delay_count;
}

static tg28_sw_register_io_t fake_io(fake_tg28_t *fake)
{
    return (tg28_sw_register_io_t) {
        .context = fake,
        .read = fake_read,
        .write = fake_write,
        .delay_ms = fake_delay_ms,
    };
}

static void fake_init(fake_tg28_t *fake, uint8_t gauge_control,
                      uint8_t module_enable)
{
    memset(fake, 0, sizeof(*fake));
    fake->registers[TEST_REG_GAUGE_CONTROL] = gauge_control;
    fake->registers[TEST_REG_MODULE_ENABLE] = module_enable;
    fake->active_sram = !!(gauge_control & TEST_SOURCE_MASK);
    for (size_t i = 0; i < TG28_SW_BATTERY_MODEL_SIZE; ++i) {
        fake->rom[i] = (uint8_t)(i ^ 0xA5);
        fake->sram[i] = (uint8_t)(i ^ 0x5A);
    }
}

static void test_chip_id_and_names(void)
{
    assert(tg28_sw_is_supported_chip_id(0x47));
    assert(tg28_sw_is_supported_chip_id(0x4A));
    assert(!tg28_sw_is_supported_chip_id(0x00));

    assert(TG28_SW_REGULATOR_COUNT == 13);
    assert(strcmp(tg28_sw_regulator_name(TG28_SW_DCDC1), "dcdc1") == 0);
    assert(strcmp(tg28_sw_regulator_name(TG28_SW_ALDO4), "aldo4") == 0);
    assert(strcmp(tg28_sw_regulator_name(TG28_SW_BLDO2), "bldo2") == 0);
    assert(strcmp(tg28_sw_regulator_name(TG28_SW_CPUSLDO), "cpusldo") == 0);
    assert(strcmp(tg28_sw_regulator_name(TG28_SW_DLDO1), "dldo1") == 0);
    assert(strcmp(tg28_sw_regulator_name(TG28_SW_DLDO2), "dldo2") == 0);
    assert(strcmp(tg28_sw_regulator_name(TG28_SW_REGULATOR_COUNT), "invalid") == 0);
}

static void test_charge_current_coding(void)
{
    uint8_t code = 0;
    assert(tg28_sw_encode_charge_current(0, &code) == ESP_OK && code == 0);
    assert(tg28_sw_encode_charge_current(25, &code) == ESP_OK && code == 1);
    assert(tg28_sw_encode_charge_current(200, &code) == ESP_OK && code == 8);
    assert(tg28_sw_encode_charge_current(300, &code) == ESP_OK && code == 9);
    assert(tg28_sw_encode_charge_current(1500, &code) == ESP_OK && code == 21);
    assert(tg28_sw_encode_charge_current(225, &code) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_charge_current(250, &code) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_charge_current(1600, &code) == ESP_ERR_INVALID_ARG);

    assert(tg28_sw_decode_charge_current(0) == 0);
    assert(tg28_sw_decode_charge_current(8) == 200);
    assert(tg28_sw_decode_charge_current(9) == 300);
    assert(tg28_sw_decode_charge_current(21) == 1500);
}

static void test_input_current_limit_coding(void)
{
    static const uint16_t levels[] = {100, 500, 900, 1000, 1500, 2000};
    for (uint8_t i = 0; i < sizeof(levels) / sizeof(levels[0]); ++i) {
        uint8_t code = 0;
        assert(tg28_sw_encode_input_current_limit(levels[i], &code) == ESP_OK);
        assert(code == i);
        assert(tg28_sw_decode_input_current_limit(code) == levels[i]);
    }

    uint8_t code = 0;
    assert(tg28_sw_encode_input_current_limit(2500, &code) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_input_current_limit(0, &code) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_decode_input_current_limit(6) == 0);
    assert(tg28_sw_decode_input_current_limit(7) == 0);
}

static void test_charge_voltage_coding(void)
{
    /* Switch-charger variant: code 0 is reserved, valid codes are 1-5
     * (datasheet 6.13.2.63). The linear-variant 3900mV level is gone. */
    static const uint16_t levels[] = {4000, 4100, 4200, 4350, 4400};
    for (uint8_t i = 0; i < sizeof(levels) / sizeof(levels[0]); ++i) {
        uint8_t code = 0;
        assert(tg28_sw_encode_charge_voltage(levels[i], &code) == ESP_OK);
        assert(code == i + 1);
        assert(tg28_sw_decode_charge_voltage(code) == levels[i]);
    }

    uint8_t code = 0;
    assert(tg28_sw_encode_charge_voltage(3900, &code) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_charge_voltage(4300, &code) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_charge_voltage(4500, &code) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_charge_voltage(3800, &code) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_decode_charge_voltage(0) == 0);
    assert(tg28_sw_decode_charge_voltage(6) == 0);
    assert(tg28_sw_decode_charge_voltage(7) == 0);
}

static void test_vindpm_coding(void)
{
    uint8_t code = 0;
    /* Full hardware window: 3880 + 80 * code, codes 0-15 (6.13.2.11). */
    assert(tg28_sw_encode_vindpm(3880, &code) == ESP_OK && code == 0);
    assert(tg28_sw_encode_vindpm(4040, &code) == ESP_OK && code == 2);
    assert(tg28_sw_encode_vindpm(4360, &code) == ESP_OK && code == 6);
    assert(tg28_sw_encode_vindpm(4680, &code) == ESP_OK && code == 10);
    assert(tg28_sw_encode_vindpm(5080, &code) == ESP_OK && code == 15);
    assert(tg28_sw_encode_vindpm(3800, &code) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_vindpm(5160, &code) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_vindpm(4000, &code) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_vindpm(4700, &code) == ESP_ERR_INVALID_ARG);

    assert(tg28_sw_decode_vindpm(0) == 3880);
    assert(tg28_sw_decode_vindpm(2) == 4040);
    assert(tg28_sw_decode_vindpm(10) == 4680);
    assert(tg28_sw_decode_vindpm(15) == 5080);
}

static void test_regulator_voltage_coding(void)
{
    uint8_t code = 0;

    /* ALDO4: the whole 500-3500mV range must be reachable, including
     * 3500mV (code 30), which the vendor two-segment table cannot encode. */
    assert(tg28_sw_encode_regulator_voltage(TG28_SW_ALDO4, 500, &code) == ESP_OK &&
           code == 0);
    assert(tg28_sw_encode_regulator_voltage(TG28_SW_ALDO4, 3400, &code) == ESP_OK &&
           code == 29);
    assert(tg28_sw_encode_regulator_voltage(TG28_SW_ALDO4, 3500, &code) == ESP_OK &&
           code == 30);
    assert(tg28_sw_encode_regulator_voltage(TG28_SW_ALDO4, 3550, &code) ==
           ESP_ERR_INVALID_ARG);
    assert(tg28_sw_decode_regulator_voltage(TG28_SW_ALDO4, 30) == 3500);
    assert(tg28_sw_decode_regulator_voltage(TG28_SW_ALDO4, 31) == 3500);

    /* DLDO1: 500-3300mV in 100mV steps. The REG99 enumeration stops at code
     * 28 (3.3V) and marks codes 29-31 reserved, so 3400mV and above are
     * rejected on encode and out-of-range codes clamp to 3300mV on decode. */
    assert(tg28_sw_encode_regulator_voltage(TG28_SW_DLDO1, 3300, &code) == ESP_OK &&
           code == 28);
    assert(tg28_sw_encode_regulator_voltage(TG28_SW_DLDO1, 3350, &code) ==
           ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_regulator_voltage(TG28_SW_DLDO1, 3400, &code) ==
           ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_regulator_voltage(TG28_SW_DLDO1, 3500, &code) ==
           ESP_ERR_INVALID_ARG);
    assert(tg28_sw_decode_regulator_voltage(TG28_SW_DLDO1, 28) == 3300);
    assert(tg28_sw_decode_regulator_voltage(TG28_SW_DLDO1, 31) == 3300);

    /* DLDO2: 500-1400mV in 50mV steps. */
    assert(tg28_sw_encode_regulator_voltage(TG28_SW_DLDO2, 500, &code) == ESP_OK &&
           code == 0);
    assert(tg28_sw_encode_regulator_voltage(TG28_SW_DLDO2, 1400, &code) == ESP_OK &&
           code == 18);
    assert(tg28_sw_encode_regulator_voltage(TG28_SW_DLDO2, 1450, &code) ==
           ESP_ERR_INVALID_ARG);
    assert(tg28_sw_decode_regulator_voltage(TG28_SW_DLDO2, 18) == 1400);

    /* CPUSLDO: 500-1400mV in 50mV steps (REG98), same coding as DLDO2. */
    assert(tg28_sw_encode_regulator_voltage(TG28_SW_CPUSLDO, 500, &code) == ESP_OK &&
           code == 0);
    assert(tg28_sw_encode_regulator_voltage(TG28_SW_CPUSLDO, 1400, &code) == ESP_OK &&
           code == 18);
    assert(tg28_sw_encode_regulator_voltage(TG28_SW_CPUSLDO, 1425, &code) ==
           ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_regulator_voltage(TG28_SW_CPUSLDO, 1450, &code) ==
           ESP_ERR_INVALID_ARG);
    assert(tg28_sw_decode_regulator_voltage(TG28_SW_CPUSLDO, 18) == 1400);
    assert(tg28_sw_decode_regulator_voltage(TG28_SW_CPUSLDO, 31) == 1400);

    /* Existing rails keep their coding: BLDO1 full range, DCDC2 two segments. */
    assert(tg28_sw_encode_regulator_voltage(TG28_SW_BLDO1, 3500, &code) == ESP_OK &&
           code == 30);
    assert(tg28_sw_encode_regulator_voltage(TG28_SW_DCDC2, 1200, &code) == ESP_OK);
    assert(tg28_sw_decode_regulator_voltage(TG28_SW_DCDC2,
                                            70) == 1200);
}

static void test_ts_current_coding(void)
{
    static const uint16_t levels[] = {20, 40, 50, 60};
    for (uint8_t i = 0; i < sizeof(levels) / sizeof(levels[0]); ++i) {
        uint8_t code = 0;
        assert(tg28_sw_encode_ts_current(levels[i], &code) == ESP_OK);
        assert(code == i);
    }

    uint8_t code = 0;
    assert(tg28_sw_encode_ts_current(0, &code) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_ts_current(30, &code) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_ts_current(80, &code) == ESP_ERR_INVALID_ARG);
}

static void test_adc_channel_coding(void)
{
    /* VBAT/VBUS/VSYS convert at 1mV/LSB; the high byte keeps only 6 bits. */
    assert(tg28_sw_decode_adc_channel(TG28_SW_ADC_CHANNEL_VBAT, 0x00, 0x00) == 0);
    assert(tg28_sw_decode_adc_channel(TG28_SW_ADC_CHANNEL_VBAT, 0x0E, 0x96) == 3734);
    assert(tg28_sw_decode_adc_channel(TG28_SW_ADC_CHANNEL_VBUS, 0x13, 0x88) == 5000);
    assert(tg28_sw_decode_adc_channel(TG28_SW_ADC_CHANNEL_VSYS, 0x0E, 0x96) == 3734);
    /* REG34[7:6] carry ch_dbg_en and must be masked out of the value. */
    assert(tg28_sw_decode_adc_channel(TG28_SW_ADC_CHANNEL_VBAT, 0xCE, 0x96) == 3734);

    /* TS converts at 0.5mV/LSB: 0x3E8 (1000) codes equal the 0.500V table
     * entry for a 10kOhm NTC at 25C (datasheet Table 6-5). */
    assert(tg28_sw_decode_adc_channel(TG28_SW_ADC_CHANNEL_TS, 0x03, 0xE8) == 500);

    /* TDIE converts at 0.1mV/LSB, truncating sub-millivolt resolution. */
    assert(tg28_sw_decode_adc_channel(TG28_SW_ADC_CHANNEL_TDIE, 0x03, 0xE8) == 100);

    assert(tg28_sw_decode_adc_channel(TG28_SW_ADC_CHANNEL_COUNT, 0x0E, 0x96) == 0);
}

static void test_precharge_and_termination_coding(void)
{
    /* REG61 precharge and REG63 termination share 25mA steps, codes 0-8. */
    for (uint8_t expected = 0; expected <= 8; ++expected) {
        uint8_t code = 0;
        assert(tg28_sw_encode_precharge_current(expected * 25, &code) == ESP_OK);
        assert(code == expected);
        assert(tg28_sw_decode_precharge_current(code) == expected * 25);
        assert(tg28_sw_encode_termination_current(expected * 25, &code) ==
               ESP_OK);
        assert(code == expected);
        assert(tg28_sw_decode_termination_current(code) == expected * 25);
    }

    uint8_t code = 0;
    assert(tg28_sw_encode_precharge_current(225, &code) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_precharge_current(250, &code) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_termination_current(225, &code) ==
           ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_termination_current(210, &code) ==
           ESP_ERR_INVALID_ARG);
    /* Codes 9-15 are reserved and decode as 0. */
    assert(tg28_sw_decode_precharge_current(9) == 0);
    assert(tg28_sw_decode_precharge_current(15) == 0);
    assert(tg28_sw_decode_termination_current(9) == 0);
    assert(tg28_sw_decode_termination_current(15) == 0);
}

static void test_switch_names(void)
{
    assert(TG28_SW_SWITCH_COUNT == 2);
    assert(strcmp(tg28_sw_switch_name(TG28_SW_SWITCH_DC1SW), "dc1sw") == 0);
    assert(strcmp(tg28_sw_switch_name(TG28_SW_SWITCH_DC4SW), "dc4sw") == 0);
    assert(strcmp(tg28_sw_switch_name(TG28_SW_SWITCH_COUNT), "invalid") == 0);
}

/* IRQ slot numbering: value = bank * 8 + bit, matching the enable registers
 * REG40-REG42 and the status registers REG48-REG4A bit for bit (datasheet
 * 6.13.2.41-46). Slot 21 (REG42 bit5) is reserved and has no symbol. */
_Static_assert(TG28_SW_IRQ_BWUT == 0 && TG28_SW_IRQ_BWOT == 1 &&
               TG28_SW_IRQ_BCUT == 2 && TG28_SW_IRQ_BCOT == 3 &&
               TG28_SW_IRQ_LOWSOC == 4 && TG28_SW_IRQ_GWDT == 5 &&
               TG28_SW_IRQ_SOCWL1 == 6 && TG28_SW_IRQ_SOCWL2 == 7,
               "IRQ bank0 slots must match REG40 bits");
_Static_assert(TG28_SW_IRQ_PONPE == 8 && TG28_SW_IRQ_PONNE == 9 &&
               TG28_SW_IRQ_PONLP == 10 && TG28_SW_IRQ_PONSP == 11 &&
               TG28_SW_IRQ_BREMOVE == 12 && TG28_SW_IRQ_BINSERT == 13 &&
               TG28_SW_IRQ_VREMOVE == 14 && TG28_SW_IRQ_VINSERT == 15,
               "IRQ bank1 slots must match REG41 bits");
_Static_assert(TG28_SW_IRQ_BOVP == 16 && TG28_SW_IRQ_CHGTE == 17 &&
               TG28_SW_IRQ_DOTL1 == 18 && TG28_SW_IRQ_CHGST == 19 &&
               TG28_SW_IRQ_CHGDN == 20 && TG28_SW_IRQ_LDOOC == 22 &&
               TG28_SW_IRQ_WDEXP == 23 && TG28_SW_IRQ_COUNT == 24,
               "IRQ bank2 slots must match REG42 bits");

static void test_irq_numbering(void)
{
    /* The legacy power-key macros must stay aliases of the bank1 bits. */
    assert(TG28_SW_POWER_KEY_IRQ_POSITIVE_EDGE ==
           (1U << (TG28_SW_IRQ_PONPE % 8)));
    assert(TG28_SW_POWER_KEY_IRQ_NEGATIVE_EDGE ==
           (1U << (TG28_SW_IRQ_PONNE % 8)));
    assert(TG28_SW_POWER_KEY_IRQ_LONG_PRESS == (1U << (TG28_SW_IRQ_PONLP % 8)));
    assert(TG28_SW_POWER_KEY_IRQ_SHORT_PRESS == (1U << (TG28_SW_IRQ_PONSP % 8)));
    assert(TG28_SW_IRQ_PONPE / 8 == TG28_SW_IRQ_BANK1);
    assert(TG28_SW_IRQ_VINSERT / 8 == TG28_SW_IRQ_BANK1);
    assert(TG28_SW_IRQ_WDEXP / 8 == TG28_SW_IRQ_BANK2);
}

static void test_low_battery_warning_coding(void)
{
    /* REG1A: bits 3:0 level1 0-15%, bits 7:4 level2 5-20% (code = % - 5). */
    uint8_t value = 0;
    assert(tg28_sw_encode_low_battery_warning(0, 5, &value) == ESP_OK &&
           value == 0x00);
    assert(tg28_sw_encode_low_battery_warning(15, 20, &value) == ESP_OK &&
           value == 0xFF);
    /* The POR reset value 0xA1 means level1 = 1% and level2 = 15%. */
    assert(tg28_sw_encode_low_battery_warning(1, 15, &value) == ESP_OK &&
           value == 0xA1);
    assert(tg28_sw_encode_low_battery_warning(16, 10, &value) ==
           ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_low_battery_warning(10, 4, &value) ==
           ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_low_battery_warning(10, 21, &value) ==
           ESP_ERR_INVALID_ARG);

    uint8_t level1 = 0;
    uint8_t level2 = 0;
    tg28_sw_decode_low_battery_warning(0xA1, &level1, &level2);
    assert(level1 == 1 && level2 == 15);
    tg28_sw_decode_low_battery_warning(0x00, &level1, &level2);
    assert(level1 == 0 && level2 == 5);
    tg28_sw_decode_low_battery_warning(0xFF, &level1, &level2);
    assert(level1 == 15 && level2 == 20);
}

static void test_powerkey_level_coding(void)
{
    /* REG27 (datasheet 6.13.2.25): bits5:4 IRQLEVEL 1000-2500 ms,
     * bits3:2 OFFLEVEL 4000-10000 ms, bits1:0 ONLEVEL 128-2000 ms. Every
     * selectable triplet round-trips, so the uint16_t struct fields can
     * carry each window without truncation. */
    static const uint16_t irqlevel[] = {1000, 1500, 2000, 2500};
    static const uint16_t offlevel[] = {4000, 6000, 8000, 10000};
    static const uint16_t onlevel[] = {128, 512, 1000, 2000};
    for (uint8_t i = 0; i < 4; ++i) {
        for (uint8_t j = 0; j < 4; ++j) {
            for (uint8_t k = 0; k < 4; ++k) {
                uint8_t value = 0;
                assert(tg28_sw_encode_powerkey_levels(irqlevel[i], offlevel[j],
                                                      onlevel[k], &value) == ESP_OK);
                assert(value == (uint8_t)((i << 4) | (j << 2) | k));
                uint16_t irq_ms = 0, off_ms = 0, on_ms = 0;
                tg28_sw_decode_powerkey_levels(value, &irq_ms, &off_ms, &on_ms);
                assert(irq_ms == irqlevel[i]);
                assert(off_ms == offlevel[j]);
                assert(on_ms == onlevel[k]);
            }
        }
    }

    uint8_t value = 0;
    assert(tg28_sw_encode_powerkey_levels(0, 4000, 128, &value) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_powerkey_levels(1000, 0, 128, &value) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_powerkey_levels(1000, 4000, 0, &value) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_powerkey_levels(3000, 4000, 128, &value) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_powerkey_levels(1000, 11000, 128, &value) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_encode_powerkey_levels(1000, 4000, 2001, &value) == ESP_ERR_INVALID_ARG);

    uint16_t irq_ms = 0, off_ms = 0, on_ms = 0;
    tg28_sw_decode_powerkey_levels(0xFF, &irq_ms, &off_ms, &on_ms);
    assert(irq_ms == 2500 && off_ms == 10000 && on_ms == 2000);

    /* The widest windows exceed 255 ms, so the config fields must stay
     * wide enough to hold them. */
    tg28_sw_poweroff_config_t poweroff = {0};
    assert(sizeof(poweroff.irqlevel_ms) == sizeof(uint16_t));
    assert(sizeof(poweroff.offlevel_ms) == sizeof(uint16_t));
    assert(sizeof(poweroff.onlevel_ms) == sizeof(uint16_t));
}

/* The feature-block enums are the register codes themselves (datasheet
 * 6.13.2.x); assert the ordering so an accidental reorder breaks the build
 * tests instead of silently programming wrong values. */
static void test_feature_block_enum_codings(void)
{
    assert(TG28_SW_BATTERY_MODEL_SIZE == 128);

    assert(TG28_SW_WATCHDOG_PERIOD_1S == 0 && TG28_SW_WATCHDOG_PERIOD_128S == 7);
    assert(TG28_SW_WATCHDOG_ACTION_IRQ_ONLY == 0 &&
           TG28_SW_WATCHDOG_ACTION_IRQ_RESET_PWRCYCLE == 3);

    assert(TG28_SW_JEITA_CV_ADJUST_NONE == 0 && TG28_SW_JEITA_CV_ADJUST_TWO_GEARS == 2);

    assert(TG28_SW_THERMAL_REGULATION_60C == 0 && TG28_SW_THERMAL_REGULATION_120C == 3);

    assert(TG28_SW_PRECHARGE_TIMER_40MIN == 0 && TG28_SW_PRECHARGE_TIMER_70MIN == 3);
    assert(TG28_SW_CHARGE_TIMER_5H == 0 && TG28_SW_CHARGE_TIMER_20H == 3);

    assert(TG28_SW_CHARGE_LED_TYPE_A == 0 && TG28_SW_CHARGE_LED_MANUAL == 2);
    assert(TG28_SW_CHARGE_LED_MANUAL_HIZ == 0 && TG28_SW_CHARGE_LED_MANUAL_LOW == 3);

    assert(TG28_SW_GPIO1_OUTPUT_HIZ == 0 && TG28_SW_GPIO1_OUTPUT_LOW == 1);
}

/* Every public entry point validates its handle before touching the bus, so
 * NULL calls fail cheaply without any hardware. */
static void test_feature_block_null_rejections(void)
{
    bool flag = false;
    uint16_t mv = 0;
    tg28_sw_watchdog_config_t watchdog = {0};
    tg28_sw_ts_thresholds_t thresholds = {0};
    tg28_sw_jeita_config_t jeita = {0};
    tg28_sw_charge_timers_t timers = {0};
    tg28_sw_charge_led_config_t charge_led = {0};
    tg28_sw_poweroff_config_t poweroff = {0};
    tg28_sw_sleep_config_t sleep_cfg = {0};
    tg28_sw_dcdc_mode_t dcdc = {0};

    assert(tg28_sw_set_charge_enable(NULL, true) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_get_charge_enable(NULL, &flag) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_set_watchdog(NULL, &watchdog) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_get_watchdog(NULL, &watchdog) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_feed_watchdog(NULL) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_set_gauge_enable(NULL, true) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_get_gauge_enable(NULL, &flag) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_set_ts_thresholds(NULL, &thresholds) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_get_ts_thresholds(NULL, &thresholds) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_set_jeita(NULL, &jeita) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_get_jeita(NULL, &jeita) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_set_ts_fixed_threshold(NULL, 0) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_get_ts_fixed_threshold(NULL, &mv) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_set_thermal_regulation(NULL, TG28_SW_THERMAL_REGULATION_80C) ==
           ESP_ERR_INVALID_ARG);
    assert(tg28_sw_get_thermal_regulation(NULL, NULL) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_set_charge_timers(NULL, &timers) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_get_charge_timers(NULL, &timers) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_set_battery_detect_enable(NULL, true) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_get_battery_detect_enable(NULL, &flag) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_set_charge_led(NULL, &charge_led) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_get_charge_led(NULL, &charge_led) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_set_backup_charge(NULL, true, 3000) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_get_backup_charge(NULL, &flag, &mv) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_set_min_sys_voltage(NULL, 3500) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_get_min_sys_voltage(NULL, &mv) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_set_poweroff_config(NULL, &poweroff) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_get_poweroff_config(NULL, &poweroff) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_set_sleep_config(NULL, &sleep_cfg) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_get_sleep_config(NULL, &sleep_cfg) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_set_gpio1_config(NULL, TG28_SW_GPIO1_OUTPUT_LOW) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_get_gpio1_config(NULL, NULL) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_set_dcdc_mode(NULL, &dcdc) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_get_dcdc_mode(NULL, &dcdc) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_soft_reset(NULL) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_set_batfet_off_state_enable(NULL, true) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_get_batfet_off_state_enable(NULL, &flag) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_write_register(NULL, 0x10, 0x00) == ESP_ERR_INVALID_ARG);
    assert(tg28_sw_read_battery_model(NULL, NULL, 0) ==
           ESP_ERR_INVALID_ARG);

    /* The 14-bit ts_cfg_data bound is validated before any bus access. */
    assert(tg28_sw_set_ts_fixed_threshold(NULL, 0x4000) == ESP_ERR_INVALID_ARG);
}

static void test_battery_model_read_state_machine(void)
{
    fake_tg28_t fake;
    uint8_t model[TG28_SW_BATTERY_MODEL_SIZE] = {0};

    /* REGA1 exposes one BROM parameter image; REGA2 bit4 selects the runtime
     * source for the gauge MCU and must remain unchanged throughout a read. */
    fake_init(&fake, TEST_SOURCE_MASK, 0x00);
    tg28_sw_register_io_t io = fake_io(&fake);
    assert(tg28_sw_read_battery_model_io(&io, model, sizeof(model)) == ESP_OK);
    assert(memcmp(model, fake.rom, sizeof(model)) == 0);
    assert(fake.registers[TEST_REG_GAUGE_CONTROL] == TEST_SOURCE_MASK);
    assert(fake.registers[TEST_REG_MODULE_ENABLE] == 0x00);
    assert(fake.reset_count == 2);

    memset(model, 0, sizeof(model));
    fake_init(&fake, 0x00,
              TEST_GAUGE_ENABLE_MASK | TEST_CHARGER_MASK | TEST_WATCHDOG_MASK);
    io = fake_io(&fake);
    assert(tg28_sw_read_battery_model_io(&io, model, sizeof(model)) == ESP_OK);
    assert(memcmp(model, fake.rom, sizeof(model)) == 0);
    assert(fake.registers[TEST_REG_GAUGE_CONTROL] == 0x00);
    assert(fake.registers[TEST_REG_MODULE_ENABLE] ==
           (TEST_GAUGE_ENABLE_MASK | TEST_CHARGER_MASK | TEST_WATCHDOG_MASK));
    assert(fake.reset_count == 2);
}

static void test_battery_model_read_failures(void)
{
    fake_tg28_t fake;
    uint8_t model[TG28_SW_BATTERY_MODEL_SIZE] = {0};

    /* If the entry REGA2 snapshot cannot be read, no register may be written
     * and no reset may occur. */
    fake_init(&fake, TEST_SOURCE_MASK,
              TEST_GAUGE_ENABLE_MASK | TEST_WATCHDOG_MASK);
    fake.fail_read_reg = TEST_REG_GAUGE_CONTROL;
    fake.fail_read_on = 1;
    tg28_sw_register_io_t io = fake_io(&fake);
    assert(tg28_sw_read_battery_model_io(&io, model, sizeof(model)) ==
           ESP_ERR_TIMEOUT);
    assert(fake.write_count[TEST_REG_GAUGE_CONTROL] == 0);
    assert(fake.write_count[TEST_REG_MODE] == 0);
    assert(fake.write_count[TEST_REG_MODULE_ENABLE] == 0);

    /* If reset assertion was accepted but its readback fails, the helper
     * must still release REG17 bit2 before restoring the entry watchdog. */
    fake_init(&fake, TEST_SOURCE_MASK,
              TEST_GAUGE_ENABLE_MASK | TEST_WATCHDOG_MASK);
    fake.fail_read_reg = TEST_REG_MODE;
    fake.fail_read_on = 2;
    io = fake_io(&fake);
    assert(tg28_sw_read_battery_model_io(&io, model, sizeof(model)) ==
           ESP_ERR_TIMEOUT);
    assert(!(fake.registers[TEST_REG_MODE] & TEST_GAUGE_RESET_MASK));
    assert(fake.registers[TEST_REG_MODULE_ENABLE] ==
           (TEST_GAUGE_ENABLE_MASK | TEST_WATCHDOG_MASK));
    assert(fake.write_count[TEST_REG_GAUGE_CONTROL] == 0);

    /* One failed deassert write is retried. The operation still reports the
     * transport error, but reset is released and modules can be restored. */
    fake_init(&fake, TEST_SOURCE_MASK,
              TEST_GAUGE_ENABLE_MASK | TEST_WATCHDOG_MASK);
    fake.fail_write_reg = TEST_REG_MODE;
    fake.fail_write_on = 2;
    io = fake_io(&fake);
    assert(tg28_sw_read_battery_model_io(&io, model, sizeof(model)) ==
           ESP_ERR_TIMEOUT);
    assert(!(fake.registers[TEST_REG_MODE] & TEST_GAUGE_RESET_MASK));
    assert(fake.registers[TEST_REG_MODULE_ENABLE] ==
           (TEST_GAUGE_ENABLE_MASK | TEST_WATCHDOG_MASK));

    /* If both deassert attempts fail, reset remains unconfirmed and cleanup
     * must not restore a watchdog that could later reset an unsafe gauge. */
    fake_init(&fake, TEST_SOURCE_MASK,
              TEST_GAUGE_ENABLE_MASK | TEST_WATCHDOG_MASK);
    fake.fail_write_reg = TEST_REG_MODE;
    fake.fail_write_on = 2;
    fake.fail_write_times = 2;
    io = fake_io(&fake);
    assert(tg28_sw_read_battery_model_io(&io, model, sizeof(model)) ==
           ESP_ERR_TIMEOUT);
    assert(fake.registers[TEST_REG_MODE] & TEST_GAUGE_RESET_MASK);
    assert(!(fake.registers[TEST_REG_MODULE_ENABLE] & TEST_WATCHDOG_MASK));

    /* A model-stream error must close BROM, restore the entry source, reset
     * on the restored source, and restore watchdog/module state. */
    fake_init(&fake, TEST_SOURCE_MASK,
              TEST_GAUGE_ENABLE_MASK | TEST_CHARGER_MASK | TEST_WATCHDOG_MASK);
    fake.fail_read_reg = TEST_REG_MODEL;
    fake.fail_read_on = 17;
    io = fake_io(&fake);
    assert(tg28_sw_read_battery_model_io(&io, model, sizeof(model)) ==
           ESP_ERR_TIMEOUT);
    assert(fake.registers[TEST_REG_GAUGE_CONTROL] == TEST_SOURCE_MASK);
    assert(fake.registers[TEST_REG_MODULE_ENABLE] ==
           (TEST_GAUGE_ENABLE_MASK | TEST_CHARGER_MASK | TEST_WATCHDOG_MASK));
    assert(fake.reset_count == 2);

    /* If the final atomic REGA2 close/restore cannot be confirmed, do not
     * reset on the temporary source and do not re-enable the watchdog. */
    fake_init(&fake, TEST_SOURCE_MASK,
              TEST_GAUGE_ENABLE_MASK | TEST_WATCHDOG_MASK);
    fake.fail_write_reg = TEST_REG_GAUGE_CONTROL;
    fake.fail_write_on = 3;
    io = fake_io(&fake);
    assert(tg28_sw_read_battery_model_io(&io, model, sizeof(model)) ==
           ESP_ERR_TIMEOUT);
    assert(fake.registers[TEST_REG_GAUGE_CONTROL] & TEST_BROM_MASK);
    assert(fake.reset_count == 1);
    assert(!(fake.registers[TEST_REG_MODULE_ENABLE] & TEST_WATCHDOG_MASK));

    /* A write timeout after the PMIC accepted the safe final byte is
     * resolved by readback and may proceed with reset/module restoration. */
    fake_init(&fake, TEST_SOURCE_MASK,
              TEST_GAUGE_ENABLE_MASK | TEST_WATCHDOG_MASK);
    fake.fail_write_reg = TEST_REG_GAUGE_CONTROL;
    fake.fail_write_on = 3;
    fake.failed_write_applies = true;
    io = fake_io(&fake);
    assert(tg28_sw_read_battery_model_io(&io, model, sizeof(model)) == ESP_OK);
    assert(fake.registers[TEST_REG_GAUGE_CONTROL] == TEST_SOURCE_MASK);
    assert(fake.registers[TEST_REG_MODULE_ENABLE] ==
           (TEST_GAUGE_ENABLE_MASK | TEST_WATCHDOG_MASK));
    assert(fake.reset_count == 2);
}

static void test_battery_model_program_state_machine(void)
{
    fake_tg28_t fake;
    uint8_t model[TG28_SW_BATTERY_MODEL_SIZE];
    for (size_t i = 0; i < sizeof(model); ++i) {
        model[i] = (uint8_t)(0xF0 ^ i);
    }

    fake_init(&fake, 0x00,
              TEST_GAUGE_ENABLE_MASK | TEST_CHARGER_MASK | TEST_WATCHDOG_MASK);
    tg28_sw_register_io_t io = fake_io(&fake);
    assert(tg28_sw_program_battery_model_io(&io, model, sizeof(model)) == ESP_OK);
    assert(memcmp(fake.sram, model, sizeof(model)) == 0);
    assert(fake.registers[TEST_REG_GAUGE_CONTROL] == TEST_SOURCE_MASK);
    assert(fake.registers[TEST_REG_MODULE_ENABLE] ==
           (TEST_GAUGE_ENABLE_MASK | TEST_CHARGER_MASK | TEST_WATCHDOG_MASK));
    assert(fake.delay_count == 1);
    assert(fake.reset_count == 2);

    /* A partial write falls back to ROM even when SRAM was selected on
     * entry, so cleanup cannot activate the incomplete SRAM image. */
    fake_init(&fake, TEST_SOURCE_MASK,
              TEST_GAUGE_ENABLE_MASK | TEST_CHARGER_MASK | TEST_WATCHDOG_MASK);
    fake.fail_write_reg = TEST_REG_MODEL;
    fake.fail_write_on = 17;
    io = fake_io(&fake);
    assert(tg28_sw_program_battery_model_io(&io, model, sizeof(model)) ==
           ESP_ERR_TIMEOUT);
    assert(fake.registers[TEST_REG_GAUGE_CONTROL] == 0x00);
    assert(fake.registers[TEST_REG_MODULE_ENABLE] ==
           (TEST_GAUGE_ENABLE_MASK | TEST_CHARGER_MASK | TEST_WATCHDOG_MASK));
    assert(fake.reset_count == 2);

    /* The same no-reset rule applies when program cleanup cannot confirm
     * that BROM is closed and the entry source is restored. */
    fake_init(&fake, 0x00,
              TEST_GAUGE_ENABLE_MASK | TEST_WATCHDOG_MASK);
    fake.fail_write_reg = TEST_REG_GAUGE_CONTROL;
    fake.fail_write_on = 5;
    io = fake_io(&fake);
    assert(tg28_sw_program_battery_model_io(&io, model, sizeof(model)) ==
           ESP_ERR_TIMEOUT);
    assert(fake.registers[TEST_REG_GAUGE_CONTROL] & TEST_BROM_MASK);
    assert(fake.reset_count == 1);
    assert(!(fake.registers[TEST_REG_MODULE_ENABLE] & TEST_WATCHDOG_MASK));
}

void app_main(void)
{
    test_chip_id_and_names();
    test_charge_current_coding();
    test_input_current_limit_coding();
    test_charge_voltage_coding();
    test_vindpm_coding();
    test_regulator_voltage_coding();
    test_ts_current_coding();
    test_adc_channel_coding();
    test_precharge_and_termination_coding();
    test_switch_names();
    test_irq_numbering();
    test_low_battery_warning_coding();
    test_powerkey_level_coding();
    test_feature_block_enum_codings();
    test_feature_block_null_rejections();
    test_battery_model_read_state_machine();
    test_battery_model_read_failures();
    test_battery_model_program_state_machine();
    printf("tg28_sw tests: PASSED\n");
}
