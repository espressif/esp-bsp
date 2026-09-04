/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "unity.h"
#include "unity_test_runner.h"

#include "driver/i2c_master.h"

#include "rx8130ce.h"

/* The pins only have to exist on the target: no case below starts an I2C
 * transfer with a configured device, so the tests run on a board without
 * a RX8130CE. */
#define TEST_I2C_SCL_IO  (GPIO_NUM_18)
#define TEST_I2C_SDA_IO  (GPIO_NUM_8)

TEST_CASE("default configuration keeps the primary-cell policy", "[rx8130ce]")
{
    const rx8130ce_config_t config = RX8130CE_CONFIG_DEFAULT();
    TEST_ASSERT_EQUAL_UINT8(RX8130CE_I2C_ADDRESS_DEFAULT, config.device_address);
    TEST_ASSERT_EQUAL_UINT32(RX8130CE_I2C_CLOCK_HZ, config.scl_speed_hz);
    TEST_ASSERT_FALSE(config.backup_charge_enable);
    /* POR default cutoff; detection on so VBLF can assert on a primary cell. */
    TEST_ASSERT_EQUAL(RX8130CE_CHARGE_CUTOFF_3_02V, config.backup_charge_cutoff);
    TEST_ASSERT_TRUE(config.backup_voltage_low_detect);
}

TEST_CASE("create rejects invalid configuration before any I2C transfer",
          "[rx8130ce]")
{
    i2c_master_bus_handle_t bus = NULL;
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = TEST_I2C_SDA_IO,
        .scl_io_num = TEST_I2C_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
    };
    TEST_ASSERT_EQUAL(ESP_OK, i2c_new_master_bus(&bus_config, &bus));

    rx8130ce_handle_t device = NULL;
    rx8130ce_config_t config = RX8130CE_CONFIG_DEFAULT();

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      rx8130ce_create(NULL, &config, &device));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      rx8130ce_create(bus, &config, NULL));
    TEST_ASSERT_NULL(device);

    config.device_address = 0x80;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      rx8130ce_create(bus, &config, &device));
    TEST_ASSERT_NULL(device);

    config.device_address = RX8130CE_I2C_ADDRESS_DEFAULT;
    config.scl_speed_hz = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      rx8130ce_create(bus, &config, &device));
    config.scl_speed_hz = RX8130CE_I2C_CLOCK_HZ + 1;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      rx8130ce_create(bus, &config, &device));
    TEST_ASSERT_NULL(device);

    /* The cutoff-less BFVSEL encoding (11b) must stay unreachable. */
    config.scl_speed_hz = RX8130CE_I2C_CLOCK_HZ;
    config.backup_charge_cutoff = (rx8130ce_charge_cutoff_t)3;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      rx8130ce_create(bus, &config, &device));
    TEST_ASSERT_NULL(device);

    TEST_ASSERT_EQUAL(ESP_OK, i2c_del_master_bus(bus));
}

TEST_CASE("time validation accepts leap days and rejects bad fields",
          "[rx8130ce]")
{
    const rx8130ce_time_t leap_day = {
        .year = 2028,
        .month = 2,
        .day = 29,
        .weekday = 2,
        .hour = 23,
        .minute = 59,
        .second = 59,
    };
    TEST_ASSERT_TRUE(rx8130ce_time_is_valid(&leap_day));

    rx8130ce_time_t invalid = leap_day;
    invalid.year = 2027;
    TEST_ASSERT_FALSE(rx8130ce_time_is_valid(&invalid));
    invalid = leap_day;
    invalid.weekday = 7;
    TEST_ASSERT_FALSE(rx8130ce_time_is_valid(&invalid));
    invalid = leap_day;
    invalid.second = 60;
    TEST_ASSERT_FALSE(rx8130ce_time_is_valid(&invalid));
    TEST_ASSERT_FALSE(rx8130ce_time_is_valid(NULL));
}

TEST_CASE("alarm validation ignores disabled fields and rejects conflicts",
          "[rx8130ce]")
{
    const rx8130ce_alarm_t daily = {
        .minute_en = true,
        .minute = 30,
        .hour_en = true,
        .hour = 7,
    };
    TEST_ASSERT_TRUE(rx8130ce_alarm_is_valid(&daily));

    /* Every field disabled is valid: an interrupt once per minute. */
    const rx8130ce_alarm_t every_minute = {0};
    TEST_ASSERT_TRUE(rx8130ce_alarm_is_valid(&every_minute));

    /* Disabled fields are ignored, even when out of range. */
    rx8130ce_alarm_t alarm = daily;
    alarm.minute_en = false;
    alarm.minute = 99;
    TEST_ASSERT_TRUE(rx8130ce_alarm_is_valid(&alarm));

    /* Enabled fields must be in range. */
    alarm = daily;
    alarm.minute = 60;
    TEST_ASSERT_FALSE(rx8130ce_alarm_is_valid(&alarm));
    alarm = daily;
    alarm.hour = 24;
    TEST_ASSERT_FALSE(rx8130ce_alarm_is_valid(&alarm));
    alarm = daily;
    alarm.day_en = true;
    alarm.day = 0;
    TEST_ASSERT_FALSE(rx8130ce_alarm_is_valid(&alarm));
    alarm = daily;
    alarm.day_en = true;
    alarm.day = 32;
    TEST_ASSERT_FALSE(rx8130ce_alarm_is_valid(&alarm));
    alarm = daily;
    alarm.weekday_en = true;
    alarm.weekday = 7;
    TEST_ASSERT_FALSE(rx8130ce_alarm_is_valid(&alarm));

    /* Day and weekday share register 19h: enabling both is rejected. */
    alarm = daily;
    alarm.day_en = true;
    alarm.day = 15;
    alarm.weekday_en = true;
    alarm.weekday = 5;
    TEST_ASSERT_FALSE(rx8130ce_alarm_is_valid(&alarm));
    TEST_ASSERT_FALSE(rx8130ce_alarm_is_valid(NULL));
}

TEST_CASE("alarm encode applies the active-low AE bits", "[rx8130ce]")
{
    const rx8130ce_alarm_t daily = {
        .minute_en = true,
        .minute = 30,
        .hour_en = true,
        .hour = 7,
    };
    const rx8130ce_alarm_t every_minute = {0};

    /* AE is active-low, so enabled fields keep bit 7 clear. */
    uint8_t registers[3];
    bool use_day_alarm = false;
    rx8130ce_alarm_encode(&daily, registers, &use_day_alarm);
    TEST_ASSERT_EQUAL_UINT8(0x30, registers[0]);
    TEST_ASSERT_EQUAL_UINT8(0x07, registers[1]);
    TEST_ASSERT_EQUAL_UINT8(0x80, registers[2]);
    TEST_ASSERT_FALSE(use_day_alarm);

    rx8130ce_alarm_encode(&every_minute, registers, &use_day_alarm);
    TEST_ASSERT_EQUAL_UINT8(0x80, registers[0]);
    TEST_ASSERT_EQUAL_UINT8(0x80, registers[1]);
    TEST_ASSERT_EQUAL_UINT8(0x80, registers[2]);
    TEST_ASSERT_FALSE(use_day_alarm);

    const rx8130ce_alarm_t monthly = {
        .minute_en = true,
        .minute = 59,
        .hour_en = true,
        .hour = 23,
        .day_en = true,
        .day = 31,
    };
    rx8130ce_alarm_encode(&monthly, registers, &use_day_alarm);
    TEST_ASSERT_EQUAL_UINT8(0x59, registers[0]);
    TEST_ASSERT_EQUAL_UINT8(0x23, registers[1]);
    TEST_ASSERT_EQUAL_UINT8(0x31, registers[2]);
    TEST_ASSERT_TRUE(use_day_alarm);

    const rx8130ce_alarm_t weekly = {
        .hour_en = true,
        .hour = 18,
        .weekday_en = true,
        .weekday = 6,
    };
    rx8130ce_alarm_encode(&weekly, registers, &use_day_alarm);
    TEST_ASSERT_EQUAL_UINT8(0x80, registers[0]);
    TEST_ASSERT_EQUAL_UINT8(0x18, registers[1]);
    TEST_ASSERT_EQUAL_UINT8(0x40, registers[2]);
    TEST_ASSERT_FALSE(use_day_alarm);
}

TEST_CASE("alarm from time targets an absolute future moment", "[rx8130ce]")
{
    const rx8130ce_time_t now = {
        .year = 2026, .month = 8, .day = 5, .weekday = 3,
        .hour = 10, .minute = 30, .second = 45,
    };
    rx8130ce_time_t target = now;
    rx8130ce_alarm_t alarm;
    uint8_t registers[3];
    bool use_day_alarm = false;

    /* One minute ahead on the same day: minute/hour/day compare, day layout. */
    target.minute = 31;
    target.second = 0;
    TEST_ASSERT_EQUAL(ESP_OK, rx8130ce_alarm_from_time(&now, &target, &alarm));
    TEST_ASSERT_TRUE(alarm.minute_en);
    TEST_ASSERT_EQUAL_UINT8(31, alarm.minute);
    TEST_ASSERT_TRUE(alarm.hour_en);
    TEST_ASSERT_EQUAL_UINT8(10, alarm.hour);
    TEST_ASSERT_TRUE(alarm.day_en);
    TEST_ASSERT_EQUAL_UINT8(5, alarm.day);
    TEST_ASSERT_FALSE(alarm.weekday_en);
    rx8130ce_alarm_encode(&alarm, registers, &use_day_alarm);
    TEST_ASSERT_EQUAL_UINT8(0x31, registers[0]);
    TEST_ASSERT_EQUAL_UINT8(0x10, registers[1]);
    TEST_ASSERT_EQUAL_UINT8(0x05, registers[2]);
    TEST_ASSERT_TRUE(use_day_alarm);

    /* The target minute must lie strictly ahead of now. */
    target = now;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      rx8130ce_alarm_from_time(&now, &target, &alarm));
    target.minute = 29;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      rx8130ce_alarm_from_time(&now, &target, &alarm));
    target = now;
    target.hour = 9;
    target.minute = 59;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      rx8130ce_alarm_from_time(&now, &target, &alarm));

    /* Horizon: 27 days out is accepted, 28 days is rejected. */
    target = now;
    target.month = 9;
    target.day = 1;
    TEST_ASSERT_EQUAL(ESP_OK, rx8130ce_alarm_from_time(&now, &target, &alarm));
    TEST_ASSERT_TRUE(alarm.day_en);
    TEST_ASSERT_EQUAL_UINT8(1, alarm.day);
    target.day = 2;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      rx8130ce_alarm_from_time(&now, &target, &alarm));

    /* Month roll-over and leap years go through the day count. */
    const rx8130ce_time_t month_end = {
        .year = 2026, .month = 8, .day = 31, .weekday = 1,
        .hour = 23, .minute = 59, .second = 0,
    };
    target = month_end;
    target.month = 9;
    target.day = 1;
    target.hour = 0;
    target.minute = 0;
    TEST_ASSERT_EQUAL(ESP_OK,
                      rx8130ce_alarm_from_time(&month_end, &target, &alarm));

    const rx8130ce_time_t leap_eve = {
        .year = 2028, .month = 2, .day = 28, .weekday = 1,
        .hour = 23, .minute = 59, .second = 0,
    };
    target = leap_eve;
    target.day = 29;
    target.hour = 0;
    target.minute = 0;
    TEST_ASSERT_EQUAL(ESP_OK,
                      rx8130ce_alarm_from_time(&leap_eve, &target, &alarm));

    const rx8130ce_time_t common_eve = {
        .year = 2027, .month = 2, .day = 28, .weekday = 0,
        .hour = 23, .minute = 59, .second = 0,
    };
    target = common_eve;
    target.month = 3;
    target.day = 1;
    target.hour = 0;
    target.minute = 0;
    TEST_ASSERT_EQUAL(ESP_OK,
                      rx8130ce_alarm_from_time(&common_eve, &target, &alarm));

    /* Invalid calendar inputs and NULL pointers are rejected. */
    target = now;
    target.month = 13;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      rx8130ce_alarm_from_time(&now, &target, &alarm));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      rx8130ce_alarm_from_time(NULL, &now, &alarm));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      rx8130ce_alarm_from_time(&now, NULL, &alarm));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      rx8130ce_alarm_from_time(&now, &now, NULL));
}

TEST_CASE("timer validation bounds the preset and source clocks",
          "[rx8130ce]")
{
    const rx8130ce_timer_t one_hour = {
        .enable = true,
        .source_clock = RX8130CE_TIMER_SOURCE_1HZ,
        .count = 3600,
    };
    TEST_ASSERT_TRUE(rx8130ce_timer_is_valid(&one_hour));

    /* A stopped timer still validates; enable does not affect validity. */
    rx8130ce_timer_t timer = one_hour;
    timer.enable = false;
    TEST_ASSERT_TRUE(rx8130ce_timer_is_valid(&timer));

    /* The preset range is 1-65535. */
    timer = one_hour;
    timer.count = 0;
    TEST_ASSERT_FALSE(rx8130ce_timer_is_valid(&timer));

    /* Only the five defined source clocks are accepted. */
    timer = one_hour;
    timer.source_clock = (rx8130ce_timer_source_t)5;
    TEST_ASSERT_FALSE(rx8130ce_timer_is_valid(&timer));
    TEST_ASSERT_FALSE(rx8130ce_timer_is_valid(NULL));
}

TEST_CASE("timer encode writes the preset low byte first", "[rx8130ce]")
{
    const rx8130ce_timer_t one_hour = {
        .enable = true,
        .source_clock = RX8130CE_TIMER_SOURCE_1HZ,
        .count = 3600,
    };
    uint8_t timer_registers[2];
    uint8_t tsel_bits = 0;
    rx8130ce_timer_encode(&one_hour, timer_registers, &tsel_bits);
    TEST_ASSERT_EQUAL_UINT8(0x10, timer_registers[0]);
    TEST_ASSERT_EQUAL_UINT8(0x0E, timer_registers[1]);
    TEST_ASSERT_EQUAL_UINT8(RX8130CE_TIMER_SOURCE_1HZ, tsel_bits);

    const rx8130ce_timer_t min_fast = {
        .enable = true,
        .source_clock = RX8130CE_TIMER_SOURCE_4096HZ,
        .count = 1,
    };
    rx8130ce_timer_encode(&min_fast, timer_registers, &tsel_bits);
    TEST_ASSERT_EQUAL_UINT8(0x01, timer_registers[0]);
    TEST_ASSERT_EQUAL_UINT8(0x00, timer_registers[1]);
    TEST_ASSERT_EQUAL_UINT8(RX8130CE_TIMER_SOURCE_4096HZ, tsel_bits);

    const rx8130ce_timer_t max_slow = {
        .enable = false,
        .source_clock = RX8130CE_TIMER_SOURCE_1_3600HZ,
        .count = 65535,
    };
    rx8130ce_timer_encode(&max_slow, timer_registers, &tsel_bits);
    TEST_ASSERT_EQUAL_UINT8(0xFF, timer_registers[0]);
    TEST_ASSERT_EQUAL_UINT8(0xFF, timer_registers[1]);
    TEST_ASSERT_EQUAL_UINT8(RX8130CE_TIMER_SOURCE_1_3600HZ, tsel_bits);
}

TEST_CASE("digital offset encode covers both sign regions", "[rx8130ce]")
{
    uint8_t offset_register = 0;
    TEST_ASSERT_EQUAL(ESP_OK,
                      rx8130ce_digital_offset_encode(true, 0, &offset_register));
    TEST_ASSERT_EQUAL_UINT8(0x80, offset_register);
    TEST_ASSERT_EQUAL(ESP_OK,
                      rx8130ce_digital_offset_encode(true, 63, &offset_register));
    TEST_ASSERT_EQUAL_UINT8(0x80 | 63, offset_register);
    TEST_ASSERT_EQUAL(ESP_OK,
                      rx8130ce_digital_offset_encode(true, -64, &offset_register));
    TEST_ASSERT_EQUAL_UINT8(0x80 | 64, offset_register);
    TEST_ASSERT_EQUAL(ESP_OK,
                      rx8130ce_digital_offset_encode(false, -1, &offset_register));
    TEST_ASSERT_EQUAL_UINT8(127, offset_register);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      rx8130ce_digital_offset_encode(true, 64, &offset_register));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      rx8130ce_digital_offset_encode(true, -65, &offset_register));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      rx8130ce_digital_offset_encode(true, 0, NULL));

    /* Decode round-trips exactly across both sign regions. */
    bool offset_enabled = false;
    int8_t offset_steps = 0;
    for (int8_t steps = -64; steps <= 63; steps++) {
        TEST_ASSERT_EQUAL(ESP_OK,
                          rx8130ce_digital_offset_encode(true, steps,
                                  &offset_register));
        rx8130ce_digital_offset_decode(offset_register, &offset_enabled,
                                       &offset_steps);
        TEST_ASSERT_TRUE(offset_enabled);
        TEST_ASSERT_EQUAL_INT8(steps, offset_steps);
    }
    rx8130ce_digital_offset_decode(0x00, &offset_enabled, &offset_steps);
    TEST_ASSERT_FALSE(offset_enabled);
    TEST_ASSERT_EQUAL_INT8(0, offset_steps);
}

TEST_CASE("user RAM and raw accessors reject invalid arguments without a device",
          "[rx8130ce]")
{
    uint8_t buffer[RX8130CE_USER_RAM_SIZE] = {0};

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      rx8130ce_write_user_ram(NULL, 0, buffer, sizeof(buffer)));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      rx8130ce_read_user_ram(NULL, 0, buffer, sizeof(buffer)));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      rx8130ce_read_registers(NULL, 0x10, buffer, sizeof(buffer)));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      rx8130ce_write_register(NULL, 0x10, 0));
}

void app_main(void)
{
    unity_run_menu();
}
