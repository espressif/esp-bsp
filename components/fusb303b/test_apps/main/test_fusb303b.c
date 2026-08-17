/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "unity.h"
#include "unity_test_runner.h"

#include "driver/i2c_master.h"

#include "fusb303b.h"

/* The pins only have to exist on the target: no case below starts an I2C
 * transfer, so the tests run on a board without a FUSB303B. */
#define TEST_I2C_SCL_IO  (GPIO_NUM_18)
#define TEST_I2C_SDA_IO  (GPIO_NUM_8)

TEST_CASE("identity check accepts the FUSB303B version and type",
          "[fusb303b]")
{
    TEST_ASSERT_TRUE(fusb303b_is_supported_identity(0x10, 0x03));
    /* The revision nibble is ignored: any revision of version 1 matches. */
    TEST_ASSERT_TRUE(fusb303b_is_supported_identity(0x1F, 0x03));
    TEST_ASSERT_FALSE(fusb303b_is_supported_identity(0x20, 0x03));
    TEST_ASSERT_FALSE(fusb303b_is_supported_identity(0x10, 0x00));
}

TEST_CASE("BC_LVL decode maps only attached states to current levels",
          "[fusb303b]")
{
    /* BC_LVL=00 is Ra or nothing attached: not a current advertisement. */
    TEST_ASSERT_EQUAL(FUSB303B_CURRENT_NONE, fusb303b_decode_bc_level(0));
    TEST_ASSERT_EQUAL(FUSB303B_CURRENT_DEFAULT, fusb303b_decode_bc_level(1));
    TEST_ASSERT_EQUAL(FUSB303B_CURRENT_1_5_A, fusb303b_decode_bc_level(2));
    TEST_ASSERT_EQUAL(FUSB303B_CURRENT_3_0_A, fusb303b_decode_bc_level(3));
    /* Bits outside the two-bit field are ignored. */
    TEST_ASSERT_EQUAL(FUSB303B_CURRENT_NONE, fusb303b_decode_bc_level(0xF0));
    TEST_ASSERT_EQUAL(FUSB303B_CURRENT_3_0_A, fusb303b_decode_bc_level(0x1B));
}

TEST_CASE("create rejects invalid arguments before any I2C transfer",
          "[fusb303b]")
{
    i2c_master_bus_handle_t bus = NULL;
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = TEST_I2C_SDA_IO,
        .scl_io_num = TEST_I2C_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
    };
    TEST_ASSERT_EQUAL(ESP_OK, i2c_new_master_bus(&bus_config, &bus));

    fusb303b_handle_t device = NULL;
    fusb303b_config_t config = FUSB303B_CONFIG_DEFAULT();

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      fusb303b_create(NULL, &config, &device));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      fusb303b_create(bus, NULL, &device));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      fusb303b_create(bus, &config, NULL));
    TEST_ASSERT_NULL(device);

    /* Only 0x21 (ADDR/ORIENT low) and 0x31 (high) are valid addresses. */
    config.device_address = 0x22;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      fusb303b_create(bus, &config, &device));
    TEST_ASSERT_NULL(device);

    config.device_address = FUSB303B_I2C_ADDRESS_LOW;
    config.scl_speed_hz = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      fusb303b_create(bus, &config, &device));
    TEST_ASSERT_NULL(device);

    /* The datasheet caps the bus at 400 kHz. */
    config.scl_speed_hz = FUSB303B_I2C_CLOCK_HZ + 1;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      fusb303b_create(bus, &config, &device));
    TEST_ASSERT_NULL(device);

    TEST_ASSERT_EQUAL(ESP_OK, i2c_del_master_bus(bus));
}

void app_main(void)
{
    unity_run_menu();
}
