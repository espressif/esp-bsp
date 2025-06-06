/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include <stdio.h>
#include "bh1750.h"


#define I2C_MASTER_SCL_IO 26      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 25      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */



i2c_master_bus_handle_t my_i2c_bus_handle = NULL;

void read_bh1750 (void *my_bh1750_handle)
{
    float bh1750_data;

    bh1750_handle_t bh1750 = (bh1750_handle_t) my_bh1750_handle; //retrieve the BH1750 device handle copy

    while (1) {
        //read the sensor
        bh1750_get_data(bh1750, &bh1750_data);

        printf("Luminenence = %.2f Lux \n", bh1750_data);
        vTaskDelay(30 / portTICK_PERIOD_MS);
    }
}

void bh1750_init(void)
{

    bh1750_handle_t bh1750 = bh1750_create(my_i2c_bus_handle, BH1750_I2C_ADDRESS_DEFAULT);

    bh1750_measure_mode_t cmd_measure = BH1750_CONTINUE_4LX_RES;
    bh1750_set_measure_mode(bh1750, cmd_measure);
    vTaskDelay( 25 / portTICK_PERIOD_MS);
    //creating a task to read from BH1750, passing the BH1750 device handle copy
    xTaskCreate(read_bh1750, "bh1750_demo", 2500, bh1750, 5, NULL);
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(void)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    printf("Requesting I2C bus handle\n");
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &my_i2c_bus_handle));
    printf("I2C bus handle acquired\n");
}

void app_main(void)
{
    i2c_master_init();
    bh1750_init();
}
