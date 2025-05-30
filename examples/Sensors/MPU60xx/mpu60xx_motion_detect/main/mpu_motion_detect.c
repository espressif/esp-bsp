/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
/*
 * mpu60xx.h
 *
 *  Created on: 15-Oct-2024
 *      Author: rohan
 */

#include "freertos/FreeRTOS.h"
#include "mpu60xx.h"



//i2c configuration values
#define I2C_MASTER_SCL_IO           (22)    // SCL pin
#define I2C_MASTER_SDA_IO           (21)    // SDA pin
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          (100000)  // I2C frequency

#define MPU_INTERRUPT_PIN           (23)



i2c_master_bus_handle_t my_bus_handle;

TaskHandle_t handler_task;
static SemaphoreHandle_t s_bin_sem;

#define ESP_INTR_FLAG_DEFAULT 0



void i2c_master_init(void)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    printf("requesting i2c bus handle\n");
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &my_bus_handle));
    printf("i2c bus handle acquired\n");


}

volatile uint32_t count = 0;
void intr_isr_handler(void *arg)
{
    //vTaskResume(handler_task);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_bin_sem, &xHigherPriorityTaskWoken);
}

void read_data(void *mpu_handle)
{

    mpu60xx_reading_t sensor_data;
    mpu60xx_handle_t my_dev_handle = (mpu60xx_handle_t)mpu_handle;
    bool status;
    while (1) {

        if (xSemaphoreTake(s_bin_sem, 20 / portTICK_PERIOD_MS) == pdPASS) {
            ESP_ERROR_CHECK( mpu60xx_read_sensor (my_dev_handle, &sensor_data ));
            printf("aX = %.3f m/s, aY = %.3f m/s, aZ = %.3f m/s, t=%.3f C, gX = %.3f dps gY = %.3f dps gZ = %.3f dps\n", sensor_data.accel.x, sensor_data.accel.y, sensor_data.accel.z, sensor_data.temperature, sensor_data.gyro.x, sensor_data.gyro.y, sensor_data.gyro.z);
            ESP_ERROR_CHECK(mpu60xx_getMotionInterruptStatus(my_dev_handle, &status));
        }
    }
}


void app_main(void)
{

    //static const mpu60xx_reading sensor_data;
    i2c_master_init();

    mpu60xx_init_config_t mpu_6050_config = {
        .gyro_res = MPU60XX_RANGE_500_DEG,
        .accel_res = MPU60XX_RANGE_8_G,
        .temp_sensor = true,
        .dlpf_bw = MPU60XX_BAND_94_HZ,
        .sample_rate = 200
    };

    mpu60xx_handle_t dev_6050_handle;

    printf("requesting mpu handle\n");

    dev_6050_handle = mpu60xx_create(my_bus_handle, MPU60xx_ADDR);

    printf("mpu handle acquired\n");

    while (mpu60xx_init(mpu_6050_config, dev_6050_handle) != ESP_OK) {
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    mpu60xx_event_detect_config_t mot_det = {
        .event_threshold = 2,
        .event_duration = 10,
        .dhpf_bw = MPU60XX_HIGHPASS_0_63_HZ
    };

    mpu60xx_intrpt_config_t md_intr_conf =  {
        .interrupt_pin = (gpio_num_t)MPU_INTERRUPT_PIN,
        .active_level = MPU60XX_INTERRUPT_PIN_ACTIVE_LOW,
        .pin_mode = MPU60XX_INTERRUPT_PIN_PUSH_PULL,
        .interrupt_latch = MPU60XX_INTERRUPT_LATCH_UNTIL_CLEARED,
        .interrupt_clear_behavior = MPU60XX_INTERRUPT_CLEAR_ON_STATUS_READ,
        .isr = intr_isr_handler
    };

    vTaskDelay(200 / portTICK_PERIOD_MS);
    mpu60xx_en_EventDetection(dev_6050_handle, &mot_det, MPU60XX_EVENT_MOTION_DETECT);
    mpu60xx_Interrupt_pin_configuration(dev_6050_handle, &md_intr_conf);
    s_bin_sem = xSemaphoreCreateBinary();

    xTaskCreate(read_data, "apna_imu_tester", 2500, dev_6050_handle, 5, &handler_task);
}
