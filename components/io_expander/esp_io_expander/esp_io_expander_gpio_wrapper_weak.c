/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_io_expander_gpio_wrapper.h"
#include "esp_io_expander_gpio_wrapper_priv.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "soc/gpio_num.h"
#include "soc/soc_caps.h"

static const char *TAG = "io_exp_gpio_wrap";

esp_err_t __real_gpio_set_level(gpio_num_t gpio_num, uint32_t level);
int __real_gpio_get_level(gpio_num_t gpio_num);
esp_err_t __real_gpio_set_direction(gpio_num_t gpio_num, gpio_mode_t mode);
esp_err_t __real_gpio_set_pull_mode(gpio_num_t gpio_num, gpio_pull_mode_t pull);
esp_err_t __real_gpio_config(const gpio_config_t *pGPIOConfig);
esp_err_t __real_gpio_reset_pin(gpio_num_t gpio_num);

__attribute__((weak)) esp_err_t esp_io_expander_gpio_wrapper_set_level(gpio_num_t gpio_num, uint32_t level)
{
    (void)level;
    ESP_LOGW(TAG,
             "GPIO %d is not a native pin; link expander wrapper by calling esp_io_expander_gpio_wrapper_append_handler()",
             (int)gpio_num);
    return ESP_ERR_INVALID_STATE;
}

__attribute__((weak)) int esp_io_expander_gpio_wrapper_get_level(gpio_num_t gpio_num)
{
    ESP_LOGW(TAG,
             "GPIO %d is not a native pin; link expander wrapper by calling esp_io_expander_gpio_wrapper_append_handler()",
             (int)gpio_num);
    return -1;
}

__attribute__((weak)) esp_err_t esp_io_expander_gpio_wrapper_set_direction(gpio_num_t gpio_num, gpio_mode_t mode)
{
    (void)mode;
    ESP_LOGW(TAG,
             "GPIO %d is not a native pin; link expander wrapper by calling esp_io_expander_gpio_wrapper_append_handler()",
             (int)gpio_num);
    return ESP_ERR_INVALID_STATE;
}

__attribute__((weak)) esp_err_t esp_io_expander_gpio_wrapper_set_pull_mode(gpio_num_t gpio_num, gpio_pull_mode_t pull)
{
    (void)pull;
    ESP_LOGW(TAG,
             "GPIO %d is not a native pin; link expander wrapper by calling esp_io_expander_gpio_wrapper_append_handler()",
             (int)gpio_num);
    return ESP_ERR_INVALID_STATE;
}

__attribute__((weak)) esp_err_t esp_io_expander_gpio_wrapper_configure_pin(gpio_num_t gpio_num,
        const gpio_config_t *config)
{
    (void)config;
    ESP_LOGW(TAG,
             "GPIO %d is not a native pin; link expander wrapper by calling esp_io_expander_gpio_wrapper_append_handler()",
             (int)gpio_num);
    return ESP_ERR_INVALID_STATE;
}

__attribute__((weak)) esp_err_t esp_io_expander_gpio_wrapper_reset_pin(gpio_num_t gpio_num)
{
    ESP_LOGW(TAG,
             "GPIO %d is not a native pin; link expander wrapper by calling esp_io_expander_gpio_wrapper_append_handler()",
             (int)gpio_num);
    return ESP_ERR_INVALID_STATE;
}

esp_err_t __wrap_gpio_set_level(gpio_num_t gpio_num, uint32_t level)
{
    if (gpio_num < GPIO_NUM_MAX) {
        return __real_gpio_set_level(gpio_num, level);
    }
    return esp_io_expander_gpio_wrapper_set_level(gpio_num, level);
}

int __wrap_gpio_get_level(gpio_num_t gpio_num)
{
    if (gpio_num < GPIO_NUM_MAX) {
        return __real_gpio_get_level(gpio_num);
    }
    return esp_io_expander_gpio_wrapper_get_level(gpio_num);
}

esp_err_t __wrap_gpio_set_direction(gpio_num_t gpio_num, gpio_mode_t mode)
{
    if (gpio_num < GPIO_NUM_MAX) {
        return __real_gpio_set_direction(gpio_num, mode);
    }
    return esp_io_expander_gpio_wrapper_set_direction(gpio_num, mode);
}

esp_err_t __wrap_gpio_set_pull_mode(gpio_num_t gpio_num, gpio_pull_mode_t pull)
{
    if (gpio_num < GPIO_NUM_MAX) {
        return __real_gpio_set_pull_mode(gpio_num, pull);
    }
    return esp_io_expander_gpio_wrapper_set_pull_mode(gpio_num, pull);
}

esp_err_t __wrap_gpio_config(const gpio_config_t *pGPIOConfig)
{
    esp_err_t ret = ESP_OK;
    gpio_config_t cfg = *pGPIOConfig;
    uint64_t invalid_mask = cfg.pin_bit_mask & ~SOC_GPIO_VALID_GPIO_MASK;

    while (invalid_mask) {
        int io_num = __builtin_ctzll(invalid_mask);
        invalid_mask &= invalid_mask - 1;
        ret = esp_io_expander_gpio_wrapper_configure_pin((gpio_num_t)io_num, &cfg);
        if (ret != ESP_OK) {
            return ret;
        }
        cfg.pin_bit_mask &= ~(1ULL << io_num);
    }

    if (cfg.pin_bit_mask) {
        return __real_gpio_config(&cfg);
    }
    return ret;
}

esp_err_t __wrap_gpio_reset_pin(gpio_num_t gpio_num)
{
    if (gpio_num < GPIO_NUM_MAX) {
        return __real_gpio_reset_pin(gpio_num);
    }
    return esp_io_expander_gpio_wrapper_reset_pin(gpio_num);
}
