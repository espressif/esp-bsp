/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <stdbool.h>
#include "esp_io_expander_gpio_wrapper.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "esp_heap_caps.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "soc/gpio_num.h"

typedef struct ioexp_range_node {
    uint32_t start_num;
    uint32_t count;
    esp_io_expander_handle_t handler;
    struct ioexp_range_node *next;
} ioexp_range_node_t;

static ioexp_range_node_t s_embedded_range_head = {
    .start_num = 0,
    .count = GPIO_NUM_MAX,
    .handler = NULL,
    .next = NULL,
};
static ioexp_range_node_t *s_ioexp_ranges = &s_embedded_range_head;
static portMUX_TYPE s_ioexp_lock = portMUX_INITIALIZER_UNLOCKED;

static char *TAG = "io_expander_wrapper";

esp_err_t __real_gpio_set_level(gpio_num_t gpio_num, uint32_t level);
int __real_gpio_get_level(gpio_num_t gpio_num);
esp_err_t __real_gpio_set_direction(gpio_num_t gpio_num, gpio_mode_t mode);
esp_err_t __real_gpio_set_pull_mode(gpio_num_t gpio_num, gpio_pull_mode_t pull);

static bool find_ioexp_for_num(uint32_t gpio_num, esp_io_expander_handle_t *out_handler, uint32_t *out_pin_mask)
{
    bool found = false;
    portENTER_CRITICAL(&s_ioexp_lock);
    ioexp_range_node_t *node = s_ioexp_ranges;
    while (node) {
        uint32_t end = node->start_num + node->count;
        if (gpio_num >= node->start_num && gpio_num < end) {
            uint32_t bit_index = gpio_num - node->start_num;
            if (out_handler) {
                *out_handler = node->handler;
            }
            if (out_pin_mask) {
                *out_pin_mask = (1U << bit_index);
            }
            found = true;
            break;
        }
        node = node->next;
    }
    portEXIT_CRITICAL(&s_ioexp_lock);
    return found;
}

esp_err_t esp_io_expander_gpio_wrapper_append_handler(esp_io_expander_handle_t handler, uint32_t start_io_num)
{
    if (handler == NULL || start_io_num < GPIO_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t cnt = handler->config.io_count;
    if (cnt == 0 || cnt > IO_COUNT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    // [start, end)
    uint32_t start = start_io_num;
    uint32_t end = start_io_num + cnt;
    // Pre-allocate node before taking the lock to minimize time spent in critical section
    ioexp_range_node_t *node = (ioexp_range_node_t *)heap_caps_malloc(sizeof(ioexp_range_node_t),
                               MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (node == NULL) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t err = ESP_OK;
    // Ensure no overlap and no duplicate handler with existing non-embedded nodes
    portENTER_CRITICAL(&s_ioexp_lock);
    for (ioexp_range_node_t *n = s_ioexp_ranges; n != NULL; n = n->next) {
        // Disallow appending the same handler more than once
        if (n->handler == handler) {
            err = ESP_ERR_INVALID_ARG;
            goto err;
        }
        if (n == &s_embedded_range_head) {
            continue; // embedded range is [0, GPIO_NUM_MAX), guaranteed disjoint by start_io_num >= GPIO_NUM_MAX
        }
        uint32_t n_start = n->start_num;
        uint32_t n_end = n->start_num + n->count;
        // Overlap if the intervals [start, end) and [n_start, n_end) intersect
        if ((start < n_end) && (n_start < end)) {
            err = ESP_ERR_INVALID_ARG;
            goto err;
        }
    }
    node->start_num = start_io_num;
    node->count = cnt;
    node->handler = handler;
    node->next = s_ioexp_ranges;
    s_ioexp_ranges = node;
err:
    portEXIT_CRITICAL(&s_ioexp_lock);
    if (err != ESP_OK) {
        ESP_LOGE(TAG,
                 "Either requested IO expander range overlaps with existing range or the same handler is already appended");
        free(node);
    }
    return err;
}

esp_err_t esp_io_expander_gpio_wrapper_remove_handler(esp_io_expander_handle_t handler)
{
    if (handler == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    ioexp_range_node_t *to_free = NULL;
    ioexp_range_node_t *prev = NULL;
    portENTER_CRITICAL(&s_ioexp_lock);
    ioexp_range_node_t *curr = s_ioexp_ranges;
    while (curr) {
        if (curr->handler == handler) {
            to_free = curr;
            if (prev) {
                prev->next = curr->next;
            } else {
                s_ioexp_ranges = curr->next;
            }
            curr = curr->next;
            break;
        }
        prev = curr;
        curr = curr->next;
    }
    portEXIT_CRITICAL(&s_ioexp_lock);
    if (to_free) {
        free(to_free);
    }
    return ESP_OK;
}

esp_err_t __wrap_gpio_set_level(gpio_num_t gpio_num, uint32_t level)
{
    if (gpio_num < GPIO_NUM_MAX) {
        // Call the ESP-IDF implementation for regular GPIOs
        return __real_gpio_set_level(gpio_num, level);
    }
    // Redirect GPIO set level calls to ESP IO Expander here
    esp_io_expander_handle_t handler = NULL;
    uint32_t pin_mask = 0;
    if (!find_ioexp_for_num((uint32_t)gpio_num, &handler, &pin_mask)) {
        ESP_LOGE(TAG, "GPIO %d is not assigned to any IO Expander", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    return esp_io_expander_set_level(handler, pin_mask, (uint8_t)(level ? 1 : 0));
}

int __wrap_gpio_get_level(gpio_num_t gpio_num)
{
    if (gpio_num < GPIO_NUM_MAX) {
        // Call the ESP-IDF implementation for regular GPIOs
        return __real_gpio_get_level(gpio_num);
    }
    // Redirect GPIO get level calls to ESP IO Expander here
    esp_io_expander_handle_t handler = NULL;
    uint32_t pin_mask = 0;
    if (!find_ioexp_for_num((uint32_t)gpio_num, &handler, &pin_mask)) {
        ESP_LOGE(TAG, "GPIO %d is not assigned to any IO Expander", gpio_num);
        return -1; // Indicate error
    }
    uint32_t level_mask = 0;
    esp_err_t err = esp_io_expander_get_level(handler, pin_mask, &level_mask);
    if (err != ESP_OK) {
        return -1; // Indicate error
    }
    return (level_mask & pin_mask) ? 1 : 0;
}

esp_err_t __wrap_gpio_set_direction(gpio_num_t gpio_num, gpio_mode_t mode)
{
    if (gpio_num < GPIO_NUM_MAX) {
        // Call the ESP-IDF implementation for regular GPIOs
        return __real_gpio_set_direction(gpio_num, mode);
    }
    // Redirect GPIO set direction calls to ESP IO Expander here
    esp_io_expander_handle_t handler = NULL;
    uint32_t pin_mask = 0;
    if (!find_ioexp_for_num((uint32_t)gpio_num, &handler, &pin_mask)) {
        ESP_LOGE(TAG, "GPIO %d is not assigned to any IO Expander", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    esp_io_expander_dir_t dir;
    esp_io_expander_output_mode_t out_mode = IO_EXPANDER_OUTPUT_MODE_PUSH_PULL;
    bool mode_valid = true;
    switch (mode) {
    case GPIO_MODE_INPUT:
        dir = IO_EXPANDER_INPUT;
        break;
    case GPIO_MODE_OUTPUT:
        dir = IO_EXPANDER_OUTPUT;
        out_mode = IO_EXPANDER_OUTPUT_MODE_PUSH_PULL;
        break;
    case GPIO_MODE_OUTPUT_OD:
        dir = IO_EXPANDER_OUTPUT;
        out_mode = IO_EXPANDER_OUTPUT_MODE_OPEN_DRAIN;
        if (!handler->write_highz_reg) {
            mode_valid = false;
        }
        break;
    default:
        mode_valid = false;
    }
    if (!mode_valid) {
        ESP_LOGE(TAG, "Unsupported GPIO mode %d for IO Expander GPIO %d", mode, gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = esp_io_expander_set_dir(handler, pin_mask, dir);
    if (err == ESP_OK && dir == IO_EXPANDER_OUTPUT && handler->write_highz_reg) {
        err = esp_io_expander_set_output_mode(handler, pin_mask, out_mode);
    }
    return err;
}

esp_err_t __wrap_gpio_set_pull_mode(gpio_num_t gpio_num, gpio_pull_mode_t pull)
{
    if (gpio_num < GPIO_NUM_MAX) {
        // Call the ESP-IDF implementation for regular GPIOs
        return __real_gpio_set_pull_mode(gpio_num, pull);
    }
    // Redirect GPIO set pull mode calls to ESP IO Expander here
    esp_io_expander_handle_t handler = NULL;
    uint32_t pin_mask = 0;
    if (!find_ioexp_for_num((uint32_t)gpio_num, &handler, &pin_mask)) {
        ESP_LOGE(TAG, "GPIO %d is not assigned to any IO Expander", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    esp_io_expander_pullupdown_t pud = IO_EXPANDER_PULL_NONE;
    bool pud_valid = true;
    switch (pull) {
    case GPIO_PULLUP_ONLY:
        if (!handler->write_pullup_en_reg) {
            pud_valid = false;
        } else {
            pud = IO_EXPANDER_PULL_UP;
        }
        break;
    case GPIO_PULLDOWN_ONLY:
        if (!handler->write_pullup_en_reg || !handler->write_pullup_sel_reg) {
            pud_valid = false;
        } else {
            pud = IO_EXPANDER_PULL_DOWN;
        }
        break;
    case GPIO_FLOATING:
        pud = IO_EXPANDER_PULL_NONE;
        break;
    default:
        pud_valid = false;
        break;
    }
    if (!pud_valid) {
        ESP_LOGE(TAG, "Unsupported GPIO pull mode %d for IO Expander GPIO %d", pull, gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    return esp_io_expander_set_pullupdown(handler, pin_mask, pud);
}
