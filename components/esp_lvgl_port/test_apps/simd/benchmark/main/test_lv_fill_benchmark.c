/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <malloc.h>
#include <sdkconfig.h>

#include "unity.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"  // for xthal_get_ccount()
#include "lvgl.h"
#include "lv_fill_common.h"

#define WIDTH 128
#define HEIGHT 128
#define STRIDE WIDTH
#define UNALIGN_BYTES 1
#define BENCHMARK_CYCLES 1000

// ------------------------------------------------- Macros and Types --------------------------------------------------

static const char *TAG_LV_FILL_BENCH = "LV Fill Benchmark";

static blend_params_t *s_blend_params = NULL;
static test_area_t *s_area = NULL;

// ------------------------------------------------ Static function headers --------------------------------------------

/**
 * @brief Initialize the benchmark test
 */
static void lv_fill_benchmark_init(bench_test_params_t *test_params);

/**
 * @brief Run the benchmark test
 */
static float lv_fill_benchmark_run(bench_test_params_t *test_params);

// ------------------------------------------------ Test cases ---------------------------------------------------------

/*
Benchmark tests

Requires:
    - To pass functionality tests first

Purpose:
    - Test that an acceleration is achieved by an assembly implementation of LVGL blending API

Procedure:
    - Depending on menuconfig settings, chose either Assembly version of LVGL blend API, or the ANSI version
    - Build the test app with selected configuration
    - Initialize structures needed for LVGL blending API
    - Initialize input parameters (test array length, width, allocate array...) of the benchmark test
    - Run LVGL blending API multiple times (1000-times or so)
    - Firstly use an input test parameters for the most ideal case (16-byte aligned array, array width and height divisible by 4 for ARGB8888 color format)
    - Then use worst-case input test parameters (1-byte aligned array, array width and height NOT divisible by 4 for ARGB8888 color format)
    - Count how many CPU cycles does it take to run a function from the LVGL blending API for each case (ideal and worst case)
    - Free test arrays and structures needed for LVGL blending API
    - If needed, in the menuconfig, select the other version of the LVGL blend API and compare the results
*/
// ------------------------------------------------ Test cases stages --------------------------------------------------

TEST_CASE("LV Fill benchmark ARGB8888", "[lv_fill][ARGB8888]")
{
    uint32_t *dest_array_align16  = (uint32_t *)memalign(16, STRIDE * HEIGHT * sizeof(uint32_t) + (UNALIGN_BYTES * sizeof(uint8_t)));
    TEST_ASSERT_NOT_EQUAL(NULL, dest_array_align16);

    // Apply byte unalignment for the worst-case test scenario
    uint32_t *dest_array_align1 = dest_array_align16 + (UNALIGN_BYTES * sizeof(uint8_t));

    bench_test_params_t test_params = {
        .height = HEIGHT,
        .width = WIDTH,
        .stride = STRIDE,
        .cc_height = HEIGHT - 1,
        .cc_width = WIDTH - 1,
        .benchmark_cycles = BENCHMARK_CYCLES,
        .array_align16 = (void *)dest_array_align16,
        .array_align1 = (void *)dest_array_align1,
    };

    TEST_ASSERT_EQUAL(ESP_OK, get_blend_params(&s_blend_params, &s_area));
    TEST_ASSERT_EQUAL(ESP_OK, set_color_format(s_blend_params, LV_COLOR_FORMAT_ARGB8888));

    ESP_LOGI(TAG_LV_FILL_BENCH, "running test for ARGB8888 color format");
    lv_fill_benchmark_init(&test_params);
    free(dest_array_align16);
}

TEST_CASE("LV Fill benchmark RGB565", "[lv_fill][RGB565]")
{
    uint16_t *dest_array_align16  = (uint16_t *)memalign(16, STRIDE * HEIGHT * sizeof(uint16_t) + (UNALIGN_BYTES * sizeof(uint8_t)));
    TEST_ASSERT_NOT_EQUAL(NULL, dest_array_align16);

    // Apply byte unalignment for the worst-case test scenario
    uint16_t *dest_array_align1 = dest_array_align16 + (UNALIGN_BYTES * sizeof(uint8_t));

    bench_test_params_t test_params = {
        .height = HEIGHT,
        .width = WIDTH,
        .stride = STRIDE,
        .cc_height = HEIGHT - 1,
        .cc_width = WIDTH - 1,
        .benchmark_cycles = BENCHMARK_CYCLES,
        .array_align16 = (void *)dest_array_align16,
        .array_align1 = (void *)dest_array_align1,
    };

    TEST_ASSERT_EQUAL(ESP_OK, get_blend_params(&s_blend_params, &s_area));
    TEST_ASSERT_EQUAL(ESP_OK, set_color_format(s_blend_params, LV_COLOR_FORMAT_RGB565));

    ESP_LOGI(TAG_LV_FILL_BENCH, "running test for RGB565 color format");
    lv_fill_benchmark_init(&test_params);
    free(dest_array_align16);
}
// ------------------------------------------------ Static test functions ----------------------------------------------

static void lv_fill_benchmark_init(bench_test_params_t *test_params)
{
    // Update all LV areas with most ideal test cases
    lv_area_set(&s_area->clip,  0, 0, test_params->stride - 1, test_params->height - 1);
    lv_area_set(&s_area->buf,   0, 0, test_params->stride - 1, test_params->height - 1);
    lv_area_set(&s_area->blend, 0, 0, test_params->width - 1,  test_params->height - 1);

    s_blend_params->draw_unit.target_layer->buf_area = s_area->buf;
    s_blend_params->draw_unit.target_layer->draw_buf->data = (void *)test_params->array_align16;

    // Run benchmark with the most ideal input parameters
    // Dest array is 16 byte aligned, dest_w and dest_h are dividable by 4
    float cycles = lv_fill_benchmark_run(test_params);     // Call LVGL API
    float per_sample = cycles / ((float)(WIDTH * STRIDE));

    ESP_LOGI(TAG_LV_FILL_BENCH, " ideal case: %.3f cycles for %dx%d matrix, %.3f cycles per sample", cycles, WIDTH, STRIDE, per_sample);

    // Update all LV areas with worst cases (Corner cases)
    lv_area_set(&s_area->clip,  0, 0, test_params->stride - 1, test_params->cc_height - 1);
    lv_area_set(&s_area->buf,   0, 0, test_params->stride - 1, test_params->cc_height - 1);
    lv_area_set(&s_area->blend, 0, 0, test_params->cc_width - 1,  test_params->cc_height - 1);

    s_blend_params->draw_unit.target_layer->buf_area = s_area->buf;
    s_blend_params->draw_unit.target_layer->draw_buf->data = (void *)test_params->array_align1;

    // Run benchmark with the corner case parameters
    // Dest array is 1 byte aligned, dest_w and dest_h are not dividable by 4
    cycles = lv_fill_benchmark_run(test_params);           // Call LVGL API
    per_sample = cycles / ((float)(WIDTH * STRIDE));

    ESP_LOGI(TAG_LV_FILL_BENCH, " common case: %.3f cycles for %dx%d matrix, %.3f cycles per sample", cycles, WIDTH, STRIDE, per_sample);
}

static float lv_fill_benchmark_run(bench_test_params_t *test_params)
{
    // Call the DUT function for the first time to init the benchmark test
    lv_draw_sw_blend(&s_blend_params->draw_unit, &s_blend_params->blend_dsc);

    const unsigned int start_b = xthal_get_ccount();
    for (int i = 0; i < test_params->benchmark_cycles; i++) {
        lv_draw_sw_blend(&s_blend_params->draw_unit, &s_blend_params->blend_dsc);
    }
    const unsigned int end_b = xthal_get_ccount();

    const float total_b = end_b - start_b;
    const float cycles = total_b / (test_params->benchmark_cycles);
    return cycles;
}
