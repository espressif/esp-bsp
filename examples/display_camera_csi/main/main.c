/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/**
 * @file
 * @brief BSP Camera Example (MIPI-CSI)
 * @details Stream camera (MIPI-CSI) output to display (LVGL)
 * @example https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_camera-
 */

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "bsp/esp-bsp.h"
#include "esp_err.h"
#include "esp_log.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include "driver/ppa.h"
#include "esp_private/esp_cache_private.h"
#include "app_video.h"

#define NUM_BUFS 2
#define ALIGN_UP(num, align)    (((num) + ((align) - 1)) & ~((align) - 1))

typedef struct {
    void  *start;
    size_t length;
} mmap_buf_t;

static const char *TAG = "example";
static int scale_level_res[] = {960, 480, 240, 120};
static ppa_client_handle_t ppa_srm_handle = NULL;
static size_t data_cache_line_size = 0;
static lv_obj_t *camera_canvas = NULL;
static uint8_t *cam_buff[NUM_BUFS];

static void app_ppa_init(void)
{
    /* Initialize PPA */
    ppa_client_config_t ppa_srm_config = {
        .oper_type = PPA_OPERATION_SRM,
    };

    esp_err_t ret = ppa_register_client(&ppa_srm_config, &ppa_srm_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register PPA client: 0x%x", ret);
    }
}

static esp_err_t app_image_process_scale_crop(
    uint8_t *in_buf, uint32_t in_width, uint32_t in_height,
    uint32_t crop_width, uint32_t crop_height,
    uint8_t *out_buf, uint32_t out_width, uint32_t out_height, size_t out_buf_size,
    ppa_srm_rotation_angle_t rotation_angle)
{
    ppa_srm_oper_config_t srm_config = {
        .in.buffer = in_buf,
        .in.pic_w = in_width,
        .in.pic_h = in_height,
        .in.block_w = crop_width,
        .in.block_h = crop_height,
        .in.block_offset_x = (in_width - crop_width) / 2,
        .in.block_offset_y = (in_height - crop_height) / 2,
        .in.srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        .out.buffer = out_buf,
        .out.buffer_size = out_buf_size,
        .out.pic_w = out_width,
        .out.pic_h = out_height,
        .out.block_offset_x = 0,
        .out.block_offset_y = 0,
        .out.srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        .rotation_angle = rotation_angle,
        .scale_x = (float)out_width / crop_width,
        .scale_y = (float)out_height / crop_height,
        .rgb_swap = 0,
        .byte_swap = 0,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };

    return ppa_do_scale_rotate_mirror(ppa_srm_handle, &srm_config);
}

static esp_err_t app_image_process_video_frame(
    uint8_t *in_buf, uint32_t in_width, uint32_t in_height,
    int scale_level, ppa_srm_rotation_angle_t rotation_angle,
    uint8_t *out_buf, size_t out_buf_size)
{
    int res_width = scale_level_res[scale_level - 1];
    int res_height = scale_level_res[scale_level - 1];

    return app_image_process_scale_crop(
               in_buf, in_width, in_height,
               res_width, res_height,
               out_buf, BSP_LCD_H_RES, BSP_LCD_V_RES, out_buf_size,
               rotation_angle
           );
}

static void camera_video_frame_operation(uint8_t *camera_buf, uint8_t camera_buf_index, uint32_t camera_buf_hes, uint32_t camera_buf_ves, size_t camera_buf_len)
{
    app_image_process_video_frame(
        camera_buf, camera_buf_hes, camera_buf_ves,
        1, PPA_SRM_ROTATION_ANGLE_0,
        cam_buff[camera_buf_index],
        ALIGN_UP(BSP_LCD_H_RES * BSP_LCD_V_RES * 2, data_cache_line_size)
    );

    bsp_display_lock(0);
    lv_canvas_set_buffer(camera_canvas, cam_buff[camera_buf_index], BSP_LCD_H_RES, BSP_LCD_V_RES, LV_COLOR_FORMAT_RGB565);
    lv_obj_invalidate(camera_canvas);
    bsp_display_unlock();
}



void app_main(void)
{
    esp_err_t ret = ESP_OK;

    bsp_display_start();
    bsp_display_backlight_on(); // Set display brightness to 100%

    /* Initialize Camera */
    bsp_camera_start();

    /* Initialize PPA for scaling */
    app_ppa_init();

    /* Get cache alignment */
    ret = esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &data_cache_line_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Get cache alignment failed: 0x%x", ret);
        return;
    }

    /* Allocate canvas buffers */
    uint32_t cam_buff_size = ALIGN_UP(BSP_LCD_H_RES * BSP_LCD_V_RES * 2, data_cache_line_size);
    for (int i = 0; i < EXAMPLE_CAM_BUF_NUM; i++) {
        cam_buff[i] = heap_caps_aligned_calloc(data_cache_line_size, 1, cam_buff_size, MALLOC_CAP_SPIRAM);
        if (cam_buff[i] == NULL) {
            ESP_LOGE(TAG, "Failed to allocate canvas buffer %d", i);
            return;
        }
    }

    /* Create LVGL canvas for camera image */
    bsp_display_lock(0);
    camera_canvas = lv_canvas_create(lv_scr_act());
    lv_canvas_set_buffer(camera_canvas, cam_buff[0], BSP_LCD_H_RES, BSP_LCD_V_RES, LV_COLOR_FORMAT_RGB565);
    assert(camera_canvas);
    lv_obj_center(camera_canvas);
    bsp_display_unlock();

    /* Open video device */
    int fd = app_video_open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, APP_VIDEO_FMT_RGB565);
    if (fd < 0) {
        ESP_LOGE(TAG, "Failed to open video device");
        return;
    }

    /* Initialize video capture device */
    ret = app_video_set_bufs(fd, NUM_BUFS, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set video buffers: 0x%x", ret);
        return;
    }

    /* Register frame process callback */
    ret = app_video_register_frame_operation_cb(camera_video_frame_operation);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register frame operation callback: 0x%x", ret);
        return;
    }

    /* Start video stream task */
    ret = app_video_stream_task_start(fd, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start video stream task: 0x%x", ret);
        return;
    }

    ESP_LOGI(TAG, "Camera example running.");
}
