/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP LVGL port
 */

#pragma once

#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "lvgl.h"

#if __has_include ("esp_lcd_touch.h")
#include "esp_lcd_touch.h"
#define ESP_LVGL_PORT_TOUCH_COMPONENT 1
#endif

#if __has_include ("iot_knob.h")
#include "iot_knob.h"
#include "iot_button.h"
#define ESP_LVGL_PORT_KNOB_COMPONENT 1
#endif

#if __has_include ("iot_button.h")
#include "iot_button.h"
#define ESP_LVGL_PORT_BUTTON_COMPONENT 1
#endif

#if __has_include ("usb/hid_host.h")
#define ESP_LVGL_PORT_USB_HOST_HID_COMPONENT 1
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Init configuration structure
 */
typedef struct {
    int task_priority;      /*!< LVGL task priority */
    int task_stack;         /*!< LVGL task stack size */
    int task_affinity;      /*!< LVGL task pinned to core (-1 is no affinity) */
    int task_max_sleep_ms;  /*!< Maximum sleep in LVGL task */
    int timer_period_ms;    /*!< LVGL timer tick period in ms */
} lvgl_port_cfg_t;

/**
 * @brief Rotation configuration
 */
typedef struct {
    bool swap_xy;  /*!< LCD Screen swapped X and Y (in esp_lcd driver) */
    bool mirror_x; /*!< LCD Screen mirrored X (in esp_lcd driver) */
    bool mirror_y; /*!< LCD Screen mirrored Y (in esp_lcd driver) */
} lvgl_port_rotation_cfg_t;


typedef void (*lv_flush_cb)(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);

typedef void (*lv_update_cb)(lv_disp_drv_t *drv);

typedef BaseType_t (*lv_transdone_cb)(esp_lcd_panel_handle_t panel, void *user_ctx);

typedef struct {
    esp_lcd_panel_io_handle_t io_handle;    /* LCD panel IO handle */
    esp_lcd_panel_handle_t    panel_handle; /* LCD panel handle */
    lvgl_port_rotation_cfg_t  rotation;     /* Default values of the screen rotation */
    lv_disp_drv_t             disp_drv;     /* LVGL display driver */

    TaskHandle_t              lvgl_task_handle;
    lv_transdone_cb           lcd_transdone_cb;
    bool                      lcd_manual_mode;
} lvgl_port_display_ctx_t;

/**
 * @brief Configuration display structure
 */
typedef struct {
    esp_lcd_panel_io_handle_t io_handle;    /*!< LCD panel IO handle */
    esp_lcd_panel_handle_t panel_handle;    /*!< LCD panel handle */
    uint32_t    buffer_size;    /*!< Size of the buffer for the screen in pixels */
    bool        double_buffer;  /*!< True, if should be allocated two buffers */
    uint32_t    hres;           /*!< LCD display horizontal resolution */
    uint32_t    vres;           /*!< LCD display vertical resolution */
    bool        monochrome;     /*!< True, if display is monochrome and using 1bit for 1px */
    lvgl_port_rotation_cfg_t rotation;    /*!< Default values of the screen rotation */

    struct {
        unsigned int buff_dma: 1;    /*!< Allocated LVGL buffer will be DMA capable */
        unsigned int buff_spiram: 1; /*!< Allocated LVGL buffer will be in PSRAM */
        unsigned int interface_RGB: 1;
    } flags;

    struct {
        unsigned int full_refresh: 1;
        unsigned int direct_mode: 1;
        unsigned int normal_mode: 1;
    } refresh_mode;

    struct {
        unsigned int audo_mode: 1;
        unsigned int manual_mode: 1;
        unsigned int bb_mode: 1;
        struct {
            unsigned int ref_period;
            unsigned int ref_priority;
        } flags;
    } trans_mode;

    lv_flush_cb user_lv_flush_cb;
    lv_update_cb user_lv_update_cb;
    lv_transdone_cb user_lcd_transdone_cb;

    void *user_buf1;
    void *user_buf2;

} lvgl_port_display_cfg_t;

#ifdef ESP_LVGL_PORT_TOUCH_COMPONENT
/**
 * @brief Configuration touch structure
 */
typedef struct {
    lv_disp_t *disp;    /*!< LVGL display handle (returned from lvgl_port_add_disp) */
    esp_lcd_touch_handle_t   handle;   /*!< LCD touch IO handle */
} lvgl_port_touch_cfg_t;
#endif

#ifdef ESP_LVGL_PORT_KNOB_COMPONENT
/**
 * @brief Configuration of the encoder structure
 */
typedef struct {
    lv_disp_t *disp;    /*!< LVGL display handle (returned from lvgl_port_add_disp) */
    const knob_config_t *encoder_a_b;
    const button_config_t *encoder_enter;  /*!< Navigation button for enter */
} lvgl_port_encoder_cfg_t;
#endif

#ifdef ESP_LVGL_PORT_BUTTON_COMPONENT
/**
 * @brief Configuration of the navigation buttons structure
 */
typedef struct {
    lv_disp_t *disp;                /*!< LVGL display handle (returned from lvgl_port_add_disp) */
    const button_config_t *button_prev;   /*!< Navigation button for previous */
    const button_config_t *button_next;   /*!< Navigation button for next */
    const button_config_t *button_enter;  /*!< Navigation button for enter */
} lvgl_port_nav_btns_cfg_t;
#endif

#ifdef ESP_LVGL_PORT_USB_HOST_HID_COMPONENT
/**
 * @brief Configuration of the mouse input
 */
typedef struct {
    lv_disp_t *disp;        /*!< LVGL display handle (returned from lvgl_port_add_disp) */
    uint8_t sensitivity;    /*!< Mouse sensitivity (cannot be zero) */
    lv_obj_t *cursor_img;   /*!< Mouse cursor image, if NULL then used default */
} lvgl_port_hid_mouse_cfg_t;

/**
 * @brief Configuration of the keyboard input
 */
typedef struct {
    lv_disp_t *disp;        /*!< LVGL display handle (returned from lvgl_port_add_disp) */
} lvgl_port_hid_keyboard_cfg_t;
#endif

/**
 * @brief LVGL port configuration structure
 *
 */
#define ESP_LVGL_PORT_INIT_CONFIG() \
    {                               \
        .task_priority = 4,       \
        .task_stack = 4096,       \
        .task_affinity = -1,      \
        .task_max_sleep_ms = 500, \
        .timer_period_ms = 5,     \
    }

/**
 * @brief Initialize LVGL portation
 *
 * @note This function initialize LVGL and create timer and task for LVGL right working.
 *
 * @return
 *      - ESP_OK                    on success
 *      - ESP_ERR_INVALID_ARG       if some of the create_args are not valid
 *      - ESP_ERR_INVALID_STATE     if esp_timer library is not initialized yet
 *      - ESP_ERR_NO_MEM            if memory allocation fails
 */
esp_err_t lvgl_port_init(const lvgl_port_cfg_t *cfg);

/**
 * @brief Deinitialize LVGL portation
 *
 * @note This function deinitializes LVGL and stops the task if running.
 * Some deinitialization will be done after the task will be stopped.
 *
 * @return
 *      - ESP_OK                    on success
 */
esp_err_t lvgl_port_deinit(void);

/**
 * @brief Add display handling to LVGL
 *
 * @note Allocated memory in this function is not free in deinit. You must call lvgl_port_remove_disp for free all memory!
 *
 * @param disp_cfg Display configuration structure
 * @return Pointer to LVGL display or NULL when error occured
 */
lv_disp_t *lvgl_port_add_disp(const lvgl_port_display_cfg_t *disp_cfg);

/**
 * @brief Remove display handling from LVGL
 *
 * @note Free all memory used for this display.
 *
 * @return
 *      - ESP_OK                    on success
 */
esp_err_t lvgl_port_remove_disp(lv_disp_t *disp);

#ifdef ESP_LVGL_PORT_TOUCH_COMPONENT
/**
 * @brief Add LCD touch as an input device
 *
 * @note Allocated memory in this function is not free in deinit. You must call lvgl_port_remove_touch for free all memory!
 *
 * @param touch_cfg Touch configuration structure
 * @return Pointer to LVGL touch input device or NULL when error occured
 */
lv_indev_t *lvgl_port_add_touch(const lvgl_port_touch_cfg_t *touch_cfg);

/**
 * @brief Remove selected LCD touch from input devices
 *
 * @note Free all memory used for this display.
 *
 * @return
 *      - ESP_OK                    on success
 */
esp_err_t lvgl_port_remove_touch(lv_indev_t *touch);
#endif

#ifdef ESP_LVGL_PORT_KNOB_COMPONENT
/**
 * @brief Add encoder as an input device
 *
 * @note Allocated memory in this function is not free in deinit. You must call lvgl_port_remove_encoder for free all memory!
 *
 * @param encoder_cfg Encoder configuration structure
 * @return Pointer to LVGL encoder input device or NULL when error occured
 */
lv_indev_t *lvgl_port_add_encoder(const lvgl_port_encoder_cfg_t *encoder_cfg);

/**
 * @brief Remove encoder from input devices
 *
 * @note Free all memory used for this input device.
 *
 * @return
 *      - ESP_OK                    on success
 */
esp_err_t lvgl_port_remove_encoder(lv_indev_t *encoder);
#endif

#ifdef ESP_LVGL_PORT_BUTTON_COMPONENT
/**
 * @brief Add buttons as an input device
 *
 * @note Allocated memory in this function is not free in deinit. You must call lvgl_port_remove_navigation_buttons for free all memory!
 *
 * @param buttons_cfg Buttons configuration structure
 * @return Pointer to LVGL buttons input device or NULL when error occured
 */
lv_indev_t *lvgl_port_add_navigation_buttons(const lvgl_port_nav_btns_cfg_t *buttons_cfg);

/**
 * @brief Remove selected buttons from input devices
 *
 * @note Free all memory used for this input device.
 *
 * @return
 *      - ESP_OK                    on success
 */
esp_err_t lvgl_port_remove_navigation_buttons(lv_indev_t *buttons);
#endif

#ifdef ESP_LVGL_PORT_USB_HOST_HID_COMPONENT
/**
 * @brief Add USB HID mouse as an input device
 *
 * @note The USB host must be initialized before. Use `usb_host_install` for host initialization.
 *
 * @param mouse_cfg mouse configuration structure
 * @return Pointer to LVGL buttons input device or NULL when error occured
 */
lv_indev_t *lvgl_port_add_usb_hid_mouse_input(const lvgl_port_hid_mouse_cfg_t *mouse_cfg);

/**
 * @brief Add USB HID keyboard as an input device
 *
 * @note The USB host must be initialized before. Use `usb_host_install` for host initialization.
 *
 * @param keyboard_cfg keyboard configuration structure
 * @return Pointer to LVGL buttons input device or NULL when error occured
 */
lv_indev_t *lvgl_port_add_usb_hid_keyboard_input(const lvgl_port_hid_keyboard_cfg_t *keyboard_cfg);

/**
 * @brief Remove selected USB HID from input devices
 *
 * @note Free all memory used for this input device. When removed all HID devices, the HID task will be freed.
 *
 * @return
 *      - ESP_OK                    on success
 */
esp_err_t lvgl_port_remove_usb_hid_input(lv_indev_t *hid);
#endif

/**
 * @brief Take LVGL mutex
 *
 * @param timeout_ms Timeout in [ms]. 0 will block indefinitely.
 * @return
 *      - true  Mutex was taken
 *      - false Mutex was NOT taken
 */
bool lvgl_port_lock(uint32_t timeout_ms);

/**
 * @brief Give LVGL mutex
 *
 */
void lvgl_port_unlock(void);

/**
 * @brief Notify LVGL, that data was flushed to LCD display
 *
 * @note It should be used only when not called inside LVGL port (more in README).
 *
 * @param disp          LVGL display handle (returned from lvgl_port_add_disp)
 */
void lvgl_port_flush_ready(lv_disp_t *disp);

/**
 * @brief Stop lvgl task
 *
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the timer is not running
 */
esp_err_t lvgl_port_stop(void);

/**
 * @brief Resume lvgl task
 *
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the timer is not running
 */
esp_err_t lvgl_port_resume(void);

#ifdef __cplusplus
}
#endif
