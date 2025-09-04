/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_spiffs.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_ldo_regulator.h"
#include "esp_vfs_fat.h"

#include "usb/usb_host.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"
#include "esp_lcd_ili9881c.h"
#include "bsp/m5stack_tab5.h"
#include "bsp/display.h"
#include "bsp/touch.h"
#include "esp_lcd_touch_gt911.h"
#include "bsp_err_check.h"
#include "esp_codec_dev_defaults.h"
#include "ili9881_init_data.h"
#include "string.h"
#include "esp_io_expander_pi4ioe5v6408.h"

static const char *TAG = "M5STACK_TAB5";

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_indev_t *disp_indev = NULL;
#endif  // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)

// Global SD card handler
static sdmmc_card_t *bsp_sdcard = NULL;

// USB Host Library task
static TaskHandle_t usb_host_task;

// sys i2c
static bool i2c_initialized               = false;
static i2c_master_bus_handle_t i2c_handle = NULL;

// i2s
static i2s_chan_handle_t i2s_tx_chan            = NULL;
static i2s_chan_handle_t i2s_rx_chan            = NULL;
static const audio_codec_data_if_t *i2s_data_if = NULL; /* Codec data interface */

esp_err_t bsp_i2c_init(void)
{
    /* I2C was initialized before */
    if (i2c_initialized) {
        return ESP_OK;
    }

    i2c_master_bus_config_t i2c_bus_conf = {
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .sda_io_num                   = BSP_I2C_SDA,
        .scl_io_num                   = BSP_I2C_SCL,
        .i2c_port                     = BSP_I2C_NUM,
        .flags.enable_internal_pullup = true,
    };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_new_master_bus(&i2c_bus_conf, &i2c_handle));

    i2c_initialized = true;

    return ESP_OK;
}

esp_err_t bsp_i2c_deinit(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(i2c_del_master_bus(i2c_handle));
    i2c_initialized = false;
    return ESP_OK;
}

static i2c_master_bus_handle_t bsp_i2c_get_handle(void)
{
    return i2c_handle;
}

//==================================================================================
// I/O Expander PI4IOE5V6408
//==================================================================================
static esp_io_expander_handle_t io_expander_pi4ioe1 = NULL;
static esp_io_expander_handle_t io_expander_pi4ioe2 = NULL;

static esp_err_t bsp_io_expander_pi4ioe_init()
{
    if (io_expander_pi4ioe1 && io_expander_pi4ioe2) {
        return ESP_OK;
    }

    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());
    i2c_master_bus_handle_t bus_handle = bsp_i2c_get_handle();

    // Initialize PI4IOE1 (address 0x43)
    ESP_RETURN_ON_ERROR(esp_io_expander_new_i2c_pi4ioe5v6408(bus_handle, ESP_IO_EXPANDER_I2C_PI4IOE5V6408_ADDRESS_LOW, &io_expander_pi4ioe1), TAG, "Create PI4IOE1 failed");

    // Initialize PI4IOE2 (address 0x44)
    ESP_RETURN_ON_ERROR(esp_io_expander_new_i2c_pi4ioe5v6408(bus_handle, ESP_IO_EXPANDER_I2C_PI4IOE5V6408_ADDRESS_HIGH, &io_expander_pi4ioe2), TAG, "Create PI4IOE2 failed");

    // P0: input, P1: SPK_EN(output), P2: EXT5V_EN(output), P3: input, P4: LCD_RST(output), P5: TP_RST(output), P6: CAM_RST(output), P7: input
    const esp_io_expander_pi4ioe5v6408_config_t pi4ioe1_config = {
        .io_dir     = 0b01111111,  // 0=input, 1=output
        .out_h_im   = 0b00000000,  // output high impedance
        .pull_sel   = 0b01111111,  // pull up/down select, 0=down, 1=up
        .pull_en    = 0b01111111,  // pull up/down enable, 0=disable, 1=enable
        .in_def_sta = 0xFF,        // skip this register
        .int_mask   = 0xFF,        // skip this register
        .out_set    = 0b01110110   // P1(SPK_EN), P2(EXT5V_EN), P4(LCD_RST), P5(TP_RST), P6(CAM_RST)
    };
    ESP_RETURN_ON_ERROR(esp_io_expander_pi4ioe5v6408_config_registers(io_expander_pi4ioe1, &pi4ioe1_config), TAG, "PI4IOE1 config failed");

    // P0: WLAN_PWR_EN(output), P1: input, P2: input, P3: USB5V_EN(output), P4: input, P5: input, P6: input, P7: CHG_EN(output)
    const esp_io_expander_pi4ioe5v6408_config_t pi4ioe2_config = {
        .io_dir     = 0b10111001,  // 0=input, 1=output
        .out_h_im   = 0b00000110,  // output high impedance
        .pull_sel   = 0b10111001,  // pull up/down select, 0=down, 1=up
        .pull_en    = 0b11111001,  // pull up/down enable, 0=disable, 1=enable
        .in_def_sta = 0b01000000,  // input default status
        .int_mask   = 0b10111111,  // interrupt mask
        .out_set    = 0b00001001   // P0(WLAN_PWR_EN)=0, P3(USB5V_EN)=1, P7(CHG_EN)=0
    };
    ESP_RETURN_ON_ERROR(esp_io_expander_pi4ioe5v6408_config_registers(io_expander_pi4ioe2, &pi4ioe2_config), TAG, "PI4IOE2 config failed");

    ESP_LOGI(TAG, "PI4IOE5V6408 IO expanders initialized");
    return ESP_OK;
}

//==================================================================================
// sd card
//==================================================================================
#define BSP_LDO_PROBE_SD_CHAN       4
#define BSP_LDO_PROBE_SD_VOLTAGE_MV 3300
#define SDMMC_BUS_WIDTH (4)
#define SDCARD_SDMMC_HOST_SLOT (SDMMC_HOST_SLOT_0)

esp_err_t bsp_sdcard_mount(void)
{
    bsp_sdcard_cfg_t cfg = {0};
    return bsp_sdcard_sdmmc_mount(&cfg);
}

esp_err_t bsp_sdcard_unmount(void)
{
    esp_err_t ret = ESP_OK;

    ret |= esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, bsp_sdcard);
    bsp_sdcard = NULL;

    return ret;
}

sdmmc_card_t *bsp_sdcard_get_handle(void)
{
    return bsp_sdcard;
}

void bsp_sdcard_get_sdmmc_host(const int slot, sdmmc_host_t *config)
{
    assert(config);
    memset(config, 0, sizeof(sdmmc_host_t));

    esp_err_t ret_val = ESP_OK;

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot         = slot;
    host.max_freq_khz                   = SDMMC_FREQ_HIGHSPEED;
    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = BSP_LDO_PROBE_SD_CHAN,  // `LDO_VO4` is used as the SDMMC IO power
    };
    static sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;

    if (pwr_ctrl_handle == NULL) {
        ret_val = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
        if (ret_val != ESP_OK) {
            ESP_LOGE(TAG, "Failed to new an on-chip ldo power control driver");
            return;
        }
    }
    host.pwr_ctrl_handle = pwr_ctrl_handle;

    memcpy(config, &host, sizeof(sdmmc_host_t));
}

void bsp_sdcard_get_sdspi_host(const int slot, sdmmc_host_t *config)
{
    assert(config);
    memset(config, 0, sizeof(sdmmc_host_t));
    ESP_LOGE(TAG, "SD card SPI mode is not supported yet");
}

void bsp_sdcard_sdmmc_get_slot(const int slot, sdmmc_slot_config_t *config)
{
    assert(config);
    memset(config, 0, sizeof(sdmmc_slot_config_t));

    config->width = SDMMC_BUS_WIDTH;
    config->clk   = BSP_SD_CLK;
    config->cmd   = BSP_SD_CMD;
    config->d0    = BSP_SD_D0;
    config->d1    = BSP_SD_D1;
    config->d2    = BSP_SD_D2;
    config->d3    = BSP_SD_D3;
}

void bsp_sdcard_sdspi_get_slot(const spi_host_device_t spi_host, sdspi_device_config_t *config)
{
    assert(config);
    memset(config, 0, sizeof(sdspi_device_config_t));
    ESP_LOGE(TAG, "SD card SPI mode is not supported yet");
}

esp_err_t bsp_sdcard_sdmmc_mount(bsp_sdcard_cfg_t *cfg)
{
    esp_err_t ret_val = ESP_OK;

    if (NULL != bsp_sdcard) {
        return ESP_ERR_INVALID_STATE;
    }

    BSP_ERROR_CHECK_RETURN_ERR(bsp_feature_enable(BSP_FEATURE_SD, true));

    /**
     * @brief Use settings defined above to initialize SD card and mount FAT filesystem.
     *   Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
     *   Please check its source code and implement error recovery when developing
     *   production applications.
     *
     */
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    bsp_sdcard_get_sdmmc_host(SDCARD_SDMMC_HOST_SLOT, &host);

    /**
     * @brief This initializes the slot without card detect (CD) and write protect (WP) signals.
     *   Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
     *
     */
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    bsp_sdcard_sdmmc_get_slot(SDCARD_SDMMC_HOST_SLOT, &slot_config);

    /**
     * @brief Options for mounting the filesystem.
     *   If format_if_mount_failed is set to true, SD card will be partitioned and
     *   formatted in case when mounting fails.
     */
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_BSP_SD_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files = 25,
        .allocation_unit_size = 16 * 1024
    };

    ret_val = esp_vfs_fat_sdmmc_mount(CONFIG_BSP_SD_MOUNT_POINT, &host, &slot_config, &mount_config, &bsp_sdcard);

    /* Check for SDMMC mount result. */
    if (ret_val != ESP_OK) {
        if (ret_val == ESP_FAIL) {
            ESP_LOGE(TAG,
                     "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG,
                     "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.",
                     esp_err_to_name(ret_val));
        }
        return ret_val;
    }

    /* Card has been initialized, print its properties. */
    sdmmc_card_print_info(stdout, bsp_sdcard);

    return ret_val;
}

esp_err_t bsp_sdcard_sdspi_mount(bsp_sdcard_cfg_t *cfg)
{
    ESP_LOGE(TAG, "SD card SPIFFS mode is not supported yet");
    return ESP_ERR_NOT_SUPPORTED;
}

//==================================================================================
// spiffs
//==================================================================================
esp_err_t bsp_spiffs_mount(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path       = CONFIG_BSP_SPIFFS_MOUNT_POINT,
        .partition_label = CONFIG_BSP_SPIFFS_PARTITION_LABEL,
        .max_files       = CONFIG_BSP_SPIFFS_MAX_FILES,
#ifdef CONFIG_BSP_SPIFFS_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
    };

    esp_err_t ret_val = esp_vfs_spiffs_register(&conf);

    BSP_ERROR_CHECK_RETURN_ERR(ret_val);

    size_t total = 0, used = 0;
    ret_val = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret_val != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret_val));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    return ret_val;
}

esp_err_t bsp_spiffs_unmount(void)
{
    return esp_vfs_spiffs_unregister(CONFIG_BSP_SPIFFS_PARTITION_LABEL);
}

//==================================================================================
// audio es7210 + es8388
//==================================================================================
typedef esp_err_t (*bsp_codec_mute_fn)(bool enable);
typedef esp_err_t (*bsp_codec_reconfig_fn)(uint32_t rate, uint32_t bps, i2s_slot_mode_t ch);
typedef esp_err_t (*bsp_i2s_reconfig_clk_fn)(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch);

/* Can be used for `i2s_std_gpio_config_t` and/or `i2s_std_config_t` initialization */
#define BSP_I2S_GPIO_CFG                                                                                           \
    {                                                                                                              \
        .mclk = BSP_I2S_MCLK, .bclk = BSP_I2S_SCLK, .ws = BSP_I2S_LCLK, .dout = BSP_I2S_DOUT, .din = BSP_I2S_DSIN, \
        .invert_flags = {                                                                                          \
            .mclk_inv = false,                                                                                     \
            .bclk_inv = false,                                                                                     \
            .ws_inv   = false,                                                                                     \
        },                                                                                                         \
    }

/* This configuration is used by default in `bsp_extra_audio_init()` */
#define BSP_I2S_DUPLEX_MONO_CFG(_sample_rate)                                                         \
    {                                                                                                 \
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(_sample_rate),                                         \
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO), \
        .gpio_cfg = BSP_I2S_GPIO_CFG,                                                                 \
    }

esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config)
{
    if (i2s_tx_chan && i2s_rx_chan) {
        /* Audio was initialized before */
        return ESP_OK;
    }

    /* Setup I2S peripheral */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(CONFIG_BSP_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear        = true;  // Auto clear the legacy data in the DMA buffer
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_tx_chan, &i2s_rx_chan));

    /* Setup I2S channels */
    // const i2s_std_config_t std_cfg_default = BSP_I2S_DUPLEX_MONO_CFG(16000);
    const i2s_std_config_t std_cfg_default = BSP_I2S_DUPLEX_MONO_CFG(48000);
    const i2s_std_config_t *p_i2s_cfg      = &std_cfg_default;
    if (i2s_config != NULL) {
        p_i2s_cfg = i2s_config;
    }

    if (i2s_tx_chan != NULL) {
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_tx_chan, p_i2s_cfg));
        ESP_ERROR_CHECK(i2s_channel_enable(i2s_tx_chan));
    }

    i2s_tdm_config_t tdm_cfg = {
        .clk_cfg =
        {
            .sample_rate_hz  = (uint32_t)48000,
            .clk_src         = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,
            .mclk_multiple   = I2S_MCLK_MULTIPLE_256,
            .bclk_div        = 8,
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode      = I2S_SLOT_MODE_STEREO,
            .slot_mask      = (I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3),
            .ws_width       = I2S_TDM_AUTO_WS_WIDTH,
            .ws_pol         = false,
            .bit_shift      = true,
            .left_align     = false,
            .big_endian     = false,
            .bit_order_lsb  = false,
            .skip_mask      = false,
            .total_slot     = I2S_TDM_AUTO_SLOT_NUM
        },
        .gpio_cfg = BSP_I2S_GPIO_CFG,
    };

    if (i2s_rx_chan != NULL) {
        ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(i2s_rx_chan, &tdm_cfg));
        ESP_ERROR_CHECK(i2s_channel_enable(i2s_rx_chan));
    }

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port      = CONFIG_BSP_I2S_NUM,
        .tx_handle = i2s_tx_chan,
        .rx_handle = i2s_rx_chan,
    };
    i2s_data_if = audio_codec_new_i2s_data(&i2s_cfg);

    return ESP_OK;
}

esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void)
{
    static esp_codec_dev_handle_t codec = NULL;
    if (codec) {
        return codec;
    }

    if (i2s_data_if == NULL) {
        /* Initilize I2C */
        bsp_i2c_init();
        /* Configure I2S peripheral and Power Amplifier */
        bsp_audio_init(NULL);
    }
    assert(i2s_data_if);

    BSP_ERROR_CHECK_RETURN_ERR(bsp_feature_enable(BSP_FEATURE_SPEAKER, true));

    i2c_master_bus_handle_t i2c_bus_handle = bsp_i2c_get_handle();
    audio_codec_i2c_cfg_t i2c_cfg          = {
        .port       = BSP_I2C_NUM,
        .addr       = ES8388_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c_bus_handle,
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    BSP_NULL_CHECK(i2c_ctrl_if, NULL);

    es8388_codec_cfg_t es8388_cfg = {
        .codec_mode  = ESP_CODEC_DEV_WORK_MODE_DAC,
        .master_mode = false,
        .ctrl_if     = i2c_ctrl_if,
        .pa_pin      = -1,  // PI4IOE1 P1 控制
    };
    const audio_codec_if_t *es8388_dev = es8388_codec_new(&es8388_cfg);
    BSP_NULL_CHECK(es8388_dev, NULL);

    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = es8388_dev,
        .data_if  = i2s_data_if,
    };
    codec = esp_codec_dev_new(&codec_dev_cfg);
    BSP_NULL_CHECK(codec, NULL);

    return codec;
}

esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void)
{
    if (i2s_data_if == NULL) {
        /* Initilize I2C */
        ESP_ERROR_CHECK(bsp_i2c_init());
        /* Configure I2S peripheral and Power Amplifier */
        ESP_ERROR_CHECK(bsp_audio_init(NULL));
        // i2s_data_if = bsp_get_codec_data_if();
    }
    assert(i2s_data_if);

    i2c_master_bus_handle_t i2c_bus_handle = bsp_i2c_get_handle();
    audio_codec_i2c_cfg_t i2c_cfg          = {
        .port       = BSP_I2C_NUM,
        .addr       = ES7210_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c_bus_handle,
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    BSP_NULL_CHECK(i2c_ctrl_if, NULL);

    es7210_codec_cfg_t es7210_cfg = {
        .ctrl_if = i2c_ctrl_if,  // Codec Control interface
    };
    es7210_cfg.mic_selected            = ES7210_SEL_MIC1;
    const audio_codec_if_t *es7210_dev = es7210_codec_new(&es7210_cfg);
    BSP_NULL_CHECK(es7210_dev, NULL);

    esp_codec_dev_cfg_t codec_es7210_dev_cfg = {
        .dev_type =
        ESP_CODEC_DEV_TYPE_IN,  // Codec device type: Codec input device like ADC (capture data from microphone)
        .codec_if = es7210_dev,     // Codec interface
        .data_if  = i2s_data_if,    // Codec data interface
    };

    return esp_codec_dev_new(&codec_es7210_dev_cfg);
}

// Bit number used to represent command and parameter
#define LCD_LEDC_CH LEDC_CHANNEL_1  // CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH
esp_err_t bsp_display_brightness_init(void)
{
    // Setup LEDC peripheral for PWM backlight control
    const ledc_timer_config_t lcd_backlight_timer = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                                     .duty_resolution = LEDC_TIMER_12_BIT,
                                                     .timer_num       = LEDC_TIMER_0,
                                                     .freq_hz         = 5000,
                                                     .clk_cfg = LEDC_AUTO_CLK
                                                    };
    ESP_ERROR_CHECK(ledc_timer_config(&lcd_backlight_timer));

    const ledc_channel_config_t lcd_backlight_channel = {.gpio_num   = BSP_LCD_BACKLIGHT,
                                                         .speed_mode = LEDC_LOW_SPEED_MODE,
                                                         .channel    = LCD_LEDC_CH,
                                                         .intr_type  = LEDC_INTR_DISABLE,
                                                         .timer_sel  = LEDC_TIMER_0,
                                                         .duty       = 0,
                                                         .hpoint     = 0
                                                        };

    ESP_ERROR_CHECK(ledc_channel_config(&lcd_backlight_channel));

    return ESP_OK;
}

const audio_codec_data_if_t *bsp_audio_get_codec_itf(void)
{
    return i2s_data_if;
}

esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    if (brightness_percent > 100) {
        brightness_percent = 100;
    }
    if (brightness_percent < 0) {
        brightness_percent = 0;
    }

    ESP_LOGI(TAG, "Setting LCD backlight: %d%%", brightness_percent);
    uint32_t duty_cycle = (4095 * brightness_percent) / 100;  // LEDC resolution set to 12bits, thus: 100% = 4095
    BSP_ERROR_CHECK_RETURN_ERR(ledc_set_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH, duty_cycle));
    BSP_ERROR_CHECK_RETURN_ERR(ledc_update_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH));
    return ESP_OK;
}

esp_err_t bsp_display_backlight_off(void)
{
    return bsp_display_brightness_set(0);
}

esp_err_t bsp_display_backlight_on(void)
{
    return bsp_display_brightness_set(100);
}

static esp_err_t bsp_enable_dsi_phy_power(void)
{
#if BSP_MIPI_DSI_PHY_PWR_LDO_CHAN > 0
    // Turn on the power for MIPI DSI PHY, so it can go from "No Power" state to "Shutdown" state
    static esp_ldo_channel_handle_t phy_pwr_chan = NULL;
    esp_ldo_channel_config_t ldo_cfg             = {
        .chan_id    = BSP_MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = BSP_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&ldo_cfg, &phy_pwr_chan), TAG, "Acquire LDO channel for DPHY failed");
    ESP_LOGI(TAG, "MIPI DSI PHY Powered on");
#endif  // BSP_MIPI_DSI_PHY_PWR_LDO_CHAN > 0

    return ESP_OK;
}

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel,
                          esp_lcd_panel_io_handle_t *ret_io)
{
    esp_err_t ret = ESP_OK;
    bsp_lcd_handles_t handles;
    ret = bsp_display_new_with_handles(config, &handles);

    *ret_panel = handles.panel;
    *ret_io    = handles.io;

    return ret;
}

esp_err_t bsp_display_new_with_handles(const bsp_display_config_t *config, bsp_lcd_handles_t *ret_handles)
{
    BSP_ERROR_CHECK_RETURN_ERR(bsp_feature_enable(BSP_FEATURE_LCD, true));

    esp_err_t ret                     = ESP_OK;
    esp_lcd_panel_io_handle_t io      = NULL;
    esp_lcd_panel_handle_t disp_panel = NULL;

    ESP_RETURN_ON_ERROR(bsp_display_brightness_init(), TAG, "Brightness init failed");
    ESP_RETURN_ON_ERROR(bsp_enable_dsi_phy_power(), TAG, "DSI PHY power failed");

    /* create MIPI DSI bus first, it will initialize the DSI PHY as well */
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus = NULL;
    esp_lcd_dsi_bus_config_t bus_config   = {
        .bus_id             = 0,
        .num_data_lanes     = BSP_LCD_MIPI_DSI_LANE_NUM,
        .phy_clk_src        = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = BSP_LCD_MIPI_DSI_LANE_BITRATE_MBPS,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus), TAG, "New DSI bus init failed");

    ESP_LOGI(TAG, "Install MIPI DSI LCD control panel");
    // we use DBI interface to send LCD commands and parameters
    esp_lcd_dbi_io_config_t dbi_config = {
        .virtual_channel = 0,
        .lcd_cmd_bits    = 8,  // according to the LCD spec
        .lcd_param_bits  = 8,  // according to the LCD spec
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &io), err, TAG, "New panel IO failed");

    ESP_LOGI(TAG, "Install LCD driver of ili9881c");
    esp_lcd_dpi_panel_config_t dpi_config = {
        .virtual_channel    = 0,
        .dpi_clk_src        = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = 60,  // 720*1280 RGB24 60Hz RGB24 // 80,
        .pixel_format       = LCD_COLOR_PIXEL_FORMAT_RGB565,
        .num_fbs            = 1,
        .video_timing =
        {
            .h_size            = BSP_LCD_H_RES,
            .v_size            = BSP_LCD_V_RES,
            .hsync_back_porch  = 140,
            .hsync_pulse_width = 40,
            .hsync_front_porch = 40,
            .vsync_back_porch  = 20,
            .vsync_pulse_width = 4,
            .vsync_front_porch = 20,
        },
        .flags.use_dma2d = true,
    };

    ili9881c_vendor_config_t vendor_config = {
        .init_cmds      = tab5_lcd_ili9881c_specific_init_code_default,
        .init_cmds_size = sizeof(tab5_lcd_ili9881c_specific_init_code_default) /
        sizeof(tab5_lcd_ili9881c_specific_init_code_default[0]),
        .mipi_config =
        {
            .dsi_bus    = mipi_dsi_bus,
            .dpi_config = &dpi_config,
            .lane_num   = 2,
        },
    };

    const esp_lcd_panel_dev_config_t lcd_dev_config = {
        .bits_per_pixel = 16,
        .rgb_ele_order  = BSP_LCD_COLOR_SPACE,
        .reset_gpio_num = -1,
        .vendor_config  = &vendor_config,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9881c(io, &lcd_dev_config, &disp_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(disp_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(disp_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(disp_panel, true));

    /* Return all handles */
    ret_handles->io           = io;
    ret_handles->mipi_dsi_bus = mipi_dsi_bus;
    ret_handles->panel        = disp_panel;
    ret_handles->control      = NULL;

    ESP_LOGI(TAG, "Display initialized with resolution %dx%d", BSP_LCD_H_RES, BSP_LCD_V_RES);

    return ret;

err:
    if (disp_panel) {
        esp_lcd_panel_del(disp_panel);
    }
    if (io) {
        esp_lcd_panel_io_del(io);
    }
    if (mipi_dsi_bus) {
        esp_lcd_del_dsi_bus(mipi_dsi_bus);
    }
    return ret;
}

esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch)
{
    BSP_ERROR_CHECK_RETURN_ERR(bsp_feature_enable(BSP_FEATURE_TOUCH, true));

    /* Initialize touch */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max        = BSP_LCD_H_RES,
        .y_max        = BSP_LCD_V_RES,
        .rst_gpio_num = -1,  // BSP_LCD_TOUCH_RST, // NC
        .int_gpio_num = 23,  // BSP_LCD_TOUCH_INT,
        .levels =
        {
            .reset     = 0,
            .interrupt = 0,
        },
        .flags =
        {
            .swap_xy  = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    esp_lcd_panel_io_handle_t tp_io_handle     = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    tp_io_config.dev_addr                      = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP;
    tp_io_config.scl_speed_hz                  = CONFIG_BSP_I2C_CLK_SPEED_HZ;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(i2c_handle, &tp_io_config, &tp_io_handle), TAG, "");
    return esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, ret_touch);
}

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_display_t *bsp_display_lcd_init(const bsp_display_lcd_config_t *cfg)
{
    assert(cfg != NULL);
    bsp_lcd_handles_t lcd_panels;
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new_with_handles(NULL, &lcd_panels));

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle      = lcd_panels.io,
        .panel_handle   = lcd_panels.panel,
        .control_handle = lcd_panels.control,
        .buffer_size    = cfg->buffer_size,
        .double_buffer  = cfg->double_buffer,
        .hres           = BSP_LCD_H_RES,
        .vres           = BSP_LCD_V_RES,
        .monochrome     = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation =
        {
            .swap_xy  = false,
            .mirror_x = false,
            .mirror_y = false,
        },
#if LVGL_VERSION_MAJOR >= 9
        .color_format = LV_COLOR_FORMAT_RGB565,
#endif
        .flags = {
            .buff_dma    = cfg->flags.buff_dma,
            .buff_spiram = cfg->flags.buff_spiram,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = (BSP_LCD_BIGENDIAN ? true : false),
#endif
#if CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
            .sw_rotate = false, /* Avoid tearing is not supported for SW rotation */
#else
            .sw_rotate   = cfg->flags.sw_rotate, /* Only SW rotation is supported for 90° and 270° */
#endif
#if CONFIG_BSP_DISPLAY_LVGL_FULL_REFRESH
            .full_refresh = true,
#elif CONFIG_BSP_DISPLAY_LVGL_DIRECT_MODE
            .direct_mode = true,
#endif
        }
    };

    const lvgl_port_display_dsi_cfg_t dpi_cfg = {.flags = {
#if CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
            .avoid_tearing = true,
#else
            .avoid_tearing = false,
#endif
        }
    };

    return lvgl_port_add_disp_dsi(&disp_cfg, &dpi_cfg);
}

esp_lcd_touch_handle_t _touch_handle = NULL;

esp_lcd_touch_handle_t bsp_display_get_touch_handle(void)
{
    return _touch_handle;
}

static void lvgl_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    if (_touch_handle == NULL) {
        data->state = LV_INDEV_STATE_REL;
        return;
    }

    uint16_t touch_x[1];
    uint16_t touch_y[1];
    uint16_t touch_strength[1];
    uint8_t touch_cnt = 0;

    esp_lcd_touch_read_data(_touch_handle);
    bool touchpad_pressed =
        esp_lcd_touch_get_coordinates(_touch_handle, touch_x, touch_y, touch_strength, &touch_cnt, 1);

    if (!touchpad_pressed) {
        data->state = LV_INDEV_STATE_REL;
    } else {
        data->state   = LV_INDEV_STATE_PR;
        data->point.x = touch_x[0];
        data->point.y = touch_y[0];
    }
}

static lv_indev_t *bsp_display_indev_init(lv_display_t *disp)
{
    esp_lcd_touch_handle_t tp;
    BSP_ERROR_CHECK_RETURN_NULL(bsp_touch_new(NULL, &tp));
    esp_lcd_touch_exit_sleep(tp);  // !!!
    assert(tp);
    _touch_handle = tp;

    disp_indev = lv_indev_create();
    lv_indev_set_type(disp_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(disp_indev, lvgl_read_cb);
    lv_indev_set_display(disp_indev, disp);

    return disp_indev;
}

lv_display_t *bsp_display_start(void)
{
    bsp_display_lcd_config_t cfg = {.lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
                                    .buffer_size   = BSP_LCD_DRAW_BUFF_SIZE,
                                    .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
    .flags         = {
        .buff_dma = true,
        .buff_spiram = false,
        .sw_rotate   = true,
    }
                                   };
    return bsp_display_start_with_config(&cfg);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_lcd_config_t *cfg)
{
    lv_display_t *disp;

    assert(cfg != NULL);
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&cfg->lvgl_port_cfg));

    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_brightness_init());

    BSP_NULL_CHECK(disp = bsp_display_lcd_init(cfg), NULL);
    BSP_NULL_CHECK(disp_indev = bsp_display_indev_init(disp), NULL);
    return disp;
}

lv_indev_t *bsp_display_get_input_dev(void)
{
    return disp_indev;
}

void bsp_display_rotate(lv_display_t *disp, lv_disp_rotation_t rotation)
{
    lv_disp_set_rotation(disp, rotation);
}

bool bsp_display_lock(uint32_t timeout_ms)
{
    return lvgl_port_lock(timeout_ms);
}

void bsp_display_unlock(void)
{
    lvgl_port_unlock();
}
#endif  // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)

//==================================================================================
// usb
//==================================================================================
static void usb_lib_task(void *arg)
{
    while (1) {
        // Start handling system events
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "USB: All devices freed");
            // Continue handling USB events to allow device reconnection
            // The only way this task can be stopped is by calling bsp_usb_host_stop()
        }
    }
}

esp_err_t bsp_usb_host_start(bsp_usb_host_power_mode_t mode, bool limit_500mA)
{
    // Install USB Host driver. Should only be called once in entire application
    ESP_LOGI(TAG, "Installing USB Host");
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags     = ESP_INTR_FLAG_LEVEL1,
    };
    BSP_ERROR_CHECK_RETURN_ERR(usb_host_install(&host_config));

    // Create a task that will handle USB library events
    if (xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 10, &usb_host_task) != pdTRUE) {
        ESP_LOGE(TAG, "Creating USB host lib task failed");
        abort();
    }

    return ESP_OK;
}

esp_err_t bsp_usb_host_stop(void)
{
    usb_host_uninstall();
    if (usb_host_task) {
        vTaskSuspend(usb_host_task);
        vTaskDelete(usb_host_task);
    }
    return ESP_OK;
}

esp_err_t bsp_feature_enable(bsp_feature_t feature, bool enable)
{
    esp_err_t err = ESP_OK;

    /* Initilize I2C */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());

    BSP_ERROR_CHECK_RETURN_ERR(bsp_io_expander_pi4ioe_init());

    switch (feature) {
    case BSP_FEATURE_LCD:
    case BSP_FEATURE_TOUCH:
    case BSP_FEATURE_SD:
    case BSP_FEATURE_SPEAKER:
    case BSP_FEATURE_BATTERY:
    }
    return err;
}
