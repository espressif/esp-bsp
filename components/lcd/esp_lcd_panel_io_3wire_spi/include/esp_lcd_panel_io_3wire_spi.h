#pragma once

#include "esp_lcd_types.h"
#include "esp_io_expander.h"

#ifdef __cplusplus
extern "C" {
#endif

// Maximum SPI clock speed
#define PANEL_IO_3WIRE_SPI_CLK_MAX      (500 * 1000UL)

/**
 * @brief Panel IO type, use GPIO or IO expander
 */
typedef enum {
    IO_TYPE_GPIO = 0,
    IO_TYPE_EXPANDER,
} panel_io_type_t;

/**
 * @brief SPI line configuration structure
 */
typedef struct {
    panel_io_type_t cs_io_type;                     /*!< IO type of CS line */
    union {
        int cs_gpio_num;                            /*!< GPIO used for CS line */
        esp_io_expander_pin_num_t cs_expander_pin;  /*!< IO expander pin used for CS line */
    };
    panel_io_type_t scl_io_type;                    /*!< IO type of CS line */
    union {
        int scl_gpio_num;                           /*!< GPIO used for SCL line */
        esp_io_expander_pin_num_t scl_expander_pin; /*!< IO expander pin used for SCL line */
    };
    panel_io_type_t sda_io_type;                    /*!< IO type of CS line */
    union {
        int sda_gpio_num;                           /*!< GPIO used for SDA line */
        esp_io_expander_pin_num_t sda_expander_pin; /*!< IO expander pin used for SDA line */
    };
} spi_line_config_t;

/**
 * @brief Panel IO configuration structure
 */
typedef struct {
    spi_line_config_t line_config;              /*!< SPI line configuration */
    esp_io_expander_handle_t io_expander;       /*!< IO expander handle, set to NULL if not used */
    uint32_t expect_clk_speed;                  /*!< Expected SPI clock speed, in Hz (1 ~ 500000)
                                                 *   If this value is 0, it will be set to `PANEL_IO_3WIRE_SPI_CLK_MAX` by default
                                                 *   The actual frequency may be very different due to the limitation of the software delay */
    int spi_mode;                               /*!< Traditional SPI mode (0 ~ 3) */
    int lcd_cmd_bytes;                          /*!< Bytes of LCD command (1 ~ 4) */
    int lcd_param_bytes;                        /*!< Bytes of LCD parameter (1 ~ 4) */
    struct {
        unsigned int dc_zero_on_data: 1;        /*!< If this flag is enabled, DC bit = 0 means transfer data, DC bit = 1 means transfer command */
        unsigned int lsb_first: 1;              /*!< If this flag is enabled, transmit LSB bit first */
        unsigned int cs_high_active: 1;         /*!< If this flag is enabled, CS line is high active */
        unsigned int del_keep_cs_inactive: 1;   /*!< If this flag is enabled, keep CS line inactive even if panel_io is deleted */
    } flags;
} esp_lcd_panel_io_3wire_spi_config_t;

/**
 * @brief Create a new panel IO instance for 3-wire SPI interface (silumate by software)
 *
 * @note  This function uses GPIO or IO expander to simulate SPI interface by software and just supports to write data.
 *        It is only suitable for some applications with low speed SPI interface. (Such as initializing RGB panel)
 *
 * @param[in]  io_config Panel IO configuration
 * @param[out] ret_io    Pointer to return the created panel IO instance
 * @return
 *      - ESP_OK:              Success
 *      - ESP_ERR_INVALID_ARG: Invalid argument
 *      - ESP_ERR_NO_MEM:      Failed to allocate memory for panel IO instance
 *      - Others:              Fail
 */
esp_err_t esp_lcd_new_panel_io_3wire_spi(const esp_lcd_panel_io_3wire_spi_config_t *io_config, esp_lcd_panel_io_handle_t *ret_io);

#ifdef __cplusplus
}
#endif
