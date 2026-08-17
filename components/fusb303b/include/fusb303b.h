/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Driver for the onsemi FUSB303B USB Type-C port controller.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FUSB303B_I2C_ADDRESS_LOW     0x21
#define FUSB303B_I2C_ADDRESS_HIGH    0x31
#define FUSB303B_I2C_CLOCK_HZ        400000
#define FUSB303B_DEVICE_TYPE_VALUE   0x03
#define FUSB303B_DEVICE_VERSION      0x01
/** Minimum delay from active-low EN assertion to the first I2C access. */
#define FUSB303B_ENABLE_TO_I2C_DELAY_MS 100

/** Opaque FUSB303B device handle. */
typedef struct fusb303b_device_t *fusb303b_handle_t;

/** I2C configuration used when creating a FUSB303B device. */
typedef struct {
    uint8_t device_address;  /*!< FUSB303B_I2C_ADDRESS_LOW or FUSB303B_I2C_ADDRESS_HIGH. */
    uint32_t scl_speed_hz;   /*!< Bus speed, 1 Hz to FUSB303B_I2C_CLOCK_HZ (400 kHz datasheet limit). */
} fusb303b_config_t;

/** Default FUSB303B I2C configuration for an ADDR/ORIENT-low device. */
#define FUSB303B_CONFIG_DEFAULT()                \
    {                                            \
        .device_address = FUSB303B_I2C_ADDRESS_LOW, \
        .scl_speed_hz = FUSB303B_I2C_CLOCK_HZ,   \
    }

/** USB Type-C power role selected in the Portrole register. */
typedef enum {
    FUSB303B_ROLE_DISABLED = 0,
    FUSB303B_ROLE_SINK,
    FUSB303B_ROLE_SOURCE,
    FUSB303B_ROLE_DRP,
} fusb303b_role_t;

/**
 * Current level for the source advertisement (fusb303b_set_role()) and for
 * the BC_LVL status field decoded by fusb303b_decode_bc_level().
 */
typedef enum {
    FUSB303B_CURRENT_NONE = 0,  /*!< BC_LVL=00: Ra or nothing attached. */
    FUSB303B_CURRENT_DEFAULT,   /*!< USB default current (BC_LVL=01). */
    FUSB303B_CURRENT_1_5_A,     /*!< 1.5 A advertisement (BC_LVL=10). */
    FUSB303B_CURRENT_3_0_A,     /*!< 3.0 A advertisement (BC_LVL=11). */
} fusb303b_current_t;

/**
 * Decode a raw BC_LVL status field (bits 1:2 of the Status register).
 *
 * Only 01/10/11 are valid advertisements; BC_LVL=00 means Ra or nothing
 * attached and decodes to FUSB303B_CURRENT_NONE.
 */
fusb303b_current_t fusb303b_decode_bc_level(uint8_t bc_level);

/** Connection and interrupt snapshot. */
typedef struct {
    uint8_t i2c_address;
    uint8_t device_id;
    uint8_t device_type;
    uint8_t status;
    uint8_t status1;
    uint8_t type;
    uint8_t interrupt;
    uint8_t interrupt1;
    bool attached;
    bool vbus_ok;
    bool vbus_safe_0v;
    bool fault;
    bool remedy_active;
    uint8_t orientation;
    fusb303b_current_t advertised_current; /*!< Partner advertisement; FUSB303B_CURRENT_NONE when nothing attached. */
} fusb303b_status_t;

/** Return true when the identity registers describe a FUSB303B. */
bool fusb303b_is_supported_identity(uint8_t device_id, uint8_t device_type);

/** Create a device on an existing I2C bus and verify its identity. */
esp_err_t fusb303b_create(i2c_master_bus_handle_t bus,
                          const fusb303b_config_t *config,
                          fusb303b_handle_t *ret_handle);

/** Delete the device and remove it from the I2C bus. */
esp_err_t fusb303b_delete(fusb303b_handle_t handle);

/** Enable or disable the autonomous Type-C state machine. */
esp_err_t fusb303b_set_enabled(fusb303b_handle_t handle, bool enabled);

/** Set or clear the global interrupt mask. */
esp_err_t fusb303b_set_global_interrupt_mask(fusb303b_handle_t handle,
        bool masked);

/**
 * @name Mask register (0Eh) bits for fusb303b_set_interrupt_mask()
 *
 * A set bit stops the matching Interrupt (14h) bit from asserting the INT_N
 * pin; the interrupt bit itself still sets (datasheet Table 19, note 12).
 * Bit 7 is reserved: keep it clear.
 * @{
 */
#define FUSB303B_MASK_ORIENT        (1U << 6)  /*!< I_ORIENT: orientation resolved */
#define FUSB303B_MASK_FAULT         (1U << 5)  /*!< I_FAULT: CC voltage outside the Rd range */
#define FUSB303B_MASK_VBUS_CHG      (1U << 4)  /*!< I_VBUS_CHG: VBUS crossed a threshold */
#define FUSB303B_MASK_AUTOSNK       (1U << 3)  /*!< I_AUTOSNK: AUTOSNK mode entered or left */
#define FUSB303B_MASK_BC_LVL        (1U << 2)  /*!< I_BC_LVL: advertised current level changed */
#define FUSB303B_MASK_DETACH        (1U << 1)  /*!< I_DETACH: device or accessory detached */
#define FUSB303B_MASK_ATTACH        (1U << 0)  /*!< I_ATTACH: device or accessory attached */
/** @} */

/**
 * @name Mask1 register (0Fh) bits for fusb303b_set_interrupt_mask()
 *
 * Same masking rule as the Mask register (datasheet Table 20, note 13).
 * Bits 7 and 4 are reserved: keep them clear.
 * @{
 */
#define FUSB303B_MASK1_REM_VBOFF    (1U << 6)  /*!< I_REM_VBOFF: remedy asks for VBUS off and discharge */
#define FUSB303B_MASK1_REM_VBON     (1U << 5)  /*!< I_REM_VBON: remedy asks for VBUS on */
#define FUSB303B_MASK1_REM_FAIL     (1U << 3)  /*!< I_REM_FAIL: remedy attach failed */
#define FUSB303B_MASK1_FRC_FAIL     (1U << 2)  /*!< I_FRC_FAIL: FORCE_SRC/FORCE_SNK failed */
#define FUSB303B_MASK1_FRC_SUCC     (1U << 1)  /*!< I_FRC_SUCC: FORCE_SRC/FORCE_SNK succeeded */
#define FUSB303B_MASK1_REMEDY       (1U << 0)  /*!< I_REMEDY: remedy attach triggered */
/** @} */

/**
 * Set the per-event interrupt masks: the Mask (0Eh) and Mask1 (0Fh)
 * registers. A set bit keeps the matching interrupt off the INT_N pin
 * without affecting the interrupt bit itself.
 */
esp_err_t fusb303b_set_interrupt_mask(fusb303b_handle_t handle, uint8_t mask,
                                      uint8_t mask1);

/** Select the power role and source current advertisement. */
esp_err_t fusb303b_set_role(fusb303b_handle_t handle,
                            fusb303b_role_t role,
                            fusb303b_current_t current);

/** Read the active power role, including the Manual.DISABLED override. */
esp_err_t fusb303b_get_role(fusb303b_handle_t handle,
                            fusb303b_role_t *role);

/**
 * Read connection state and optionally clear all reported interrupts.
 *
 * The status, type, partner-advertised current, and interrupt fields are a
 * live register snapshot; device_id and device_type are the read-only identity
 * values verified and cached by fusb303b_create().
 */
esp_err_t fusb303b_get_status(fusb303b_handle_t handle,
                              fusb303b_status_t *status,
                              bool clear_interrupts);

/**
 * Read and clear both write-one-to-clear interrupt registers.
 *
 * Task context only: the call takes the device lock (portMAX_DELAY) and
 * performs synchronous I2C transfers. A GPIO ISR wired to INT_N must only
 * notify a task, which then calls this function.
 */
esp_err_t fusb303b_get_and_clear_interrupts(fusb303b_handle_t handle,
        uint8_t *interrupt,
        uint8_t *interrupt1);

/**
 * Board-level escape hatch: read one FUSB303B register directly.
 *
 * It exists for board policy the typed API does not cover (for example
 * audio-accessory support or Try.SNK/Try.SRC configuration); regular
 * applications should not need it.
 */
esp_err_t fusb303b_read_register(fusb303b_handle_t handle, uint8_t reg,
                                 uint8_t *value);

/**
 * Board-level escape hatch: write one FUSB303B register directly.
 * Same scope and caution as fusb303b_read_register().
 */
esp_err_t fusb303b_write_register(fusb303b_handle_t handle, uint8_t reg,
                                  uint8_t value);

#ifdef __cplusplus
}
#endif
