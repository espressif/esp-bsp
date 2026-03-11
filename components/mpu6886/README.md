# MPU-6886 Driver Component

C driver for MPU-6886 6-axis gyroscope and accelerometer based on I2C communication.

## Features

### Sensor Data
- 3-axis accelerometer data (raw 16-bit or floating-point g values)
- 3-axis gyroscope data (raw 16-bit or floating-point dps values)
- Internal temperature sensor
- Complimentary filter for roll and pitch angle estimation

### Configuration
- Accelerometer full-scale range: ±2g, ±4g, ±8g, ±16g
- Gyroscope full-scale range: ±250, ±500, ±1000, ±2000 dps
- Clock source selection (internal 20 MHz, auto-select PLL, stop)
- Sample rate divider (0–255)
- Gyroscope DLPF bandwidth (8 settings, 5–3281 Hz)
- Accelerometer DLPF bandwidth (8 settings, 5.1–420 Hz)
- FSYNC pin configuration (disabled or routed to any sensor output LSB)

### Power Management
- Sleep / wake control
- Device reset (all registers restored to defaults)
- Individual axis enable/disable (accelerometer and gyroscope independently)
- Temperature sensor disable
- Gyroscope standby mode (drive + PLL on, sense paths off)
- Accelerometer low-power duty-cycle mode
- Gyroscope low-power mode with configurable averaging (1–128 samples)
- Accelerometer averaging filter for low-power mode (4–32 samples)

### Interrupts
- Configurable INT pin: active level, push-pull/open-drain, latch mode, clear behavior
- Interrupt sources: data ready, FIFO overflow, gyroscope drive ready, wake-on-motion (per-axis)
- GPIO ISR registration

### Wake-on-Motion (WoM)
- Per-axis motion threshold (0–255 mg, scaled by accelerometer FSR)
- OR / AND mode for multi-axis threshold comparison
- Hardware-accelerated motion detection with programmable intelligence

### FIFO
- Enable/disable FIFO buffering for gyroscope and/or accelerometer data
- FIFO reset
- Read FIFO byte count and bulk-read FIFO data
- Configurable watermark threshold (0–1023 bytes) with watermark interrupt status

### Calibration & Self-Test
- Gyroscope user offset registers (16-bit, 2's complement)
- Accelerometer offset registers (15-bit, 0.98 mg steps)
- Factory-stored accelerometer and gyroscope self-test values
- Signal path reset and signal condition reset
- OUTPUT_LIMIT configuration (required after every power-up per datasheet)

## Important Notes

- The MPU-6886 I2C address depends on the level of its AD0/SA0 pin: **0x68** when low, **0x69** when high (default: 0x68).
- The INT pin must be connected to a GPIO on the ESP32 to receive interrupts.
- After every power-up, call `mpu6886_set_output_limit()` to avoid limiting sensor output (per datasheet requirement).
- For full gyroscope performance, set the clock source to `MPU6886_CLK_AUTO` using `mpu6886_set_clock_source()`.
- The FIFO watermark interrupt (register 0x39) is **not** cleared by reading INT_STATUS — it is cleared by reading FIFO data via `mpu6886_read_fifo()`.

## Limitations

- Only I2C communication is supported (SPI is not implemented).

## Get Started

This driver can be used as a package from [Espressif's IDF Component Registry](https://components.espressif.com). To include this driver in your project, run the following idf.py from the project's root directory:

```
    idf.py add-dependency "espressif/mpu6886^1.0.0"
```

Another option is to manually create a `idf_component.yml` file. You can find more about using .yml files for components from [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## Example Usage

### Basic Reading (ESP-IDF >= 5.2, new I2C master API)

```c
#include "mpu6886.h"
#include "driver/i2c_master.h"

// Initialize I2C bus
i2c_master_bus_config_t bus_cfg = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = GPIO_NUM_21,
    .scl_io_num = GPIO_NUM_22,
    .clk_source = I2C_CLK_SRC_DEFAULT,
};
i2c_master_bus_handle_t bus;
i2c_new_master_bus(&bus_cfg, &bus);

// Create sensor handle
mpu6886_handle_t sensor = mpu6886_create(bus, MPU6886_I2C_ADDRESS);

// Wake up the sensor (device starts in sleep mode)
mpu6886_wake_up(sensor);

// Required after every power-up per datasheet
mpu6886_set_output_limit(sensor);

// Use auto clock for best gyroscope performance
mpu6886_set_clock_source(sensor, MPU6886_CLK_AUTO);

// Configure full scale ranges
mpu6886_config(sensor, MPU6886_ACCE_FS_8G, MPU6886_GYRO_FS_2000DPS);

// Read accelerometer and gyroscope data
mpu6886_acce_value_t acce;
mpu6886_gyro_value_t gyro;
mpu6886_get_acce(sensor, &acce);
mpu6886_get_gyro(sensor, &gyro);

// Read temperature
mpu6886_temp_value_t temp;
mpu6886_get_temp(sensor, &temp);

// Cleanup
mpu6886_delete(sensor);
```

### Basic Reading (ESP-IDF < 5.2, legacy I2C API)

```c
#include "mpu6886.h"

// Create sensor handle (I2C bus must be initialized beforehand)
mpu6886_handle_t sensor = mpu6886_create(I2C_NUM_0, MPU6886_I2C_ADDRESS);

// Wake up the sensor (device starts in sleep mode)
mpu6886_wake_up(sensor);

// Required after every power-up per datasheet
mpu6886_set_output_limit(sensor);

// Use auto clock for best gyroscope performance
mpu6886_set_clock_source(sensor, MPU6886_CLK_AUTO);

// Configure full scale ranges
mpu6886_config(sensor, MPU6886_ACCE_FS_8G, MPU6886_GYRO_FS_2000DPS);

// Read accelerometer and gyroscope data
mpu6886_acce_value_t acce;
mpu6886_gyro_value_t gyro;
mpu6886_get_acce(sensor, &acce);
mpu6886_get_gyro(sensor, &gyro);

// Read temperature
mpu6886_temp_value_t temp;
mpu6886_get_temp(sensor, &temp);

// Cleanup
mpu6886_delete(sensor);
```

### Wake-on-Motion Detection

```c
// Configure WoM with 100mg threshold on all axes (OR mode)
mpu6886_config_wom(sensor, 100, 100, 100, MPU6886_WOM_OR);

// Enable WoM interrupts
mpu6886_enable_interrupts(sensor, MPU6886_WOM_X_INT_BIT | MPU6886_WOM_Y_INT_BIT | MPU6886_WOM_Z_INT_BIT);
```

### FIFO Buffering

```c
// Enable FIFO for both accel and gyro
mpu6886_enable_fifo(sensor, true, true, true);

// Set watermark at 512 bytes
mpu6886_set_fifo_watermark(sensor, 512);

// Read FIFO data
uint16_t count;
mpu6886_get_fifo_count(sensor, &count);
uint8_t buf[512];
mpu6886_read_fifo(sensor, buf, count > 512 ? 512 : (uint8_t)count);
```

### Low-Power Mode

```c
// Configure accelerometer low-power cycle with 16-sample averaging
mpu6886_set_acce_averaging(sensor, MPU6886_ACCE_AVG_16);
mpu6886_set_acce_low_power_cycle(sensor, true);

// Or gyroscope low-power mode with 32-sample averaging
mpu6886_set_gyro_low_power(sensor, true, MPU6886_GYRO_AVG_32);
```

## API Reference

| Function | Description |
|---|---|
| `mpu6886_create` | Create sensor handle |
| `mpu6886_delete` | Delete sensor handle |
| `mpu6886_get_deviceid` | Read WHO_AM_I register |
| `mpu6886_reset` | Reset device to defaults |
| `mpu6886_wake_up` / `mpu6886_sleep` | Power mode control |
| `mpu6886_set_clock_source` | Set clock source (internal/auto/stop) |
| `mpu6886_config` | Set accel + gyro full-scale ranges |
| `mpu6886_get_acce_sensitivity` | Get accel LSB/g value |
| `mpu6886_get_gyro_sensitivity` | Get gyro LSB/dps value |
| `mpu6886_set_sample_rate_divider` | Set sample rate divider |
| `mpu6886_set_gyro_dlpf` | Set gyro/temp DLPF bandwidth |
| `mpu6886_set_acce_dlpf` | Set accel DLPF bandwidth |
| `mpu6886_set_fsync` | Configure FSYNC pin |
| `mpu6886_set_gyro_low_power` | Gyro low-power mode + averaging |
| `mpu6886_set_acce_averaging` | Accel low-power averaging |
| `mpu6886_set_acce_low_power_cycle` | Accel duty-cycle mode |
| `mpu6886_config_wom` | Wake-on-motion thresholds + mode |
| `mpu6886_set_axes_disable` | Enable/disable individual axes |
| `mpu6886_set_temp_disable` | Enable/disable temperature sensor |
| `mpu6886_set_gyro_standby` | Gyro standby mode |
| `mpu6886_enable_fifo` | Enable FIFO for accel/gyro |
| `mpu6886_reset_fifo` | Reset FIFO buffer |
| `mpu6886_get_fifo_count` | Get FIFO byte count |
| `mpu6886_read_fifo` | Read FIFO data |
| `mpu6886_set_fifo_watermark` | Set FIFO watermark threshold |
| `mpu6886_get_fifo_wm_status` | Read FIFO watermark status |
| `mpu6886_reset_signal_path` | Reset signal paths |
| `mpu6886_reset_signal_cond` | Reset signal paths + registers |
| `mpu6886_set_output_limit` | Set OUTPUT_LIMIT bit (post power-up) |
| `mpu6886_set_gyro_offset` | Set gyro user offsets |
| `mpu6886_set_acce_offset` | Set accel offsets |
| `mpu6886_get_acce_self_test` | Read accel self-test values |
| `mpu6886_get_gyro_self_test` | Read gyro self-test values |
| `mpu6886_config_interrupts` | Configure INT pin |
| `mpu6886_register_isr` | Register interrupt handler |
| `mpu6886_enable_interrupts` | Enable interrupt sources |
| `mpu6886_disable_interrupts` | Disable interrupt sources |
| `mpu6886_get_interrupt_status` | Read interrupt status |
| `mpu6886_get_raw_acce` / `mpu6886_get_acce` | Read accelerometer data |
| `mpu6886_get_raw_gyro` / `mpu6886_get_gyro` | Read gyroscope data |
| `mpu6886_get_temp` | Read temperature |
| `mpu6886_complimentary_filter` | Compute roll/pitch angles |

## See Also
* [MPU-6886 datasheet](https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/core/MPU-6886-000193%2Bv1.1_GHIC_en.pdf)
