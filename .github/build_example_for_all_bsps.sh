#!/usr/bin/env bash

set -e
set -u

# BSP dictionary. README strings -> component names
# Don't forget to update this variable and examples/bsp_ext.py when adding new BSP
declare -A bsp_dictionary
bsp_dictionary['Wrover kit']='esp_wrover_kit'
bsp_dictionary['ESP-BOX']='esp-box'
bsp_dictionary['Azure IoT kit']='esp32_azure_iot_kit'
bsp_dictionary['Kaluga kit']='esp32_s2_kaluga_kit'
bsp_dictionary['ESP32-S3-EYE']='esp32_s3_eye'
bsp_dictionary['ESP32-S3-USB-OTG']='esp32_s3_usb_otg'
bsp_dictionary['ESP32-S3-LCD-EV-BOARD']='esp32_s3_lcd_ev_board'
bsp_dictionary['ESP32-S3-Korvo-2']='esp32_s3_korvo_2'

# Read supported BSPs line from README
bsp_line=$(head -n 1 README.md)

# Find supported BSPs
declare -a supported_bsps=()
echo "Detected supported BSPs for $1:"
for key in "${!bsp_dictionary[@]}"; do
  if [[ "$bsp_line" == *"$key"* ]]; then
    supported_bsps+=("${bsp_dictionary[$key]}")
    echo "${bsp_dictionary[$key]}"
  fi
done

# Build example for all supported BSPs
for bsp in ${supported_bsps[@]}; do
  echo "Building $1 for $bsp"
  idf.py set-bsp $bsp
  idf.py -B build_$bsp build
done
