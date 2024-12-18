# SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: CC0-1.0

import pytest
from pytest_embedded import Dut


@pytest.mark.parametrize(
    'port,flash_port',
    [
        pytest.param('/dev/boards/esp-box-3', '/dev/boards/esp-box-3', id='esp_box_3'),
        pytest.param('/dev/boards/esp32_c3_lcdkit', '/dev/boards/esp32_c3_lcdkit', id='esp32_c3_lcdkit'),
        pytest.param('/dev/boards/esp32_p4_box', '/dev/boards/esp32_p4_box', id='esp32_p4_box'),
        pytest.param('/dev/boards/esp32_p4_function_ev_board', '/dev/boards/esp32_p4_function_ev_board', id='esp32_p4_function_ev_board'),
        pytest.param('/dev/boards/esp32_s3_eye', '/dev/boards/esp32_s3_eye', id='esp32_s3_eye'),
        pytest.param('/dev/boards/esp32_s3_lcd_ev_board', '/dev/boards/esp32_s3_lcd_ev_board', id='esp32_s3_lcd_ev_board'),
        pytest.param('/dev/boards/esp32_s3_usb_otg', '/dev/boards/esp32_s3_usb_otg', id='esp32_s3_usb_otg'),
        pytest.param('/dev/boards/esp_wrover_kit', '/dev/boards/esp_wrover_kit', id='esp_wrover_kit'),
        pytest.param('/dev/boards/m5dial', '/dev/boards/m5dial', id='m5dial'),
        pytest.param('/dev/boards/m5stack_core', '/dev/boards/m5stack_core', id='m5stack_core'),
        pytest.param('/dev/boards/m5stack_core_2', '/dev/boards/m5stack_core_2', id='m5stack_core_2'),
        pytest.param('/dev/boards/m5stack_core_s3', '/dev/boards/m5stack_core_s3', id='m5stack_core_s3'),
        pytest.param('/dev/boards/m5stack_core_s3_se', '/dev/boards/m5stack_core_s3_se', id='m5stack_core_s3_se'),
    ],
    indirect=True,
)
@pytest.mark.esp_box_3
@pytest.mark.esp32_c3_lcdkit
@pytest.mark.esp32_p4_box
@pytest.mark.esp32_p4_function_ev_board
@pytest.mark.esp32_s3_eye
@pytest.mark.esp32_s3_lcd_ev_board
@pytest.mark.esp32_s3_usb_otg
@pytest.mark.esp_wrover_kit
@pytest.mark.m5dial
@pytest.mark.m5stack_core
@pytest.mark.m5stack_core_2
@pytest.mark.m5stack_core_s3
@pytest.mark.m5stack_core_s3_se
def test_display_example(dut: Dut) -> None:
    dut.expect_exact('example: Display LVGL demo')
    dut.expect_exact('main_task: Returned from app_main()')
