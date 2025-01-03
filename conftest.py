# SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
import pytest


def pytest_generate_tests(metafunc):
    # Pokud test očekává parametr 'port' nebo 'flash_port', přidáme parametrizaci
    if 'port' in metafunc.fixturenames and 'flash_port' in metafunc.fixturenames:
        metafunc.parametrize(
            'port, flash_port',
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
            ]
        )


# Check marker with params and test ID
def pytest_collection_modifyitems(config, items):
    for item in items:
        marker_option = "[" + config.getoption("-m") + "]"
        if marker_option not in item.nodeid:
            item.add_marker(pytest.mark.skip(reason="Not for selected params"))
