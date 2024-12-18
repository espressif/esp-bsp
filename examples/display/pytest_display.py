# SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: CC0-1.0

import pytest
from pytest_embedded import Dut


@pytest.mark.esp_box_3
@pytest.mark.esp32_p4_function_ev_board
@pytest.mark.parametrize(
    'port,flash_port',
    [
        pytest.param('/dev/boards/esp-box-3', '/dev/boards/esp-box-3', id='esp_box_3'),
        pytest.param('/dev/boards/esp32_p4_function_ev_board', '/dev/boards/esp32_p4_function_ev_board', id='esp32_p4_function_ev_board'),
    ],
    indirect=True,
)
def test_display_example(dut: Dut) -> None:
    dut.expect_exact('example: Display LVGL animation')
    dut.expect_exact('ESP-BOX-3: Setting LCD backlight: 100%')
    dut.expect_exact('main_task: Returned from app_main()')
