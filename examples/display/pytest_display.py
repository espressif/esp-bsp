# SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: CC0-1.0

import pytest
from pytest_embedded import Dut


@pytest.mark.esp_box_3
@pytest.mark.esp32_p4_function_ev_board
@pytest.mark.esp32_c3_lcdkit
@pytest.mark.esp32_s3_eye
@pytest.mark.esp32_s3_lcd_ev_board
@pytest.mark.esp32_s3_lcd_ev_board_2
@pytest.mark.esp32_s3_usb_otg
@pytest.mark.esp_wrover_kit
@pytest.mark.esp32_s3_korvo_2
@pytest.mark.m5dial
@pytest.mark.m5stack_core
@pytest.mark.m5stack_core_2
@pytest.mark.m5stack_core_s3
@pytest.mark.m5stack_core_s3_se
@pytest.mark.m5_atom_s3
def test_example_display(dut: Dut) -> None:
    dut.expect_exact('example: Display LVGL animation')
    dut.expect_exact('main_task: Returned from app_main()')
