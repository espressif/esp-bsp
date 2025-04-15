# SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: CC0-1.0

import pytest
from pytest_embedded import Dut


@pytest.mark.esp_bsp_devkit
@pytest.mark.esp_bsp_generic
def test_example_generic_button_led(dut: Dut) -> None:
    dut.expect_exact('led_indicator: Indicator create successfully.')
    dut.expect_exact('main_task: Returned from app_main()')
