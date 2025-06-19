# SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: CC0-1.0

import pytest
from pytest_embedded import Dut


@pytest.mark.esp32_azure_iot_kit
def test_example_display_sensors(dut: Dut) -> None:
    dut.expect_exact('main_task: Returned from app_main()')
    dut.expect(r'example: temperature: (\d+[.]\d+), humidity: (\d+[.]\d+), luminance: (\d+[.]\d+)')
