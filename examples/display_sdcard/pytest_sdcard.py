# SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: CC0-1.0

import pytest
from pytest_embedded import Dut


@pytest.mark.esp32_p4_function_ev_board
@pytest.mark.esp32_s3_korvo_2
def test_example_sdcard(dut: Dut) -> None:
    dut.expect_exact('example: Testing of SD card passed!')
