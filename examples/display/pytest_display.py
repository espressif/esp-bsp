# SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Unlicense OR CC0-1.0
from __future__ import unicode_literals

import pytest
from pytest_embedded import Dut


@pytest.mark.generic
def test_display_example(dut: Dut) -> None:
    dut.expect_exact('Add LCD screen')
    dut.expect_exact('Display LVGL animation')
