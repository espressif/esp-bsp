# SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: CC0-1.0

import pytest
from pytest_embedded import Dut


@pytest.mark.esp_box_3
@pytest.fixture
def build_dir(board: str) -> str:
    return f'build_{board}'


@pytest.mark.esp_box_3
@pytest.mark.parametrize(
    'board,port',
    [
        pytest.param('esp-box-3', '/dev/boards/esp-box-3', id='esp-box-3'),
    ],
    indirect=True,
)
def test_display_example(dut: Dut) -> None:
    dut.expect_exact('Add LCD screen')
    dut.expect_exact('Display LVGL animation')
