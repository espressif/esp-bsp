# SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: CC0-1.0

import pytest
from pytest_embedded import Dut


@pytest.mark.esp32_p4_eye
@pytest.mark.esp32_s3_eye
@pytest.mark.esp32_s3_korvo_2
@pytest.mark.m5stack_core_s3
@pytest.mark.m5stack_tab5
def test_example_camera_video(dut: Dut) -> None:
    dut.expect_exact('example: Camera example running.')
    dut.expect_exact('main_task: Returned from app_main()')
