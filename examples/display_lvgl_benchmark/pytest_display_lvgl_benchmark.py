# SPDX-FileCopyrightText: 2023-2026 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: CC0-1.0

import os
import csv
import json
import pytest
from pytest_embedded import Dut
from io import StringIO
from datetime import datetime


@pytest.mark.esp_box_3
@pytest.mark.esp32_p4_function_ev_board
@pytest.mark.esp32_s3_eye
@pytest.mark.esp32_s3_lcd_ev_board
@pytest.mark.esp32_s3_lcd_ev_board_2
@pytest.mark.m5dial
@pytest.mark.m5stack_core_s3
@pytest.mark.m5stack_core_s3_se
def test_example_lvgl_benchmark(dut: Dut, request, build_dir: str) -> None:
    board_name = request.node.callspec.id
    benchmark_result = {
        "time": datetime.now().strftime('%d.%m.%Y %H:%M'),
        "board": board_name
    }

    # Benchmark starts after returning from the main app
    dut.expect_exact('app_main: Display LVGL demo')
    dut.expect_exact('main_task: Returned from app_main()')
    # Get LVGL version
    outdata = dut.expect(r'Benchmark Summary \((.*) \)', timeout=200)
    benchmark_result["LVGL"] = outdata[1].decode()

    outdata = dut.expect(r'I (.*)app_main: LVGL demo ended', return_what_before_match=True)
    # Convert to regular string
    decoded_output = outdata.decode("utf-8")
    # Fix newlines
    cleaned_output = decoded_output.replace('\r\r\n', '\n').strip()
    # Convert test results to a dictionary
    benchmark_result["tests"] = list(csv.DictReader(StringIO(cleaned_output), skipinitialspace=True))

    test_dirname = os.path.dirname(__file__)
    with open(os.path.join(test_dirname, build_dir, f'benchmark_{board_name}.json'), "w+") as f:
        json.dump(benchmark_result, f, indent=4)
