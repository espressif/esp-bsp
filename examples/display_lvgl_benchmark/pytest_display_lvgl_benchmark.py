# SPDX-FileCopyrightText: 2023-2026 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: CC0-1.0

import os
import csv
import json
import pytest
from pytest_embedded import Dut
from io import StringIO
from datetime import datetime, timezone

# Mapping from LVGL CSV column names to snake_case field names.
_CSV_FIELD_MAP = {
    'Name':        'scene',
    'Avg. CPU':    'avg_cpu',
    'Avg. FPS':    'avg_fps',
    'Avg. time':   'avg_time_ms',
    'render time': 'render_time_ms',
    'flush time':  'flush_time_ms',
}


def _parse_number(value: str) -> int | float | str:
    stripped = value.strip().rstrip('%')
    try:
        f = float(stripped)
        return int(f) if f.is_integer() else f
    except ValueError:
        return stripped


def _normalise_row(row: dict) -> dict:
    """Rename CSV columns to snake_case and parse numeric values."""
    result = {}
    for csv_key, field_key in _CSV_FIELD_MAP.items():
        if csv_key not in row:
            raise ValueError(f'{csv_key} field is missing from the csv map.')
        raw = row[csv_key]
        result[field_key] = raw if field_key == 'scene' else _parse_number(raw)
    return result


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
    benchmark_result: dict = {
        'time': datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
        'board': board_name,
    }

    # Benchmark starts after returning from the main app
    dut.expect_exact('app_main: Display LVGL demo')
    dut.expect_exact('main_task: Returned from app_main()')
    # Get LVGL version
    outdata = dut.expect(r'Benchmark Summary \((.*) \)', timeout=200)
    benchmark_result['lvgl_version'] = outdata[1].decode()

    outdata = dut.expect(r'I (.*)app_main: LVGL demo ended', return_what_before_match=True)
    decoded_output = outdata.decode('utf-8')
    cleaned_output = decoded_output.replace('\r\r\n', '\n').strip()
    benchmark_result['tests'] = [
        _normalise_row(row)
        for row in csv.DictReader(StringIO(cleaned_output), skipinitialspace=True)
    ]

    test_dirname = os.path.dirname(__file__)
    with open(os.path.join(test_dirname, build_dir, f'benchmark_{board_name}.json'), 'w+') as f:
        json.dump(benchmark_result, f, indent=4)
