# SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
import os
import uuid
import pytest


def pytest_generate_tests(metafunc):
    allowed_ids = set()

    # get markers from test case (e.g. 'esp_box_3')
    for marker in metafunc.definition.iter_markers():
        allowed_ids.add(marker.name)

    all_params = [
        pytest.param('/dev/boards/esp-box-3',
                     '/dev/boards/esp-box-3',
                     'build_esp-box-3',
                     id='esp_box_3'),
        pytest.param('/dev/boards/esp32_c3_lcdkit',
                     '/dev/boards/esp32_c3_lcdkit',
                     'build_esp32_c3_lcdkit',
                     id='esp32_c3_lcdkit'),
        pytest.param('/dev/boards/esp32_p4_box',
                     '/dev/boards/esp32_p4_box',
                     'build_esp32_p4_box',
                     id='esp32_p4_box'),
        pytest.param('/dev/boards/esp32_p4_function_ev_board',
                     '/dev/boards/esp32_p4_function_ev_board',
                     'build_esp32_p4_function_ev_board',
                     id='esp32_p4_function_ev_board'),
        pytest.param('/dev/boards/esp32_s3_eye',
                     '/dev/boards/esp32_s3_eye',
                     'build_esp32_s3_eye',
                     id='esp32_s3_eye'),
        pytest.param('/dev/boards/esp32_s3_lcd_ev_board',
                     '/dev/boards/esp32_s3_lcd_ev_board',
                     'build_esp32_s3_lcd_ev_board',
                     id='esp32_s3_lcd_ev_board'),
        pytest.param('/dev/boards/esp32_s3_lcd_ev_board-2',
                     '/dev/boards/esp32_s3_lcd_ev_board-2',
                     'build_esp32_s3_lcd_ev_board',
                     id='esp32_s3_lcd_ev_board_2'),
        pytest.param('/dev/boards/esp32_s3_usb_otg',
                     '/dev/boards/esp32_s3_usb_otg',
                     'build_esp32_s3_usb_otg',
                     id='esp32_s3_usb_otg'),
        pytest.param('/dev/boards/esp_wrover_kit',
                     '/dev/boards/esp_wrover_kit',
                     'build_esp_wrover_kit',
                     id='esp_wrover_kit'),
        pytest.param('/dev/boards/esp32_azure_iot_kit',
                     '/dev/boards/esp32_azure_iot_kit',
                     'build_esp32_azure_iot_kit',
                     id='esp32_azure_iot_kit'),
        pytest.param('/dev/boards/m5dial',
                     '/dev/boards/m5dial',
                     'build_m5dial',
                     id='m5dial'),
        pytest.param('/dev/boards/m5stack_core',
                     '/dev/boards/m5stack_core',
                     'build_m5stack_core',
                     id='m5stack_core'),
        pytest.param('/dev/boards/m5stack_core_2',
                     '/dev/boards/m5stack_core_2',
                     'build_m5stack_core',
                     id='m5stack_core_2'),
        pytest.param('/dev/boards/m5stack_core_s3',
                     '/dev/boards/m5stack_core_s3',
                     'build_m5stack_core_s3',
                     id='m5stack_core_s3'),
        pytest.param('/dev/boards/m5stack_core_s3_se',
                     '/dev/boards/m5stack_core_s3_se',
                     'build_m5stack_core_s3',
                     id='m5stack_core_s3_se'),
        pytest.param('/dev/boards/m5_atom_s3',
                     '/dev/boards/m5_atom_s3',
                     'build_m5_atom_s3',
                     id='m5_atom_s3'),
        pytest.param('/dev/boards/esp32_s3_devkitc_1_1',
                     '/dev/boards/esp32_s3_devkitc_1_1',
                     'build_esp_bsp_devkit',
                     id='esp_bsp_devkit'),
        pytest.param('/dev/boards/esp32_s2_devkitc_1',
                     '/dev/boards/esp32_s2_devkitc_1',
                     'build_esp_bsp_generic',
                     id='esp_bsp_generic'),
        pytest.param('/dev/boards/esp32_s3_korvo_2',
                     '/dev/boards/esp32_s3_korvo_2',
                     'build_esp32_s3_korvo_2',
                     id='esp32_s3_korvo_2'),
        pytest.param('/dev/boards/esp32_p4_eye',
                     '/dev/boards/esp32_p4_eye',
                     'build_esp32_p4_eye',
                     id='esp32_p4_eye'),
    ]

    # filter by markers
    selected_params = [
        p for p in all_params if p.id in allowed_ids
    ]

    # if not markers, use all
    if not selected_params:
        selected_params = all_params

    if 'port' in metafunc.fixturenames and 'flash_port' in metafunc.fixturenames:
        metafunc.parametrize(
            'port, flash_port, build_dir',
            selected_params
        )


@pytest.fixture
def build_dir(build_dir: str) -> str:
    return f'{build_dir}'


# This fixing using cache when used "-n auto" (parallel)
def pytest_configure(config):
    # If run pytest-xdist (parallel), set unique cache dir
    worker_id = os.getenv("PYTEST_XDIST_WORKER", None)
    if worker_id:
        cache_dir = f"/tmp/pytest-embedded-cache-{uuid.uuid4()}"
        os.environ["PYTEST_EMBEDDED_CACHE_DIR"] = cache_dir
        os.makedirs(cache_dir, exist_ok=True)
        print(f"Using embedded cache dir: {cache_dir}")
