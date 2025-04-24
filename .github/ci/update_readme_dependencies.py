#!/usr/bin/env python
#
# SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0

"""
Check if BSP's README.md contains list of dependencies
"""

import os
import sys
import re
import argparse
import subprocess
import urllib.request
from pathlib import Path
from idf_component_tools.manifest import ManifestManager
from py_markdown_table.markdown_table import markdown_table
from typing import Any

DEPENDENCIES_START = '<!-- START_DEPENDENCIES -->'
DEPENDENCIES_END = '<!-- END_DEPENDENCIES -->'
EXAMPLES_START = '<!-- START_EXAMPLES -->'
EXAMPLES_END = '<!-- END_EXAMPLES -->'
BENCHMARK_START = '<!-- START_BENCHMARK -->'
BENCHMARK_END = '<!-- END_BENCHMARK -->'
ESP_REGISTRY_URL = 'https://components.espressif.com/components/'
BENCHMARK_RELEASE_URL = "https://github.com/espressif/esp-bsp/releases/download/benchmark-latest/"
CAPABILITIES_PREFIX = '#define BSP_CAPS_'

EXAMPLES_TABLE_HEADER = """| Example | Description | Try with ESP Launchpad |
| ------- | ----------- | ---------------------- |"""

EXAMPLES_DIR = Path("examples")

"""
This constant dictionary maps 'BSP Capabilities' to 'Component names'
TODO: Make this 'smart'. ATM, we must update this table when e.g. a new IMU component is implemented
"""
capability_dict = {
    'DISPLAY': r'(esp_lcd_[^touch].*|^idf$)',  # All esp_lcd prefixes, but no esp_lcd_touch prefix. If not present, assume that the driver is native to idf
    'TOUCH': r'(esp_lcd_touch.*)',  # esp_lcd_touch prefix
    'BUTTONS': r'(button$)',  # TODO currently only this button driver is supported
    'AUDIO': r'(esp_codec_dev$)',  # TODO currently only this audio driver is supported
    # Currently, Speaker and Microphone drivers are the same as AUDIO driver
    # Do not add any component link, to avoid duplication
    # 'AUDIO_SPEAKER' : r'',
    # 'AUDIO_MIC' : r'',
    'SDCARD': r'(^idf$)',  # SD card driver is native to ESP-IDF
    'IMU': r'(icm42670$|mpu6050$|qma6100p$)',
    'LED': r'(led_indicator$|^idf$)',  # Provided by led_indicator or idf
    'BAT': r'(^idf$)',  # Battery (ADC driver) is native to ESP-IDF
    'CAMERA': r'(esp32-camera$)',  # esp32-camera component
    'KNOB': r'(knob$)',  # knob component
    'SENSOR_TEMPERATURE': r'(hts221$|ds18b20$)',
    'SENSOR_HUMIDITY': r'(hts221$)',
    'SENSOR_PRESSURE': r'(fbm320$)',
    'SENSOR_LIGHT': r'(bh1750$)',
    'SENSOR_MAG': r'(mag3110$)',
    'LVGL_PORT': r'(esp_lvgl_port$)',
}
# Emoji map for capabilities
capability_emojis = {
    "DISPLAY": ":pager:",
    "TOUCH": ":point_up:",
    "AUDIO": ":musical_note:",
    "AUDIO_MIC": ":microphone:",
    "AUDIO_SPEAKER": ":speaker:",
    "SDCARD": ":floppy_disk:",
    "IMU": ":video_game:",
    "USB": ":electric_plug:",
    "BUTTONS": ":radio_button:",
    "LED": ":bulb:",
    "CAMERA": ":camera:",
    "BAT": ":battery:",
    "KNOB": ":white_circle:",
}


def map_capability_to_driver(capability, manifest):
    components = []
    try:
        component_regex = capability_dict[capability]
        for item in manifest.dependencies:
            result = re.findall(component_regex, item.name)
            if result:
                version = item.version_spec if item.version_spec else ""
                component = item.name
                # Add hyperlinks to components (but not to IDF component)
                if component != "idf":
                    component = "[{}]({}{})".format(component, ESP_REGISTRY_URL, component)

                components.append((component, version))

        return components
    except KeyError:
        return components


def extract_function_body(c_code, function_name):
    # Start of function
    func_pos = c_code.find(function_name)
    if func_pos == -1:
        return None

    # First bracket
    open_brace_pos = c_code.find('{', func_pos)
    if open_brace_pos == -1:
        return None

    # Looking all inside brackets
    brace_count = 0
    function_body = ''
    for i in range(open_brace_pos, len(c_code)):
        char = c_code[i]
        if char == '{':
            brace_count += 1
        elif char == '}':
            brace_count -= 1

        function_body += char

        if brace_count == 0:
            # End of function
            return function_body.strip()

    return None


def get_capabilities_table(header_path, main_path, manifest):
    """
    Get markdown formatted table of manifest's capabilities
    """
    with open(header_path, 'r') as f:
        content = f.readlines()
        table_data = []
        for line in content:
            if line.startswith(CAPABILITIES_PREFIX):
                result = re.search(r"^" + CAPABILITIES_PREFIX + "([A-Z,_]*) *(0|1)", line)
                capability = result.group(1)
                available = (result.group(2) == "1")
                component_versions = map_capability_to_driver(capability, manifest) if available else []

                # Check for specific driver/controller names in C file
                additional_info = []
                if main_path and main_path.exists():
                    with open(main_path, "r", encoding="utf-8") as cf:
                        c_code = cf.read()

                        # DISPLAY: find all esp_lcd_new_panel_* excluding io_*
                        if capability == "DISPLAY":
                            display_matches = re.findall(r"esp_lcd_new_panel_([a-z0-9_]+)\s*\(", c_code)
                            additional_info = [m for m in display_matches if not m.startswith("io_")]

                        # TOUCH: esp_lcd_touch_new_i2c_*
                        if capability == "TOUCH":
                            additional_info = re.findall(r"esp_lcd_touch_new_i2c_([a-z0-9_]+)\s*\(", c_code)

                        # AUDIO_SPEAKER
                        if capability == "AUDIO_SPEAKER":
                            speaker_code_section = extract_function_body(c_code, 'bsp_audio_codec_speaker_init')
                            if speaker_code_section is not None:
                                matches = re.findall(r"([a-z0-9_]+)_codec_new", speaker_code_section)
                                additional_info = [m for m in matches if not m.startswith("audio")]
                            if not additional_info:
                                speaker_code_section = extract_function_body(c_code, 'bsp_audio_codec_init')
                                if speaker_code_section is not None:
                                    matches = re.findall(r"([a-z0-9_]+)_codec_new", speaker_code_section)
                                    additional_info = [m for m in matches if not m.startswith("audio")]

                        # AUDIO_MIC
                        if capability == "AUDIO_MIC":
                            mic_code_section = extract_function_body(c_code, 'bsp_audio_codec_microphone_init')
                            if mic_code_section is not None:
                                matches = re.findall(r"([a-z0-9_]+)_codec_new", mic_code_section)
                                additional_info = [m for m in matches if not m.startswith("audio")]
                            if not additional_info:
                                mic_code_section = extract_function_body(c_code, 'bsp_audio_codec_init')
                                if mic_code_section is not None:
                                    matches = re.findall(r"([a-z0-9_]+)_codec_new", mic_code_section)
                                    additional_info = [m for m in matches if not m.startswith("audio")]

                components = []
                versions = []
                if component_versions is not None:
                    components = [c for c, _ in component_versions]
                    versions = [v for _, v in component_versions]
                table_data.append(
                    {
                        "Available": ":heavy_check_mark:" if available else ":x:",
                        "Capability": capability_emojis.get(capability, ":black_circle:") + " " + capability,
                        "Controller/Codec": ", ".join(additional_info),
                        "Component": "<br/>".join(components),
                        "Version": "<br/>".join(versions),
                    })
                # Special handling of 'DISPLAY' capability, as it often includes esp_lvgl_port component
                if capability == 'DISPLAY' and available:
                    component_versions = map_capability_to_driver('LVGL_PORT', manifest)
                    components = []
                    versions = []
                    if component_versions is not None:
                        components = [c for c, _ in component_versions]
                        versions = [v for _, v in component_versions]
                    table_data.append(
                        {
                            "Available": ":heavy_check_mark:",
                            "Capability": ":black_circle: " + 'LVGL_PORT',
                            "Controller/Codec":  "",
                            "Component": "<br/>".join(components),
                            "Version": "<br/>".join(versions),
                        })
        return table_data


def extract_metadata_from_c_file(c_file_path):
    brief = ""
    details = ""
    example = ""
    try:
        with open(c_file_path, "r", encoding="utf-8") as f:
            content = f.read()
            brief_match = re.search(r"@brief\s+(.*)", content)
            if brief_match:
                brief = brief_match.group(1).strip()
            details_match = re.search(r"@details\s+(.*)", content, re.DOTALL)
            if details_match:
                details = details_match.group(1).strip().split("\n")[0]
            example_match = re.search(r"@example\s+(.*)", content)
            if example_match:
                example = example_match.group(1).strip()
    except Exception as e:
        print(f"Failed to parse {c_file_path}: {e}")
    return brief or "", details or "", example or ""


def extract_supported_boards(example_path):
    boards = []
    for file in sorted(example_path.glob("sdkconfig.bsp.*")):
        board_name = file.name.split("sdkconfig.bsp.")[-1]
        boards.append(board_name)
    return boards


def is_tracked_by_git(path: Path) -> bool:
    try:
        result = subprocess.run(["git", "ls-files", "--error-unmatch", str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return result.returncode == 0
    except Exception:
        return False


def get_examples_table(bsp_name):
    rows = []
    for example_dir in sorted(EXAMPLES_DIR.iterdir()):
        if not example_dir.is_dir():
            continue
        if not is_tracked_by_git(example_dir):      # Only files in git are real examples
            print(f"Ignoring {example_dir} - not tracked by Git")
            continue

        c_file = next(example_dir.glob("main/main.c"), None)
        if not c_file:
            c_files = list(example_dir.rglob("*.c"))
            if c_files:
                c_file = c_files[0]
            else:
                print(f"No .c file found in {example_dir}")
                continue

        brief, details, example_ref = extract_metadata_from_c_file(c_file)
        boards = extract_supported_boards(example_dir)
        if bsp_name not in boards:
            continue

        example_folder = example_dir.name
        flash_link = f'[Flash Example]({example_ref})' if example_ref != "" else "-"
        example_name = brief.split("BSP ")[-1]
        row = f"| [{example_name}](https://github.com/espressif/esp-bsp/tree/master/examples/{example_folder}) | {details} | {flash_link} |"
        rows.append(row)

    active_table = "\n".join([EXAMPLES_TABLE_HEADER] + rows)
    return active_table


def index_regexp(yourlist, yourstring):
    for i, line in enumerate(yourlist):
        if yourstring in line:
            return i
    return -1


def check_bsp_readme(bsp_path) -> Any:
    # Get list of capabilities and dependencies of this BSP
    bsp_name = bsp_path.stem if not bsp_path.stem.endswith('_noglib') else bsp_path.stem[:-7]
    header_path = bsp_path / "include" / "bsp" / (bsp_name + '.h')
    main_path = bsp_path / (bsp_name + '.c')
    readme_path = bsp_path / 'README.md'
    manager = ManifestManager(bsp_path, 'bsp')

    # Some BSPs has main C file in src/ folder
    if not main_path.exists():
        main_path = bsp_path / "src" / (bsp_name + '.c')

    table_capabilities_data = get_capabilities_table(header_path, main_path, manager.load())
    table_examples_md = get_examples_table(bsp_name)

    # Convert table to markdown format
    try:
        table_capabilities_md = markdown_table(table_capabilities_data).set_params(row_sep='markdown', quote=False).get_markdown()
    except Exception:
        return

    with open(readme_path, "r", encoding="utf-8") as f:
        content = f.read()

    # UPDATE DEPENDENCIES TABLE
    pattern = re.compile(
        rf"{DEPENDENCIES_START}.*?{DEPENDENCIES_END}", re.DOTALL
    )
    new_section = f"{DEPENDENCIES_START}\n\n{table_capabilities_md}\n"
    new_section += f"\n{DEPENDENCIES_END}"

    if pattern.search(content):
        updated_content = pattern.sub(new_section, content)
        if content.strip() != updated_content.strip():
            with open(readme_path, "w", encoding="utf-8") as f:
                f.write(updated_content.strip())
            print(f"{bsp_name} README.md capabilities updated")
        else:
            print(f"{bsp_name} README.md capabilities is up to date")
    else:
        print(f"Markers capabilities not in {bsp_name} README.md")

    # UPDATE EXAMPLES TABLE
    pattern = re.compile(
        rf"{EXAMPLES_START}.*?{EXAMPLES_END}", re.DOTALL
    )
    new_section = f"{EXAMPLES_START}\n\n{table_examples_md}\n"
    new_section += f"\n{EXAMPLES_END}"

    if pattern.search(content):
        updated_content = pattern.sub(new_section, content)
        if content.strip() != updated_content.strip():
            with open(readme_path, "w", encoding="utf-8") as f:
                f.write(updated_content.strip())
            print(f"{bsp_name} README.md examples updated")
        else:
            print(f"{bsp_name} README.md examples is up to date")
    else:
        print(f"Markers examples not in {bsp_name} README.md")

    if os.getenv("GITHUB_ACTIONS") != "true":
        # UPDATE BENCHMARK TABLE
        try:
            benchmark_path = BENCHMARK_RELEASE_URL + "benchmark_" + bsp_name.replace('-', '_') + ".md"
            with urllib.request.urlopen(benchmark_path) as response:
                benchmark_content = response.read().decode("utf-8")
            pattern = re.compile(
                rf"{BENCHMARK_START}.*?{BENCHMARK_END}", re.DOTALL
            )
            new_section = f"{BENCHMARK_START}\n\n{benchmark_content}\n"
            new_section += f"\n{BENCHMARK_END}"

            if pattern.search(content):
                updated_content = pattern.sub(new_section, content)
                if content.strip() != updated_content.strip():
                    with open(readme_path, "w", encoding="utf-8") as f:
                        f.write(updated_content.strip())
                    print(f"{bsp_name} README.md benchmark updated")
                else:
                    print(f"{bsp_name} README.md benchmark is up to date")
            else:
                print(f"Markers benchmark not in {bsp_name} README.md")
        except urllib.error.HTTPError:
            print(f"Benchmarks for {bsp_name} does not exist")
        except Exception as e:
            print(f"[WARNING] Failed to update benchmarks: {e}")


def check_all_bsps(filenames):
    ret = 0
    for f in filenames:
        bsp_path = Path(f)
        if not bsp_path.exists():
            print("[Error] Argument {} does not point to existing BSP".format(f))
            raise Exception

        if not bsp_path.is_dir():
            if bsp_path.suffix == '.h':
                # In case a BSP header file is passed (from pre-commit)
                bsp_path = bsp_path.parents[2]
            else:
                # In case idf_component.yml or README.md is passed here (e.g. from pre-commit)
                bsp_path = bsp_path.parent

        try:
            check_bsp_readme(bsp_path)
        except Exception:
            ret += 1
    return ret


if __name__ == '__main__':
    os.environ["IDF_VERSION"] = "5.3.0"  # Let's assume IDF v5.3.0 for optional dependencies
    parser = argparse.ArgumentParser()
    parser.add_argument('bsps', nargs='*', help='BSPs to check. Can be BSP path, BSP README.md, BSP component.yml or BSP header.h file')
    args = parser.parse_args()
    sys.exit(check_all_bsps(args.bsps))
