#!/usr/bin/env python
#
# SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0

import re
import yaml
from pathlib import Path

BSP_DIR = Path("bsp")
README_PATH = Path("README.md")

START_MARKER = "<!-- START_SUPPORTED_BOARDS -->"
END_MARKER = "<!-- END_SUPPORTED_BOARDS -->"


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


def extract_board_info(board_path):
    board_name = board_path.name
    h_file = next(board_path.glob(f"include/bsp/{board_name}.h"), None)
    yml_file = board_path / "idf_component.yml"

    if not h_file or not yml_file.exists():
        return None

    with open(h_file, "r", encoding="utf-8") as f:
        content = f.read()
        lines = content.splitlines()

        # Find @brief ESP BSP: *
        bsp_name = ""
        match = re.search(r"@brief\s+ESP BSP:\s+(.*)", content)
        if match:
            bsp_name = match.group(1).strip()

        # Find @deprecated *
        deprecated = re.search(r"@deprecated(.*)", content)

        # Extract features
        features = set()
        display_types = []
        touch_types = []
        speaker_codec = []
        mic_codec = []
        for line in lines:
            match = re.match(r"#define\s+BSP_CAPS_([A-Z0-9_]+)\s+1\b", line)
            if match:
                features.add(match.group(1))

        # Check for specific driver/controller names in C file
        c_file = next(board_path.glob(f"{board_name}.c"), None)
        # Some BSPs has main C file in src/ folder
        if not c_file or not c_file.exists():
            c_file = next(board_path.glob(f"src/{board_name}.c"), None)
        if c_file and c_file.exists():
            with open(c_file, "r", encoding="utf-8") as f:
                c_code = f.read()

                # DISPLAY: find all esp_lcd_new_panel_* excluding io_*
                display_matches = re.findall(r"esp_lcd_new_panel_([a-z0-9_]+)\s*\(", c_code)
                display_types = [m for m in display_matches if not m.startswith("io_")]

                # TOUCH: esp_lcd_touch_new_i2c_*
                touch_types = re.findall(r"esp_lcd_touch_new_i2c_([a-z0-9_]+)\s*\(", c_code)

                # AUDIO_SPEAKER
                if "AUDIO_SPEAKER" in features:
                    speaker_code_section = extract_function_body(c_code, 'bsp_audio_codec_speaker_init')
                    if speaker_code_section is not None:
                        matches = re.findall(r"([a-z0-9_]+)_codec_new", speaker_code_section)
                        speaker_codec = [m for m in matches if not m.startswith("audio")]
                    if not speaker_codec:
                        speaker_code_section = extract_function_body(c_code, 'bsp_audio_codec_init')
                        if speaker_code_section is not None:
                            matches = re.findall(r"([a-z0-9_]+)_codec_new", speaker_code_section)
                            speaker_codec = [m for m in matches if not m.startswith("audio")]

                # AUDIO_MIC
                if "AUDIO_MIC" in features:
                    mic_code_section = extract_function_body(c_code, 'bsp_audio_codec_microphone_init')
                    if mic_code_section is not None:
                        matches = re.findall(r"([a-z0-9_]+)_codec_new", mic_code_section)
                        mic_codec = [m for m in matches if not m.startswith("audio")]
                    if not mic_codec:
                        mic_code_section = extract_function_body(c_code, 'bsp_audio_codec_init')
                        if mic_code_section is not None:
                            matches = re.findall(r"([a-z0-9_]+)_codec_new", mic_code_section)
                            mic_codec = [m for m in matches if not m.startswith("audio")]

    # Extract SoC from YML
    with open(yml_file, "r", encoding="utf-8") as f:
        yml_data = yaml.safe_load(f)
        soc = yml_data.get("targets", ["-"])[0]

    # Prepare image path
    image_path = next(board_path.glob(f"doc/{board_name}.webp"), None)
    img_html = f'<img src="{image_path.as_posix()}" width="150">' if image_path.exists() else "-"

    # Emoji map for features
    feature_emojis = {
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

    # Name map for features
    feature_names = {
        "DISPLAY": "LCD Display",
        "TOUCH": "Display Touch",
        "AUDIO": "Audio",
        "AUDIO_MIC": "Audio Microphone",
        "AUDIO_SPEAKER": "Audio Speaker",
        "SDCARD": "uSD Card",
        "IMU": "IMU",
        "USB": "USB-OTG",
        "BUTTONS": "Button",
        "LED": "LED",
        "CAMERA": "Camera",
        "BAT": "Battery",
        "KNOB": "Knob",
    }

    feature_list = ""
    for f in sorted(features):
        f_emoji = feature_emojis.get(f, ":black_circle:")
        f_name = feature_names.get(f, f)
        f_suffix = ""

        if f == "DISPLAY":
            f_suffix = f" ({', '.join(display_types)})" if display_types else ""
        elif f == "TOUCH":
            f_suffix = f" ({', '.join(touch_types)})" if touch_types else ""
        elif f == "AUDIO_SPEAKER":
            f_suffix = f" ({', '.join(speaker_codec)})" if speaker_codec else ""
        elif f == "AUDIO_MIC":
            f_suffix = f" ({', '.join(mic_codec)})" if mic_codec else ""

        feature_list += f"{f_emoji} {f_name} {f_suffix}<br/>"
    feature_list += ""

    return {
        "name": bsp_name,
        "link": f"[{bsp_name}](bsp/{board_name})",
        "soc": soc,
        "features": f"{feature_list}",
        "photo": img_html,
        "deprecated": deprecated,
    }


def generate_table():
    header = "| Board name | SoC | Supported Features | Photo |\n|:----------:|:---:|:-------------------|:-----:|"
    active_rows = []
    deprecated_rows = []

    for board_path in sorted(BSP_DIR.iterdir()):
        if board_path.is_dir():
            info = extract_board_info(board_path)
            if info:
                row = f"| {info['link']} | {info['soc']} | {info['features']} | {info['photo']} |"
                if info["deprecated"]:
                    deprecated_rows.append(row)
                else:
                    active_rows.append(row)

    active_table = "\n".join([header] + active_rows)
    deprecated_table = "\n".join([header] + deprecated_rows) if deprecated_rows else ""

    return active_table, deprecated_table


def update_readme():
    active_table, deprecated_table = generate_table()

    with open(README_PATH, "r", encoding="utf-8") as f:
        content = f.read()

    pattern = re.compile(
        rf"{START_MARKER}.*?{END_MARKER}", re.DOTALL
    )
    new_section = f"{START_MARKER}\n\n{active_table}\n"

    if deprecated_table:
        new_section += "\n### Deprecated Boards\n\n" + deprecated_table + "\n"

    new_section += f"\n{END_MARKER}"

    if pattern.search(content):
        updated_content = pattern.sub(new_section, content)
        if content.strip() != updated_content.strip():
            with open(README_PATH, "w", encoding="utf-8") as f:
                f.write(updated_content.strip())
            print("README.md boards table updated")
        else:
            print("README.md boards table is up to date")
    else:
        print("Markers (boards table) not found in README.md")


if __name__ == "__main__":
    update_readme()
