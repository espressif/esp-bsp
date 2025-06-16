#!/usr/bin/env python
#
# SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0

import re
import subprocess
from pathlib import Path

EXAMPLES_DIR = Path("examples")
README_PATH = Path("README.md")

START_MARKER = "<!-- EXAMPLES_TABLE_START -->"
END_MARKER = "<!-- EXAMPLES_TABLE_END -->"

TABLE_HEADER = """| Example | Description | Supported Boards | Try with ESP Launchpad |
| ------- | ----------- | ---------------- | ---------------------- |"""


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
        boards.append(f"[{board_name}](bsp/{board_name})")
    return boards


def is_tracked_by_git(path: Path) -> bool:
    try:
        result = subprocess.run(["git", "ls-files", "--error-unmatch", str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return result.returncode == 0
    except Exception:
        return False


def collect_example_data():
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
        if len(boards) > 1:
            boards_str = f"<details><summary>{len(boards)} boards</summary>" + "<br/>".join(boards) + "</details>"
        else:
            boards_str = f"<details><summary>{len(boards)} board</summary>" + "<br/>".join(boards) + "</details>"

        example_folder = example_dir.name
        flash_link = f'[Flash Example]({example_ref})' if example_ref != "" else "-"
        example_name = brief.split("BSP ")[-1]
        row = f"| [{example_name}](examples/{example_folder}) | {details} | {boards_str} | {flash_link} |"
        rows.append(row)

    active_table = "\n".join([TABLE_HEADER] + rows)
    return active_table


def update_readme_with_table():
    table = collect_example_data()

    with open(README_PATH, "r", encoding="utf-8") as f:
        content = f.read()

    pattern = re.compile(
        rf"{START_MARKER}.*?{END_MARKER}", re.DOTALL
    )
    new_section = f"{START_MARKER}\n\n{table}\n"

    new_section += f"\n{END_MARKER}"

    if pattern.search(content):
        updated_content = pattern.sub(new_section, content)
        if content.strip() != updated_content.strip():
            with open(README_PATH, "w", encoding="utf-8") as f:
                f.write(updated_content.strip())
            print("README.md examples table updated")
        else:
            print("README.md examples table is up to date")
    else:
        print("Markers (examples table) not found in README.md")


if __name__ == "__main__":
    update_readme_with_table()
