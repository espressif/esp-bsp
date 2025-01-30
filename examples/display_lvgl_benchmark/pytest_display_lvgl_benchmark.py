# SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: CC0-1.0

import os
import datetime
import json
from pathlib import Path
import pytest
from pytest_embedded import Dut


def write_to_file(board, ext, text):
    with open("benchmark_" + board + ext, "a") as file:
        file.write(text)


def read_json_file(board):
    repo_root = Path(__file__).resolve().parent
    while repo_root.name != "esp-bsp" and repo_root.parent != repo_root:
        repo_root = repo_root.parent
    file_path = f"{repo_root}/bsp/{board}/benchmark.json"
    if not os.path.exists(file_path):
        print(f"\nFile \"{file_path}\" not exists\n")
        return []
    try:
        with open(file_path, "r") as file:
            return json.load(file)
    except json.JSONDecodeError:
        return []


def write_json_file(board, data):
    repo_root = Path(__file__).resolve().parent
    while repo_root.name != "esp-bsp" and repo_root.parent != repo_root:
        repo_root = repo_root.parent
    file_path = f"{repo_root}/bsp/{board}/benchmark.json"
    try:
        os.remove(file_path)
        with open(file_path, "a") as file:
            file.write(data)
    except OSError:
        pass


def find_test_results(json_obj, test):
    if json_obj:
        for t in json_obj["tests"]:
            if t["Name"] == test:
                return t


def get_test_diff(test1, test2, name, positive):
    if not test1 or not test2 or not test1[name] or not test2[name]:
        return ""
    test1[name] = test1[name].replace("%", "")
    test2[name] = test2[name].replace("%", "")
    diff = int(test1[name]) - int(test2[name])
    if diff == 0:
        return ""
    else:
        if positive:
            color = "red" if diff < 0 else "green"
        else:
            color = "green" if diff < 0 else "red"
        sign = "+" if diff > 0 else ""
        return f"*<span style=\"color:{color}\"><sub>({sign}{diff})</sub></span>*"


@pytest.mark.esp_box_3
@pytest.mark.esp32_p4_function_ev_board
@pytest.mark.esp32_s3_eye
@pytest.mark.esp32_s3_lcd_ev_board
@pytest.mark.esp32_s3_lcd_ev_board_2
@pytest.mark.m5dial
@pytest.mark.m5stack_core_s3
@pytest.mark.m5stack_core_s3_se
def test_example(dut: Dut, request) -> None:
    date = datetime.datetime.now()
    board = request.node.callspec.id

    # Wait for start benchmark
    dut.expect_exact('app_main: Display LVGL demo')
    dut.expect_exact('main_task: Returned from app_main()')

    try:
        os.remove("benchmark_" + board + ".md")
        os.remove("benchmark_" + board + ".json")
    except OSError:
        pass

    output = {
        "date": date.strftime('%d.%m.%Y %H:%M'),
        "board": board
    }

    # Write board into file
    write_to_file(board, ".md", f"# Benchmark for BOARD " + board + "\n\n")
    write_to_file(board, ".md", f"**DATE:** " + date.strftime('%d.%m.%Y %H:%M') + "\n\n")
    # Get LVGL version write it into file
    outdata = dut.expect(r'Benchmark Summary \((.*) \)', timeout=200)
    output["LVGL"] = outdata[1].decode()
    write_to_file(board, ".md", f"**LVGL version:** " + outdata[1].decode() + "\n\n")
    outdata = dut.expect(r'Name, Avg. CPU, Avg. FPS, Avg. time, render time, flush time', timeout=200)
    write_to_file(board, ".md", f"| Name | Avg. CPU | Avg. FPS | Avg. time | render time | flush time |\n")
    write_to_file(board, ".md", f"| ---- | :------: | :------: | :-------: | :---------: | :--------: |\n")  # noqa: E203

    last_results = read_json_file(board)

    # Benchmark lines
    output["tests"] = []
    for x in range(17):
        outdata = dut.expect(r'([\w \.]+),[ ]?(\d+%),[ ]?(\d+),[ ]?(\d+),[ ]?(\d+),[ ]?(\d+)', timeout=200)
        test_entry = {
            "Name": outdata[1].decode(),
            "Avg. CPU": outdata[2].decode(),
            "Avg. FPS": outdata[3].decode(),
            "Avg. time": outdata[4].decode(),
            "Render time": outdata[5].decode(),
            "Flush time": outdata[6].decode()
        }
        output["tests"].append(test_entry)

        last_test_result = find_test_results(last_results, test_entry["Name"])
        write_to_file(board, ".md", f"| " +
                      test_entry["Name"] + " | " +
                      test_entry["Avg. CPU"] + " " + get_test_diff(test_entry, last_test_result, "Avg. CPU", False) + " | " +
                      test_entry["Avg. FPS"] + " " + get_test_diff(test_entry, last_test_result, "Avg. FPS", True) + " | " +
                      test_entry["Avg. time"] + " " + get_test_diff(test_entry, last_test_result, "Avg. time", False) + " | " +
                      test_entry["Render time"] + " " + get_test_diff(test_entry, last_test_result, "Render time", False) + " | " +
                      test_entry["Flush time"] + " " + get_test_diff(test_entry, last_test_result, "Flush time", False) + " |\n")

    write_to_file(board, ".md", "\n")
    write_to_file(board, ".md", "***")
    write_to_file(board, ".md", "\n\n")

    # Save JSON to file
    json_output = json.dumps(output, indent=4)
    write_to_file(board, ".json", json_output)
    write_json_file(board, json_output)
