# SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: CC0-1.0

import os
import datetime
import pytest
import json
from pytest_embedded import Dut


def write_to_file(board, ext, text):
    with open("benchmark_" + board + ext, "a") as h:
        h.write(text)


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

    # Benchmark lines
    output["tests"] = []
    for x in range(17):
        outdata = dut.expect(r'([\w \.]+),[ ]?(\d+%),[ ]?(\d+),[ ]?(\d+),[ ]?(\d+),[ ]?(\d+)', timeout=200)
        test_entry = {
            "Name": outdata[1].decode(),
            "Avg. CPU": outdata[2].decode(),
            "Avg. FPS": outdata[3].decode(),
            "Avg. time": outdata[4].decode(),
            "render time": outdata[5].decode(),
            "flush time": outdata[6].decode()
        }
        output["tests"].append(test_entry)

        write_to_file(board, ".md", f"| " +
                      outdata[1].decode() + " | " +
                      outdata[2].decode() + " | " +
                      outdata[3].decode() + " | " +
                      outdata[4].decode() + " | " +
                      outdata[5].decode() + " | " +
                      outdata[6].decode() + " |\n")

    write_to_file(board, ".md", "\n")
    write_to_file(board, ".md", "***")
    write_to_file(board, ".md", "\n\n")

    # Save JSON to file
    json_output = json.dumps(output, indent=4)
    write_to_file(board, ".json", json_output)
