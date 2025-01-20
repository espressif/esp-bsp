# SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: CC0-1.0

import os
import datetime
import pytest
from pytest_embedded import Dut


def write_to_file(board, text):
    with open("benchmark_" + board + ".md", "a") as h:
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
    except OSError:
        pass

    # Write board into file
    write_to_file(board, f"# Benchmark for BOARD " + board + "\n\n")
    write_to_file(board, f"**DATE:** " + date.strftime('%d.%m.%Y %H:%M') + "\n\n")
    # Get LVGL version write it into file
    outdata = dut.expect(r'Benchmark Summary \((.*) \)', timeout=200)
    write_to_file(board, f"**LVGL version:** " + outdata[1].decode() + "\n\n")
    outdata = dut.expect(r'Name, Avg. CPU, Avg. FPS, Avg. time, render time, flush time', timeout=200)
    write_to_file(board, f"| Name | Avg. CPU | Avg. FPS | Avg. time | render time | flush time |\n")
    write_to_file(board, f"| ---- | :------: | :------: | :-------: | :---------: | :--------: |\n")  # noqa: E203

    # Benchmark lines
    for x in range(17):
        outdata = dut.expect(r'([\w \.]+),[ ]?(\d+%),[ ]?(\d+),[ ]?(\d+),[ ]?(\d+),[ ]?(\d+)', timeout=200)
        write_to_file(board, f"| " +
                      outdata[1].decode() + " | " +
                      outdata[2].decode() + " | " +
                      outdata[3].decode() + " | " +
                      outdata[4].decode() + " | " +
                      outdata[5].decode() + " | " +
                      outdata[6].decode() + " |\n")

    write_to_file(board, "\n")
    write_to_file(board, "***")
    write_to_file(board, "\n\n")
