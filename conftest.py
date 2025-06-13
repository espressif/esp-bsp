# SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
import pytest
import cv2
import subprocess
import numpy as np
from pathlib import Path


def pytest_generate_tests(metafunc):
    # If test expects param 'port' or 'flash_port', add parametrize
    if 'port' in metafunc.fixturenames and 'flash_port' in metafunc.fixturenames:
        metafunc.parametrize(
            'port, flash_port',
            [
                pytest.param('/dev/boards/esp-box-3', '/dev/boards/esp-box-3', id='esp_box_3'),
                pytest.param('/dev/boards/esp32_c3_lcdkit', '/dev/boards/esp32_c3_lcdkit', id='esp32_c3_lcdkit'),
                pytest.param('/dev/boards/esp32_p4_box', '/dev/boards/esp32_p4_box', id='esp32_p4_box'),
                pytest.param('/dev/boards/esp32_p4_function_ev_board', '/dev/boards/esp32_p4_function_ev_board', id='esp32_p4_function_ev_board'),
                pytest.param('/dev/boards/esp32_s3_eye', '/dev/boards/esp32_s3_eye', id='esp32_s3_eye'),
                pytest.param('/dev/boards/esp32_s3_lcd_ev_board', '/dev/boards/esp32_s3_lcd_ev_board', id='esp32_s3_lcd_ev_board'),
                pytest.param('/dev/boards/esp32_s3_lcd_ev_board-2', '/dev/boards/esp32_s3_lcd_ev_board-2', id='esp32_s3_lcd_ev_board_2'),
                pytest.param('/dev/boards/esp32_s3_usb_otg', '/dev/boards/esp32_s3_usb_otg', id='esp32_s3_usb_otg'),
                pytest.param('/dev/boards/esp_wrover_kit', '/dev/boards/esp_wrover_kit', id='esp_wrover_kit'),
                pytest.param('/dev/boards/esp32_azure_iot_kit', '/dev/boards/esp32_azure_iot_kit', id='esp32_azure_iot_kit'),
                pytest.param('/dev/boards/m5dial', '/dev/boards/m5dial', id='m5dial'),
                pytest.param('/dev/boards/m5stack_core', '/dev/boards/m5stack_core', id='m5stack_core'),
                pytest.param('/dev/boards/m5stack_core_2', '/dev/boards/m5stack_core_2', id='m5stack_core_2'),
                pytest.param('/dev/boards/m5stack_core_s3', '/dev/boards/m5stack_core_s3', id='m5stack_core_s3'),
                pytest.param('/dev/boards/m5stack_core_s3_se', '/dev/boards/m5stack_core_s3_se', id='m5stack_core_s3_se'),
                pytest.param('/dev/boards/m5_atom_s3', '/dev/boards/m5_atom_s3', id='m5_atom_s3'),
                pytest.param('/dev/boards/esp32_s3_devkitc_1_1', '/dev/boards/esp32_s3_devkitc_1_1', id='esp_bsp_devkit'),
                pytest.param('/dev/boards/esp32_s3_devkitc_1_1', '/dev/boards/esp32_s3_devkitc_1_1', id='esp_bsp_generic'),
                pytest.param('/dev/boards/esp32_s3_korvo_2', '/dev/boards/esp32_s3_korvo_2', id='esp32_s3_korvo_2'),
            ]
        )


# Check marker with params and test ID
def pytest_collection_modifyitems(config, items):
    for item in items:
        marker_option = "[" + config.getoption("-m") + "]"
        if marker_option not in item.nodeid:
            item.add_marker(pytest.mark.skip(reason="Not for selected params"))


def bsp_image_correction(image, width, height):
    pts_src = np.float32([
        [160, 80],    # top-left
        [1044, 87],   # top-right
        [279, 720],    # bottom-left
        [913, 718]   # bottom-right
    ])

    pts_dst = np.float32([
        [0, 0],
        [width, 0],
        [0, height],
        [width, height]
    ])

    matrix = cv2.getPerspectiveTransform(pts_src, pts_dst)
    warped = cv2.warpPerspective(image, matrix, (width, height))
    return warped


def bsp_fisheye_correction(image, width, height):
    K = np.array([
        [width, 0, width / 2],    # Focal length x
        [0, width, height / 2],    # Focal length y
        [0, 0, 1]])       # Principal point
    # [k1, k2, p1, p2, k3] - main is k1 a k2
    dist_coeffs = np.array([-0.3, 0.1, 0, 0, 0])

    new_K, _ = cv2.getOptimalNewCameraMatrix(K, dist_coeffs, (width, height), 1, (width, height))
    map1, map2 = cv2.initUndistortRectifyMap(K, dist_coeffs, None, new_K, (width, height), 5)
    undistorted = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR)

    return undistorted


def bsp_capture_image(image_path, board):
    # Enable auto-focus
    # subprocess.run(["v4l2-ctl", "-d", "/dev/video0", "--set-ctrl=focus_auto=1"])
    # Manual focus
    subprocess.run(["v4l2-ctl", "-d", "/dev/video0", "--set-ctrl=focus_auto=0"])
    subprocess.run(["v4l2-ctl", "-d", "/dev/video0", "--set-ctrl=focus_absolute=50"])
    # Manual exposition
    subprocess.run(["v4l2-ctl", "-d", "/dev/video0", "--set-ctrl=exposure_auto=1"])
    subprocess.run(["v4l2-ctl", "-d", "/dev/video0", "--set-ctrl=exposure_absolute=1"])

    # Return video from the first webcam on your computer.
    cap = cv2.VideoCapture(0)
    # Set FullHD resolution (1920x1080)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # TODO: Camera calibration

    # reads frames from a camera
    # ret checks return at each frame
    ret, frame = cap.read()
    if ret:
        # Image rotation
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        # TODO: Camera calibration / Perspective transform
        # TODO: Change size image
        # TODO: Crop image for {board}

        # correction image perspective and crop
        frame = bsp_image_correction(frame, 1000, 884)
        # correction of fisheye
        frame = bsp_fisheye_correction(frame, 1000, 884)
        # crop
        frame = frame[30:848, 38:980]

        # Save image
        cv2.imwrite(image_path, frame)
        print(f"Image saved {image_path}")
    else:
        print("Cannot save image.")

    # Close the window / Release webcam
    cap.release()


def bsp_test_image(board, example, expectation):
    image_file = f"snapshot_{board}_{example}.jpg"
    bsp_capture_image(image_file, board)


@pytest.fixture()
def bsp_test(request):
    board = request.node.callspec.id
    path = Path(str(request.node.fspath))
    test_name = path.parent.name
    yield
    print(f"Capturing image for: {board}")
    bsp_test_image(board, test_name, "")
