# SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
import cv2


def bsp_capture_image(image_path):
    # Return video from the first webcam on your computer.
    cap = cv2.VideoCapture(0)
    # reads frames from a camera
    # ret checks return at each frame
    ret, frame = cap.read()
    if ret:
        # TODO: Change size image
        # TODO: Crop image

        # Save image
        cv2.imwrite(image_path, frame)
        print(f"Image saved {image_path}")
    else:
        print("Cannot save image.")

    # Close the window / Release webcam
    cap.release()


def bsp_test_image(board, example, expectation):
    image_file = f"snapshot_{example}_{board}.jpg"
    bsp_capture_image(image_file)
