# SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0

import cv2

# This will return video from the first webcam on your computer.
cap = cv2.VideoCapture(0)

# reads frames from a camera
# ret checks return at each frame
ret, frame = cap.read()
if ret:
    # Save image
    image_path = "snapshot.jpg"
    cv2.imwrite(image_path, frame)
    print(f"Image saved {image_path}")
else:
    print("Cannot save image.")

# Close the window / Release webcam
cap.release()
