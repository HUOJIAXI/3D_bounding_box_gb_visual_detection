#!/usr/bin/env python3
import urllib.request
import os

# Try multiple mirrors for YOLOv2-tiny weights
urls = [
    "https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov2-tiny.weights",
    "https://drive.google.com/uc?export=download&id=1wv_LiFeCRYwtpkqREPeI13-gPELBDwuJ",
]

weights_path = "/home/nvidia/gb_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights/yolov2-tiny.weights"

# Expected size is around 44MB (44661544 bytes)
expected_size = 40000000  # At least 40MB

for url in urls:
    try:
        print(f"Trying to download from: {url}")
        urllib.request.urlretrieve(url, weights_path)

        # Check if file size is reasonable
        file_size = os.path.getsize(weights_path)
        print(f"Downloaded file size: {file_size} bytes")

        if file_size > expected_size:
            print("Download successful!")
            break
        else:
            print(f"File too small (< {expected_size} bytes), trying next mirror...")
    except Exception as e:
        print(f"Failed: {e}")
        continue
else:
    print("\nAll download attempts failed.")
    print("Please manually download yolov2-tiny.weights from:")
    print("https://github.com/AlexeyAB/darknet/releases")
    print("Or use: gdown to download from Google Drive")
