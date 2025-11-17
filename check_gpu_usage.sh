#!/bin/bash
# Script to check GPU usage for YOLO V8

echo "=== GPU Information ==="
nvidia-smi --query-gpu=name,driver_version,memory.total,memory.used,memory.free,utilization.gpu,temperature.gpu --format=csv

echo ""
echo "=== PyTorch CUDA Check ==="
python3 -c "
import torch
print(f'PyTorch version: {torch.__version__}')
print(f'CUDA available: {torch.cuda.is_available()}')
if torch.cuda.is_available():
    print(f'CUDA version: {torch.version.cuda}')
    print(f'cuDNN version: {torch.backends.cudnn.version()}')
    print(f'GPU count: {torch.cuda.device_count()}')
    for i in range(torch.cuda.device_count()):
        print(f'  GPU {i}: {torch.cuda.get_device_name(i)}')
        props = torch.cuda.get_device_properties(i)
        print(f'    Total memory: {props.total_memory / 1024**3:.2f} GB')
else:
    print('CUDA not available - YOLO will run on CPU (slow!)')
"

echo ""
echo "=== Checking YOLO Config ==="
if [ -f "src/ultralytics_ros/config/yolov8m.yaml" ]; then
    echo "YOLO V8 Medium config:"
    grep "device:" src/ultralytics_ros/config/yolov8m.yaml
else
    echo "Config file not found"
fi

echo ""
echo "=== Monitoring GPU Usage (Ctrl+C to stop) ==="
echo "Run this while YOLO is running to see real-time GPU usage:"
echo "  watch -n 1 nvidia-smi"
