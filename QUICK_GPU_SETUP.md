# Quick GPU Setup Guide

## Current Status
❌ **darknet_ros is NOT using GPU** - CUDA toolkit is not installed.

## Quick Setup (Run these commands):

### Step 1: Install CUDA Toolkit
```bash
sudo apt update
sudo apt install -y nvidia-cuda-toolkit
```

### Step 2: Verify CUDA Installation
```bash
nvcc --version
```

### Step 3: Rebuild darknet_ros with GPU Support
```bash
cd /home/huojiaxi/3D_detector/3D_bounding_box_gb_visual_detection
rm -rf build/darknet_ros install/darknet_ros
colcon build --packages-select darknet_ros
```

### Step 4: Verify GPU Support is Enabled
Look for these messages during build:
- `CUDA Version: X.X`
- `Building with GPU support`

Or check for CUDA object files:
```bash
find build/darknet_ros -name "*.cu.o" | wc -l
# Should show > 0 if GPU support is enabled
```

### Step 5: Test GPU Usage
```bash
# Terminal 1: Monitor GPU
watch -n 1 nvidia-smi

# Terminal 2: Run darknet_ros
source install/setup.bash
ros2 launch darknet_ros darknet_ros.launch.py
```

You should see GPU memory usage and compute activity in nvidia-smi.

## Alternative: Use the Installation Script
```bash
cd /home/huojiaxi/3D_detector/3D_bounding_box_gb_visual_detection
./install_cuda_and_rebuild.sh
```

This script will:
1. Install CUDA toolkit (requires sudo password)
2. Rebuild darknet_ros with GPU support
3. Verify GPU support is enabled
