# Enabling GPU Support for darknet_ros

## Current Status
Your system has an NVIDIA GPU with compute capability 7.5, but darknet_ros is currently built **without GPU support** because CUDA toolkit is not installed.

## Steps to Enable GPU Support

### 1. Install CUDA Toolkit

You have CUDA driver version 13.0 installed. You need to install the CUDA toolkit that matches your driver.

**Option A: Install CUDA Toolkit via apt (Recommended)**
```bash
sudo apt update
sudo apt install nvidia-cuda-toolkit
```

**Option B: Install CUDA Toolkit from NVIDIA (for latest version)**
1. Visit: https://developer.nvidia.com/cuda-downloads
2. Select your Linux distribution
3. Follow the installation instructions

**Verify CUDA Installation:**
```bash
nvcc --version
```

### 2. Rebuild darknet_ros with GPU Support

After installing CUDA, rebuild the package:

```bash
cd /home/huojiaxi/3D_detector/3D_bounding_box_gb_visual_detection
rm -rf build install log
colcon build --packages-select darknet_ros
```

### 3. Verify GPU Support

During the build, you should see messages like:
```
-- CUDA Version: X.X
-- CUDA Libraries: ...
-- Building with GPU support
```

### 4. Check GPU Usage at Runtime

To verify GPU is being used:
```bash
# Monitor GPU usage
watch -n 1 nvidia-smi

# In another terminal, run darknet_ros
ros2 launch darknet_ros darknet_ros.launch.py
```

You should see GPU memory usage and compute activity in nvidia-smi.

## GPU Architecture Compatibility

The CMakeLists.txt has been updated to support multiple GPU architectures:
- **compute_75** (sm_75): RTX 20xx series, GTX 16xx series (your GPU)
- **compute_86** (sm_86): RTX 30xx series
- **compute_87** (sm_87): RTX 40xx series

This ensures compatibility with various NVIDIA GPUs.

## Troubleshooting

If CUDA is still not found after installation:
1. Check CUDA installation path: `ls /usr/local/cuda/`
2. Set CUDA_PATH environment variable if needed:
   ```bash
   export CUDA_PATH=/usr/local/cuda
   ```
3. Verify CUDA libraries are accessible:
   ```bash
   ldconfig -p | grep cuda
   ```

## Performance Benefits

With GPU support enabled, you should see:
- **10-50x faster inference** depending on your GPU
- Lower CPU usage
- Ability to process higher resolution images in real-time
