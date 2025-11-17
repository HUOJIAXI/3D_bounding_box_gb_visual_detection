# YOLO Selection Guide: V8 vs V3

This guide helps you choose between YOLO V8 (Ultralytics) and YOLO V3 (Darknet) for your 3D detection system.

## Quick Comparison

| Feature | YOLO V8 (Ultralytics) | YOLO V3 (Darknet) |
|---------|----------------------|-------------------|
| **Accuracy** | ⭐⭐⭐⭐⭐ (10-20% higher mAP) | ⭐⭐⭐⭐ |
| **Speed** | ⭐⭐⭐⭐⭐ (2-3x faster) | ⭐⭐⭐ |
| **Model Size** | ⭐⭐⭐⭐⭐ (30x smaller) | ⭐⭐ |
| **GPU Support** | ✅ PyTorch/CUDA | ✅ CUDA |
| **Active Development** | ✅ Yes | ⚠️ Legacy |
| **Ease of Use** | ✅ Easy | ⚠️ Requires weights download |

## When to Use YOLO V8 (Recommended)

✅ **Use YOLO V8 if:**
- You want better accuracy and speed
- You have limited storage (smaller models)
- You want the latest features and updates
- You're starting a new project
- You need faster inference

## When to Use YOLO V3 (Legacy)

⚠️ **Use YOLO V3 if:**
- You need compatibility with existing systems
- You have pre-trained custom weights for YOLO V3
- You're maintaining legacy code
- You have specific requirements for Darknet framework

## Usage

### Option 1: YOLO V8 (Default)

```bash
# Default (YOLO V8 Medium)
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py

# Or explicitly specify YOLO V8
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py use_yolov8:=true

# Choose YOLO V8 model variant
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py use_yolov8:=true yolo_model:=yolov8n
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py use_yolov8:=true yolo_model:=yolov8m
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py use_yolov8:=true yolo_model:=yolov8l
```

### Option 2: YOLO V3 (Legacy)

```bash
# Use YOLO V3
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py use_yolov8:=false

# Or use dedicated YOLO V3 launch file
ros2 launch darknet_ros_3d yolov3_3d.launch.py
```

### Option 3: Direct Launch Files

```bash
# YOLO V8 only
ros2 launch darknet_ros_3d yolov8_3d.launch.py

# YOLO V3 only
ros2 launch darknet_ros_3d yolov3_3d.launch.py
```

## Model Variants (YOLO V8 Only)

| Model | Size | mAP | Speed | Use Case |
|-------|------|-----|-------|----------|
| `yolov8n` | 6.2MB | 37.3 | 80+ FPS | Speed priority |
| `yolov8s` | 22MB | 44.9 | 60+ FPS | Balanced |
| `yolov8m` | 52MB | 50.2 | 40+ FPS | **Recommended** |
| `yolov8l` | 87MB | 52.9 | 25+ FPS | Accuracy priority |
| `yolov8x` | 136MB | 53.9 | 20+ FPS | Best accuracy |
| `yolov11m` | 40MB | 51.5 | 45+ FPS | Latest generation |

## Performance Comparison

### YOLO V8 Medium vs YOLO V3

- **Accuracy**: YOLO V8m has ~10-20% higher mAP
- **Speed**: YOLO V8m is 2-3x faster on same hardware
- **Model Size**: YOLO V8m (52MB) vs YOLO V3 (235MB)
- **Memory**: YOLO V8m uses less GPU memory

## Configuration

### YOLO V8 Configuration

Edit: `src/ultralytics_ros/config/yolov8m.yaml`

```yaml
yolo_detector_node:
  ros__parameters:
    model_path: 'yolov8m.pt'
    device: 'cuda:0'  # or 'cpu'
    confidence_threshold: 0.25
    input_topic: '/camera/camera/color/image_raw'
```

### YOLO V3 Configuration

Edit: `src/darknet_ros/darknet_ros/config/yolov3.yaml`

```yaml
darknet_ros:
  ros__parameters:
    yolo_model:
      config_file:
        name: yolov3.cfg
      weight_file:
        name: yolov3.weights
      threshold:
        value: 0.3
```

## Switching Between Versions

### From YOLO V3 to YOLO V8

1. Install dependencies:
   ```bash
   pip3 install -r src/ultralytics_ros/requirements.txt
   ```

2. Build package:
   ```bash
   colcon build --packages-select ultralytics_ros
   ```

3. Launch with YOLO V8:
   ```bash
   ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py use_yolov8:=true
   ```

### From YOLO V8 to YOLO V3

1. Ensure YOLO V3 weights are downloaded:
   ```bash
   # Check if weights exist
   ls src/darknet_ros/darknet_ros/yolo_network_config/weights/yolov3.weights
   
   # If not, download:
   cd src/darknet_ros/darknet_ros/yolo_network_config/weights/
   wget https://pjreddie.com/media/files/yolov3.weights
   ```

2. Launch with YOLO V3:
   ```bash
   ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py use_yolov8:=false
   ```

## Troubleshooting

### YOLO V8 Issues

**Problem**: Model not loading
- **Solution**: Models auto-download on first run. Check internet connection.

**Problem**: CUDA out of memory
- **Solution**: Use smaller model (`yolov8n`) or reduce `image_size` in config

### YOLO V3 Issues

**Problem**: Weights file not found
- **Solution**: Download yolov3.weights (235MB) from https://pjreddie.com/media/files/yolov3.weights

**Problem**: CUDA compilation errors
- **Solution**: See `ENABLE_GPU.md` for CUDA setup instructions

## Recommendations

### For New Projects
👉 **Use YOLO V8** - Better performance, easier setup, active development

### For Existing Projects
- If already using YOLO V3: Can continue using it
- If migrating: Consider YOLO V8 for better performance

### For Production
- **YOLO V8 Medium** (`yolov8m`) - Best balance of accuracy and speed
- **YOLO V8 Large** (`yolov8l`) - If accuracy is critical

### For Development/Testing
- **YOLO V8 Nano** (`yolov8n`) - Fastest, good for quick testing

## Summary

**Default Choice**: YOLO V8 (Ultralytics)
- Better accuracy and speed
- Smaller models
- Easier to use
- Active development

**Legacy Choice**: YOLO V3 (Darknet)
- Use only if you have specific requirements
- Or for compatibility with existing systems
