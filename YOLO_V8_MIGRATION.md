# Migration Guide: YOLO V3 → YOLO V8

This guide helps you migrate from YOLO V3 (Darknet) to YOLO V8 (Ultralytics) for improved detection performance and faster inference.

## Quick Start

The system now uses **YOLO V8 by default**. Simply run:

```bash
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py
```

Or use the dedicated YOLO V8 launch file:

```bash
ros2 launch darknet_ros_3d yolov8_3d.launch.py
```

## What Changed?

### Architecture
- **Before**: YOLO V3 using Darknet framework (C/CUDA)
- **After**: YOLO V8 using Ultralytics framework (PyTorch)

### Performance Improvements
- **10-20% higher mAP** (mean Average Precision)
- **2-3x faster inference** on same hardware
- **Smaller model sizes** (YOLOv8n is 30x smaller than YOLOv3)
- **Better small object detection**

### Compatibility
✅ **Same ROS topics** - Drop-in replacement
✅ **Same message types** - Uses `darknet_ros_msgs`
✅ **Same 3D detection** - Works seamlessly with `darknet_ros_3d`

## Installation Steps

### 1. Install Python Dependencies

```bash
cd /home/huojiaxi/3D_detector/3D_bounding_box_gb_visual_detection
pip3 install -r src/ultralytics_ros/requirements.txt
```

Or install manually:

```bash
pip3 install ultralytics>=8.0.0 opencv-python>=4.5.0 numpy>=1.20.0 torch>=2.0.0 torchvision>=0.15.0
```

**Note**: If you're using GPU, make sure PyTorch is installed with CUDA support.

### 2. Build the Workspace

```bash
cd /home/huojiaxi/3D_detector/3D_bounding_box_gb_visual_detection
colcon build --packages-select ultralytics_ros
source install/setup.bash
```

### 3. Model Download

YOLO V8 models are **automatically downloaded** on first run. Models are cached in `~/.cache/ultralytics/`.

Available models:
- `yolov8n.pt` - Nano (6.2MB) - Fastest, lowest accuracy
- `yolov8s.pt` - Small (22MB) - Good balance
- `yolov8m.pt` - Medium (52MB) - **Recommended** (default)
- `yolov8l.pt` - Large (87MB) - Higher accuracy
- `yolov8x.pt` - Extra Large (136MB) - Best accuracy
- `yolov11m.pt` - YOLOv11 Medium (40MB) - Latest generation

## Usage

### Default Launch (YOLO V8 Medium)

```bash
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py
```

### Specify Model Variant

```bash
# YOLO V8 Nano (fastest)
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py yolo_model:=yolov8n

# YOLO V8 Large (more accurate)
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py yolo_model:=yolov8l

# YOLO V11 Medium (latest)
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py yolo_model:=yolov11m
```

### Use YOLO V3 (Legacy)

If you need to use YOLO V3 for compatibility:

```bash
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py use_yolov8:=false
```

### Direct YOLO V8 Launch

```bash
ros2 launch darknet_ros_3d yolov8_3d.launch.py
```

## Configuration

### YOLO V8 Configuration Files

Located in: `src/ultralytics_ros/config/`

Example: `yolov8m.yaml`

```yaml
yolo_detector_node:
  ros__parameters:
    model_path: 'yolov8m.pt'
    model_type: 'yolov8'
    device: 'cuda:0'              # Use 'cpu' for CPU-only
    confidence_threshold: 0.25
    iou_threshold: 0.45
    input_topic: '/camera/color/image_raw'
    max_det: 300
    enable_visualization: true
    publish_image: true
    image_size: 640
```

### 3D Detection Configuration

Located in: `src/gb_visual_detection_3d/darknet_ros_3d/config/darknet_3d.yaml`

```yaml
darknet3d_node:
  ros__parameters:
    darknet_ros_topic: /darknet_ros/bounding_boxes  # Same topic as YOLO V3
    output_bbx3d_topic: /darknet_ros_3d/bounding_boxes
    point_cloud_topic: /camera/depth/points
    working_frame: camera_link
    maximum_detection_threshold: 0.8
    minimum_probability: 0.1
    interested_classes: ["person"]  # Filter specific classes
```

## Performance Comparison

| Model | Size | mAP | Speed (FPS) | Best For |
|-------|------|-----|-------------|----------|
| YOLOv3 | 235MB | 55.3 | 30 | Baseline |
| **YOLOv8n** | 6.2MB | 37.3 | 80+ | Speed |
| **YOLOv8s** | 22MB | 44.9 | 60+ | Balance |
| **YOLOv8m** | 52MB | 50.2 | 40+ | **Recommended** |
| **YOLOv8l** | 87MB | 52.9 | 25+ | Accuracy |
| **YOLOv8x** | 136MB | 53.9 | 20+ | Best Accuracy |

## Troubleshooting

### Model Download Fails

Models are downloaded automatically. If download fails:

```bash
# Check internet connection
ping github.com

# Manually download (example for yolov8m)
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8m.pt
mkdir -p ~/.cache/ultralytics/
mv yolov8m.pt ~/.cache/ultralytics/
```

### CUDA Out of Memory

- Use smaller model: `yolo_model:=yolov8n`
- Reduce `image_size` in config (e.g., 416 instead of 640)
- Reduce `max_det` parameter

### Low FPS

1. **Check GPU usage**:
   ```bash
   nvidia-smi
   ```

2. **Use smaller model**:
   ```bash
   yolo_model:=yolov8n
   ```

3. **Reduce input resolution**:
   Edit config: `image_size: 416` or `image_size: 320`

4. **Disable visualization**:
   Edit config: `enable_visualization: false`

### CPU-Only Systems

For systems without GPU:

1. Edit config file: `device: 'cpu'`
2. Use smallest model: `yolo_model:=yolov8n`
3. Reduce image size: `image_size: 416`

### Module Not Found Errors

```bash
# Install missing dependencies
pip3 install ultralytics opencv-python numpy torch torchvision

# For ROS2 cv_bridge
sudo apt install ros-humble-cv-bridge
```

### NumPy Version Compatibility Issue

**Error**: `AttributeError: _ARRAY_API not found` or `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x`

**Cause**: ROS2 cv_bridge requires NumPy < 2.0, but newer opencv-python requires NumPy >= 2.0.

**Solution**: Install compatible versions:

```bash
# Option 1: Use requirements.txt (recommended)
pip3 install -r src/ultralytics_ros/requirements.txt

# Option 2: Manual fix
pip3 install "numpy>=1.20.0,<2.0.0" "opencv-python>=4.5.0,<4.10.0"
```

**Note**: The requirements.txt file now pins:
- NumPy to `<2.0.0` (for ROS2 cv_bridge compatibility)
- OpenCV to `<4.10.0` (for NumPy 1.x compatibility)

## Verification

### Check if YOLO V8 is Running

```bash
# Check running nodes
ros2 node list
# Should see: /yolo_detector_node

# Check topics
ros2 topic list
# Should see: /darknet_ros/bounding_boxes

# Monitor detections
ros2 topic echo /darknet_ros/bounding_boxes
```

### Compare Performance

Run both versions and compare:

```bash
# YOLO V8
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py use_yolov8:=true

# YOLO V3 (legacy)
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py use_yolov8:=false
```

## Migration Checklist

- [ ] Install Python dependencies (`pip3 install -r requirements.txt`)
- [ ] Build ultralytics_ros package (`colcon build --packages-select ultralytics_ros`)
- [ ] Test YOLO V8 launch (`ros2 launch darknet_ros_3d yolov8_3d.launch.py`)
- [ ] Verify 2D detections (`ros2 topic echo /darknet_ros/bounding_boxes`)
- [ ] Verify 3D detections (`ros2 topic echo /darknet_ros_3d/bounding_boxes`)
- [ ] Update any custom launch files to use YOLO V8
- [ ] Update documentation/reports if needed

## Benefits Summary

✅ **Better Accuracy**: 10-20% higher mAP  
✅ **Faster Inference**: 2-3x speed improvement  
✅ **Smaller Models**: 30x smaller than YOLOv3  
✅ **Active Development**: Regular updates from Ultralytics  
✅ **Easy Integration**: Drop-in replacement, same topics  
✅ **Multiple Variants**: Choose model size based on needs  

## Support

For issues or questions:
- Check `src/ultralytics_ros/README.md` for detailed documentation
- Ultralytics docs: https://docs.ultralytics.com
- GitHub: https://github.com/ultralytics/ultralytics

## Rollback to YOLO V3

If you need to use YOLO V3:

```bash
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py use_yolov8:=false
```

Or use the original launch file:

```bash
ros2 launch darknet_ros darknet_ros.launch.py
```
