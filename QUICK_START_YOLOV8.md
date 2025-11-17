# Quick Start: YOLO V8 Migration

## One-Command Setup

```bash
# 1. Install dependencies
pip3 install -r src/ultralytics_ros/requirements.txt

# 2. Build package
colcon build --packages-select ultralytics_ros
source install/setup.bash

# 3. Run (YOLO V8 is now default!)
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py
```

That's it! YOLO V8 models download automatically on first run.

## Model Selection

```bash
# Fastest (YOLOv8n - 6MB)
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py yolo_model:=yolov8n

# Recommended (YOLOv8m - 52MB) - DEFAULT
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py yolo_model:=yolov8m

# Most Accurate (YOLOv8x - 136MB)
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py yolo_model:=yolov8x

# Latest Generation (YOLOv11m - 40MB)
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py yolo_model:=yolov11m
```

## Verify It's Working

```bash
# Check nodes
ros2 node list
# Should see: /yolo_detector_node

# Check topics
ros2 topic list | grep darknet
# Should see: /darknet_ros/bounding_boxes

# Monitor detections
ros2 topic echo /darknet_ros/bounding_boxes
```

## Switch to YOLO V3 (Legacy)

```bash
# Option 1: Use launch argument
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py use_yolov8:=false

# Option 2: Use dedicated launch file
ros2 launch darknet_ros_3d yolov3_3d.launch.py
```

**Note**: YOLO V3 requires manual download of weights file (235MB). See [YOLO_SELECTION_GUIDE.md](YOLO_SELECTION_GUIDE.md) for details.

## Troubleshooting

**NumPy compatibility error?**
```bash
# Fix: Install compatible versions (ROS2 cv_bridge requires NumPy < 2.0)
pip3 install -r src/ultralytics_ros/requirements.txt

# Or manually:
pip3 install "numpy>=1.20.0,<2.0.0" "opencv-python>=4.5.0,<4.10.0"
```

**Model download fails?**
- Models auto-download on first run
- Check internet connection
- Models cached in `~/.cache/ultralytics/`

**Low FPS?**
- Use smaller model: `yolo_model:=yolov8n`
- Check GPU: `nvidia-smi`
- Edit config: `image_size: 416` (smaller = faster)

**CUDA out of memory?**
- Use smaller model: `yolo_model:=yolov8n`
- Edit config: `device: 'cpu'` (if no GPU)

For detailed migration guide, see [YOLO_V8_MIGRATION.md](YOLO_V8_MIGRATION.md)
