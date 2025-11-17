# Quick Reference: YOLO V8 vs V3

## 🚀 Quick Commands

### YOLO V8 (Recommended - Default)

```bash
# Default (YOLO V8 Medium)
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py

# Choose model size
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py yolo_model:=yolov8n  # Fastest
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py yolo_model:=yolov8m  # Recommended
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py yolo_model:=yolov8l  # Accurate
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py yolo_model:=yolov8x  # Best accuracy

# Direct launch
ros2 launch darknet_ros_3d yolov8_3d.launch.py
```

### YOLO V3 (Legacy)

```bash
# Option 1: Launch argument
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py use_yolov8:=false

# Option 2: Direct launch
ros2 launch darknet_ros_3d yolov3_3d.launch.py
```

## 📊 Quick Comparison

| Aspect | YOLO V8 | YOLO V3 |
|--------|---------|---------|
| **Accuracy** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ |
| **Speed** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |
| **Model Size** | 6-136 MB | 235 MB |
| **Setup** | Auto-download | Manual weights |

## 🎯 When to Use What?

- **New projects**: YOLO V8 ✅
- **Better performance**: YOLO V8 ✅
- **Legacy compatibility**: YOLO V3 ⚠️
- **Custom YOLO V3 weights**: YOLO V3 ⚠️

## 📝 Configuration Files

- **YOLO V8**: `src/ultralytics_ros/config/yolov8m.yaml`
- **YOLO V3**: `src/darknet_ros/darknet_ros/config/yolov3.yaml`
- **3D Detection**: `src/gb_visual_detection_3d/darknet_ros_3d/config/darknet_3d.yaml`

## 🔍 Verify Which Version is Running

```bash
# Check nodes
ros2 node list

# YOLO V8 node: /yolo_detector_node
# YOLO V3 node: /darknet_ros

# Check topics (both use same topics)
ros2 topic echo /darknet_ros/bounding_boxes
```

## 📚 More Information

- Full guide: [YOLO_SELECTION_GUIDE.md](YOLO_SELECTION_GUIDE.md)
- Migration: [YOLO_V8_MIGRATION.md](YOLO_V8_MIGRATION.md)
- Quick start: [QUICK_START_YOLOV8.md](QUICK_START_YOLOV8.md)
