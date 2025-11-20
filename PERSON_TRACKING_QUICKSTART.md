# Person Speed Tracker - Quick Start Guide

## 🚀 Fastest Way to Launch Everything

```bash
cd ~/3D_detector/3D_bounding_box_gb_visual_detection
source install/setup.bash
ros2 launch person_speed_tracker person_tracking_system.launch.py
```

This single command launches:
- ✅ YOLO detector (person detection)
- ✅ 3D bounding box generator
- ✅ Person speed tracker

## 📊 Monitor the Output

In a new terminal:
```bash
source install/setup.bash
ros2 topic echo /person_speed_tracker/person_info
```

## 🎯 What You'll See

Example output:
```yaml
persons:
  - person_id: 0
    position:
      x: 2.45
      y: -1.23
      z: 0.85
    velocity:
      x: 0.45
      y: 0.12
      z: 0.0
    speed: 0.47  # m/s (walking speed)
    confidence: 0.92
    tracking_duration: 45
```

## ⚙️ Launch Options

### Use Different YOLO Model
```bash
# Faster (lighter model)
ros2 launch person_speed_tracker person_tracking_system.launch.py yolo_model:=yolov8n

# More accurate (heavier model)
ros2 launch person_speed_tracker person_tracking_system.launch.py yolo_model:=yolov8l
```

### Available YOLO Models
- `yolov8n` - Nano (fastest, least accurate)
- `yolov8s` - Small
- `yolov8m` - Medium (default, balanced)
- `yolov8l` - Large (more accurate)
- `yolov8x` - Extra large (most accurate, slowest)
- `yolov11m` - YOLO v11 Medium

### Use Real Robot (not simulation)
```bash
ros2 launch person_speed_tracker person_tracking_system.launch.py use_sim_time:=false
```

## 📋 Alternative Launch Files

### Option 1: Complete System (Recommended)
```bash
ros2 launch person_speed_tracker person_tracking_system.launch.py
```

### Option 2: Standalone Complete
```bash
ros2 launch person_speed_tracker person_tracking_complete.launch.py
```

### Option 3: Tracker Only
If YOLO and 3D bbox are already running:
```bash
ros2 launch person_speed_tracker person_speed_tracker.launch.py
```

## 🔍 Useful Commands

### Check if everything is running
```bash
ros2 node list
```
You should see:
- `yolo_detector_node`
- `darknet3d_node`
- `person_speed_tracker`

### Check topics
```bash
ros2 topic list | grep -E "(darknet|person)"
```

### Monitor detection rates
```bash
ros2 topic hz /darknet_ros_3d/bounding_boxes
ros2 topic hz /person_speed_tracker/person_info
```

### View specific person data
```bash
ros2 topic echo /person_speed_tracker/person_info --field persons[0].speed
```

## ⚡ Tuning for Your Environment

Edit configuration file:
```bash
nano src/person_speed_tracker/config/tracker_params.yaml
```

**For noisy/crowded environments:**
- Increase `min_speed_threshold: 0.1`
- Increase `velocity_history_size: 8`

**For fast-moving persons:**
- Increase `max_tracking_distance: 2.0`
- Increase `max_speed_threshold: 5.0`

**For smoother tracking:**
- Decrease `smoothing_alpha: 0.2`

**For more responsive tracking:**
- Increase `smoothing_alpha: 0.5`

After editing, rebuild:
```bash
colcon build --packages-select person_speed_tracker --symlink-install
source install/setup.bash
```

## 🛠️ Troubleshooting

### No persons detected
1. Check camera is running:
   ```bash
   ros2 topic hz /camera/color/image_raw
   ```

2. Verify YOLO is detecting:
   ```bash
   ros2 topic echo /darknet_ros/bounding_boxes
   ```

### Speed always shows 0.0
- Persons need to be tracked for at least 2 frames
- Try decreasing `min_speed_threshold`
- Check persons are actually moving in simulation

### Tracking is jittery
- Decrease `smoothing_alpha` (e.g., 0.2)
- Increase `velocity_history_size` (e.g., 8)

## 📦 System Architecture

```
Camera Image ──────► YOLO Detector ──────► 2D Bounding Boxes
                     (ultralytics_ros)
                                                    │
Point Cloud ─────────────────────────────────────► │
                                                    ▼
                                           3D Bbox Generator
                                           (darknet_ros_3d)
                                                    │
                                                    ▼
                                           Person Speed Tracker
                                           (person_speed_tracker)
                                                    │
                                                    ▼
                                           PersonInfoArray
                                           (ID, position, speed, etc.)
```

## 📚 More Information

- Detailed usage: `src/person_speed_tracker/USAGE.md`
- Package README: `src/person_speed_tracker/README.md`
- Configuration: `src/person_speed_tracker/config/tracker_params.yaml`

## 🎓 Example Use Cases

1. **Social Navigation**: Use person positions and velocities for path planning
2. **Crowd Monitoring**: Count and track multiple persons
3. **Safety Systems**: Alert when persons approach robot too quickly
4. **Data Collection**: Log pedestrian movement patterns
5. **Interaction**: Approach persons based on their speed and position
