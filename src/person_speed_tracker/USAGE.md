# Person Speed Tracker - Usage Guide

This guide explains how to launch the complete person tracking system in your Gazebo simulation.

## System Overview

The person tracking system consists of three main components:

1. **YOLO Detector** (ultralytics_ros) - Detects persons in 2D images
2. **3D Bounding Box Generator** (darknet_ros_3d) - Converts 2D detections to 3D using depth information
3. **Person Speed Tracker** (person_speed_tracker) - Tracks persons and calculates walking speed

## Launch Options

### Option 1: Complete System (Recommended)

Launch everything in one command using the complete launch file:

```bash
source install/setup.bash
ros2 launch person_speed_tracker person_tracking_system.launch.py
```

This launch file:
- ✅ Includes the existing darknet_ros_3d_complete.launch.py
- ✅ Automatically launches YOLO detector
- ✅ Automatically launches 3D bounding box generator
- ✅ Launches person speed tracker
- ✅ Configures all nodes with proper parameters

**With custom YOLO model:**
```bash
ros2 launch person_speed_tracker person_tracking_system.launch.py yolo_model:=yolov8l
```

**Available models:** yolov8n, yolov8s, yolov8m (default), yolov8l, yolov8x, yolov11m

### Option 2: Standalone Complete Launch

Alternative complete launch file with more control:

```bash
ros2 launch person_speed_tracker person_tracking_complete.launch.py
```

**With options:**
```bash
ros2 launch person_speed_tracker person_tracking_complete.launch.py \
  yolo_model:=yolov8m \
  use_sim_time:=true
```

### Option 3: Person Tracker Only

If you already have YOLO and darknet_ros_3d running:

```bash
ros2 launch person_speed_tracker person_speed_tracker.launch.py
```

### Option 4: Manual Component Launch

Launch each component separately in different terminals:

**Terminal 1 - YOLO Detector:**
```bash
source install/setup.bash
ros2 launch darknet_ros_3d yolov8_3d.launch.py
```

**Terminal 2 - Person Speed Tracker:**
```bash
source install/setup.bash
ros2 launch person_speed_tracker person_speed_tracker.launch.py
```

## Complete Workflow Example

### Step 1: Start Gazebo Simulation
```bash
# Terminal 1
source install/setup.bash
# Your Gazebo launch command (e.g., for TIAGo robot)
ros2 launch <your_gazebo_package> <your_world>.launch.py
```

### Step 2: Launch Person Tracking System
```bash
# Terminal 2
source install/setup.bash
ros2 launch person_speed_tracker person_tracking_system.launch.py
```

### Step 3: Monitor Outputs

**View tracked persons:**
```bash
# Terminal 3
source install/setup.bash
ros2 topic echo /person_speed_tracker/person_info
```

**Monitor detection rates:**
```bash
ros2 topic hz /darknet_ros_3d/bounding_boxes
ros2 topic hz /person_speed_tracker/person_info
```

**List all active topics:**
```bash
ros2 topic list
```

## Published Topics

The complete system publishes:

| Topic | Type | Description |
|-------|------|-------------|
| `/darknet_ros/bounding_boxes` | `darknet_ros_msgs/BoundingBoxes` | 2D detections from YOLO |
| `/darknet_ros_3d/bounding_boxes` | `gb_visual_detection_3d_msgs/BoundingBoxes3d` | 3D bounding boxes |
| `/person_speed_tracker/person_info` | `person_speed_tracker/PersonInfoArray` | Tracked persons with speed |

## Subscribed Topics

The system requires:

| Topic | Type | Description |
|-------|------|-------------|
| Camera RGB image | `sensor_msgs/Image` | Input images for YOLO |
| Point cloud | `sensor_msgs/PointCloud2` | Depth data for 3D estimation |

## Configuration

### Person Speed Tracker Parameters

Edit `config/tracker_params.yaml`:

```yaml
person_speed_tracker:
  ros__parameters:
    max_tracking_distance: 1.5    # meters
    max_missing_frames: 10        # frames
    velocity_history_size: 5      # samples
    position_history_size: 10     # samples
    smoothing_alpha: 0.3          # 0.0-1.0
    min_speed_threshold: 0.05     # m/s
    max_speed_threshold: 3.0      # m/s
```

### 3D Bounding Box Parameters

Edit darknet_ros_3d config if needed:
```bash
nano src/gb_visual_detection_3d/darknet_ros_3d/config/darknet_3d.yaml
```

Key parameters:
- `point_cloud_topic`: Your robot's depth camera topic
- `working_frame`: Your robot's camera frame
- `interested_classes`: ["person"]

## Visualization with RViz

To visualize the tracking in RViz:

```bash
rviz2
```

Add displays for:
1. **Camera** - Subscribe to camera image topic
2. **PointCloud2** - Subscribe to point cloud topic
3. **MarkerArray** - For bounding box visualization (if available)

## Troubleshooting

### No detections appearing

1. Check camera topics are publishing:
   ```bash
   ros2 topic list | grep camera
   ros2 topic hz /camera/camera/color/image_raw
   ```

2. Check YOLO is detecting:
   ```bash
   ros2 topic echo /darknet_ros/bounding_boxes
   ```

3. Verify point cloud is available:
   ```bash
   ros2 topic hz <your_point_cloud_topic>
   ```

### Person tracker not publishing

1. Check if 3D bounding boxes are being published:
   ```bash
   ros2 topic echo /darknet_ros_3d/bounding_boxes
   ```

2. Verify "person" class is in detections:
   ```bash
   ros2 topic echo /darknet_ros_3d/bounding_boxes | grep "person"
   ```

3. Check node is running:
   ```bash
   ros2 node list | grep person_speed_tracker
   ```

### Low detection rate

- Try a larger YOLO model: `yolo_model:=yolov8l` or `yolov8x`
- Adjust `minimum_probability` in darknet_3d.yaml
- Check camera is pointing at persons in scene

## Performance Tuning

### For faster processing:
- Use smaller YOLO model: `yolov8n` or `yolov8s`
- Reduce image resolution in YOLO config

### For better accuracy:
- Use larger YOLO model: `yolov8l` or `yolov8x`
- Increase `velocity_history_size` for smoother speed estimates
- Decrease `smoothing_alpha` for more stable tracking

### For noisy environments:
- Increase `min_speed_threshold` (e.g., 0.1 m/s)
- Increase `velocity_history_size` (e.g., 8-10)
- Decrease `smoothing_alpha` (e.g., 0.2)

## Example: Quick Start

The absolute quickest way to get started:

```bash
# In workspace root
cd ~/3D_detector/3D_bounding_box_gb_visual_detection

# Source workspace
source install/setup.bash

# Launch everything
ros2 launch person_speed_tracker person_tracking_system.launch.py

# In another terminal, monitor output
source install/setup.bash
ros2 topic echo /person_speed_tracker/person_info
```

That's it! The system will automatically:
- Start YOLO detector
- Generate 3D bounding boxes
- Track persons and calculate speeds
- Publish comprehensive tracking information

## Next Steps

- Integrate with your navigation system
- Use person positions for obstacle avoidance
- Create custom visualization nodes
- Log tracking data for analysis
- Implement trajectory prediction
