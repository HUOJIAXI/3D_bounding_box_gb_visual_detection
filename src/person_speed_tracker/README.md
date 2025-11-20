# Person Speed Tracker

A ROS2 package for tracking persons detected in 3D bounding boxes and calculating their walking speed with noise filtering.

## Overview

This package subscribes to 3D bounding box detections from `darknet_ros_3d`, tracks individual persons across frames, and publishes comprehensive tracking information including:

- Person ID (for persistent tracking)
- 3D bounding box
- 3D position
- Velocity vector
- Speed magnitude
- Tracking confidence
- Tracking duration

## Features

- **Person Tracking**: Associates detections across frames using distance-based matching
- **Speed Calculation**: Computes walking speed using position history
- **Noise Filtering**:
  - Exponential smoothing for velocity estimation
  - Minimum speed threshold to filter sensor noise
  - Maximum speed threshold to reject outliers
- **Robust Tracking**: Handles temporary occlusions and missing detections

## Topics

### Subscribed Topics

- `/darknet_ros_3d/bounding_boxes` (`gb_visual_detection_3d_msgs/BoundingBoxes3d`)
  - Input 3D bounding box detections (filters for "person" class)

### Published Topics

- `/person_speed_tracker/person_info` (`person_speed_tracker/PersonInfoArray`)
  - Array of tracked persons with position, velocity, and speed information

## Parameters

Configuration file: `config/tracker_params.yaml`

### Tracking Parameters

- `max_tracking_distance` (default: 1.5): Maximum distance (m) to associate detections with existing tracks
- `max_missing_frames` (default: 10): Maximum number of frames a person can be missing before track removal

### Velocity Calculation Parameters

- `velocity_history_size` (default: 5): Number of velocity samples for smoothing
- `position_history_size` (default: 10): Number of position samples for calculation
- `smoothing_alpha` (default: 0.3): Exponential smoothing factor (0.0-1.0, higher = more responsive)

### Speed Filtering Parameters

- `min_speed_threshold` (default: 0.05): Minimum speed (m/s) to report (filters noise)
- `max_speed_threshold` (default: 3.0): Maximum speed (m/s) to report (rejects outliers)

## Usage

### Build the Package

```bash
cd ~/3D_detector/3D_bounding_box_gb_visual_detection
colcon build --packages-select person_speed_tracker --symlink-install
source install/setup.bash
```

### Run the Node

Using the launch file (recommended):

```bash
ros2 launch person_speed_tracker person_speed_tracker.launch.py
```

Run the node directly:

```bash
ros2 run person_speed_tracker person_speed_tracker_node
```

### View Published Data

```bash
ros2 topic echo /person_speed_tracker/person_info
```

### Monitor with RViz

You can visualize the tracking data by subscribing to the published topics in RViz.

## Custom Messages

### PersonInfo.msg

Contains tracking information for a single person:

```
int32 person_id
gb_visual_detection_3d_msgs/BoundingBox3d bounding_box
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64 speed
float64 confidence
int32 tracking_duration
builtin_interfaces/Time last_seen
```

### PersonInfoArray.msg

Array of tracked persons:

```
std_msgs/Header header
PersonInfo[] persons
int32 num_persons
```

## Algorithm Details

### Person Tracking

1. **Detection Filtering**: Filters incoming bounding boxes for "person" class
2. **Data Association**: Uses nearest-neighbor matching based on 3D Euclidean distance
3. **Track Management**: Creates new tracks for unmatched detections, removes stale tracks

### Speed Calculation

1. **Position History**: Maintains a sliding window of recent positions with timestamps
2. **Velocity Estimation**: Calculates instantaneous velocity from consecutive positions
3. **Exponential Smoothing**: Applies smoothing to reduce noise in velocity estimates
4. **Thresholding**: Filters very low speeds (noise) and caps very high speeds (outliers)

### Noise Avoidance

- **Temporal Filtering**: Uses position and velocity history for smoothing
- **Exponential Moving Average**: Smooths velocity estimates over time
- **Threshold-based Filtering**: Removes noise below minimum threshold
- **Outlier Rejection**: Caps speeds above maximum realistic threshold

## Tuning Tips

- **For smoother tracking**: Increase `smoothing_alpha` (0.5-0.7)
- **For more responsive tracking**: Decrease `smoothing_alpha` (0.1-0.3)
- **For noisy environments**: Increase `min_speed_threshold` and `velocity_history_size`
- **For fast-moving persons**: Increase `max_tracking_distance` and `max_speed_threshold`

## Dependencies

- rclcpp
- std_msgs
- geometry_msgs
- gb_visual_detection_3d_msgs
- builtin_interfaces

## License

Apache-2.0

## Author

Maintained by huojiaxi (jiaxi.huo@outlook.com)
