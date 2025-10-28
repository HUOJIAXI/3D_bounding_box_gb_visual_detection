# 3D Bounding Box Detection System

A ROS 2 system for real-time 3D object detection using YOLO-based 2D detection combined with RGB-D camera depth information.

## Overview

This system integrates **darknet_ros** (2D object detection) with **gb_visual_detection_3d** (3D bounding box calculation) to provide complete 3D localization of detected objects in real-world coordinates.

### How It Works

1. **2D Detection**: darknet_ros uses YOLO neural network to detect objects in RGB images
2. **3D Projection**: gb_visual_detection_3d maps 2D bounding boxes to 3D space using point cloud data
3. **Output**: 3D bounding boxes with position (x,y,z) and dimensions in meters

```
RGB Image → YOLO Detection → 2D Bounding Boxes
                                      ↓
Point Cloud → Depth Information → 3D Bounding Boxes
```

## Prerequisites

### Hardware
- **RGB-D Camera** (one of the following):
  - Intel RealSense (D435, D455, etc.)
  - Asus Xtion Pro
  - Orbbec Astra
  - Any camera publishing RGB image + PointCloud2

### Software
- Ubuntu 22.04
- ROS 2 Humble
- CUDA (optional, for GPU acceleration)
- OpenCV
- Eigen3

## Installation

### 1. Clone the Repository

```bash
git clone git@github.com:HUOJIAXI/3D_bounding_box_gb_visual_detection.git gb_ws
cd gb_ws
```

### 2. Install Dependencies

```bash
# Install ROS 2 dependencies
sudo apt update
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    ros-humble-image-transport \
    ros-humble-tf2-ros \
    ros-humble-tf2-sensor-msgs \
    ros-humble-visualization-msgs

# Install system dependencies
sudo apt install -y \
    libopencv-dev \
    libeigen3-dev
```

### 3. Download YOLO Weights

```bash
# Download YOLOv3 weights (or use the provided script)
python3 download_weights.py

# Or manually download:
cd src/darknet_ros/darknet_ros/yolo_network_config/weights/
wget https://pjreddie.com/media/files/yolov3.weights
```

### 4. Build the Workspace

```bash
cd gb_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Configuration

### Camera Topics

Edit the configuration files to match your camera topics:

#### 1. darknet_ros Configuration
File: `src/darknet_ros/darknet_ros/config/ros.yaml`

```yaml
darknet_ros:
  ros__parameters:
    subscribers:
      camera_reading:
        topic: /camera/color/image_raw  # Change to your RGB topic
```

#### 2. darknet_ros_3d Configuration
File: `src/gb_visual_detection_3d/darknet_ros_3d/config/darknet_3d.yaml`

```yaml
darknet3d_node:
  ros__parameters:
    darknet_ros_topic: /darknet_ros/bounding_boxes
    output_bbx3d_topic: /darknet_ros_3d/bounding_boxes
    point_cloud_topic: /camera/depth/points  # Change to your point cloud topic
    working_frame: camera_link                # Change to your camera frame
    maximum_detection_threshold: 0.8          # Max depth variation (meters)
    minimum_probability: 0.1                  # Min detection confidence
    interested_classes: ["person", "book", "bottle"]  # Objects to detect
```

### Parameters Explained

| Parameter | Description | Default |
|-----------|-------------|---------|
| `point_cloud_topic` | PointCloud2 topic from RGB-D camera | `/camera/depth/points` |
| `working_frame` | TF frame for 3D coordinates | `camera_link` |
| `maximum_detection_threshold` | Max depth difference within object (m) | 0.8 |
| `minimum_probability` | Minimum detection confidence (0-1) | 0.1 |
| `interested_classes` | List of object classes to process | `["person", "book", "bottle"]` |

## Usage

### Quick Start

```bash
# Terminal 1: Start your camera driver (example for RealSense)
ros2 launch realsense2_camera rs_launch.py \
    enable_depth:=true \
    enable_color:=true \
    pointcloud.enable:=true

# Terminal 2: Run the complete 3D detection system
cd ~/gb_ws
source install/setup.bash
./run_3d_detection.sh
```

Or use the launch file directly:

```bash
source install/setup.bash
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py
```

### Step-by-Step Launch

If you prefer to run components separately:

```bash
# Terminal 1: Camera driver
ros2 launch realsense2_camera rs_launch.py

# Terminal 2: 2D object detection
source install/setup.bash
ros2 launch darknet_ros darknet_ros.launch.py

# Terminal 3: 3D bounding box calculation
source install/setup.bash
ros2 launch darknet_ros_3d darknet_ros_3d.launch.py
```

### Visualization in RViz2

```bash
# Terminal 4: Launch RViz
rviz2
```

**RViz Configuration:**
1. Set **Fixed Frame** to: `camera_link` (or your camera frame)
2. Add displays:
   - **Image** → Topic: `/darknet_ros/detection_image` (2D detections)
   - **PointCloud2** → Topic: `/camera/depth/points` (depth data)
   - **MarkerArray** → Topic: `/darknet_ros_3d/markers` (3D bounding boxes)

## ROS 2 Topics

### Published Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/darknet_ros/bounding_boxes` | `darknet_ros_msgs/BoundingBoxes` | 2D detections |
| `/darknet_ros/detection_image` | `sensor_msgs/Image` | Annotated image with 2D boxes |
| `/darknet_ros_3d/bounding_boxes` | `gb_visual_detection_3d_msgs/BoundingBoxes3d` | 3D detections |
| `/darknet_ros_3d/markers` | `visualization_msgs/MarkerArray` | 3D boxes for RViz |

### Subscribed Topics

| Topic | Message Type | Required |
|-------|--------------|----------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | ✅ RGB image |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | ✅ Point cloud |

### Message Format: BoundingBox3d

```yaml
string object_name       # Object class (e.g., "person")
float64 probability      # Detection confidence [0-1]
float64 xmin, xmax      # X-axis bounds (meters)
float64 ymin, ymax      # Y-axis bounds (meters)
float64 zmin, zmax      # Z-axis bounds (meters)
```

## Monitoring and Debugging

### Check Active Topics

```bash
# List all topics
ros2 topic list

# Check if camera is publishing
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/points

# View 2D detections
ros2 topic echo /darknet_ros/bounding_boxes

# View 3D detections
ros2 topic echo /darknet_ros_3d/bounding_boxes
```

### Verify TF Frames

```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo base_link camera_link
```

### Performance Monitoring

```bash
# CPU and GPU usage
htop
nvidia-smi  # If using GPU

# Node computational performance
ros2 run rqt_top rqt_top
```

## Troubleshooting

### Issue: No 3D bounding boxes appearing

**Possible Causes:**
1. Point cloud not publishing
   ```bash
   ros2 topic hz /camera/depth/points
   ```

2. Object class not in `interested_classes`
   - Edit `config/darknet_3d.yaml` to add desired classes

3. Probability threshold too high
   - Lower `minimum_probability` in `config/darknet_3d.yaml`

4. TF frame mismatch
   - Verify `working_frame` matches your camera's frame name

### Issue: darknet_ros not detecting objects

**Solutions:**
1. Check if weights file exists:
   ```bash
   ls src/darknet_ros/darknet_ros/yolo_network_config/weights/
   ```

2. Verify camera topic is correct in `config/ros.yaml`

3. Check image quality and lighting conditions

### Issue: "Transform error" messages

**Solution:**
- Ensure camera driver publishes TF transforms
- Check TF tree: `ros2 run tf2_tools view_frames`
- Verify `working_frame` parameter matches actual frame name

### Issue: High CPU usage

**Solutions:**
1. Use GPU acceleration (requires CUDA)
2. Use smaller YOLO model (yolov3-tiny)
3. Reduce camera resolution
4. Increase detection threshold to process fewer objects

## System Architecture

```
┌─────────────────┐
│   RGB-D Camera  │
└────────┬────────┘
         │
    ┌────┴─────────────────┐
    │                      │
┌───▼────────┐    ┌────────▼────────┐
│ RGB Image  │    │  Point Cloud    │
│ (640x480)  │    │  (PointCloud2)  │
└───┬────────┘    └────────┬────────┘
    │                      │
    │                      │
┌───▼─────────────┐        │
│  darknet_ros    │        │
│  (YOLO Detection)│       │
└───┬─────────────┘        │
    │                      │
    │ 2D Bounding Boxes    │
    │                      │
    └──────┬───────────────┘
           │
    ┌──────▼──────────────────┐
    │  darknet_ros_3d         │
    │  (3D Projection)        │
    │  - Map 2D→3D            │
    │  - Calculate bounds     │
    │  - Filter outliers      │
    └──────┬──────────────────┘
           │
    ┌──────┴───────────────────┐
    │                          │
┌───▼──────────────┐  ┌────────▼────────┐
│ 3D Bounding Boxes│  │  RViz Markers   │
│ (x,y,z + size)   │  │  (Visualization)│
└──────────────────┘  └─────────────────┘
```

## Supported Object Classes

Default YOLO (COCO dataset) supports 80 classes including:
- person, bicycle, car, motorcycle, bus, truck
- bottle, cup, bowl, chair, couch, bed
- laptop, mouse, keyboard, cell phone, book
- [Full list](https://github.com/pjreddie/darknet/blob/master/data/coco.names)

Edit `interested_classes` parameter to filter specific objects.

## Performance Tips

1. **Use GPU**: Significant speedup with CUDA-enabled darknet
2. **Optimize camera resolution**: 640x480 is usually sufficient
3. **Limit interested_classes**: Process only needed objects
4. **Adjust thresholds**: Balance accuracy vs. false positives
5. **Use yolov3-tiny**: Faster but less accurate alternative

## Citation

This project uses:
- [darknet_ros](https://github.com/leggedrobotics/darknet_ros) by Leggedrobotics
- [gb_visual_detection_3d](https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d) by Intelligent Robotics Labs

## License

- darknet_ros: BSD License
- gb_visual_detection_3d: Apache 2.0 License

## Contributing

Feel free to submit issues and pull requests to improve the system.

## Support

For issues and questions:
- Check the Troubleshooting section above
- Review ROS 2 logs: `ros2 run rqt_console rqt_console`
- Open an issue on GitHub

---

**Maintained by:** HUOJIAXI
**Contact:** JasonHuoeclille@gmail.com
**Repository:** https://github.com/HUOJIAXI/3D_bounding_box_gb_visual_detection
