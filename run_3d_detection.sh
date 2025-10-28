#!/bin/bash

# Script to run 3D bounding box detection
# This combines darknet_ros (2D detection) with darknet_ros_3d (3D detection)

echo "=========================================="
echo "Starting 3D Bounding Box Detection System"
echo "=========================================="
echo ""

# Source the workspace
source /opt/ros/humble/setup.bash
source /home/nvidia/gb_ws/install/setup.bash

echo "Workspace sourced successfully"
echo ""
echo "Prerequisites:"
echo "  1. Camera driver must be running and publishing:"
echo "     - /camera/color/image_raw (RGB image)"
echo "     - /camera/depth/points (PointCloud2)"
echo ""
echo "  2. Make sure your camera_link frame is published in TF"
echo ""
echo "Launching darknet_ros (2D) + darknet_ros_3d (3D detection)..."
echo ""

# Launch the complete system
ros2 launch darknet_ros_3d darknet_ros_3d_complete.launch.py
