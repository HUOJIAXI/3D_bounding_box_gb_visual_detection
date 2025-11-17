// Copyright 2020 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* Author: Francisco Martín fmrico@gmail.com */
/* Author: Fernando González fergonzaramos@yahoo.es */

#include "darknet_ros_3d/Darknet3D.hpp"
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <algorithm>
#include <memory>
#include <limits>
#include <cmath>
#include <vector>
#include "gb_visual_detection_3d_msgs/msg/bounding_box3d.hpp"

using std::placeholders::_1;
using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace darknet_ros_3d
{
Darknet3D::Darknet3D()
: LifecycleNode("darknet3d_node"), clock_(RCL_SYSTEM_TIME),
  tfBuffer_(std::make_shared<rclcpp::Clock>(clock_)), tfListener_(tfBuffer_, true),
  pc_received_(false)
{
  // Init Params

  this->declare_parameter("darknet_ros_topic", "/darknet_ros/bounding_boxes");
  this->declare_parameter("output_bbx3d_topic", "/darknet_ros_3d/bounding_boxes");
  this->declare_parameter("point_cloud_topic", "/camera/depth_registered/points");
  this->declare_parameter("working_frame", "camera_link");
  this->declare_parameter("maximum_detection_threshold", 0.3f);
  this->declare_parameter("minimum_probability", 0.3f);
  this->declare_parameter("interested_classes", std::vector<std::string>());

  last_detection_ts_ = clock_.now();
}

void
Darknet3D::pointCloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  point_cloud_ = *msg;
  pc_received_ = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "Received point cloud with frame_id: %s", point_cloud_.header.frame_id.c_str());
}

void
Darknet3D::darknetCb(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg)
{
  original_bboxes_ = msg->bounding_boxes;
  last_detection_ts_ = clock_.now();
  RCLCPP_INFO(this->get_logger(), "Received %zu bounding boxes", original_bboxes_.size());
  for (const auto& bbx : original_bboxes_) {
    RCLCPP_INFO(this->get_logger(), "  - class_id='%s', probability=%.3f", 
      bbx.class_id.c_str(), bbx.probability);
  }
}

void
Darknet3D::calculate_boxes(
  sensor_msgs::msg::PointCloud2 cloud_pc2,
  sensor_msgs::msg::PointCloud cloud_pc,
  gb_visual_detection_3d_msgs::msg::BoundingBoxes3d * boxes)
{
  boxes->header.stamp = cloud_pc2.header.stamp;
  boxes->header.frame_id = cloud_pc2.header.frame_id;

  // Determine if point cloud is organized (height > 1) or unorganized (height == 1)
  bool is_organized = (cloud_pc2.height > 1);
  uint32_t image_width, image_height;
  
  if (is_organized) {
    // Organized point cloud: width and height represent image dimensions
    image_width = cloud_pc2.width;
    image_height = cloud_pc2.height;
  } else {
    // Unorganized point cloud: need to infer image dimensions
    // Strategy: Use bounding box coordinates to determine image size
    // The bounding boxes from darknet_ros correspond to the actual image dimensions
    uint32_t max_x = 0, max_y = 0;
    
    // Find maximum coordinates from all bounding boxes
    for (const auto& bbx : original_bboxes_) {
      if (bbx.xmax > max_x) max_x = bbx.xmax;
      if (bbx.ymax > max_y) max_y = bbx.ymax;
    }
    
    // If we have bounding boxes, use them to infer image dimensions
    // Add some padding to account for potential edge cases
    if (max_x > 0 && max_y > 0) {
      image_width = max_x + 10;  // Add small padding
      image_height = max_y + 10;
      
      RCLCPP_INFO_ONCE(this->get_logger(), 
        "Unorganized point cloud detected. Inferred image dimensions from bounding boxes: %ux%u (max_bbox: %ux%u, total_points=%u)", 
        image_width, image_height, max_x, max_y, cloud_pc2.width);
    } else {
      // Fallback: try to estimate from point cloud structure
      uint32_t total_points = cloud_pc2.width;
      uint32_t estimated_width = 0, estimated_height = 0;
      
      // Try common resolutions
      std::vector<std::pair<uint32_t, uint32_t>> common_resolutions = {
        {1920, 1080}, {1280, 720}, {640, 480}, {640, 360}, {320, 240},
        {1024, 768}, {800, 600}, {640, 400}, {512, 424}, {424, 240}
      };
      
      for (const auto& res : common_resolutions) {
        if (res.first * res.second == total_points) {
          estimated_width = res.first;
          estimated_height = res.second;
          break;
        }
      }
      
      // If no exact match, use row_step to estimate
      if (estimated_width == 0 && cloud_pc2.row_step > 0 && cloud_pc2.point_step > 0) {
        estimated_width = cloud_pc2.row_step / cloud_pc2.point_step;
        estimated_height = total_points / estimated_width;
      }
      
      // Final fallback: use sqrt approximation
      if (estimated_width == 0) {
        estimated_width = static_cast<uint32_t>(std::sqrt(total_points));
        estimated_height = total_points / estimated_width;
      }
      
      if (estimated_width == 0 || estimated_height == 0) {
        RCLCPP_WARN(this->get_logger(), 
          "Cannot determine image dimensions for unorganized point cloud (total_points=%u). "
          "Skipping 3D box calculation.", total_points);
        return;
      }
      
      image_width = estimated_width;
      image_height = estimated_height;
      
      RCLCPP_INFO_ONCE(this->get_logger(), 
        "Unorganized point cloud detected. Estimated image dimensions: %ux%u (total_points=%u)", 
        image_width, image_height, total_points);
    }
  }

  for (auto bbx : original_bboxes_) {
    // Check probability threshold
    if (bbx.probability < minimum_probability_) {
      RCLCPP_INFO(this->get_logger(), "Skipping bbox: class='%s', prob=%.3f < min=%.3f", 
        bbx.class_id.c_str(), bbx.probability, minimum_probability_);
      continue;
    }
    
    // Check class filter only if interested_classes_ is not empty
    // If empty, accept all classes
    if (!interested_classes_.empty() &&
        (std::find(interested_classes_.begin(), interested_classes_.end(),
        bbx.class_id) == interested_classes_.end()))
    {
      RCLCPP_INFO(this->get_logger(), "Skipping bbox: class='%s' not in interested_classes (size=%zu)", 
        bbx.class_id.c_str(), interested_classes_.size());
      continue;
    }
    RCLCPP_INFO(this->get_logger(), "Processing bbox: class_id=%s, probability=%.3f", 
      bbx.class_id.c_str(), bbx.probability);

    int center_x, center_y;

    center_x = (bbx.xmax + bbx.xmin) / 2;
    center_y = (bbx.ymax + bbx.ymin) / 2;

    // Bounds checking using image dimensions
    if (center_x < 0 || center_x >= static_cast<int>(image_width) ||
        center_y < 0 || center_y >= static_cast<int>(image_height))
    {
      RCLCPP_INFO(this->get_logger(), "Skipping bbox: center (%d, %d) out of bounds (image: %ux%u, cloud: %ux%u)", 
        center_x, center_y, image_width, image_height, cloud_pc2.width, cloud_pc2.height);
      continue;
    }

    // Calculate point cloud index using image dimensions
    // For unorganized clouds, points are often still stored in row-major order
    // even though height=1. We use the inferred image dimensions for indexing.
    int pc_index = (center_y * image_width) + center_x;
    
    // Validate index - for unorganized clouds, the point cloud size might not match
    // image_width * image_height exactly (due to invalid points), but we can still
    // use the indexing if the point cloud is stored in row-major order
    if (pc_index < 0 || pc_index >= static_cast<int>(cloud_pc.points.size())) {
      RCLCPP_INFO(this->get_logger(), 
        "Skipping bbox: pc_index %d out of range (size: %zu, image: %ux%u, expected_max_index: %u)", 
        pc_index, cloud_pc.points.size(), image_width, image_height, image_width * image_height);
      continue;
    }
    
    geometry_msgs::msg::Point32 center_point = cloud_pc.points[pc_index];

    if (std::isnan(center_point.x)) {
      RCLCPP_INFO(this->get_logger(), "Skipping bbox: center point is NaN");
      continue;
    }

    float maxx, minx, maxy, miny, maxz, minz;

    maxx = maxy = maxz = -std::numeric_limits<float>::max();
    minx = miny = minz = std::numeric_limits<float>::max();

    for (int i = bbx.xmin; i < bbx.xmax; i++) {
      for (int j = bbx.ymin; j < bbx.ymax; j++) {
        // Bounds checking using image dimensions
        if (i < 0 || i >= static_cast<int>(image_width) ||
            j < 0 || j >= static_cast<int>(image_height))
        {
          continue;
        }
        pc_index = (j * image_width) + i;
        if (pc_index < 0 || pc_index >= static_cast<int>(cloud_pc.points.size())) {
          continue;
        }
        geometry_msgs::msg::Point32 point = cloud_pc.points[pc_index];

        if (std::isnan(point.x)) {
          continue;
        }

        if (fabs(point.x - center_point.x) > maximum_detection_threshold_) {
          continue;
        }

        maxx = std::max(point.x, maxx);
        maxy = std::max(point.y, maxy);
        maxz = std::max(point.z, maxz);
        minx = std::min(point.x, minx);
        miny = std::min(point.y, miny);
        minz = std::min(point.z, minz);
      }
    }

    // Check if we found any valid points (min/max should have changed from initial values)
    if (maxx == -std::numeric_limits<float>::max() || minx == std::numeric_limits<float>::max()) {
      RCLCPP_INFO(this->get_logger(), "Skipping bbox: no valid points found in bounding box region");
      continue;
    }

    gb_visual_detection_3d_msgs::msg::BoundingBox3d bbx_msg;
    bbx_msg.object_name = bbx.class_id;
    bbx_msg.probability = bbx.probability;

    bbx_msg.xmin = minx;
    bbx_msg.xmax = maxx;
    bbx_msg.ymin = miny;
    bbx_msg.ymax = maxy;
    bbx_msg.zmin = minz;
    bbx_msg.zmax = maxz;

    RCLCPP_INFO(this->get_logger(), "Created 3D bbox: class='%s', size=(%.3f, %.3f, %.3f)", 
      bbx.class_id.c_str(), maxx - minx, maxy - miny, maxz - minz);
    boxes->bounding_boxes.push_back(bbx_msg);
  }
}

void
Darknet3D::publish_markers(gb_visual_detection_3d_msgs::msg::BoundingBoxes3d boxes)
{
  visualization_msgs::msg::MarkerArray msg;

  int counter_id = 0;
  for (auto bb : boxes.bounding_boxes) {
    visualization_msgs::msg::Marker bbx_marker;

    bbx_marker.header.frame_id = working_frame_;
    bbx_marker.header.stamp = boxes.header.stamp;
    bbx_marker.ns = "darknet3d";
    bbx_marker.id = counter_id++;
    bbx_marker.type = visualization_msgs::msg::Marker::CUBE;
    bbx_marker.action = visualization_msgs::msg::Marker::ADD;
    bbx_marker.frame_locked = false;
    bbx_marker.pose.position.x = (bb.xmax + bb.xmin) / 2.0;
    bbx_marker.pose.position.y = (bb.ymax + bb.ymin) / 2.0;
    bbx_marker.pose.position.z = (bb.zmax + bb.zmin) / 2.0;
    bbx_marker.pose.orientation.x = 0.0;
    bbx_marker.pose.orientation.y = 0.0;
    bbx_marker.pose.orientation.z = 0.0;
    bbx_marker.pose.orientation.w = 1.0;
    bbx_marker.scale.x = (bb.xmax - bb.xmin);
    bbx_marker.scale.y = (bb.ymax - bb.ymin);
    bbx_marker.scale.z = (bb.zmax - bb.zmin);
    bbx_marker.color.b = 0;
    bbx_marker.color.g = bb.probability * 255.0;
    bbx_marker.color.r = (1.0 - bb.probability) * 255.0;
    bbx_marker.color.a = 0.4;
    bbx_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    bbx_marker.text = bb.object_name;

    msg.markers.push_back(bbx_marker);
  }

  if (markers_pub_->is_activated()) {
    markers_pub_->publish(msg);
  }
}

void
Darknet3D::update()
{
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  if ((clock_.now() - last_detection_ts_).seconds() > 2.0) {
    RCLCPP_DEBUG(this->get_logger(), "No recent detections (last: %.2f seconds ago)", 
      (clock_.now() - last_detection_ts_).seconds());
    return;
  }
  if (!pc_received_) {
    RCLCPP_DEBUG(this->get_logger(), "Point cloud not received yet");
    return;
  }

  sensor_msgs::msg::PointCloud2 local_pointcloud;
  geometry_msgs::msg::TransformStamped transform;
  sensor_msgs::msg::PointCloud cloud_pc;
  gb_visual_detection_3d_msgs::msg::BoundingBoxes3d msg;

  // Check if transform is needed
  if (working_frame_ != point_cloud_.header.frame_id) {
    try {
      transform = tfBuffer_.lookupTransform(working_frame_, point_cloud_.header.frame_id,
          point_cloud_.header.stamp, tf2::durationFromSec(2.0));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform error of sensor data: %s, %s\n",
        ex.what(), "quitting callback");
      return;
    }
    tf2::doTransform<sensor_msgs::msg::PointCloud2>(point_cloud_, local_pointcloud, transform);
  } else {
    // No transform needed, use point cloud directly
    local_pointcloud = point_cloud_;
  }
  sensor_msgs::convertPointCloud2ToPointCloud(local_pointcloud, cloud_pc);

  calculate_boxes(local_pointcloud, cloud_pc, &msg);
  publish_markers(msg);

  if (darknet3d_pub_->is_activated()) {
    RCLCPP_INFO(this->get_logger(), "Publishing %zu 3D bounding boxes", msg.bounding_boxes.size());
    darknet3d_pub_->publish(msg);
  } else {
    RCLCPP_WARN(this->get_logger(), "Publisher not activated, cannot publish %zu 3D bounding boxes", msg.bounding_boxes.size());
  }
}

CallbackReturnT
Darknet3D::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Configuring from [%s] state...",
    this->get_name(), state.label().c_str());

  this->get_parameter("darknet_ros_topic", input_bbx_topic_);
  this->get_parameter("output_bbx3d_topic", output_bbx3d_topic_);
  this->get_parameter("point_cloud_topic", pointcloud_topic_);
  this->get_parameter("working_frame", working_frame_);
  this->get_parameter("maximum_detection_threshold", maximum_detection_threshold_);
  this->get_parameter("minimum_probability", minimum_probability_);
  this->get_parameter("interested_classes", interested_classes_);

  RCLCPP_INFO(this->get_logger(), "Configuration:");
  RCLCPP_INFO(this->get_logger(), "  darknet_ros_topic: %s", input_bbx_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  output_bbx3d_topic: %s", output_bbx3d_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  point_cloud_topic: %s", pointcloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  working_frame: %s", working_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  maximum_detection_threshold: %.3f", maximum_detection_threshold_);
  RCLCPP_INFO(this->get_logger(), "  minimum_probability: %.3f", minimum_probability_);
  if (interested_classes_.empty()) {
    RCLCPP_INFO(this->get_logger(), "  interested_classes: [ALL]");
  } else {
    std::string classes_str;
    for (size_t i = 0; i < interested_classes_.size(); ++i) {
      if (i > 0) classes_str += ", ";
      classes_str += "'" + interested_classes_[i] + "'";
    }
    RCLCPP_INFO(this->get_logger(), "  interested_classes: [%s]", classes_str.c_str());
  }

  // Use sensor data QoS profile with BEST_EFFORT reliability to match point cloud publishers
  auto qos = rclcpp::SensorDataQoS();
  pointCloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_, qos, std::bind(&Darknet3D::pointCloudCb, this, std::placeholders::_1));

  darknet_ros_sub_ = this->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>(
    input_bbx_topic_, 1, std::bind(&Darknet3D::darknetCb, this, std::placeholders::_1));

  darknet3d_pub_ = this->create_publisher<gb_visual_detection_3d_msgs::msg::BoundingBoxes3d>(
    output_bbx3d_topic_, 100);

  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/darknet_ros_3d/markers", 1);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
Darknet3D::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Activating from [%s] state...",
    this->get_name(), state.label().c_str());

  darknet3d_pub_->on_activate();
  markers_pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
Darknet3D::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Deactivating from [%s] state...",
    this->get_name(), state.label().c_str());

  darknet3d_pub_->on_deactivate();
  markers_pub_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
Darknet3D::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Cleanning Up from [%s] state...",
    this->get_name(), state.label().c_str());

  darknet3d_pub_.reset();
  markers_pub_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
Darknet3D::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Shutting Down from [%s] state...",
    this->get_name(), state.label().c_str());

  darknet3d_pub_.reset();
  markers_pub_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
Darknet3D::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Shutting Down from [%s] state...",
    this->get_name(), state.label().c_str());
  return CallbackReturnT::SUCCESS;
}

}  // namespace darknet_ros_3d
