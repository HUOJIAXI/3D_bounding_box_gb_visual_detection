/**
 * @file person_speed_tracker_node.cpp
 * @brief ROS2 node for tracking persons and calculating their walking speed
 *
 * This node subscribes to 3D bounding box detections, tracks individual persons
 * across frames, and estimates their walking speed with noise filtering.
 */

#include <rclcpp/rclcpp.hpp>
#include <gb_visual_detection_3d_msgs/msg/bounding_boxes3d.hpp>
#include <gb_visual_detection_3d_msgs/msg/bounding_box3d.hpp>
#include "person_tracker/msg/person_info.hpp"
#include "person_tracker/msg/person_info_array.hpp"
#include "person_tracker/msg/human_cluster.hpp"
#include "person_tracker/msg/human_cluster_array.hpp"
#include "person_tracker/human_clusterer.hpp"
#include "person_tracker/kalman_filter.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <map>
#include <vector>
#include <cmath>
#include <deque>
#include <algorithm>

/**
 * @brief Structure to hold tracked person state
 */
struct TrackedPerson {
    int id;
    geometry_msgs::msg::Point position;
    geometry_msgs::msg::Vector3 velocity;
    double speed;
    double confidence;
    int tracking_duration;
    rclcpp::Time last_seen;

    // Kalman filter for smooth position and velocity estimation
    person_tracker::KalmanFilter3D kalman_filter;
    rclcpp::Time last_update_time;

    // Store the last bounding box
    gb_visual_detection_3d_msgs::msg::BoundingBox3d last_bbox;
};

/**
 * @brief Main person tracker node class with speed tracking and clustering
 */
class PersonTrackerNode : public rclcpp::Node {
public:
    PersonTrackerNode() : Node("person_tracker_node"), next_person_id_(0),
                          tf_buffer_(this->get_clock()),
                          tf_listener_(tf_buffer_) {
        // Declare and get parameters
        this->declare_parameter("max_tracking_distance", 1.5);  // meters
        this->declare_parameter("max_missing_frames", 10);
        this->declare_parameter("velocity_history_size", 5);
        this->declare_parameter("position_history_size", 10);
        this->declare_parameter("min_speed_threshold", 0.05);  // m/s, ignore speeds below this
        this->declare_parameter("max_speed_threshold", 3.0);   // m/s, cap speeds above this
        this->declare_parameter("smoothing_alpha", 0.3);       // Exponential smoothing factor
        this->declare_parameter("map_frame", "map");           // Map frame for velocity calculation

        // Clustering parameters
        this->declare_parameter("cluster_distance_threshold", 1.2);  // meters
        this->declare_parameter("publish_singletons", true);         // publish clusters of size 1

        max_tracking_distance_ = this->get_parameter("max_tracking_distance").as_double();
        max_missing_frames_ = this->get_parameter("max_missing_frames").as_int();
        velocity_history_size_ = this->get_parameter("velocity_history_size").as_int();
        position_history_size_ = this->get_parameter("position_history_size").as_int();
        min_speed_threshold_ = this->get_parameter("min_speed_threshold").as_double();
        max_speed_threshold_ = this->get_parameter("max_speed_threshold").as_double();
        smoothing_alpha_ = this->get_parameter("smoothing_alpha").as_double();
        map_frame_ = this->get_parameter("map_frame").as_string();

        cluster_distance_threshold_ = this->get_parameter("cluster_distance_threshold").as_double();
        publish_singletons_ = this->get_parameter("publish_singletons").as_bool();

        // Create subscriber
        bbox_sub_ = this->create_subscription<gb_visual_detection_3d_msgs::msg::BoundingBoxes3d>(
            "/darknet_ros_3d/bounding_boxes", 10,
            std::bind(&PersonTrackerNode::bboxCallback, this, std::placeholders::_1));

        // Create publishers
        person_info_pub_ = this->create_publisher<person_tracker::msg::PersonInfoArray>(
            "/person_tracker/person_info", 10);

        cluster_pub_ = this->create_publisher<person_tracker::msg::HumanClusterArray>(
            "/person_tracker/human_clusters", 10);

        // Initialize clusterer with threshold
        clusterer_.setDistanceThreshold(cluster_distance_threshold_);

        RCLCPP_INFO(this->get_logger(), "Person Tracker Node initialized");
        RCLCPP_INFO(this->get_logger(), "  - Map frame: %s", map_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  - Max tracking distance: %.2f m", max_tracking_distance_);
        RCLCPP_INFO(this->get_logger(), "  - Velocity smoothing alpha: %.2f", smoothing_alpha_);
        RCLCPP_INFO(this->get_logger(), "  - Cluster distance threshold: %.2f m", cluster_distance_threshold_);
        RCLCPP_INFO(this->get_logger(), "  - Publish singletons: %s", publish_singletons_ ? "true" : "false");
    }

private:
    /**
     * @brief Calculate 3D center position from bounding box
     */
    geometry_msgs::msg::Point calculateCenter(const gb_visual_detection_3d_msgs::msg::BoundingBox3d& bbox) {
        geometry_msgs::msg::Point center;
        center.x = (bbox.xmin + bbox.xmax) / 2.0;
        center.y = (bbox.ymin + bbox.ymax) / 2.0;
        center.z = (bbox.zmin + bbox.zmax) / 2.0;
        return center;
    }

    /**
     * @brief Transform a point to the map frame
     * This ensures velocities are calculated in the fixed map frame,
     * independent of robot movement
     */
    bool transformToMapFrame(const geometry_msgs::msg::Point& point_in,
                            const std::string& source_frame,
                            const rclcpp::Time& timestamp,
                            geometry_msgs::msg::Point& point_out) {
        try {
            // Create stamped point in source frame
            geometry_msgs::msg::PointStamped point_stamped_in;
            point_stamped_in.header.frame_id = source_frame;
            point_stamped_in.header.stamp = timestamp;
            point_stamped_in.point = point_in;

            // Transform to map frame
            geometry_msgs::msg::PointStamped point_stamped_out;
            point_stamped_out = tf_buffer_.transform(point_stamped_in, map_frame_,
                                                     tf2::durationFromSec(0.5));

            point_out = point_stamped_out.point;
            return true;
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Failed to transform point to map frame: %s", ex.what());
            return false;
        }
    }

    /**
     * @brief Calculate Euclidean distance between two 3D points
     */
    double calculateDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    /**
     * @brief Update Kalman filter with new measurement and get estimated velocity
     */
    void updateKalmanFilter(TrackedPerson& person,
                           const geometry_msgs::msg::Point& measured_position,
                           const rclcpp::Time& current_time) {
        // Store previous velocity for change limiting
        geometry_msgs::msg::Vector3 prev_velocity = person.velocity;
        double prev_speed = person.speed;

        // Calculate dt since last update
        double dt = 0.1;  // Default 10Hz
        if (person.kalman_filter.isInitialized()) {
            dt = (current_time - person.last_update_time).seconds();
            // Sanity check on dt
            if (dt <= 0.0 || dt > 1.0) {
                dt = 0.1;
            }
        }

        // Predict step
        person.kalman_filter.predict(dt);

        // Update step with measurement
        person.kalman_filter.update(measured_position);

        // Get filtered estimates
        person.position = person.kalman_filter.getPosition();
        person.velocity = person.kalman_filter.getVelocity();
        person.speed = person.kalman_filter.getSpeed();

        // Constrain velocity change rate (max 1.0 m/s change per second)
        // This prevents sudden spikes from detection jitter
        if (person.kalman_filter.isInitialized() && person.tracking_duration > 1) {
            double max_velocity_change = 1.0 * dt;  // m/s per timestep

            double velocity_change = std::sqrt(
                std::pow(person.velocity.x - prev_velocity.x, 2) +
                std::pow(person.velocity.y - prev_velocity.y, 2) +
                std::pow(person.velocity.z - prev_velocity.z, 2)
            );

            // If velocity changed too much, limit it
            if (velocity_change > max_velocity_change) {
                double scale = max_velocity_change / velocity_change;
                person.velocity.x = prev_velocity.x + (person.velocity.x - prev_velocity.x) * scale;
                person.velocity.y = prev_velocity.y + (person.velocity.y - prev_velocity.y) * scale;
                person.velocity.z = prev_velocity.z + (person.velocity.z - prev_velocity.z) * scale;
                person.speed = std::sqrt(person.velocity.x * person.velocity.x +
                                       person.velocity.y * person.velocity.y +
                                       person.velocity.z * person.velocity.z);

                RCLCPP_DEBUG(this->get_logger(),
                            "Velocity change limited for person %d: %.2f -> %.2f m/s (change: %.2f)",
                            person.id, prev_speed, person.speed, velocity_change);
            }
        }

        // Apply speed thresholds
        if (person.speed < min_speed_threshold_) {
            person.speed = 0.0;
            person.velocity.x = 0.0;
            person.velocity.y = 0.0;
            person.velocity.z = 0.0;
        }
        if (person.speed > max_speed_threshold_) {
            double scale = max_speed_threshold_ / person.speed;
            person.velocity.x *= scale;
            person.velocity.y *= scale;
            person.velocity.z *= scale;
            person.speed = max_speed_threshold_;
        }

        person.last_update_time = current_time;
    }


    /**
     * @brief Data association: match detected persons to tracked persons
     */
    void associateDetections(const std::vector<gb_visual_detection_3d_msgs::msg::BoundingBox3d>& detections,
                            const std::string& source_frame,
                            const rclcpp::Time& current_time) {
        // Mark all tracked persons as not seen this frame
        std::map<int, bool> seen_this_frame;
        for (auto& [id, person] : tracked_persons_) {
            seen_this_frame[id] = false;
        }

        // Try to associate each detection with existing tracks
        for (const auto& detection : detections) {
            geometry_msgs::msg::Point detected_center = calculateCenter(detection);

            // Transform to map frame for consistent tracking across robot movement
            geometry_msgs::msg::Point detected_center_map;
            if (!transformToMapFrame(detected_center, source_frame, current_time, detected_center_map)) {
                RCLCPP_WARN(this->get_logger(), "Skipping detection due to transform failure");
                continue;
            }

            // Find closest tracked person
            int best_match_id = -1;
            double min_distance = max_tracking_distance_;

            for (auto& [id, person] : tracked_persons_) {
                if (seen_this_frame[id]) continue;  // Already matched

                double dist = calculateDistance(detected_center_map, person.position);
                if (dist < min_distance) {
                    min_distance = dist;
                    best_match_id = id;
                }
            }

            if (best_match_id >= 0) {
                // Update existing track
                updatePerson(best_match_id, detection, detected_center_map, current_time);
                seen_this_frame[best_match_id] = true;
            } else {
                // Create new track
                createNewPerson(detection, detected_center_map, current_time);
            }
        }

        // Remove tracks that haven't been seen for too long
        std::vector<int> to_remove;
        for (auto& [id, person] : tracked_persons_) {
            if (!seen_this_frame[id]) {
                // Increment missing frames counter (using tracking_duration as proxy)
                if ((current_time - person.last_seen).seconds() > max_missing_frames_ * 0.1) {  // Assuming ~10Hz
                    to_remove.push_back(id);
                }
            }
        }

        for (int id : to_remove) {
            RCLCPP_DEBUG(this->get_logger(), "Removing person ID %d (lost track)", id);
            tracked_persons_.erase(id);
        }
    }

    /**
     * @brief Create a new tracked person
     */
    void createNewPerson(const gb_visual_detection_3d_msgs::msg::BoundingBox3d& bbox,
                        const geometry_msgs::msg::Point& center,
                        const rclcpp::Time& current_time) {
        TrackedPerson person;
        person.id = next_person_id_++;
        person.confidence = bbox.probability;
        person.tracking_duration = 1;
        person.last_seen = current_time;
        person.last_bbox = bbox;
        person.last_update_time = current_time;

        // Initialize Kalman filter with first measurement
        person.kalman_filter.initialize(center);
        person.position = center;
        person.velocity.x = 0.0;
        person.velocity.y = 0.0;
        person.velocity.z = 0.0;
        person.speed = 0.0;

        tracked_persons_[person.id] = person;

        RCLCPP_DEBUG(this->get_logger(), "Created new person ID %d at (%.2f, %.2f, %.2f)",
                    person.id, center.x, center.y, center.z);
    }

    /**
     * @brief Update an existing tracked person
     */
    void updatePerson(int id, const gb_visual_detection_3d_msgs::msg::BoundingBox3d& bbox,
                     const geometry_msgs::msg::Point& center,
                     const rclcpp::Time& current_time) {
        auto& person = tracked_persons_[id];

        // Update Kalman filter with new measurement
        updateKalmanFilter(person, center, current_time);

        // Update other tracking info
        person.confidence = bbox.probability;
        person.tracking_duration++;
        person.last_seen = current_time;
        person.last_bbox = bbox;

        RCLCPP_DEBUG(this->get_logger(), "Updated person ID %d, speed: %.3f m/s", id, person.speed);
    }

    /**
     * @brief Callback for bounding box messages
     */
    void bboxCallback(const gb_visual_detection_3d_msgs::msg::BoundingBoxes3d::SharedPtr msg) {
        // Filter for person detections only
        std::vector<gb_visual_detection_3d_msgs::msg::BoundingBox3d> person_detections;
        for (const auto& bbox : msg->bounding_boxes) {
            if (bbox.object_name == "person") {
                person_detections.push_back(bbox);
            }
        }

        rclcpp::Time current_time = msg->header.stamp;
        std::string source_frame = msg->header.frame_id;

        // Associate detections with tracked persons (positions will be transformed to map frame)
        associateDetections(person_detections, source_frame, current_time);

        // Publish tracking results
        publishPersonInfo(msg->header);

        // Perform clustering and publish results
        performClustering(msg->header);

        RCLCPP_DEBUG(this->get_logger(), "Detected %zu persons, tracking %zu persons",
                    person_detections.size(), tracked_persons_.size());
    }

    /**
     * @brief Publish person tracking information
     */
    void publishPersonInfo(const std_msgs::msg::Header& header) {
        auto person_info_array = person_tracker::msg::PersonInfoArray();
        person_info_array.header = header;
        // Positions are in map frame, so update the frame_id
        person_info_array.header.frame_id = map_frame_;

        for (const auto& [id, person] : tracked_persons_) {
            auto person_info = person_tracker::msg::PersonInfo();
            person_info.person_id = person.id;
            person_info.bounding_box = person.last_bbox;
            person_info.position = person.position;
            person_info.velocity = person.velocity;
            person_info.speed = person.speed;
            person_info.confidence = person.confidence;
            person_info.tracking_duration = person.tracking_duration;
            person_info.last_seen = rclcpp::Time(person.last_seen);

            person_info_array.persons.push_back(person_info);
        }

        person_info_array.num_persons = static_cast<int>(tracked_persons_.size());
        person_info_pub_->publish(person_info_array);
    }

    /**
     * @brief Perform clustering on tracked persons and publish results
     */
    void performClustering(const std_msgs::msg::Header& header) {
        // Convert tracked persons to HumanDetection format
        std::vector<person_tracker::HumanDetection> detections;
        for (const auto& [id, person] : tracked_persons_) {
            person_tracker::HumanDetection det;
            det.id = person.id;
            det.x = person.position.x;
            det.y = person.position.y;
            det.z = person.position.z;
            detections.push_back(det);
        }

        // Perform clustering
        auto result = clusterer_.cluster(detections);

        // Create cluster array message
        auto cluster_array = person_tracker::msg::HumanClusterArray();
        cluster_array.header = header;
        // Cluster centroids are computed from positions in map frame
        cluster_array.header.frame_id = map_frame_;

        // Convert clusters to ROS messages
        for (const auto& cluster : result.clusters) {
            // Filter singletons if publish_singletons_ is false
            if (!publish_singletons_ && cluster.member_ids.size() == 1) {
                continue;
            }

            auto cluster_msg = person_tracker::msg::HumanCluster();
            cluster_msg.cluster_id = cluster.cluster_id;
            cluster_msg.member_ids = cluster.member_ids;
            cluster_msg.cluster_size = static_cast<int>(cluster.member_ids.size());

            // Set centroid
            cluster_msg.centroid.x = cluster.centroid_x;
            cluster_msg.centroid.y = cluster.centroid_y;
            cluster_msg.centroid.z = 0.0;  // Ground plane

            cluster_array.clusters.push_back(cluster_msg);
        }

        cluster_array.num_clusters = static_cast<int>(cluster_array.clusters.size());

        // Log cluster information
        RCLCPP_INFO(this->get_logger(), "Found %d clusters from %zu tracked persons",
                    cluster_array.num_clusters, tracked_persons_.size());

        for (const auto& cluster : cluster_array.clusters) {
            std::string member_str;
            for (size_t i = 0; i < cluster.member_ids.size(); ++i) {
                if (i > 0) member_str += ", ";
                member_str += std::to_string(cluster.member_ids[i]);
            }
            RCLCPP_DEBUG(this->get_logger(), "  Cluster %d: size=%d, members=[%s], centroid=(%.2f, %.2f)",
                        cluster.cluster_id, cluster.cluster_size, member_str.c_str(),
                        cluster.centroid.x, cluster.centroid.y);
        }

        // Publish clusters
        cluster_pub_->publish(cluster_array);
    }

    // ROS2 components
    rclcpp::Subscription<gb_visual_detection_3d_msgs::msg::BoundingBoxes3d>::SharedPtr bbox_sub_;
    rclcpp::Publisher<person_tracker::msg::PersonInfoArray>::SharedPtr person_info_pub_;
    rclcpp::Publisher<person_tracker::msg::HumanClusterArray>::SharedPtr cluster_pub_;

    // Tracking state
    std::map<int, TrackedPerson> tracked_persons_;
    int next_person_id_;

    // Parameters
    double max_tracking_distance_;
    int max_missing_frames_;
    int velocity_history_size_;
    int position_history_size_;
    double min_speed_threshold_;
    double max_speed_threshold_;
    double smoothing_alpha_;

    // Clustering parameters
    double cluster_distance_threshold_;
    bool publish_singletons_;

    // Clusterer
    person_tracker::HumanClusterer clusterer_;

    // TF2 for coordinate transformation
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string map_frame_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PersonTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
