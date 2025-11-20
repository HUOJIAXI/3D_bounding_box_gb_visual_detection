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
#include "person_speed_tracker/msg/person_info.hpp"
#include "person_speed_tracker/msg/person_info_array.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

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

    // For velocity filtering - store recent positions and timestamps
    std::deque<std::pair<geometry_msgs::msg::Point, rclcpp::Time>> position_history;
    std::deque<geometry_msgs::msg::Vector3> velocity_history;

    // Store the last bounding box
    gb_visual_detection_3d_msgs::msg::BoundingBox3d last_bbox;
};

/**
 * @brief Main person speed tracker node class
 */
class PersonSpeedTrackerNode : public rclcpp::Node {
public:
    PersonSpeedTrackerNode() : Node("person_speed_tracker_node"), next_person_id_(0) {
        // Declare and get parameters
        this->declare_parameter("max_tracking_distance", 1.5);  // meters
        this->declare_parameter("max_missing_frames", 10);
        this->declare_parameter("velocity_history_size", 5);
        this->declare_parameter("position_history_size", 10);
        this->declare_parameter("min_speed_threshold", 0.05);  // m/s, ignore speeds below this
        this->declare_parameter("max_speed_threshold", 3.0);   // m/s, cap speeds above this
        this->declare_parameter("smoothing_alpha", 0.3);       // Exponential smoothing factor

        max_tracking_distance_ = this->get_parameter("max_tracking_distance").as_double();
        max_missing_frames_ = this->get_parameter("max_missing_frames").as_int();
        velocity_history_size_ = this->get_parameter("velocity_history_size").as_int();
        position_history_size_ = this->get_parameter("position_history_size").as_int();
        min_speed_threshold_ = this->get_parameter("min_speed_threshold").as_double();
        max_speed_threshold_ = this->get_parameter("max_speed_threshold").as_double();
        smoothing_alpha_ = this->get_parameter("smoothing_alpha").as_double();

        // Create subscriber
        bbox_sub_ = this->create_subscription<gb_visual_detection_3d_msgs::msg::BoundingBoxes3d>(
            "/darknet_ros_3d/bounding_boxes", 10,
            std::bind(&PersonSpeedTrackerNode::bboxCallback, this, std::placeholders::_1));

        // Create publisher
        person_info_pub_ = this->create_publisher<person_speed_tracker::msg::PersonInfoArray>(
            "/person_speed_tracker/person_info", 10);

        RCLCPP_INFO(this->get_logger(), "Person Speed Tracker Node initialized");
        RCLCPP_INFO(this->get_logger(), "  - Max tracking distance: %.2f m", max_tracking_distance_);
        RCLCPP_INFO(this->get_logger(), "  - Velocity smoothing alpha: %.2f", smoothing_alpha_);
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
     * @brief Calculate Euclidean distance between two 3D points
     */
    double calculateDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    /**
     * @brief Calculate velocity using position history with exponential smoothing
     */
    geometry_msgs::msg::Vector3 calculateSmoothedVelocity(TrackedPerson& person,
                                                          const geometry_msgs::msg::Point& new_position,
                                                          const rclcpp::Time& current_time) {
        geometry_msgs::msg::Vector3 velocity;
        velocity.x = 0.0;
        velocity.y = 0.0;
        velocity.z = 0.0;

        // Add new position to history
        person.position_history.push_back({new_position, current_time});
        if (person.position_history.size() > static_cast<size_t>(position_history_size_)) {
            person.position_history.pop_front();
        }

        // Need at least 2 positions to calculate velocity
        if (person.position_history.size() < 2) {
            return velocity;
        }

        // Calculate instantaneous velocity using recent positions
        auto& latest = person.position_history.back();
        auto& previous = person.position_history[person.position_history.size() - 2];

        double dt = (latest.second - previous.second).seconds();

        if (dt > 0.0001) {  // Avoid division by very small numbers
            velocity.x = (latest.first.x - previous.first.x) / dt;
            velocity.y = (latest.first.y - previous.first.y) / dt;
            velocity.z = (latest.first.z - previous.first.z) / dt;

            // Apply exponential smoothing if we have velocity history
            if (!person.velocity_history.empty()) {
                auto& last_vel = person.velocity_history.back();
                velocity.x = smoothing_alpha_ * velocity.x + (1.0 - smoothing_alpha_) * last_vel.x;
                velocity.y = smoothing_alpha_ * velocity.y + (1.0 - smoothing_alpha_) * last_vel.y;
                velocity.z = smoothing_alpha_ * velocity.z + (1.0 - smoothing_alpha_) * last_vel.z;
            }

            // Add to velocity history
            person.velocity_history.push_back(velocity);
            if (person.velocity_history.size() > static_cast<size_t>(velocity_history_size_)) {
                person.velocity_history.pop_front();
            }
        }

        return velocity;
    }

    /**
     * @brief Calculate speed magnitude and apply thresholding
     */
    double calculateSpeed(const geometry_msgs::msg::Vector3& velocity) {
        double speed = std::sqrt(velocity.x * velocity.x +
                                velocity.y * velocity.y +
                                velocity.z * velocity.z);

        // Apply minimum threshold (noise filtering)
        if (speed < min_speed_threshold_) {
            speed = 0.0;
        }

        // Apply maximum threshold (outlier rejection)
        if (speed > max_speed_threshold_) {
            speed = max_speed_threshold_;
        }

        return speed;
    }

    /**
     * @brief Data association: match detected persons to tracked persons
     */
    void associateDetections(const std::vector<gb_visual_detection_3d_msgs::msg::BoundingBox3d>& detections,
                            const rclcpp::Time& current_time) {
        // Mark all tracked persons as not seen this frame
        std::map<int, bool> seen_this_frame;
        for (auto& [id, person] : tracked_persons_) {
            seen_this_frame[id] = false;
        }

        // Try to associate each detection with existing tracks
        for (const auto& detection : detections) {
            geometry_msgs::msg::Point detected_center = calculateCenter(detection);

            // Find closest tracked person
            int best_match_id = -1;
            double min_distance = max_tracking_distance_;

            for (auto& [id, person] : tracked_persons_) {
                if (seen_this_frame[id]) continue;  // Already matched

                double dist = calculateDistance(detected_center, person.position);
                if (dist < min_distance) {
                    min_distance = dist;
                    best_match_id = id;
                }
            }

            if (best_match_id >= 0) {
                // Update existing track
                updatePerson(best_match_id, detection, detected_center, current_time);
                seen_this_frame[best_match_id] = true;
            } else {
                // Create new track
                createNewPerson(detection, detected_center, current_time);
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
        person.position = center;
        person.velocity.x = 0.0;
        person.velocity.y = 0.0;
        person.velocity.z = 0.0;
        person.speed = 0.0;
        person.confidence = bbox.probability;
        person.tracking_duration = 1;
        person.last_seen = current_time;
        person.last_bbox = bbox;

        // Initialize position history
        person.position_history.push_back({center, current_time});

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

        // Calculate velocity with smoothing
        person.velocity = calculateSmoothedVelocity(person, center, current_time);
        person.speed = calculateSpeed(person.velocity);

        // Update position
        person.position = center;
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

        // Associate detections with tracked persons
        associateDetections(person_detections, current_time);

        // Publish tracking results
        publishPersonInfo(msg->header);

        RCLCPP_DEBUG(this->get_logger(), "Detected %zu persons, tracking %zu persons",
                    person_detections.size(), tracked_persons_.size());
    }

    /**
     * @brief Publish person tracking information
     */
    void publishPersonInfo(const std_msgs::msg::Header& header) {
        auto person_info_array = person_speed_tracker::msg::PersonInfoArray();
        person_info_array.header = header;

        for (const auto& [id, person] : tracked_persons_) {
            auto person_info = person_speed_tracker::msg::PersonInfo();
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

    // ROS2 components
    rclcpp::Subscription<gb_visual_detection_3d_msgs::msg::BoundingBoxes3d>::SharedPtr bbox_sub_;
    rclcpp::Publisher<person_speed_tracker::msg::PersonInfoArray>::SharedPtr person_info_pub_;

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
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PersonSpeedTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
