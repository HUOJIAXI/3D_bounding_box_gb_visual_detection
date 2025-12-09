/**
 * @file person_tracker_visualizer.cpp
 * @brief ROS2 node for visualizing person tracking data in RViz2
 *
 * This node subscribes to person_info and human_clusters topics and publishes
 * visualization markers for RViz2.
 */

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "person_tracker/msg/person_info_array.hpp"
#include "person_tracker/msg/human_cluster_array.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <set>

class PersonTrackerVisualizer : public rclcpp::Node {
public:
    PersonTrackerVisualizer() : Node("person_tracker_visualizer") {
        // Declare parameters
        this->declare_parameter("person_marker_scale", 0.5);
        this->declare_parameter("velocity_arrow_scale", 0.5);
        this->declare_parameter("cluster_marker_scale", 0.8);
        this->declare_parameter("show_velocity_arrows", true);
        this->declare_parameter("show_speed_text", true);
        this->declare_parameter("show_person_ids", true);

        person_marker_scale_ = this->get_parameter("person_marker_scale").as_double();
        velocity_arrow_scale_ = this->get_parameter("velocity_arrow_scale").as_double();
        cluster_marker_scale_ = this->get_parameter("cluster_marker_scale").as_double();
        show_velocity_arrows_ = this->get_parameter("show_velocity_arrows").as_bool();
        show_speed_text_ = this->get_parameter("show_speed_text").as_bool();
        show_person_ids_ = this->get_parameter("show_person_ids").as_bool();

        // Create subscribers
        person_info_sub_ = this->create_subscription<person_tracker::msg::PersonInfoArray>(
            "/person_tracker/person_info", 10,
            std::bind(&PersonTrackerVisualizer::personInfoCallback, this, std::placeholders::_1));

        cluster_sub_ = this->create_subscription<person_tracker::msg::HumanClusterArray>(
            "/person_tracker/human_clusters", 10,
            std::bind(&PersonTrackerVisualizer::clusterCallback, this, std::placeholders::_1));

        // Create publishers
        person_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/person_tracker/person_markers", 10);

        cluster_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/person_tracker/cluster_markers", 10);

        RCLCPP_INFO(this->get_logger(), "Person Tracker Visualizer initialized");
        RCLCPP_INFO(this->get_logger(), "  - Show velocity arrows: %s", show_velocity_arrows_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  - Show speed text: %s", show_speed_text_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  - Show person IDs: %s", show_person_ids_ ? "true" : "false");
    }

private:
    /**
     * @brief Create a sphere marker for person position
     */
    visualization_msgs::msg::Marker createPersonMarker(
        const person_tracker::msg::PersonInfo& person,
        const std_msgs::msg::Header& header, int marker_id) {

        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "persons";
        marker.id = marker_id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position = person.position;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = person_marker_scale_;
        marker.scale.y = person_marker_scale_;
        marker.scale.z = person_marker_scale_;

        // Color based on speed (blue = stationary, red = moving fast)
        double speed_normalized = std::min(person.speed / 2.0, 1.0);  // Normalize to 0-1 (assuming max 2 m/s)
        marker.color.r = speed_normalized;
        marker.color.g = 0.3;
        marker.color.b = 1.0 - speed_normalized;
        marker.color.a = 0.8;

        marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        return marker;
    }

    /**
     * @brief Create a text marker for person ID
     */
    visualization_msgs::msg::Marker createPersonIDMarker(
        const person_tracker::msg::PersonInfo& person,
        const std_msgs::msg::Header& header, int marker_id) {

        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "person_ids";
        marker.id = marker_id;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position = person.position;
        marker.pose.position.z += 0.5;  // Offset above person
        marker.pose.orientation.w = 1.0;

        marker.scale.z = 0.3;  // Text height

        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker.text = "ID:" + std::to_string(person.person_id);
        marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        return marker;
    }

    /**
     * @brief Create a text marker for speed display
     */
    visualization_msgs::msg::Marker createSpeedMarker(
        const person_tracker::msg::PersonInfo& person,
        const std_msgs::msg::Header& header, int marker_id) {

        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "speed_text";
        marker.id = marker_id;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position = person.position;
        marker.pose.position.z += 0.8;  // Offset above ID
        marker.pose.orientation.w = 1.0;

        marker.scale.z = 0.25;  // Text height

        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        char speed_str[32];
        snprintf(speed_str, sizeof(speed_str), "%.2f m/s", person.speed);
        marker.text = speed_str;
        marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        return marker;
    }

    /**
     * @brief Create an arrow marker for velocity vector
     */
    visualization_msgs::msg::Marker createVelocityMarker(
        const person_tracker::msg::PersonInfo& person,
        const std_msgs::msg::Header& header, int marker_id) {

        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "velocity_arrows";
        marker.id = marker_id;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Arrow from person position in direction of velocity
        geometry_msgs::msg::Point start = person.position;
        geometry_msgs::msg::Point end;
        end.x = start.x + person.velocity.x * velocity_arrow_scale_;
        end.y = start.y + person.velocity.y * velocity_arrow_scale_;
        end.z = start.z + person.velocity.z * velocity_arrow_scale_;

        marker.points.push_back(start);
        marker.points.push_back(end);

        marker.scale.x = 0.1;  // Arrow shaft diameter
        marker.scale.y = 0.2;  // Arrow head diameter
        marker.scale.z = 0.0;  // Not used for arrow type

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;

        marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        return marker;
    }

    /**
     * @brief Callback for person info messages
     */
    void personInfoCallback(const person_tracker::msg::PersonInfoArray::SharedPtr msg) {
        visualization_msgs::msg::MarkerArray marker_array;

        // Track current person IDs
        std::set<int32_t> current_person_ids;

        for (const auto& person : msg->persons) {
            current_person_ids.insert(person.person_id);

            // Use person_id as base for marker IDs to ensure consistency
            int base_id = person.person_id * 10;

            // Person position marker
            marker_array.markers.push_back(createPersonMarker(person, msg->header, base_id + 0));

            // Person ID text
            if (show_person_ids_) {
                marker_array.markers.push_back(createPersonIDMarker(person, msg->header, base_id + 1));
            }

            // Speed text
            if (show_speed_text_ && person.speed > 0.01) {
                marker_array.markers.push_back(createSpeedMarker(person, msg->header, base_id + 2));
            }

            // Velocity arrow
            if (show_velocity_arrows_ && person.speed > 0.01) {
                marker_array.markers.push_back(createVelocityMarker(person, msg->header, base_id + 3));
            }
        }

        // Delete markers for persons that are no longer present
        for (const auto& prev_id : previous_person_ids_) {
            if (current_person_ids.find(prev_id) == current_person_ids.end()) {
                // Person disappeared, delete all its markers
                int base_id = prev_id * 10;
                for (int i = 0; i < 4; ++i) {
                    visualization_msgs::msg::Marker delete_marker;
                    delete_marker.header = msg->header;
                    delete_marker.id = base_id + i;
                    delete_marker.action = visualization_msgs::msg::Marker::DELETE;

                    // Delete from all namespaces
                    if (i == 0) delete_marker.ns = "persons";
                    else if (i == 1) delete_marker.ns = "person_ids";
                    else if (i == 2) delete_marker.ns = "speed_text";
                    else if (i == 3) delete_marker.ns = "velocity_arrows";

                    marker_array.markers.push_back(delete_marker);
                }
            }
        }

        // Update tracking
        previous_person_ids_ = current_person_ids;

        person_markers_pub_->publish(marker_array);

        RCLCPP_DEBUG(this->get_logger(), "Published %zu person markers", marker_array.markers.size());
    }

    /**
     * @brief Create a cylinder marker for cluster centroid
     */
    visualization_msgs::msg::Marker createClusterMarker(
        const person_tracker::msg::HumanCluster& cluster,
        const std_msgs::msg::Header& header, int marker_id) {

        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "clusters";
        marker.id = marker_id;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position = cluster.centroid;
        marker.pose.position.z = 0.5;  // Raise cylinder to ground level
        marker.pose.orientation.w = 1.0;

        marker.scale.x = cluster_marker_scale_;
        marker.scale.y = cluster_marker_scale_;
        marker.scale.z = 1.0;  // Height

        // Color based on cluster size
        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
        marker.color.a = 0.4;

        marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        return marker;
    }

    /**
     * @brief Create a text marker for cluster info
     */
    visualization_msgs::msg::Marker createClusterTextMarker(
        const person_tracker::msg::HumanCluster& cluster,
        const std_msgs::msg::Header& header, int marker_id) {

        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "cluster_text";
        marker.id = marker_id;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position = cluster.centroid;
        marker.pose.position.z = 1.2;  // Above cluster
        marker.pose.orientation.w = 1.0;

        marker.scale.z = 0.3;  // Text height

        marker.color.r = 1.0;
        marker.color.g = 0.8;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        std::string member_ids_str;
        for (size_t i = 0; i < cluster.member_ids.size(); ++i) {
            if (i > 0) member_ids_str += ",";
            member_ids_str += std::to_string(cluster.member_ids[i]);
        }

        marker.text = "Cluster " + std::to_string(cluster.cluster_id) +
                      "\nSize: " + std::to_string(cluster.cluster_size) +
                      "\nIDs: [" + member_ids_str + "]";
        marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        return marker;
    }

    /**
     * @brief Callback for cluster messages
     */
    void clusterCallback(const person_tracker::msg::HumanClusterArray::SharedPtr msg) {
        visualization_msgs::msg::MarkerArray marker_array;

        // Track current cluster IDs
        std::set<int32_t> current_cluster_ids;

        for (const auto& cluster : msg->clusters) {
            current_cluster_ids.insert(cluster.cluster_id);

            // Use cluster_id as base for marker IDs
            int base_id = cluster.cluster_id * 10;

            // Cluster centroid marker
            marker_array.markers.push_back(createClusterMarker(cluster, msg->header, base_id + 0));

            // Cluster info text
            marker_array.markers.push_back(createClusterTextMarker(cluster, msg->header, base_id + 1));
        }

        // Delete markers for clusters that are no longer present
        for (const auto& prev_id : previous_cluster_ids_) {
            if (current_cluster_ids.find(prev_id) == current_cluster_ids.end()) {
                // Cluster disappeared, delete its markers
                int base_id = prev_id * 10;
                for (int i = 0; i < 2; ++i) {
                    visualization_msgs::msg::Marker delete_marker;
                    delete_marker.header = msg->header;
                    delete_marker.id = base_id + i;
                    delete_marker.action = visualization_msgs::msg::Marker::DELETE;

                    if (i == 0) delete_marker.ns = "clusters";
                    else if (i == 1) delete_marker.ns = "cluster_text";

                    marker_array.markers.push_back(delete_marker);
                }
            }
        }

        // Update tracking
        previous_cluster_ids_ = current_cluster_ids;

        cluster_markers_pub_->publish(marker_array);

        RCLCPP_DEBUG(this->get_logger(), "Published %zu cluster markers for %d clusters",
                     marker_array.markers.size(), msg->num_clusters);
    }

    // ROS2 components
    rclcpp::Subscription<person_tracker::msg::PersonInfoArray>::SharedPtr person_info_sub_;
    rclcpp::Subscription<person_tracker::msg::HumanClusterArray>::SharedPtr cluster_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr person_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_markers_pub_;

    // Parameters
    double person_marker_scale_;
    double velocity_arrow_scale_;
    double cluster_marker_scale_;
    bool show_velocity_arrows_;
    bool show_speed_text_;
    bool show_person_ids_;

    // Track person and cluster IDs for proper marker deletion
    std::set<int32_t> previous_person_ids_;
    std::set<int32_t> previous_cluster_ids_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PersonTrackerVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
