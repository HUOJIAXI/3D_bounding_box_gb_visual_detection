/**
 * @file human_clusterer.hpp
 * @brief Non-learning, geometry-based clustering for grouping pedestrians
 *
 * This module implements distance-based clustering on the ground plane to identify
 * groups of people standing or walking together. It uses connected components in
 * an undirected graph where edges connect people within a distance threshold.
 */

#ifndef PERSON_TRACKER__HUMAN_CLUSTERER_HPP_
#define PERSON_TRACKER__HUMAN_CLUSTERER_HPP_

#include <vector>
#include <map>
#include <set>
#include <cmath>

namespace person_tracker
{

/**
 * @brief Structure representing a detected human with ID and 3D position
 */
struct HumanDetection {
    int id;          // Unique tracking ID
    double x;        // 3D position x
    double y;        // 3D position y
    double z;        // 3D position z (typically ignored for ground plane clustering)
};

/**
 * @brief Structure representing a cluster of humans
 */
struct Cluster {
    int cluster_id;               // Unique cluster ID
    std::vector<int> member_ids;  // IDs of humans in this cluster
    double centroid_x;            // Cluster centroid x on ground plane
    double centroid_y;            // Cluster centroid y on ground plane
};

/**
 * @brief Result of clustering operation
 */
struct ClusteringResult {
    std::vector<Cluster> clusters;           // List of all clusters
    std::map<int, int> id_to_cluster;        // Mapping from human ID to cluster ID
};

/**
 * @brief Non-learning human clustering using distance-based graph connectivity
 *
 * This class provides a simple, geometry-based clustering algorithm for grouping
 * pedestrians. It projects 3D positions to the ground plane and constructs an
 * undirected graph where edges connect people within a distance threshold.
 * Clusters are then computed as connected components of this graph.
 */
class HumanClusterer
{
public:
    /**
     * @brief Constructor with default distance threshold
     * @param distance_threshold Maximum distance (in meters) between two humans to be in same cluster
     */
    explicit HumanClusterer(double distance_threshold = 1.2)
    : distance_threshold_(distance_threshold) {}

    /**
     * @brief Set the distance threshold for clustering
     * @param threshold Distance threshold in meters
     */
    void setDistanceThreshold(double threshold) {
        distance_threshold_ = threshold;
    }

    /**
     * @brief Get the current distance threshold
     * @return Distance threshold in meters
     */
    double getDistanceThreshold() const {
        return distance_threshold_;
    }

    /**
     * @brief Perform clustering on a list of human detections
     *
     * This is the main clustering function. It:
     * 1. Projects 3D positions to ground plane (x, y)
     * 2. Constructs an undirected graph with edges between humans within distance_threshold
     * 3. Computes connected components (clusters)
     * 4. Calculates cluster centroids
     *
     * @param detections Vector of human detections with IDs and 3D positions
     * @return ClusteringResult containing clusters and ID-to-cluster mapping
     */
    ClusteringResult cluster(const std::vector<HumanDetection>& detections);

private:
    /**
     * @brief Calculate Euclidean distance on the ground plane (x, y only)
     * @param det1 First detection
     * @param det2 Second detection
     * @return Distance in meters
     */
    double calculateGroundPlaneDistance(const HumanDetection& det1, const HumanDetection& det2) const;

    /**
     * @brief Depth-first search for finding connected components
     * @param human_id Current human ID
     * @param adjacency_list Graph adjacency list
     * @param visited Set of visited human IDs
     * @param component Current connected component being built
     */
    void dfs(int human_id,
             const std::map<int, std::vector<int>>& adjacency_list,
             std::set<int>& visited,
             std::vector<int>& component);

    /**
     * @brief Calculate centroid of a cluster on the ground plane
     * @param member_ids IDs of cluster members
     * @param detections_map Map from human ID to detection
     * @param centroid_x Output: centroid x coordinate
     * @param centroid_y Output: centroid y coordinate
     */
    void calculateCentroid(const std::vector<int>& member_ids,
                          const std::map<int, HumanDetection>& detections_map,
                          double& centroid_x,
                          double& centroid_y) const;

    double distance_threshold_;  // Distance threshold for clustering (meters)
};

}  // namespace person_tracker

#endif  // PERSON_TRACKER__HUMAN_CLUSTERER_HPP_
