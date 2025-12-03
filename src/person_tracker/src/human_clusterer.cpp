/**
 * @file human_clusterer.cpp
 * @brief Implementation of non-learning, geometry-based clustering for pedestrians
 */

#include "person_tracker/human_clusterer.hpp"
#include <algorithm>
#include <cmath>

namespace person_tracker
{

ClusteringResult HumanClusterer::cluster(const std::vector<HumanDetection>& detections)
{
    ClusteringResult result;

    // Handle empty input
    if (detections.empty()) {
        return result;
    }

    // Build a map for quick lookup of detections by ID
    std::map<int, HumanDetection> detections_map;
    for (const auto& det : detections) {
        detections_map[det.id] = det;
    }

    // Step 1: Construct undirected graph
    // adjacency_list[id] = list of IDs connected to this ID
    std::map<int, std::vector<int>> adjacency_list;

    // Initialize adjacency list
    for (const auto& det : detections) {
        adjacency_list[det.id] = std::vector<int>();
    }

    // Add edges between humans within distance_threshold
    for (size_t i = 0; i < detections.size(); ++i) {
        for (size_t j = i + 1; j < detections.size(); ++j) {
            double dist = calculateGroundPlaneDistance(detections[i], detections[j]);
            if (dist < distance_threshold_) {
                // Add edge in both directions (undirected graph)
                adjacency_list[detections[i].id].push_back(detections[j].id);
                adjacency_list[detections[j].id].push_back(detections[i].id);
            }
        }
    }

    // Step 2: Find connected components using DFS
    std::set<int> visited;
    int cluster_id_counter = 0;

    for (const auto& det : detections) {
        if (visited.find(det.id) == visited.end()) {
            // Start a new cluster (connected component)
            std::vector<int> component;
            dfs(det.id, adjacency_list, visited, component);

            // Create cluster
            Cluster cluster;
            cluster.cluster_id = cluster_id_counter++;
            cluster.member_ids = component;

            // Calculate centroid
            calculateCentroid(component, detections_map, cluster.centroid_x, cluster.centroid_y);

            // Add to results
            result.clusters.push_back(cluster);

            // Update ID-to-cluster mapping
            for (int member_id : component) {
                result.id_to_cluster[member_id] = cluster.cluster_id;
            }
        }
    }

    return result;
}

double HumanClusterer::calculateGroundPlaneDistance(
    const HumanDetection& det1, const HumanDetection& det2) const
{
    // Project to ground plane: use (x, y) and ignore z
    double dx = det1.x - det2.x;
    double dy = det1.y - det2.y;
    return std::sqrt(dx * dx + dy * dy);
}

void HumanClusterer::dfs(
    int human_id,
    const std::map<int, std::vector<int>>& adjacency_list,
    std::set<int>& visited,
    std::vector<int>& component)
{
    // Mark as visited
    visited.insert(human_id);
    component.push_back(human_id);

    // Visit all neighbors
    auto it = adjacency_list.find(human_id);
    if (it != adjacency_list.end()) {
        for (int neighbor_id : it->second) {
            if (visited.find(neighbor_id) == visited.end()) {
                dfs(neighbor_id, adjacency_list, visited, component);
            }
        }
    }
}

void HumanClusterer::calculateCentroid(
    const std::vector<int>& member_ids,
    const std::map<int, HumanDetection>& detections_map,
    double& centroid_x,
    double& centroid_y) const
{
    centroid_x = 0.0;
    centroid_y = 0.0;

    if (member_ids.empty()) {
        return;
    }

    // Calculate average position on ground plane
    for (int id : member_ids) {
        auto it = detections_map.find(id);
        if (it != detections_map.end()) {
            centroid_x += it->second.x;
            centroid_y += it->second.y;
        }
    }

    centroid_x /= static_cast<double>(member_ids.size());
    centroid_y /= static_cast<double>(member_ids.size());
}

}  // namespace person_tracker
