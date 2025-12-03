# Human Clustering Module - Implementation Summary

## Overview
A non-learning, distance-based clustering module has been successfully added to the `person_tracker` package. This module groups pedestrians based on their proximity on the ground plane using connected component analysis.

## What Was Implemented

### 1. **New Message Types**
Created custom ROS2 messages for cluster output:

- **`HumanCluster.msg`** (person_tracker/msg/HumanCluster.msg:1)
  - `int32 cluster_id` - Unique cluster identifier
  - `int32[] member_ids` - Array of person tracking IDs in this cluster
  - `int32 cluster_size` - Number of members
  - `geometry_msgs/Point centroid` - Cluster centroid on ground plane

- **`HumanClusterArray.msg`** (person_tracker/msg/HumanClusterArray.msg:1)
  - `std_msgs/Header header` - Timestamp and frame info
  - `HumanCluster[] clusters` - Array of all clusters
  - `int32 num_clusters` - Total number of clusters

### 2. **HumanClusterer Class** (Pure C++, No ROS Dependencies)

**Header:** `include/person_tracker/human_clusterer.hpp`
**Source:** `src/human_clusterer.cpp`

**Key Features:**
- **Algorithm:** Distance-based graph connectivity with connected components
- **Input:** List of human detections with (id, x, y, z)
- **Output:** Clusters and ID-to-cluster mapping
- **Method:**
  1. Projects 3D positions to ground plane (x, y)
  2. Constructs undirected graph with edges between humans within `distance_threshold`
  3. Computes connected components using depth-first search (DFS)
  4. Calculates cluster centroids

**API:**
```cpp
HumanClusterer clusterer(1.2);  // 1.2 meter threshold
clusterer.setDistanceThreshold(1.5);  // Change threshold
ClusteringResult result = clusterer.cluster(detections);
```

### 3. **Integration into person_tracker_node**

Modified `src/person_tracker_node.cpp`:
- Added clustering module instance
- Added ROS2 parameters:
  - `cluster_distance_threshold` (double, default: 1.2m)
  - `publish_singletons` (bool, default: true)
- Added publisher for `/person_tracker/human_clusters` topic
- Integrated clustering into the main callback pipeline (person_tracker_node.cpp:318)
- Added detailed logging for cluster information (person_tracker_node.cpp:394)

### 4. **Configuration File**

Created `config/clustering_params.yaml`:
- All tracking parameters
- Clustering distance threshold
- Singleton publication control

### 5. **Updated Build System**

Modified `CMakeLists.txt`:
- Added new message generation for HumanCluster messages
- Created `human_clusterer` library
- Linked library to main node
- Installed headers and library

## Architecture

```
[Camera/Point Cloud]
    |
    v
[Darknet3D Node] -> publishes 3D bounding boxes
    |
    v
[PersonTrackerNode]
    |-- Tracking: Assigns unique IDs to people
    |-- Clustering: Groups people by proximity
    |
    v
[Outputs]
    |-- /person_tracker/person_info (PersonInfoArray)
    |-- /person_tracker/human_clusters (HumanClusterArray) **NEW**
```

## Key Design Decisions

1. **Modular Design:** `HumanClusterer` is a standalone C++ class with no ROS dependencies, making it easy to replace with a learning-based method later.

2. **Ground Plane Clustering:** Uses only (x, y) coordinates, ignoring z-axis for pedestrian grouping.

3. **Connected Components:** Groups are formed by graph connectivity, not k-means or other centroid-based methods. This allows for:
   - Variable cluster sizes
   - No need to specify number of clusters
   - Natural handling of isolated individuals

4. **Integration Point:** Added to `person_tracker_node` (not `darknet3d_node`) because:
   - This node already has tracking IDs
   - Avoids additional message passing latency
   - Keeps all tracking/clustering logic together

## ROS2 Topics

### New Output Topic
- **Topic:** `/person_tracker/human_clusters`
- **Type:** `person_tracker/msg/HumanClusterArray`
- **Rate:** Same as detection rate (~10 Hz typically)

### Existing Topics Used
- **Input:** `/darknet_ros_3d/bounding_boxes` (BoundingBoxes3d)
- **Output:** `/person_tracker/person_info` (PersonInfoArray) - unchanged

## Parameters

### Clustering Parameters (New)
```yaml
cluster_distance_threshold: 1.2  # meters - max distance to be in same cluster
publish_singletons: true         # publish clusters of size 1 (isolated people)
```

### Tracking Parameters (Existing)
```yaml
max_tracking_distance: 1.5       # meters
max_missing_frames: 10
velocity_history_size: 5
position_history_size: 10
min_speed_threshold: 0.05        # m/s
max_speed_threshold: 3.0         # m/s
smoothing_alpha: 0.3
```

## Usage

### 1. Build the Package
```bash
cd /home/huojiaxi/3D_detector/3D_bounding_box_gb_visual_detection
colcon build --packages-select person_tracker
source install/setup.bash
```

### 2. Launch with Default Parameters
```bash
ros2 launch person_tracker person_tracker.launch.py
```

### 3. Launch with Custom Parameters
```bash
ros2 launch person_tracker person_tracker.launch.py \
  config_file:=/path/to/your/params.yaml
```

### 4. Change Parameters at Runtime
```bash
# Change clustering threshold
ros2 param set /person_tracker cluster_distance_threshold 2.0

# Toggle singleton publishing
ros2 param set /person_tracker publish_singletons false
```

### 5. View Cluster Output
```bash
ros2 topic echo /person_tracker/human_clusters
```

### 6. Monitor Logs
```bash
ros2 run person_tracker person_tracker_node --ros-args --log-level info
```

## Example Output

When 3 people are detected (2 close together, 1 isolated):

```
[INFO] [person_tracker]: Found 2 clusters from 3 tracked persons
[DEBUG] [person_tracker]:   Cluster 0: size=2, members=[0, 1], centroid=(1.50, 2.30)
[DEBUG] [person_tracker]:   Cluster 1: size=1, members=[2], centroid=(5.20, 1.80)
```

## Code Quality Features

- **Separation of Concerns:** ROS-specific code separated from clustering logic
- **Detailed Logging:** INFO level for cluster counts, DEBUG level for details
- **Configurable:** All thresholds exposed as ROS2 parameters
- **Efficient:** O(N²) edge construction, O(N) DFS for connected components
- **Clean API:** Easy to replace with learning-based clustering later

## Future Improvements (Suggestions)

1. **Learning-Based Clustering:**
   - Replace `HumanClusterer` with a neural network-based method
   - Keep the same interface for easy drop-in replacement
   - Consider features: position, velocity, appearance, pose

2. **Temporal Consistency:**
   - Add cluster ID tracking across frames
   - Smooth cluster membership changes

3. **Visualization:**
   - Add RViz markers for cluster visualization
   - Color-code people by cluster

4. **Advanced Clustering:**
   - Consider velocity alignment (people moving in same direction)
   - Add social force models
   - Use F-formations for group detection

## Files Modified/Created

### New Files
- `src/person_tracker/msg/HumanCluster.msg`
- `src/person_tracker/msg/HumanClusterArray.msg`
- `src/person_tracker/include/person_tracker/human_clusterer.hpp`
- `src/person_tracker/src/human_clusterer.cpp`
- `src/person_tracker/config/clustering_params.yaml`

### Modified Files
- `src/person_tracker/src/person_tracker_node.cpp` (renamed from person_speed_tracker_node.cpp)
- `src/person_tracker/CMakeLists.txt`
- `src/person_tracker/package.xml`
- `src/person_tracker/launch/person_tracker.launch.py` (renamed from person_speed_tracker.launch.py)

## Build Status
✅ Successfully built with `colcon build --packages-select person_tracker`

---

**Implementation Date:** 2025-12-02
**Package:** person_tracker (ROS2 Humble) - Renamed from person_speed_tracker
**Language:** C++
**Build System:** colcon + ament_cmake
