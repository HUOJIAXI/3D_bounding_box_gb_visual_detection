# Represents a cluster of persons (people walking/standing together)

# Unique cluster ID
int32 cluster_id

# Array of person IDs that belong to this cluster
int32[] member_ids

# Number of members in this cluster
int32 cluster_size

# Centroid position of the cluster on the ground plane (x, y, z=0)
geometry_msgs/Point centroid
