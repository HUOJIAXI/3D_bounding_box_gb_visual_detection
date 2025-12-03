# Person tracking information message

# Unique tracking ID for this person
int32 person_id

# 3D Bounding Box information
gb_visual_detection_3d_msgs/BoundingBox3d bounding_box

# Person's center position in 3D space (in camera/world frame)
geometry_msgs/Point position

# Walking speed vector (velocity in m/s)
geometry_msgs/Vector3 velocity

# Speed magnitude (m/s)
float64 speed

# Confidence score (0.0 to 1.0)
float64 confidence

# Number of consecutive frames this person has been tracked
int32 tracking_duration

# Time since last update (for filtering stale tracks)
builtin_interfaces/Time last_seen
