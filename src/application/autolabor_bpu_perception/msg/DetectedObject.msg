uint64 object_id
int32 class_id
string class_name
float32 confidence
# Source image coordinates; zeros for LiDAR detections.
float32[4] bbox
bool position_valid
geometry_msgs/Point position
bool dimensions_valid
geometry_msgs/Vector3 dimensions
float32 yaw
bool velocity_valid
geometry_msgs/Vector3 velocity
float32 depth_m
float32 depth_mad_m
uint32 depth_samples
