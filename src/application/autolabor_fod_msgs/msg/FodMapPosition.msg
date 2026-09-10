# Current visible foreground surface representative point, metres, in array.header.frame_id.
# X/Y are planar map coordinates. Z is unused (NaN), not an estimated height.
uint64 object_id
uint32 track_id
string material_class
sensor_msgs/RegionOfInterest bbox
bool position_valid
geometry_msgs/Point position
string status
# Same current foreground point in the MID360-origin frame; independent of map localization.
# Vehicle axes: X forward, Y left. local_frame="mid360"; local_position.z is unused (NaN).
bool local_position_valid
geometry_msgs/Point local_position
string local_frame
string local_status
float32 depth_m
float32 depth_mad_m
uint32 depth_sample_count
# Full 3D observation in the current FAST-LIO session; display/control fields above unchanged.
bool odom_position_valid
geometry_msgs/Point odom_position
