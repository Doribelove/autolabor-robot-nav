int32 class_id
string class_name
float32 confidence
sensor_msgs/RegionOfInterest bbox
geometry_msgs/Point32 anchor_px
geometry_msgs/Polygon mask_px
# Current-frame identity and registered stereo depth. Zero ID means untracked.
# Invalid/missing depth leaves both distances zero; never interpret it as near.
uint32 track_id
bool depth_valid
float32 depth_m
float32 range_m
