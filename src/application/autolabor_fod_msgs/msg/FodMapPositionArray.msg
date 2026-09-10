# Observation only. No cached coordinates and no motion authorization.
std_msgs/Header header
std_msgs/Header source_header
string backend_id
string calibration_sha256
string status
FodMapPosition[] positions
# NVIDIA observation-only landmark context; never a navigation transform or command.
string odom_session_id
string odom_frame
bool map_stable
string stability_status
uint64 correction_generation
# Row-major rigid 4x4 map<-camera_init estimate, valid only when map_stable.
float64[16] stable_map_from_odom
