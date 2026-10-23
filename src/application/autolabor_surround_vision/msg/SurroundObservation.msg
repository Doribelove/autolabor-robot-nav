string observation_id
string position_source
int32 detector_class_id
string material
float32 detect_confidence
float32 classify_confidence
bool material_valid
float32[5] material_probabilities
geometry_msgs/Point point_map
float64[4] covariance_xy
autolabor_surround_vision/SurroundSource source
# Always false: monocular plane estimates are not measured stereo depth.
bool depth_valid
