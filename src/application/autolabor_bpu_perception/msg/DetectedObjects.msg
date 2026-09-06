# header.stamp is the ORIGINAL sensor observation time, never inference completion.
# All metric values use header.frame_id. source_frame records the sensor frame.
std_msgs/Header header
string source_frame
string session_id
string model_name
string model_sha256
bool valid
bool calibrated
# Reference models remain ineligible for automatic garbage collection.
bool motion_eligible
float32 latency_ms
string detail
autolabor_bpu_perception/DetectedObject[] objects
