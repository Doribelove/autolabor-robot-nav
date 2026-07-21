uint8 GEOMETRY_BALANCED=0
uint8 GEOMETRY_CRUISE=1
uint8 GEOMETRY_STATIC_DENSE=2
uint8 GEOMETRY_CORRIDOR=3
uint8 GEOMETRY_MANEUVER=4

uint8 DYNAMIC_NONE=0
uint8 DYNAMIC_CROSSING=1
uint8 DYNAMIC_HEAD_ON=2
uint8 DYNAMIC_FOLLOW=3
uint8 DYNAMIC_OVERTAKE_OR_YIELD=4

uint8 TRANSITION_STABLE=0
uint8 TRANSITION_ENTERING=1
uint8 TRANSITION_EXITING=2
uint8 TRANSITION_HOLDING=3
uint8 TRANSITION_SAFE_OVERRIDE=4
uint8 TRANSITION_FAULTED=5

std_msgs/Header header
string schema_version
uint64 world_model_seq
uint64 mode_seq
uint8 geometry_mode
uint8 dynamic_overlay
uint8 transition_state
float32 mode_confidence
duration mode_dwell
duration minimum_dwell_remaining
bool valid
string reason
