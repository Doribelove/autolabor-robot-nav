uint8 OUTCOME_SUCCESS=0
uint8 OUTCOME_TIMEOUT=1
uint8 OUTCOME_NO_PATH=2
uint8 OUTCOME_INVALID=3

uint32 candidate_index
uint32 transition_index
uint8 outcome
nav_msgs/Path path
bool reaches_final_goal
geometry_msgs/PoseStamped planned_goal
uint64 expansions
float64 estimated_time_sec
float64 reverse_distance_m
uint32 direction_changes
string reason
