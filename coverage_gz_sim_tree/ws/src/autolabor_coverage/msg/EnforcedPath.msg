uint8 MODE_ORDINARY_NAVFN=0
uint8 MODE_COVERAGE_NAVFN=1
uint8 MODE_HYBRID_TRANSIT=2
uint8 MODE_ENFORCED_SWEEP=3

int8 GEAR_REVERSE=-1
int8 GEAR_AUTO=0
int8 GEAR_FORWARD=1

std_msgs/Header header
string plan_id
uint32 segment_index
uint32 path_generation
uint8 planner_mode
int8 expected_gear
bool active
nav_msgs/Path path
