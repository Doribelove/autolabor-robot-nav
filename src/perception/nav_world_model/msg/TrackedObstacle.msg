uint8 MOTION_UNKNOWN=0
uint8 MOTION_STATIONARY=1
uint8 MOTION_CROSSING=2
uint8 MOTION_HEAD_ON=3
uint8 MOTION_FOLLOWING=4
uint8 MOTION_DEPARTING=5

uint32 track_id
uint8 motion_class
geometry_msgs/Polygon footprint
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance velocity
float32 confidence
duration age
uint32 miss_count
time last_update
nav_world_model/PredictedState[] predictions
