# Current GPS Development Handoff

Date: 2026-07-09

This file records the current GPS navigation development state for the Autolabor M2 robot in `/home/robot/robot_ws_base_rl`.

## Current Goal

The current development focus is GPS-mode outdoor navigation for an Autolabor M2 Ackermann robot in a mostly open airport-like environment.

The desired behavior is:

- Receive GPS targets and drive to them reliably.
- Prefer straight, efficient motion in open areas.
- Keep enough local-planner maneuverability to avoid obstacles and recover from blocked paths.
- Use dual-antenna GNSS heading as the vehicle yaw source.
- Use a test task menu for GPS target, electronic fence, and obstacle-avoidance testing.

## Startup

GPS navigation:

```bash
cd /home/robot/robot_ws_base_rl
./scripts/bringup.sh gps
```

RabbitMQ GPS target bridge:

```bash
cd /home/robot/robot_ws_base_rl
source /opt/ros/noetic/setup.bash
source /home/robot/robot_ws_base_rl/devel/setup.bash
./scripts/rabbitmq_gps_goal_bridge.py
```

GPS test task menu:

```bash
cd /home/robot/robot_ws_base_rl
source /opt/ros/noetic/setup.bash
source /home/robot/robot_ws_base_rl/devel/setup.bash
./scripts/gps_test_tasks.py
```

## GPS Mode Chain

```text
Main GNSS GGA position + dual-antenna UNIHEADINGA heading
  -> gps_localization_node.py
  -> /gps/fix
  -> /gps/heading
  -> /gps/odom
  -> camera_init -> base_link TF
  -> move_base + TEB
  -> /cmd_vel
  -> m2_driver
```

GPS targets:

```text
RabbitMQ or test task
  -> /gps/goal_fix
  -> gps_goal_node.py
  -> /move_base_simple/goal
  -> move_base + TEB
```

## Current GPS Defaults

`scripts/bringup.sh gps` currently defaults to:

```text
GPS_PORT=/dev/ttyUSB1
GPS_BAUD_RATE=115200
GPS_HEADING_SOURCE=dual_antenna
GPS_HEADING_REQUIRED_SOLUTION_STATUS=SOL_COMPUTED
GPS_HEADING_REQUIRED_POSITION_TYPES=NARROW_INT
GPS_ANTENNA_OFFSET_X=-0.3
GPS_ANTENNA_OFFSET_Y=0.0
GPS_USE_WHEEL_ODOM=false
```

The main GNSS antenna is treated as mounted at `x=-0.3m`, `y=0.0m` in `base_link`. The localization node compensates this offset so `/gps/odom` represents the chassis center.

## GPS Goal Conversion

File:

```text
src/application/gps_module/scripts/gps_goal_node.py
```

Current behavior:

- Subscribes `/gps/goal_fix`.
- Publishes `/move_base_simple/goal`.
- Subscribes current odom, usually `/gps/odom` in GPS mode.
- `goal_yaw_mode=bearing` by default.
- Converted goal yaw is the bearing from current odom position to target, not fixed yaw 0.
- Publisher is no longer latched, to avoid old GPS goals reappearing and overriding RViz goals.

Current `bringup.sh gps` launches it with:

```text
frame_id:=camera_init
odom_topic:=/gps/odom
goal_yaw_mode:=bearing
```

RViz manual goals must use:

```text
Fixed Frame: camera_init
2D Nav Goal topic: /move_base_simple/goal
```

If a RViz goal appears to change unexpectedly, check:

```bash
rostopic info /move_base_simple/goal
rostopic echo /move_base_simple/goal
```

## GPS Test Task Menu

File:

```text
scripts/gps_test_tasks.py
```

Menu:

```text
1: Publish current vehicle-front 8m GPS target.
2: Save permanent 20m x 20m electronic fence around current pose.
3: Publish random GPS target within current front/back/left/right 10m area.
4: Show current fence.
5: Clear permanent fence.
q: Quit.
```

Fence file:

```text
/home/robot/robot_ws_base_rl/config/gps_test_fence.json
```

The fence is only active while `scripts/gps_test_tasks.py` is running. Normal `./scripts/bringup.sh gps` is not constrained by this fence.

RViz fence display:

```text
Fixed Frame: camera_init
Display: GPS Test Fence / MarkerArray
Topic: /gps/test_fence_markers
```

`arena_bringup/rviz/nav_LP.rviz` already includes this display, so the fence appears in the same RViz opened by `./scripts/bringup.sh gps` when `scripts/gps_test_tasks.py` publishes or loads a fence.

The test task no longer latches `/gps/goal_fix`, to avoid old test targets being resent.

## Current TEB / Ackermann Tuning

The robot is an Autolabor M2 Ackermann vehicle. Hardware minimum turning radius is currently provided by the user as:

```text
1.2 m
```

M2 driver behavior:

- `/cmd_vel.linear.x` is target speed.
- `/cmd_vel.angular.z` is interpreted by `m2_driver` as angular velocity.
- `m2_driver` converts angular velocity to steering angle internally.
- Therefore TEB must keep:

```text
cmd_angle_instead_rotvel=false
```

Current GPS/nomap TEB tuning file:

```text
src/navigation_arena/arena-rosnav-3D/arena_navigation/arena_local_planer/model_based/conventional/config/dingo/teb_local_planner_params_nomap.yaml
```

Current important parameters:

```text
cmd_angle_instead_rotvel: False
max_vel_theta: 1.5
acc_lim_theta: 0.5
min_turning_radius: 1.2
global_plan_viapoint_sep: 0.8
weight_shortest_path: 4.0
weight_viapoint: 8.0
enable_homotopy_class_planning: True
max_number_classes: 3
costmap_obstacles_behind_robot_dist: 0.8
```

Intent:

- Keep straight-line preference in open airport-like areas.
- Retain enough steering and path alternatives to avoid obstacles.
- Avoid the previous overly rigid behavior where the robot became slow and oscillated forward/backward near obstacles.

After restarting GPS navigation, verify:

```bash
rosparam get /move_base/TebLocalPlannerROS/cmd_angle_instead_rotvel
rosparam get /move_base/TebLocalPlannerROS/max_vel_theta
rosparam get /move_base/TebLocalPlannerROS/acc_lim_theta
rosparam get /move_base/TebLocalPlannerROS/min_turning_radius
rosparam get /move_base/TebLocalPlannerROS/global_plan_viapoint_sep
rosparam get /move_base/TebLocalPlannerROS/weight_shortest_path
rosparam get /move_base/TebLocalPlannerROS/weight_viapoint
rosparam get /move_base/TebLocalPlannerROS/enable_homotopy_class_planning
rosparam get /move_base/TebLocalPlannerROS/costmap_obstacles_behind_robot_dist
```

Expected:

```text
False
1.5
0.5
1.2
0.8
4.0
8.0
True
0.8
```

## Current Speed / Goal Tuning

GPS mode overrides TEB through launch args:

```text
max_vel_x=1.5
max_vel_x_backwards=1.0
xy_goal_tolerance=0.5
yaw_goal_tolerance=6.283
weight_kinematics_forward_drive=20.0
penalty_epsilon=0.03
```

This means GPS targets do not require a strict final orientation.

## Costmap / Perception State

Current intended rates:

```text
/scan: about 10 Hz
move_base controller_frequency: 10 Hz
local_costmap update_frequency: 10 Hz
```

Current local map / laser obstacle settings:

```text
local_costmap width: 20.0
local_costmap height: 20.0
obstacle_range: 10.0
raytrace_range: 11.0
scan range_max: 12.0
```

Current clearance:

```text
TebLocalPlannerROS/min_obstacle_dist: 0.3
```

## GPS Static Error Monitor

Files:

```text
src/tools/robot_diagnostics/scripts/gps_static_error_monitor.py
src/tools/robot_diagnostics/launch/gps_static_error_monitor.launch
```

Run after GPS navigation starts:

```bash
cd /home/robot/robot_ws_base_rl
source /opt/ros/noetic/setup.bash
source /home/robot/robot_ws_base_rl/devel/setup.bash
roslaunch robot_diagnostics gps_static_error_monitor.launch
```

Useful topics:

```text
/gps/static_error/summary
/gps/static_error/current
/gps/static_error/rms
/gps/static_error/max
```

User previously observed acceptable static drift, roughly 5cm RMS and 12cm max in one longer run.

## If Obstacle Recovery Still Fails

Collect:

```bash
rostopic echo /cmd_vel
rostopic echo /move_base/TebLocalPlannerROS/local_plan
rostopic echo /move_base/status
rostopic info /move_base_simple/goal
```

Also capture RViz with:

```text
local_costmap
global_costmap
local_plan
global_plan
/scan
/gps/test_fence_markers, if the test task is running
```

This will distinguish:

- TEB local optimum / oscillation.
- Hidden costmap obstacle.
- Wrong goal publisher.
- Chassis not executing `/cmd_vel` as expected.

## Verification Already Run

Recent checks passed:

```bash
python3 -m py_compile src/application/gps_module/scripts/gps_goal_node.py scripts/gps_test_tasks.py
python3 -c "import yaml; yaml.safe_load(open('src/navigation_arena/arena-rosnav-3D/arena_navigation/arena_local_planer/model_based/conventional/config/dingo/teb_local_planner_params_nomap.yaml'))"
bash -n scripts/bringup.sh
```

## Git / Working Tree Notes

There are many existing modified and untracked files in the workspace, including submodule dirty states. Do not infer all dirty files were created in the last step.

Generated ROS outputs and bag files should not be committed:

```text
build/
devel/
install/
log/
*.bag
*.bag.active
```

The runtime fence file is ignored:

```text
config/gps_test_fence.json
```

No git commit has been made for the latest GPS navigation tuning unless the user explicitly requests one.
