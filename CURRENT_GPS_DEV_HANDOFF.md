# Current GPS Development Handoff

Date: 2026-07-15

This file records the current GPS navigation development state for the Autolabor M2 robot in `/home/robot/robot_ws`.

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
cd /home/robot/robot_ws
./scripts/bringup.sh gps
```

GPS navigation maximum speed and TEB scene profile:

```bash
./scripts/bringup.sh gps 2.0 cruise
./scripts/bringup.sh gps 1.0 obstacle
```

The second positional argument overrides the GPS TEB forward `max_vel_x`; the third selects `cruise` (open road/long straight) or `obstacle` (dense fixed obstacles). Omitting the third argument selects `cruise`. Reverse speed remains capped by its configured limit and is never allowed to exceed the positional maximum. The M2 driver still clamps commands to the chassis-reported hardware maximum.

RabbitMQ GPS target bridge:

```bash
cd /home/robot/robot_ws
source /opt/ros/noetic/setup.bash
source /home/robot/robot_ws/devel/setup.bash
./scripts/rabbitmq_gps_goal_bridge.py
```

GPS test task menu:

```bash
cd /home/robot/robot_ws
source /opt/ros/noetic/setup.bash
source /home/robot/robot_ws/devel/setup.bash
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
  -> /cmd_vel_navigation
  -> gps_goal_speed_limiter.py
  -> /cmd_vel
  -> m2_driver
```

GPS targets:

```text
RabbitMQ
  -> rabbitmq_gps_goal_bridge.py saves and prints latest valid target
  -> operator enters 1 in the bridge terminal
  -> /gps/goal_fix
  -> gps_goal_node.py
  -> /move_base_simple/goal
  -> move_base + TEB

Test task
  -> /gps/goal_fix
  -> gps_goal_node.py
  -> /move_base_simple/goal
  -> move_base + TEB
```

RabbitMQ bridge operator behavior:

```text
On valid message: save and print the last valid item in TARGETS; do not navigate automatically.
Input 1: publish the saved point to /gps/goal_fix.
Input 2: clear the saved point.
New messages replace the in-memory saved point.
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
GPS_GOAL_SLOWDOWN_ENABLED=true
GPS_GOAL_COMFORTABLE_DECEL=0.4
GPS_GOAL_MIN_APPROACH_SPEED=0.15
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
6: Open guided FOD final test menu T01-T08.
q: Quit.
```

FOD final test records:

```text
/home/robot/robot_ws/test_results/fod_final_test_records.jsonl
/home/robot/robot_ws/test_results/fod_final_test_records.csv
```

For T02-T07, the test script arms first and waits for a new `/gps/goal_fix`. The RabbitMQ bridge operator then enters `1`; goal reception is the timer start. The operator presses Enter after cleaning and confirmed recovery to stop the timer, then records the manual pass/fail criteria. T03/T04 summarize `S / t_avg`; T07 summarizes three-run success rates.

Fence file:

```text
/home/robot/robot_ws/config/gps_test_fence.json
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

The common GPS/nomap TEB tuning file is:

```text
src/navigation_arena/arena-rosnav-3D/arena_navigation/arena_local_planer/model_based/conventional/config/dingo/teb_local_planner_params_nomap.yaml
```

Two small overlays are loaded after that common file:

```text
config/teb_profiles/gps_cruise.yaml
config/teb_profiles/gps_obstacle.yaml
```

`cruise` intent and key values:

- Open roads, campus roads, and long straight segments.
- Raise longitudinal acceleration (`acc_lim_x=2.5`).
- Suppress angular fluctuation (`max_vel_theta=0.8`, `acc_lim_theta=0.3`, `weight_acc_lim_theta=200`).
- Favor time and straight/short paths (`weight_optimaltime=6`, `weight_shortest_path=8`, `weight_viapoint=12`).
- Disable homotopy-class candidates to avoid needless topology changes in open space.
- Use `weight_kinematics_forward_drive=100` to discourage reverse/forward dithering.

`obstacle` intent and key values:

- Warehouse shelving, fixed facilities, and dense static obstacles.
- Expand the local rolling costmap to `24 x 24 m`, which makes the `max_global_plan_lookahead_dist=10` TEB horizon effective; also increase obstacle influence to `obstacle_poses_affected=30`.
- Retain four homotopy classes and increase roadmap sampling.
- Make topology switches less frequent (`selection_cost_hysteresis=1.5`, `switching_blocking_period=10`).
- Increase obstacle clearance/cost (`min_obstacle_dist=0.35`, `inflation_dist=0.7`, `weight_obstacle=80`).
- Use `weight_kinematics_forward_drive=60` and delete backward detours to reduce reversing while retaining escape maneuverability.

The positional speed argument is passed after the overlay, so it remains the final `max_vel_x` value.

After restarting GPS navigation, verify:

```bash
rosparam get /move_base/TebLocalPlannerROS/cmd_angle_instead_rotvel
rosparam get /move_base/TebLocalPlannerROS/max_vel_theta
rosparam get /move_base/TebLocalPlannerROS/acc_lim_theta
rosparam get /move_base/TebLocalPlannerROS/min_turning_radius
rosparam get /move_base/TebLocalPlannerROS/global_plan_viapoint_sep
rosparam get /move_base/TebLocalPlannerROS/max_global_plan_lookahead_dist
rosparam get /move_base/local_costmap/width
rosparam get /move_base/TebLocalPlannerROS/weight_shortest_path
rosparam get /move_base/TebLocalPlannerROS/weight_viapoint
rosparam get /move_base/TebLocalPlannerROS/enable_homotopy_class_planning
rosparam get /move_base/TebLocalPlannerROS/max_number_classes
rosparam get /move_base/TebLocalPlannerROS/switching_blocking_period
rosparam get /move_base/TebLocalPlannerROS/costmap_obstacles_behind_robot_dist
rosparam get /move_base/TebLocalPlannerROS/weight_kinematics_forward_drive
```

Expected for `cruise`:

```text
False
0.8
0.3
1.2
1.0
8.0
20.0
8.0
12.0
False
1
5.0
0.5
100.0
```

Expected for `obstacle`:

```text
False
1.2
0.4
1.2
0.6
10.0
24.0
3.0
4.0
True
4
10.0
0.8
60.0
```

## Current Speed / Goal Tuning

GPS mode overrides TEB through launch args:

```text
max_vel_x=1.5
max_vel_x_backwards=1.0
xy_goal_tolerance=0.5
yaw_goal_tolerance=6.283
weight_kinematics_forward_drive=100.0 (cruise) or 60.0 (obstacle)
penalty_epsilon=0.03
```

This means GPS targets do not require a strict final orientation.

## Goal-Approach Slowdown

GPS navigation inserts `gps_goal_speed_limiter.py` between TEB and the M2 driver:

```text
/move_base -> /cmd_vel_navigation -> /gps_goal_speed_limiter -> /cmd_vel -> /m2_driver
```

The node tracks `/move_base/current_goal` and `/gps/odom`. Its default forward cap is based on `v = sqrt(2 * 0.4 * (distance - 0.5))`, with a `0.15m/s` minimum while outside the `0.5m` goal tolerance. At `2.0m/s`, limiting begins about `5.5m` from the goal center; at `1.5m/s`, about `3.3m`.

The limiter changes only an excessive positive `linear.x` near the goal. A lower/zero obstacle command passes immediately, negative recovery velocity passes unchanged, and `angular.z` always passes unchanged. If navigation commands stop for `0.5s`, the relay publishes zero instead of holding the last command. `/move_base/cancel` also latches a zero output until move_base publishes a new current goal, preserving electronic-fence stop authority.

Runtime tuning:

```bash
GPS_GOAL_COMFORTABLE_DECEL=0.3 ./scripts/bringup.sh gps 2.0 cruise
GPS_GOAL_SLOWDOWN_ENABLED=false ./scripts/bringup.sh gps 2.0 cruise
```

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
cd /home/robot/robot_ws
source /opt/ros/noetic/setup.bash
source /home/robot/robot_ws/devel/setup.bash
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
