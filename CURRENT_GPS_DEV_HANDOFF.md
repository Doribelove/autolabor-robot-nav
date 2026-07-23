# Current GPS Development Handoff

Date: 2026-07-23

This file records the current GPS navigation development state for the Autolabor M2 robot in `/home/robot/robot_ws`.

## 2026-07-23 Live Heading and Near-Goal Latch

A straight-line steering calibration observed a transient dual-antenna heading
change of about `23.99deg/s` while chassis yaw rate was only about
`0.015deg/s`, wheel angle was approximately `0.021deg`, and the driven path
remained straight. This is treated as a GNSS heading outlier rather than real
body rotation.

An initial jump guard was added for this observation, but live navigation then
showed that a small persistent mismatch could leave it holding an old yaw for
minutes; one recorded run grew to roughly `140deg` disagreement and TEB began
oscillating. Both `cruise` and `obstacle` therefore now default
`GPS_HEADING_JUMP_GUARD_ENABLED=false`. Navigation directly uses each fresh,
quality-valid dual-antenna heading. The old guard remains only as an explicit
diagnostic opt-in and must not be enabled for the normal `2.7m/s cruise` run.

The RViz goal from the affected run remained fixed in `camera_init`; the
apparent moving target came from the incorrect live robot orientation/TF, not
from mutation of the target `PoseStamped`.

The same run exposed a separate terminal-control defect: after a one-cycle
hard stop at `0.17m`, GPS noise reported `0.21m` and the old action moved again.
The goal limiter now latches a complete stop on first entry into `0.20m`; only
a new `/move_base/goal` releases it. A bounded final-approach fence also arms
inside `1.0m` and latches stopped if the action takes more than `15s` or moves
`0.5m` farther away than its closest point. This prevents indefinite roaming
around a goal even if TEB never reaches a terminal action state.

## 2026-07-22 Strict Dual-Antenna Startup Gate

After the FAST_LIO libusb fix, live GPS bringup passed CAN, Livox,
`/cloud_registered_body`, `/scan`, and chassis `/odom`. GPS then advertised
all topics but correctly withheld `/gps/odom`: every `UNIHEADINGA` sample was
`SOL_COMPUTED + NARROW_FLOAT`, while production requires `NARROW_INT`. A
subsequent direct sample reported a `0.5715m` baseline, `0.7200deg` heading
standard deviation, 23 tracked satellites and 15 solution satellites. This is
a real float ambiguity solution, not a missing serial stream.

Bringup now waits up to `120s` for strict GPS odometry instead of failing after
`15s`, prints the required heading quality in the main terminal, and explains
that persistent `NARROW_FLOAT` is intentionally rejected. Rejected-heading
logs now include baseline and heading standard deviation. Do not weaken the
default gate for the `2.7m/s cruise` run; if float persists outdoors, inspect
both antenna views/cables and the receiver's heading/fixed-baseline setup.

## 2026-07-22 FAST_LIO / Hikrobot MVS libusb Isolation

Installing Hikrobot MVS added `/opt/MVS/lib/64` to the global loader cache.
Its bundled `libusb-1.0.so.0` lacks `libusb_set_option`, which PCL 1.10 needs,
so `fastlio_mapping` exited with code 127 before publishing
`/cloud_registered_body`. Navigation bringup now prepends
`/lib/x86_64-linux-gnu` only for the FAST_LIO roslaunch child. The camera and
all other processes keep their original environment, so navigation and the
Hikrobot camera can run concurrently with the original commands. The one-shot
CAN preflight also runs directly through `rosrun`, avoiding the misleading
`REQUIRED process ... has died` message after a successful check.

Concurrent startup:

```bash
./scripts/bringup.sh gps 2.7 cruise
# In a second sourced terminal, after bringup is ready:
roslaunch hikrobot_mvs_camera fod_camera.launch
```

## 2026-07-22 GPS Antenna Lateral Offset

The confirmed main GNSS antenna position in `base_link` is now
`x=-0.30m`, `y=-0.05m`: 0.30m behind and 0.05m to the right of the chassis
center. GPS localization subtracts the yaw-rotated antenna offset, so the
published chassis-center pose is corrected 0.30m forward and 0.05m left from
the antenna position in the vehicle frame.

## 2026-07-17 Qt Operator Console

The first optional Qt5/librviz operator console is under
`src/application/autolabor_operator_gui`, with structured RabbitMQ messages in
`src/application/autolabor_operator_msgs`. It displays ROS, CAN, GNSS, heading,
laser, navigation and RabbitMQ health; GPS/local pose and existing static-error
metrics; an embedded RViz; cached remote targets; and event logs. The first
test controls are an 8m forward GPS goal, move_base cancellation, static-error
reset and start/stop of the existing `mode1` rosbag script. Camera/YOLO and
cleaning-device pages are placeholders for later ROS interfaces.

The console is deliberately a sidecar and never publishes `/cmd_vel`. Missing
robot-side nodes or RabbitMQ only produce offline cards. RabbitMQ connection
failures keep the bridge ROS services and status publisher alive. The bridge
still supports the original terminal `1/2` confirmation commands, and now also
publishes latched `/rabbitmq_bridge/status` and
`/rabbitmq_bridge/latest_target`, with bounded Trigger services at
`/rabbitmq_bridge/publish_latest` and `/rabbitmq_bridge/clear_latest`.

Recommended launch without a duplicate RViz window:

```bash
NAV_START_RVIZ=false ./scripts/bringup.sh gps 0.3 cruise
./scripts/operator_gui.sh
```

`NAV_START_RVIZ` defaults to `true`, so the original standalone-RViz bringup
continues unchanged when the GUI is not used.

## 2026-07-17 GPS Cruise Costmap Reduction

The GPS `cruise` overlay now reduces the local rolling costmap from
`20 x 20m` to `16 x 16m` at the unchanged `0.1m` resolution. This changes the
grid from `200 x 200` to `160 x 160` cells, a 36% reduction in cells per layer.
TEB only transforms global-plan points inside 85% of the costmap half-width,
so the theoretical local-plan boundary is `6.8m`; cruise explicitly uses a
`6.5m` lookahead to retain boundary margin. An 8m GPS goal remains valid and is
followed as successive local segments while the rolling window moves. The
`obstacle` overlay remains `24 x 24m` with a `10m` TEB lookahead.

## 2026-07-16 GPS Test Task Link Fix

The observed no-response test was an entry-link failure, not a planner or
chassis-command failure. `/gps_test_tasks` registered with an old ROS master;
bringup then stopped that master and started a new one. The still-running test
process did not re-register. Its own `/gps/goal_fix` subscriber could observe
its own publication, while the new `/gps_goal` process received nothing.

The test script now remembers the ROS master PID and exits on master loss or
replacement. Before publishing or arming an external FOD goal, it verifies the
current graph from `/gps/goal_fix` through `/gps_goal` and
`/move_base_simple/goal` to `/move_base`, requires fresh `/gps/odom`, and waits
for the matching converted `PoseStamped` before reporting link success.
Bringup also verifies both goal-topic connections before declaring itself
ready. Always wait until bringup prints its ready message, then start the test
script in a second terminal. Restart the test script after every bringup or
roscore restart.

## 2026-07-16 Reverse-Incident Fix

The latest live run was actually `./scripts/bringup.sh gps 2.7 cruise`, with
`max_vel_x_backwards=1.0`, even though the initial report named `2.0`. Five
goals were published from RViz. The only goal farther than 12 m from the local
origin was `(3.963, 14.584)`; it was replaced after about 40 s and was never
reported reached. No rosbag captured the incident command sign, so do not
claim that the historical `/cmd_vel_navigation.linear.x` was directly
observed negative.

Two deterministic defects found in the command path have now been fixed:

- The TEB carlike graph ignored `weight_kinematics_forward_drive`, so the
  configured cruise value `100` had no effect. `EdgeKinematicsCarlike` now
  contains separate nonholonomic, backward-drive, and turning-radius errors,
  and the optimizer wires all three configured weights.
- The M2 driver converted a reverse `Twist` with
  `atan(angular.z * wheelbase / abs(linear.x))`. It now retains the sign of
  `linear.x`, so a reverse arc produces the yaw direction requested by TEB.

The forward-drive weight remains a soft preference: it strongly discourages
unnecessary reverse but still permits a genuinely required maneuver. The
changes compiled successfully. Six TEB tests and twelve M2 steering/safety
tests pass.
They are not active in a process that was already running before the rebuild;
restart bringup before the controlled road test. Keep the software emergency
stop asserted until the restart is complete and record both command topics,
GPS/chassis odometry, steering, goals, status, and TEB plans during the first
low-speed test.

## 2026-07-16 Premature-Stop Live Test

The second controlled `gps 0.3 cruise` run used target `(1.976, 13.863)`, about
`8.85m` from the vehicle when sent. The vehicle travelled about `5.70m`, then
stopped with about `3.21m` still remaining. This was not a navigation success,
obstacle stop, software emergency, command dropout, or reverse command:

- `move_base` remained `ACTIVE` until the goal was manually cancelled.
- `/cmd_vel_navigation` and `/cmd_vel` continued at about
  `v=+0.300m/s, omega=+0.243rad/s` for another `84.7s` while the wheels were
  stationary.
- The laser had no nearby obstacle and both command streams remained fresh.
- The steering swept repeatedly between almost full left and full right. On
  the final sweep the wheels braked and stopped even though the positive
  command continued.

The strongest deterministic cause was non-proportional TEB saturation. The
local plan segment had a geometric radius of about `2.20m`, but TEB clipped
only its estimated `0.53m/s` linear velocity to `0.30m/s` while retaining the
same angular velocity. The command radius therefore collapsed to about
`1.23m`, placing steering at roughly `99%` of the M2 limit and causing repeated
extreme steering reversals. The exact VCU/TCU protection bit cannot be recovered
from that bag because it did not contain `0x23`/`0x24` diagnostics.

The corrective set for the next controlled run is:

- Both GPS TEB profiles set `use_proportional_saturation=true` and use a
  `1.35m` planning minimum radius, above the approximately `1.22m` theoretical
  chassis limit.
- The GPS goal-speed limiter and the M2 final chassis-speed clamp scale
  `angular.z` whenever they reduce linear velocity, preserving Ackermann
  curvature through the complete command path.
- `/gps/odom.twist` now uses fresh signed `/odom` linear and angular velocity
  while GPS position and dual-antenna yaw remain unchanged. The M2 `/odom`
  timestamp is the oldest of the four fresh feedback samples used to form it.
- Strict dual-antenna mode suppresses navigation pose/odom/TF until heading is
  fresh and quality-valid, and suppresses them again if heading becomes stale.
  Stationary position filtering prefers fresh wheel speed over RMC speed.
- VCU `0x23` now logs raw controller bytes and identifies bit 2 as current
  overlimit; `0x24` control-timeout publication/logging is enabled.
- M2 command handling rejects invalid chassis parameters and non-finite Twist
  input, with a final CAN-boundary sanitizer that can only encode finite,
  chassis-limited motion or an explicit zero command.
- The recorder includes both command topics, raw CAN, controller monitor,
  timeout, wheel speeds, and steering angle.
- The goal-speed limiter uses a `0.20m` arrival radius strictly inside the GPS
  planner's `0.30m` success radius. Entry is stop-latched against GPS noise;
  the `1.0m` final-approach fence also latches stopped after `15s` or `0.5m`
  regression from the closest point.
- Goal cancellation and terminal states are GoalID-aware and stop-latched;
  final timer publishing is serialized with stops. Missing/stale/non-finite
  GPS odometry during an active goal fails to a complete zero command.
- Clean restart removes any old goal-speed limiter, and startup requires one
  and only one publisher on each command topic.

The full workspace builds. The isolated `gps_module` and `robot_bringup`
package run for this change passes 79 tests: 32 GPS motion/heading tests and
47 goal-limiter/profile/bringup tests. The previously passing 12 M2
command-safety and 6 TEB tests were not rerun for this Python/launch-only
change. The running bringup still requires a restart under software emergency
before the next road test.

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

The second positional argument requests the GPS TEB forward `max_vel_x`; the third selects `cruise` (open road/long straight) or `obstacle` (dense fixed obstacles). Omitting the third argument selects `cruise`. Reverse speed remains capped by its configured limit and is never allowed to exceed the positional maximum. After the CAN driver starts, bringup reads `/m2_driver/chassis_parameter` and caps both TEB speed limits to the chassis-reported `max_speed`; the M2 driver still provides a final clamp.

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
GPS_ODOM_STARTUP_TIMEOUT=120.0
GPS_HEADING_REQUIRED_SOLUTION_STATUS=SOL_COMPUTED
GPS_HEADING_REQUIRED_POSITION_TYPES=NARROW_INT
GPS_ANTENNA_OFFSET_X=-0.3
GPS_ANTENNA_OFFSET_Y=-0.05
GPS_USE_WHEEL_ODOM=false
GPS_USE_WHEEL_TWIST=true
GPS_WHEEL_TWIST_TIMEOUT=0.5
GPS_RMC_SPEED_TIMEOUT=1.0
GPS_NAV_MAX_VEL_X_BACKWARDS=1.4
GPS_XY_GOAL_TOLERANCE=0.3
GPS_GLOBAL_COSTMAP_SIZE=200.0
GPS_GLOBAL_COSTMAP_RESOLUTION=0.25
GPS_GOAL_SLOWDOWN_ENABLED=true
GPS_GOAL_COMFORTABLE_DECEL=0.4
GPS_GOAL_MIN_APPROACH_SPEED=0.15
GPS_GOAL_HARD_STOP_DISTANCE=0.2
```

The main GNSS antenna is treated as mounted at `x=-0.3m`, `y=-0.05m` in `base_link`. The localization node compensates this offset so `/gps/odom` represents the chassis center. Position and yaw remain GNSS-based by default, while `/gps/odom.twist` uses fresh signed chassis `/odom` linear and angular velocity without enabling wheel-pose integration. The M2 driver timestamps `/odom` with the oldest velocity/wheel/steering measurement used to form that twist and stops publishing when any required feedback is stale. The GPS node checks that source timestamp rather than callback receipt time. If chassis twist is older than `0.5s`, it falls back to RMC course/speed and dual-antenna heading rate; cached RMC motion is discarded after `1.0s`.

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
- Use a `16 x 16m` local rolling costmap and a `6.5m` TEB lookahead. The
  theoretical 85%-of-half-width boundary is `6.8m`.
- Keep `planner_frequency=0.0` while controlling so the global reference line
  is not rebuilt once per second from an already-offset vehicle pose. New
  goals and planning failures still trigger planning. `obstacle` remains at
  `1.0Hz`.
- Use `position_filter_alpha=0.70` in cruise only. At a 10Hz position update
  and 2.7m/s, the first-order filter's constant-motion lag falls from about
  `0.81m` at alpha 0.25 to about `0.12m`. Obstacle/direct launch stays at 0.25.
- Raise longitudinal acceleration (`acc_lim_x=2.5`).
- Damp high-speed lateral recovery with `control_look_ahead_poses=2`,
  `global_plan_viapoint_sep=1.5`, `max_vel_theta=0.85`,
  `acc_lim_theta=0.45`, and `weight_acc_lim_theta=250`. A genuinely tight bend
  can make TEB reduce linear speed instead of commanding a sharp correction at
  full cruise speed.
- Retain moderate time/path attraction without snapping across the reference
  (`weight_optimaltime=4`, `weight_shortest_path=5`, `weight_viapoint=6`).
- Disable homotopy-class candidates to avoid needless topology changes in open space.
- `weight_kinematics_forward_drive=100` is now applied by this fork's carlike
  graph. It is a strong forward preference, not a hard prohibition of reverse.

`obstacle` intent and key values:

- Warehouse shelving, fixed facilities, and dense static obstacles.
- Expand the local rolling costmap to `24 x 24 m`, which makes the
  `max_global_plan_lookahead_dist=10` TEB horizon effective.
- Use three homotopy classes, ten roadmap samples, and `5 x 4` optimizer
  iterations. This reduces the configured upper search budget from
  `4 x 10 x 6 = 240` to `3 x 5 x 4 = 60` optimizer passes per cycle.
- Favor the previous route by 5% while allowing a replacement after `0.5s`
  (`selection_cost_hysteresis=0.95`, `selection_prefer_initial_plan=0.95`,
  `switching_blocking_period=0.5`). The former hysteresis value `1.5` actually
  favored a new route in this fork and then locked it for `10s`.
- Recover from an infeasible plan's shortened horizon after `3s`, not `10s`.
- Treat laser/costmap points as static (`include_dynamic_obstacles=false`),
  because no costmap-converter source currently supplies obstacle velocities.
- Increase obstacle clearance/cost (`min_obstacle_dist=0.35`, `inflation_dist=0.7`, `weight_obstacle=80`).
- Raise angular speed/acceleration to `1.4rad/s` and `0.8rad/s^2`, and the
  time-optimal weight to `4.0`, so an avoidance arc builds and resumes sooner.
- Raise the symmetric longitudinal planning limit to `acc_lim_x=2.0m/s^2`.
  This remains below the cruise profile's `2.5m/s^2`, but it is a planning
  constraint rather than a measured chassis guarantee; retain it only after
  commanded-versus-measured acceleration and braking pass the staged road test.
- `delete_detours_backwards=true` remains active. The configured
  `weight_kinematics_forward_drive=60` now applies to the carlike graph, but it
  remains a soft preference and does not enforce a no-reverse policy.

`obstacle_poses_affected` is also ineffective while
`legacy_obstacle_association=false`; it must not be credited for the current
clearance behavior.

The positional speed argument is passed after the overlay, so it remains the final `max_vel_x` value.

After restarting GPS navigation, verify:

```bash
rosparam get /move_base/TebLocalPlannerROS/cmd_angle_instead_rotvel
rosparam get /move_base/planner_frequency
rosparam get /move_base/TebLocalPlannerROS/max_vel_theta
rosparam get /move_base/TebLocalPlannerROS/acc_lim_x
rosparam get /move_base/TebLocalPlannerROS/acc_lim_theta
rosparam get /move_base/TebLocalPlannerROS/control_look_ahead_poses
rosparam get /move_base/TebLocalPlannerROS/min_turning_radius
rosparam get /move_base/TebLocalPlannerROS/global_plan_viapoint_sep
rosparam get /move_base/TebLocalPlannerROS/max_global_plan_lookahead_dist
rosparam get /move_base/local_costmap/width
rosparam get /move_base/TebLocalPlannerROS/weight_shortest_path
rosparam get /move_base/TebLocalPlannerROS/weight_viapoint
rosparam get /move_base/TebLocalPlannerROS/enable_homotopy_class_planning
rosparam get /move_base/TebLocalPlannerROS/max_number_classes
rosparam get /move_base/TebLocalPlannerROS/no_inner_iterations
rosparam get /move_base/TebLocalPlannerROS/no_outer_iterations
rosparam get /move_base/TebLocalPlannerROS/roadmap_graph_no_samples
rosparam get /move_base/TebLocalPlannerROS/selection_cost_hysteresis
rosparam get /move_base/TebLocalPlannerROS/selection_prefer_initial_plan
rosparam get /move_base/TebLocalPlannerROS/switching_blocking_period
rosparam get /move_base/TebLocalPlannerROS/include_dynamic_obstacles
rosparam get /move_base/TebLocalPlannerROS/shrink_horizon_min_duration
rosparam get /move_base/TebLocalPlannerROS/costmap_obstacles_behind_robot_dist
rosparam get /move_base/TebLocalPlannerROS/weight_kinematics_forward_drive
rosparam get /move_base/global_costmap/width
rosparam get /move_base/global_costmap/resolution
rosparam get /gps_localization/position_filter_alpha
```

Expected for `cruise`:

```text
False
1.3
2.5
0.8
1.35
1.0
6.5
16.0
8.0
12.0
False
1
10
6
8
1.2
0.95
5.0
True
10
0.5
100.0
200.0
0.25
```

Expected for `obstacle`:

```text
False
1.4
2.0
0.8
1.2
0.6
10.0
24.0
3.0
4.0
True
3
5
4
10
0.95
0.95
0.5
False
3.0
0.8
60.0
200.0
0.25
```

### Obstacle response evidence and staged acceptance

The pre-tuning `1.0m/s obstacle` session logged 71
`trajectory is not feasible` resets. The main burst lasted about `5.92s` at
almost every 10Hz control cycle, while only one control-loop overrun was logged
(`0.1049s`). This maps the observed stop primarily to planner rejection and
zero-command resets, not to a generic CPU stall or the goal limiter.

Restart before testing so the overlay is reloaded:

```bash
cd /home/robot/robot_ws
./scripts/bringup.sh gps 0.8 obstacle
```

Record the first repeatable bypass run:

```bash
rosbag record -O ~/m2_obstacle_tune_01.bag \
  /cmd_vel_navigation /cmd_vel /odom /gps/odom \
  /m2_driver/wheel_angle /m2_driver/left_wheel_vel \
  /m2_driver/right_wheel_vel /move_base/status \
  /move_base/TebLocalPlannerROS/local_plan /scan /rosout
```

Acceptance for this first stage:

- No continuous `trajectory is not feasible` burst longer than two 10Hz cycles.
- No visible zero-command plateau unless the footprint really has no safe path.
- No repeated control-loop miss warning.
- Re-test the efficiency tune at `0.8`, `1.4`, `1.8`, then `2.2m/s`, with the
  physical remote and emergency stop ready. Record front-wheel angle during
  every forward/reverse transition; motion must not outrun steering alignment.

For acceleration calibration, compare `/cmd_vel_navigation`, `/cmd_vel`, and
`/gps/odom.twist.twist.linear.x` during both `0 -> 0.5m/s` acceleration and
`0.5 -> 0m/s` braking. Fast command rise with slow odometry rise identifies a
VCU/drive-layer limit; it is not fixed by raising TEB acceleration. Set the
final `acc_lim_x` no higher than the reliably measured chassis deceleration.

## Current Speed / Goal Tuning

GPS mode overrides TEB through launch args:

```text
max_vel_x=1.5
max_vel_x_backwards=1.4
xy_goal_tolerance=0.3
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

The node tracks `/move_base/current_goal`, `/move_base/goal`, `/move_base/status`, and `/gps/odom`. TEB's GPS `xy_goal_tolerance=0.3m` is independent of the limiter's `hard_stop_distance=0.2m`. The default forward cap is based on `v = sqrt(2 * 0.4 * (distance - 0.2))`, with a `0.15m/s` minimum outside the hard-stop radius. At `2.0m/s`, limiting begins about `5.2m` from the goal center; at `1.5m/s`, about `3.0m`.

Outside `0.2m`, the limiter changes only an excessive positive `linear.x`; whenever it reduces forward speed, it scales `angular.z` by the same ratio to preserve the Ackermann trajectory curvature already checked by TEB. Lower/zero obstacle commands and negative recovery velocity pass unchanged. At or inside `0.2m`, it latches a full zero `Twist`, so later GPS jitter cannot restart that action. Once distance first falls within `1.0m`, the final approach is also bounded to `15s` and `0.5m` regression from the closest point; violating either condition latches stopped until a new goal. If navigation commands stop for `0.5s`, the relay publishes zero instead of holding the last command. `/move_base/cancel` follows actionlib `GoalID` matching: a specific ID stops only that active goal, an empty ID with zero stamp cancels all current goals, and an empty ID with a timestamp cancels goals at or before that time. Stopping or terminal `/move_base/status` states also stop the relay; a genuinely new action goal releases it. Timer and cancellation output are serialized so an old nonzero timer command cannot overwrite a cancellation stop.

Runtime tuning:

```bash
GPS_GOAL_COMFORTABLE_DECEL=0.3 ./scripts/bringup.sh gps 2.0 cruise
GPS_XY_GOAL_TOLERANCE=0.3 GPS_GOAL_HARD_STOP_DISTANCE=0.2 ./scripts/bringup.sh gps
GPS_GOAL_NEAR_COMMIT_DISTANCE=1.0 GPS_GOAL_NEAR_TIMEOUT=15.0 GPS_GOAL_NEAR_MAX_REGRESSION=0.5 ./scripts/bringup.sh gps 2.0 cruise
GPS_GOAL_SLOWDOWN_ENABLED=false ./scripts/bringup.sh gps 2.0 cruise
```

## Ackermann Recovery And Reverse Commands

`navigation_arena.launch` now replaces move_base's default in-place rotation
recovery with this explicit chain:

```text
conservative costmap clear
  -> footprint-checked reverse Ackermann arc
  -> footprint-checked forward Ackermann arc
```

The arc plugin is `robot_bringup/AckermannArcRecovery`. Each motion ramps at
`0.60m/s^2` to at most `0.30m/s`, uses no more than `0.24rad/s`, keeps a
turning radius of at least `1.30m`, and stops after at most `0.55m` or `4.0s`.
It runs at `20Hz`, checks the complete padded footprint every `0.05m`,
and refuses unknown, inscribed, lethal, over-threshold, stale-costmap, or
out-of-map trajectories. A cancel or new action goal immediately publishes a
zero command. An interrupt arriving just before `runBehavior()` is retained for
`0.5s` and consumed once, closing the recovery-start race without permanently
latching old messages. In GPS mode the plugin publishes through
`/cmd_vel_navigation`, so the goal limiter and electronic-fence cancel authority
remain in the path.

The M2 `/cmd_vel` conversion now uses signed linear velocity:

```text
steering = atan(angular.z * wheelbase / linear.x)
```

This is required for a reverse command's actual yaw rate to have the same sign
as standard `Twist.angular.z` and as the trajectory collision-checked by TEB.

The tested steering-center correction value `-0.3` is a chassis calibration
and is independent of `GPS_ANTENNA_OFFSET_X=-0.3m`. Verify after a chassis
power cycle whether the VCU persists the steering-center setting.

## Costmap / Perception State

Current intended rates:

```text
/scan: about 10 Hz
move_base controller_frequency: 10 Hz
local_costmap update_frequency: 10 Hz
```

Current local map / laser obstacle settings:

```text
local_costmap width: 16.0 (cruise), 24.0 (obstacle)
local_costmap height: 16.0 (cruise), 24.0 (obstacle)
obstacle_range: 10.0
raytrace_range: 11.0
scan range_max: 12.0
```

GPS uses a separate coarse global rolling map for long-range path generation:

```text
global_costmap width/height: 200.0 m
global_costmap resolution: 0.25 m/cell
global_costmap initial origin: (-100.0, -100.0)
```

This is an `800 x 800` grid (640,000 cells) and accepts goals with useful
margin beyond 50m. `GPS_GLOBAL_COSTMAP_SIZE` and
`GPS_GLOBAL_COSTMAP_RESOLUTION` can be changed together for farther targets;
bringup rejects configurations above one million cells. The detailed local
costmap remains at `0.1m/cell`, so local obstacle geometry is not coarsened.

Current clearance:

```text
TebLocalPlannerROS/min_obstacle_dist: 0.3 (cruise), 0.35 (obstacle)
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

Checks for the GPS twist, goal limiter, and Ackermann recovery changes passed:

```bash
catkin_make --pkg autolabor_canbus_driver gps_module robot_bringup -j2
cmake --build build --target run_tests_gps_module run_tests_robot_bringup -- -j2
catkin_test_results build/test_results/gps_module
catkin_test_results build/test_results/robot_bringup
python3 -m py_compile src/application/gps_module/scripts/gps_localization_node.py src/scripts/robot_bringup/scripts/gps_goal_speed_limiter.py
bash -n scripts/bringup.sh
git diff --check
```

The Qt/RabbitMQ increment was also validated with the package whitelist
explicitly cleared, so the complete 69-package workspace remains buildable:

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES='' -j2
python3 -m unittest -v src/scripts/robot_bringup/test/test_teb_profile_tuning.py
python3 -m py_compile scripts/rabbitmq_gps_goal_bridge.py
bash -n scripts/bringup.sh scripts/operator_gui.sh
```

Additional smoke tests covered GUI startup with a fresh master and no robot
nodes, absence of any GUI `/cmd_vel` publisher, clean required-launch shutdown,
legacy RViz default/disabled selection, and RabbitMQ retry/status/services with
an unreachable broker.

There are 61 directly added regression cases: 16 GPS motion/timestamp cases,
17 limiter/cancel cases, 10 recovery geometry/interrupt cases, 9 obstacle
profile/efficiency cases, 6 long-range costmap cases, and 3 startup/runtime
isolation cases. The user reported a
successful `2.2m/s obstacle` run before this efficiency increment; the new
acceleration, reverse-speed, and recovery-speed defaults still require the
staged acceptance above.

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

On 2026-07-16, the user requested a complete GitHub checkpoint. The current
workspace state is being saved on the `experiment/715` branch with the commit
note `715实验分支：保存当前项目完整进度`. Modified third-party submodules are
stored on matching `experiment/715` branches in Doribelove forks so the parent
repository can be cloned recursively without missing local commits.
