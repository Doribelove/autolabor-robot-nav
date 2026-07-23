# Current GPS Development Handoff

Date: 2026-07-23

This file records the current GPS navigation development state for the Autolabor M2 robot in `/home/robot/robot_ws`.

## 2026-07-23 Local Checkpoint and Next Session

The current development branch is `pre-safety-runtime`. This checkpoint
contains the rolling 15m GPS goal manager, final-goal safety relay changes,
GPS cruise turn-entry tuning, and the GPS/FOD visual-recovery mode arbiter
described below. The camera/YOLO launch is intentionally still separate.

The next required work is an attended low-speed vehicle validation; the
integrated GPS/FOD transition has passed automated ROS tests but has not yet
driven the physical vehicle. Start with:

```bash
cd /home/robot/robot_ws
FOD_RECOVERY_BLIND_DISTANCE_M=0.20 ./scripts/bringup.sh gps 0.3 cruise

# In a second sourced terminal:
roslaunch autolabor_fod_vision hikrobot_fod_detection.launch \
  start_camera:=true \
  enable_image_quality_controller:=true \
  image_quality_exposure_max_us:=12000

# After detections are healthy and the operator holds the physical stop:
./scripts/fod_mode.sh start
./scripts/fod_mode.sh watch
```

Confirm the complete state sequence
`GPS_ACTIVE -> ENTERING_FOD -> FOD_ACTIVE -> FOD_COMPLETE_STOP ->
RETURNING_GPS -> GPS_ACTIVE`, verify that the retained final GPS route resumes
from the post-recovery pose, and record `mode1` during the run. If the visual
controller reaches `ABORT`, diagnose it while stopped and use
`./scripts/fod_mode.sh stop` only when it is safe to resume GPS.

Do not start the standalone `visual_recovery.launch` alongside integrated GPS
bringup, and do not call `/fod_visual_servo/set_enabled` directly. Keep
`FOD_RECOVERY_EXTERNAL_ESTOP_OVERRIDE=false` unless an attended test explicitly
requires the override.

The following pre-existing untracked user files are deliberately outside this
checkpoint and must not be deleted or committed without review:

```text
_allow_motion:=true
_current_bias_deg:=-0.4
_distance_m:=5.0
_external_estop_override:=true
_speed_mps:=0.20
src/SweepDeviceControl/
```

## 2026-07-23 GPS/FOD Visual-Recovery Mode Arbitration

GPS bringup now defaults to
`FOD_RECOVERY_STANDBY_ENABLED=true`. The camera/YOLO perception launch remains
separate and may run continuously, while the visual motion controller starts
in `DISABLED` standby. The chassis command chain is now:

```text
move_base -> /cmd_vel_navigation -> gps_goal_speed_limiter -> /cmd_vel_gps
visual servo -----------------------------------------------> /cmd_vel_fod
                      both -> /fod_navigation_mode -> /cmd_vel -> m2_driver
```

`/fod_navigation_mode` is the sole `/cmd_vel` publisher. It forwards only one
fresh, finite input and publishes zero during every transition or fault. Its
graph watchdog latches `FAULT_STOP` if another `/cmd_vel` publisher appears or
the expected M2 subscriber disappears.

Operator commands:

```bash
./scripts/fod_mode.sh start
./scripts/fod_mode.sh status
./scripts/fod_mode.sh watch
./scripts/fod_mode.sh stop
```

On `start`, the manager blocks GPS output first, calls
`/gps/long_range/set_paused`, cancels the current move_base segment, and
requires fresh `/odom` below `0.03m/s` and `0.05rad/s` for `0.5s` before it
enables visual motion. The long-range manager retains the final WGS84 target
while paused and ignores its expected cancel/preempt echoes. On resume it
selects a new rolling segment from the post-recovery position rather than
continuing the canceled subgoal.

Visual `COMPLETE` automatically disables and resets the visual controller,
reconfirms the stop, resumes the retained GPS route, and returns to
`GPS_ACTIVE`. Visual `ABORT`, missing/stale stop odometry, service failure, or
a command-graph fault stays stopped with GPS paused; only an explicit
`fod_mode.sh stop` attempts recovery. Integrated operation must not call
`/fod_visual_servo/set_enabled` directly. The old direct
`visual_recovery.launch` remains available only for CAN-only standalone tests.

Defaults:

```text
FOD_RECOVERY_STANDBY_ENABLED=true
FOD_RECOVERY_EXTERNAL_ESTOP_OVERRIDE=false
FOD_RECOVERY_BLIND_DISTANCE_M=0.50
FOD_RECOVERY_TRANSITION_TIMEOUT=12.0
```

The mode1 recorder includes `/cmd_vel_gps`, `/cmd_vel_fod`, both mode-manager
status topics, and visual controller state/status/completion.

Verification for this increment:

- Incremental `gps_module`, `autolabor_fod_control`, and `robot_bringup` build
  passes.
- 85 direct long-range/visual/mode unit tests and 26 startup/profile
  architecture tests pass.
- The new ROS graph test verifies exclusive command selection, retained GPS
  pause, ABORT stop-latching, explicit recovery, and automatic COMPLETE resume.
- The existing eight-scenario full visual recovery ROS test still passes,
  including the 0.50m completion loop and all transient fail-closed cases.
- A full 73-package `catkin_make -j2` completes successfully; the focused
  GPS/FOD/bringup suite covers 196 test cases with zero failures.

New `/gps/goal_fix` messages are deliberately rejected while the manager is
paused. The retained pre-recovery final target therefore cannot be silently
replaced; send a replacement only after the mode returns to `GPS_ACTIVE`.

## 2026-07-23 Rolling Lookahead GPS Goals

GPS mode now accepts one distant final WGS84 target and keeps the existing
`40 x 40m`, `0.1m/cell` rolling global costmap. This section supersedes older
handoff notes below that describe a `200 x 200m` coarse global map or direct
GPS-mode publication to `/move_base_simple/goal`.

The default GPS target chain is:

```text
/gps/goal_fix (final WGS84 target)
  -> /gps_long_range_goal_manager
  -> /move_base/goal (bounded action goal)
  -> move_base + TEB
```

The manager converts and retains the final point, then selects a point on the
current-vehicle-to-final straight line at a default `15m` lookahead. It
replaces a non-final segment when the vehicle is within `5m` of it, has made
equivalent radial progress, has passed it during an obstacle detour, when the
segment succeeds early, or when the final point enters the `15m` horizon. Once
the final point is selected, that exact final action goal is sent only once
and retained until completion, cancellation, failure, or replacement by a new
external goal. A 200m target therefore advances in roughly 10m increments
without enlarging the costmap.

Managed action GoalIDs distinguish `intermediate` from `final` segments.
`gps_goal_speed_limiter.py` bypasses the `1m` near-goal fence and the `0.2m`
arrival latch only for strictly formatted intermediate IDs.
Odom/frame/command freshness, cancel, action failure, and zero-output
protections remain active. The final segment uses those terminal safety
checks, but the additional distance-based forward-speed cap now defaults off
through `GPS_GOAL_SPEED_CAP_ENABLED=false`; final TEB commands otherwise pass
through unchanged. Consecutive segment IDs within the same route retain a
still-fresh TEB command instead of injecting the relay's normal new-goal zero
pulse; a different route or non-consecutive ID still fences the old command
with zero.

Default parameters:

```text
GPS_LONG_RANGE_GOAL_ENABLED=true
GPS_LONG_RANGE_LOOKAHEAD_DISTANCE=15.0
GPS_LONG_RANGE_ADVANCE_DISTANCE=5.0
GPS_LONG_RANGE_MAX_LOOKAHEAD_DISTANCE=18.0
GPS_LONG_RANGE_MAX_FINAL_DISTANCE=1000.0
GPS_LONG_RANGE_ODOM_TIMEOUT=1.0
GPS_LONG_RANGE_MOVE_BASE_STATUS_TIMEOUT=2.0
GPS_LONG_RANGE_UPDATE_RATE=10.0
```

The manager rejects a new goal unless the GPS origin, fresh `/gps/odom`, and
fresh `/move_base/status` are available. Invalid coordinates and targets over
the configured 1000m bound are rejected. Runtime loss of odom or move_base
status cancels the active segment and deactivates the route. A foreign RViz or
action goal supersedes the managed route. Bringup enforces exactly one
subscriber to `/gps/goal_fix` in GPS mode so the old direct converter cannot
run in parallel.

Observability topics:

```text
/gps/long_range/final_goal
/gps/long_range/subgoal
/gps/long_range/status
/gps/long_range/active
```

`scripts/gps_test_tasks.py` detects the new action route and verifies the
complete final local coordinate using `/gps/long_range/final_goal`; it retains
fallback support for GPS mode with `GPS_LONG_RANGE_GOAL_ENABLED=false`. The
mode1 rosbag list includes all four rolling-route topics.

Verification completed for this increment:

- The complete 73-package workspace builds.
- The 119 directly relevant GPS/manager/limiter/bringup tests pass:
  32 localization, 27 rolling-goal, 40 limiter, and 20 profile/startup tests.
- A live ROS graph smoke test with synthetic odometry and move_base status
  converted one 200m GPS target into action targets at `15m`, `25m`, then the
  exact `200m` final point. Their identities were
  `intermediate`, `intermediate`, `final`; the final diagnostic pose was
  published once.

## 2026-07-23 Cruise Turn-Entry Response

GPS `cruise` now uses `acc_lim_theta=0.70rad/s^2`, up from `0.45rad/s^2`, to
shorten the few-tenths-of-a-metre-per-second phase while initial steering
curvature builds. This is deliberately narrower than restoring the former
fully aggressive tune: `max_vel_theta=0.85rad/s`,
`weight_acc_lim_theta=250`, `control_look_ahead_poses=2`,
`min_turning_radius=1.35m`, proportional Twist saturation, the stable
`planner_frequency=0` reference route, and the `position_filter_alpha=0.70`
moving-position filter all remain unchanged. Consequently straight-line
maximum velocity is not capped by this change, while sustained tight turns
and abrupt left/right reversals retain their existing damping.

The profile regression test locks both sides of this tradeoff: quicker
turn-entry acceleration must remain exactly `0.70`, while the steady yaw cap,
strong acceleration weight, steering-radius margin, proportional saturation,
two-pose command averaging, and absence of a profile-level `max_vel_x`
override must remain in place. This is configuration-level protection, not a
guarantee against physical snaking; validate with wheel-angle and both command
topics before the first `2.7m/s` run.

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
  -> /cmd_vel_gps
  -> fod_navigation_mode_manager.py
  -> /cmd_vel
  -> m2_driver
```

GPS targets:

```text
RabbitMQ
  -> rabbitmq_gps_goal_bridge.py saves and prints latest valid target
  -> operator enters 1 in the bridge terminal
  -> /gps/goal_fix
  -> gps_long_range_goal_manager.py
  -> rolling /move_base/goal
  -> move_base + TEB

Test task
  -> /gps/goal_fix
  -> gps_long_range_goal_manager.py
  -> rolling /move_base/goal
  -> move_base + TEB
```

FAST_LIO and `fast_lio_gps` compatibility modes still use
`gps_goal_node.py -> /move_base_simple/goal`.

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
GPS_NAV_MAX_VEL_X_BACKWARDS=1.0
GPS_XY_GOAL_TOLERANCE=0.3
GPS_GOAL_SLOWDOWN_ENABLED=true
GPS_GOAL_SPEED_CAP_ENABLED=false
GPS_GOAL_COMFORTABLE_DECEL=0.4
GPS_GOAL_MIN_APPROACH_SPEED=0.15
GPS_GOAL_HARD_STOP_DISTANCE=0.2
GPS_LONG_RANGE_GOAL_ENABLED=true
GPS_LONG_RANGE_LOOKAHEAD_DISTANCE=15.0
GPS_LONG_RANGE_ADVANCE_DISTANCE=5.0
GPS_LONG_RANGE_MAX_LOOKAHEAD_DISTANCE=18.0
GPS_LONG_RANGE_MAX_FINAL_DISTANCE=1000.0
```

The main GNSS antenna is treated as mounted at `x=-0.3m`, `y=-0.05m` in `base_link`. The localization node compensates this offset so `/gps/odom` represents the chassis center. Position and yaw remain GNSS-based by default, while `/gps/odom.twist` uses fresh signed chassis `/odom` linear and angular velocity without enabling wheel-pose integration. The M2 driver timestamps `/odom` with the oldest velocity/wheel/steering measurement used to form that twist and stops publishing when any required feedback is stale. The GPS node checks that source timestamp rather than callback receipt time. If chassis twist is older than `0.5s`, it falls back to RMC course/speed and dual-antenna heading rate; cached RMC motion is discarded after `1.0s`.

## GPS Goal Management

GPS-mode files:

```text
src/application/gps_module/scripts/gps_long_range_goal_manager.py
src/application/gps_module/src/gps_module/long_range.py
src/application/gps_module/launch/gps_long_range_goal.launch
```

Current behavior:

- Subscribes `/gps/goal_fix`.
- Treats that message as the final target and publishes bounded action goals on
  `/move_base/goal`.
- Uses `/gps/odom`, a 15m lookahead, and a 5m advance threshold.
- Publishes final/subgoal/status observability topics under `/gps/long_range`.
- Uses the bearing from current odom position to each target as its target yaw.
- Yields the route when a foreign RViz/action goal takes control.

Current `bringup.sh gps` launches it with:

```text
frame_id:=camera_init
odom_topic:=/gps/odom
lookahead_distance:=15.0
advance_distance:=5.0
```

FAST_LIO compatibility modes still launch
`src/application/gps_module/scripts/gps_goal_node.py` and publish
`/move_base_simple/goal`.

RViz manual goals still use:

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
  `acc_lim_theta=0.70`, and `weight_acc_lim_theta=250`. The higher acceleration
  bound shortens the low-speed steering-build phase, while the unchanged
  steady-yaw cap, strong acceleration weight, command averaging, and stable
  global route continue to suppress rapid left/right corrections at full
  cruise speed. A genuinely tight bend can still make TEB reduce linear speed
  instead of commanding a sharp correction at full cruise speed.
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
cmd_angle_instead_rotvel=False
planner_frequency=0.0
max_vel_theta=0.85
acc_lim_x=2.5
acc_lim_theta=0.70
control_look_ahead_poses=2
min_turning_radius=1.35
global_plan_viapoint_sep=1.5
max_global_plan_lookahead_dist=6.5
local_costmap/width=16.0
weight_shortest_path=5.0
weight_viapoint=6.0
enable_homotopy_class_planning=False
max_number_classes=1
no_inner_iterations=10
no_outer_iterations=6
roadmap_graph_no_samples=8
selection_cost_hysteresis=1.2
selection_prefer_initial_plan=0.95
switching_blocking_period=5.0
include_dynamic_obstacles=True
shrink_horizon_min_duration=10
costmap_obstacles_behind_robot_dist=0.5
weight_kinematics_forward_drive=100.0
global_costmap/width=40.0
global_costmap/resolution=0.1
gps_localization/position_filter_alpha=0.70
```

Expected for `obstacle`:

```text
cmd_angle_instead_rotvel=False
planner_frequency=1.0
max_vel_theta=1.2
acc_lim_x=1.2
acc_lim_theta=0.4
control_look_ahead_poses=1
min_turning_radius=1.35
global_plan_viapoint_sep=0.6
max_global_plan_lookahead_dist=10.0
local_costmap/width=24.0
weight_shortest_path=3.0
weight_viapoint=4.0
enable_homotopy_class_planning=True
max_number_classes=4
no_inner_iterations=10
no_outer_iterations=6
roadmap_graph_no_samples=15
selection_cost_hysteresis=1.5
selection_prefer_initial_plan=0.8
switching_blocking_period=10.0
include_dynamic_obstacles=True
shrink_horizon_min_duration=10
costmap_obstacles_behind_robot_dist=0.8
weight_kinematics_forward_drive=60.0
global_costmap/width=40.0
global_costmap/resolution=0.1
gps_localization/position_filter_alpha=0.25
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

## Goal Command Safety Relay And Optional Speed Cap

GPS navigation inserts `gps_goal_speed_limiter.py` between TEB and the M2
driver:

```text
/move_base -> /cmd_vel_navigation -> /gps_goal_speed_limiter
  -> /cmd_vel_gps -> /fod_navigation_mode -> /cmd_vel -> /m2_driver
```

The node tracks `/move_base/current_goal`, `/move_base/goal`,
`/move_base/status`, and `/gps/odom`. `GPS_GOAL_SPEED_CAP_ENABLED=false` is the
default, so outside a stop condition it forwards the final-goal `linear.x` and
`angular.z` from TEB unchanged even near the target. TEB may still choose to
slow down on its own. This switch disables only the relay's extra
distance-based cap.

TEB's GPS `xy_goal_tolerance=0.3m` remains independent of the relay's
`hard_stop_distance=0.2m`. At or inside `0.2m`, the relay latches a full zero
`Twist`, so later GPS jitter cannot restart that action. Once distance first
falls within `1.0m`, the final approach is bounded to `15s` and `0.5m`
regression from the closest point; violating either condition latches stopped
until a new goal. If navigation commands stop for `0.5s`, the relay publishes
zero instead of holding the last command. `/move_base/cancel` follows
actionlib `GoalID` matching, stopping or terminal `/move_base/status` states
also stop the relay, and only a genuinely new action goal releases it. Timer
and cancellation output are serialized so an old nonzero timer command cannot
overwrite a cancellation stop.

The former comfort cap remains available as an opt-in. When enabled, it uses
`v = sqrt(2 * 0.4 * (distance - 0.2))`, with a `0.15m/s` minimum outside the
hard-stop radius. It changes only excessive positive `linear.x` and scales
`angular.z` by the same ratio to preserve Ackermann curvature. At `1.5m/s`,
`2.0m/s`, and `2.7m/s`, it begins limiting at about `3.0m`, `5.2m`, and
`9.3m` respectively. Lower/zero obstacle commands and negative recovery
velocity pass unchanged.

Runtime tuning:

```bash
GPS_GOAL_SPEED_CAP_ENABLED=true GPS_GOAL_COMFORTABLE_DECEL=0.3 ./scripts/bringup.sh gps 2.0 cruise
GPS_XY_GOAL_TOLERANCE=0.3 GPS_GOAL_HARD_STOP_DISTANCE=0.2 ./scripts/bringup.sh gps
GPS_GOAL_NEAR_COMMIT_DISTANCE=1.0 GPS_GOAL_NEAR_TIMEOUT=15.0 GPS_GOAL_NEAR_MAX_REGRESSION=0.5 ./scripts/bringup.sh gps 2.0 cruise
```

`GPS_GOAL_SLOWDOWN_ENABLED=false` remains a legacy master bypass for
diagnostics. It removes the entire relay, including arrival, stale-input,
cancel, and near-goal protections, so it is not needed to disable speed
limiting and should normally remain `true`.

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

GPS keeps the existing rolling global map:

```text
global_costmap width/height: 40.0 m
global_costmap resolution: 0.1 m/cell
global_costmap rolling_window: true
```

This is a `400 x 400` grid. The long-range manager keeps each intermediate
move_base target at `15m`, leaving about `5m` inside the nominal `20m`
half-width for GPS error, rolling-window latency, and obstacle detours. Far
final targets therefore do not require a larger or coarser map.

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
