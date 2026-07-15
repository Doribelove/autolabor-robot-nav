# Project Health Fix Roadmap

Last audit: 2026-07-15
Workspace: `/home/robot/robot_ws`
Target platform: ROS Noetic / Python 3

## Purpose

This document records the verified project-health problems and the recommended
repair order. It is intended to be the handoff point for future repair sessions.

## Implementation Checkpoint: 2026-07-15

The following requested fixes are implemented in the current worktree and have
build/test coverage, but have not yet been committed. The preceding fixes have
passed the user's low-speed vehicle acceptance; the newly listed obstacle tune
still awaits its separate staged acceptance:

- `/gps/odom` now reports signed chassis `linear.x`, a credible `angular.z`,
  source-timestamp freshness, and bounded RMC fallback lifetime.
- TEB GPS success tolerance (`0.3m`) and the independent limiter hard stop
  (`0.2m`) no longer share one `0.5m` value.
- The goal limiter now applies actionlib GoalID/timestamp cancel semantics and
  serializes cancellation/status stops with timer output.
- Default in-place rotation recovery is disabled; bounded, footprint-checked
  reverse/forward Ackermann arc recoveries are configured instead.
- Supporting fixes correct reverse `Twist` steering conversion and close both
  running and just-before-start recovery cancellation races.
- The obstacle TEB overlay has a first-stage response tune: safety clearances
  are unchanged, the contradictory 1.5/10s topology selection pair is removed,
  infeasible-plan short-horizon recovery is reduced from 10s to 3s, and the
  optimizer budget is bounded. This tune is covered by configuration tests but
  still requires staged physical bypass acceptance.
- GPS mode now uses a configurable `200 x 200m @ 0.25m` global rolling map,
  with a one-million-cell guard, so goals beyond the old 20m radius can be
  planned without coarsening the local obstacle map.
- The obstacle efficiency increment raises reverse speed, symmetric TEB
  acceleration/turn authority, and bounded Ackermann recovery speed while
  preserving clearance, minimum turning radius, and recovery distance limits.

The user reported that the preceding low-speed vehicle acceptance passed on
2026-07-15. That result does not yet accept the new obstacle-response tune or
establish the chassis acceleration/braking envelope.

This checkpoint does **not** complete all items in Batch 1 or Batch 2. In
particular, GPS-health fail-closed gating, the M2 command watchdog, heading-loss
health state, covariance/validity reporting, and GNSS epoch/CRC hardening remain
open in the order documented below.

Until Batch 1 is completed and verified on a safely lifted or restrained robot,
do not treat unattended navigation as safe.

## Working Rules

- Fix one batch at a time and keep unrelated submodule changes separate.
- Add a regression test before or together with every core behavior fix.
- After each batch, run a clean out-of-tree Catkin build and the relevant tests.
- Before a real driving test, first test with wheels lifted or the vehicle
  restrained, then use a low-speed open-area test.
- Do not hide a failed localization or sensor state by publishing fresh
  timestamps for stale data.
- Safety gates and watchdogs must fail closed: invalid state means zero velocity.

## Batch 1: Motion Safety and Emergency Stop Chain

Priority: critical. Complete this batch before navigation tuning.

### 1.1 Correct reverse Ackermann steering

File:
`src/autolabor_core/autolabor_canbus_driver/autolabor_canbus_driver/src/m2_driver.cpp`

Problem: `handle_twist_msg()` uses `abs(target_vel)` when converting requested
yaw rate to steering angle. With negative linear velocity, the resulting yaw
rate has the opposite sign from the command.

Required work:

- Use signed velocity in the Ackermann conversion.
- Define behavior for zero or near-zero linear velocity explicitly.
- Add forward, reverse, saturation, and near-zero unit tests.

Acceptance:

- For positive and negative velocity, reconstructed
  `v * tan(steer) / wheelbase` has the requested yaw-rate sign.
- Reverse recovery no longer turns opposite to the TEB command.

### 1.2 Make localization loss stop the robot

Files:

- `src/scripts/robot_bringup/scripts/gps_goal_speed_limiter.py`
- `src/application/gps_module/scripts/gps_localization_node.py`
- GPS-mode costmap configuration

Problem: missing or stale GPS odometry currently causes the limiter to forward
the planner command. Costmaps also tolerate transforms up to five seconds old.

Required work:

- Change the GPS safety gate to publish zero on missing, stale, invalid, or
  frame-mismatched localization.
- Publish an explicit localization-health state with a reason.
- Reduce transform tolerance to a value justified by measured sensor latency.
- Require a healthy state for a short, configurable recovery interval before
  motion resumes.

Acceptance:

- Disconnecting GPS or freezing its timestamp produces zero velocity within the
  configured timeout.
- Reconnection does not cause an immediate uncontrolled resume.

### 1.3 Add a software command watchdog to the M2 driver

File:
`src/autolabor_core/autolabor_canbus_driver/autolabor_canbus_driver/src/m2_driver.cpp`

Required work:

- Track the last accepted command using a monotonic timeout mechanism.
- Send an explicit zero/brake command after timeout.
- Define interaction with emergency stop and hardware timeout reporting.
- Test planner exit, ROS publisher loss, and frozen command streams.

Acceptance:

- Killing the only command publisher always stops the chassis within the
  configured bound.

### 1.4 Enforce a single command source

Files:

- `scripts/bringup.sh`
- M2 driver launch/configuration

Required work:

- Clean up stale `gps_goal_speed_limiter` and keyboard teleop nodes.
- Reject unexpected additional `/cmd_vel` publishers during startup.
- Introduce or clearly define command arbitration for `/cmd_vel` and
  `/ackerman_vel`.

## Batch 2: GPS State Estimation and Data Integrity

Priority: critical/high.

### 2.1 Require valid dual-antenna heading

Problem: startup does not wait for valid heading, and a heading timeout silently
reuses the previous yaw while publishing newly stamped pose and TF messages.

Required work:

- Require a heading solution that passes configured status/type checks.
- Include `/gps/heading` data validity in startup readiness.
- Mark localization unhealthy when heading becomes stale in dual-antenna mode.
- Do not disguise frozen yaw as a fully fresh localization result.

### 2.2 Add GPS jump recovery

File: `src/application/gps_module/scripts/gps_localization_node.py`

Problem: a position more than `max_fix_jump` from the filtered pose is rejected
forever because the reference is never advanced or reacquired.

Required work:

- Add consecutive-rejection and elapsed-time tracking.
- Use speed/time-aware gating instead of a single fixed-distance decision.
- Add an explicit reacquisition state and controlled reset policy.
- Test RTK convergence, initial bad fixes, outage while moving, and recovery.

Acceptance:

- A stable new RTK position is reacquired after the configured validation
  period, while isolated outliers remain rejected.

### 2.3 Publish physically correct odometry

Required work:

- Compute signed longitudinal velocity; reverse must be negative.
- Populate angular velocity from wheel feedback or validated yaw differences.
- Define covariances and validity for pose and twist.
- Correct or disable the current wheel-odometry fusion until map/odom rotation
  and asynchronous prediction are implemented properly.

### 2.4 Harden serial parsing and GNSS quality control

Required work:

- Validate NMEA checksums and UNIHEADING CRC.
- Reject non-finite and out-of-range values.
- Gate navigation on configured RTK quality, HDOP, satellite count, and heading
  solution type.
- Catch per-sentence errors and reconnect after serial disconnection.
- Merge GGA, RMC, and heading by GNSS epoch. Do not rate-limit every input line
  or stamp buffered old data with the current ROS time.

### 2.5 Reduce estimator latency

Review the default EMA (`alpha=0.25` at 10 Hz), which can lag about 0.45 m at
1.5 m/s. Replace it with a low-latency estimator or compensate using a tested
motion model so goal braking is based on current position.

## Batch 3: FAST_LIO Runtime Stability

Priority: critical/high.

### 3.1 Remove the deterministic profiling-array overflow

File: `src/localization_fastlio/FAST_LIO/src/laserMapping.cpp`

Problem: `scan_count` writes into fixed arrays of length 720000 without a bounds
check. At 10 Hz, the first out-of-bounds write occurs after about 20 hours.

Required work:

- Remove unused unconditional profiling storage or replace it with a bounded
  ring buffer.
- Ensure logging-disabled operation performs no unbounded profiling writes.
- Add a long-run counter/bounds test.

### 3.2 Bound all ROS and synchronization queues

Current subscriber queues reach 200000 and publisher queues reach 100000.
Sensor synchronization buffers can also grow when one stream stops.

Required work:

- Choose bounded queues based on measured rates and acceptable latency.
- Drop old data deliberately and report the drop/staleness condition.
- Add diagnostics for LiDAR/IMU skew, queue depth, and processing lag.

### 3.3 Verify logging defaults

Keep path publishing and PCD saving disabled by default for production. Prevent
an unlimited single-PCD accumulation mode from being enabled accidentally.

## Batch 4: Navigation Geometry and Long-Range Goals

Priority: high.

### 4.1 Align GPS ENU and FAST_LIO frames automatically

Problem: GPS ENU x points east, while FAST_LIO `camera_init` yaw depends on the
robot's startup orientation. A default static yaw offset of zero is only correct
when the robot starts in the assumed direction.

Required work:

- Estimate and publish an explicit ENU-to-`camera_init` transform from the
  simultaneous dual-antenna and FAST_LIO orientations.
- Validate translation, antenna offset, and restart behavior.
- Refuse GPS goals until alignment is valid.

### 4.2 Support goals outside the 40 x 40 m rolling costmap

Status: implemented in the current worktree; static launch expansion and
configuration tests pass. A motion-disabled `/move_base/make_plan` check at
55m+ and staged field validation remain.

Implemented strategy:

- GPS mode passes a configurable global costmap size/resolution through all
  launch layers; the default is `200 x 200m @ 0.25m` (`640,000` cells).
- Width, height, resolution, and initial origin are overridden together after
  the base YAML/profile, while the local costmap stays at `0.1m` resolution.
- bringup rejects a global grid above one million cells before hardware start.

Test goals inside, on the boundary of, and far outside the current window.

### 4.3 Review reverse behavior in the active TEB carlike mode

The configured `weight_kinematics_forward_drive` is not applied by the current
carlike edge path. Decide whether reverse is allowed, then enforce that policy in
an effective constraint rather than relying on an inactive weight.

## Batch 5: Reproducible Build, Submodules, and CI

Priority: high; required before sharing or deploying from a fresh machine.

### 5.1 Make Livox ROS1 setup reproducible

Problems:

- `livox_ros_driver2/package.xml` is ignored and generated locally.
- The upstream build script assumes a different directory layout.
- Livox SDK2 installation under `/usr/local` is an undocumented prerequisite.

Required work:

- Provide a tracked ROS1-ready wrapper/fork or a root bootstrap command.
- Remove directory-layout assumptions.
- Document and check the SDK2 installation explicitly.

### 5.2 Publish reachable submodule commits

Required work:

- Commit/publish the FAST_LIO Livox2 adaptation and launch parameters.
- Publish the Arena commit referenced by the parent gitlink.
- Preserve the required C++17/Qt compatibility fixes in reachable forks.
- Verify `git clone --recurse-submodules` in an empty directory.

### 5.3 Complete dependency manifests

At minimum address:

- `python3-serial` for `gps_module`;
- `rosgraph`, `pointcloud_to_laserscan`, and `tf` where used;
- `eigen_conversions` for FAST_LIO;
- Arena launch-time dependencies;
- the `pika` dependency for the RabbitMQ bridge;
- unresolved rosdep keys for optional packages.

### 5.4 Add minimum CI

CI should start from a recursive clean clone and run:

1. dependency/bootstrap verification;
2. clean Catkin build;
3. core unit tests;
4. launch argument/include validation;
5. Python 3, shell, XML, and YAML checks.

Core tests must cover reverse steering, GPS jump recovery, heading timeout,
localization fail-stop, signed odometry, frame alignment, and long goals.

## Batch 6: Legacy and Maintenance Debt

Priority: medium; isolate from production work where possible.

- Replace obsolete `CV_FILLED` in the `light_scan_sim` test and restore the
  fifth registered test.
- Port or exclude the ten Python 2 Arena/Pedsim scripts under Noetic.
- Repair or remove the truncated
  `pedsim_gazebo_plugin/worlds/social_activities.world`.
- Fix non-void C++ functions without returns and format-string mismatches.
- Remove obsolete `/home/robot/arena_ws/...` absolute launch paths.
- Add a `use_rviz` option and disable evaluation visualization in headless
  production launches.
- Complete install-space rules or document that only devel-space is supported.

## Audit Baseline

The 2026-07-15 audit established the following baseline:

- The current locally modified worktree completes an isolated clean Catkin
  build and discovers 68 buildable packages.
- Primary CAN, Livox, FAST_LIO, scan, GPS, and navigation launches resolve.
- 69 package manifests, 143 launch XML files, 676 YAML files, and 16 shell
  scripts pass syntax/parse checks.
- Four registered tests pass. The fifth, `light_scan_sim`, does not compile with
  OpenCV 4 because it uses `CV_FILLED`.
- Ten optional legacy Python files fail Python 3 syntax checks; current
  first-party GPS/bringup Python scripts compile.
- The main GPS/navigation safety paths have no dedicated automated tests or CI.
- No connected GPS/Livox hardware test was performed during this audit.

## Resume Instruction

For a later Codex session, use a request such as:

> Read `AGENTS.md`, `CURRENT_GPS_DEV_HANDOFF.md`, and
> `PROJECT_HEALTH_FIX_ROADMAP.md`. Start Batch 1, keep existing unrelated changes,
> add regression tests, and stop before any real-hardware motion test.
