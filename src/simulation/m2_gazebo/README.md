# Autolabor M2 Gazebo candidate

This package is an isolated, simulation-only M2 interface for thesis work. It
does not load the serial/CAN driver and it does not modify Arena robot models.

The initial model is a deterministic kinematic Ackermann/bicycle model. Its
geometry and limits are explicitly uncalibrated candidates in
`config/simulation_candidates.yaml`; it is not yet a tire, suspension, latency,
or braking dynamics model.

Quantitative chassis regression without `move_base` (11 cases, about one minute):

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch m2_gazebo m2_regression.launch run_test:=true seed:=42 \
  report_path:=/tmp/m2_chassis_regression.yaml
```

Fixed-TEB simulation (starts no goal and therefore no motion by itself):

```bash
roslaunch m2_gazebo m2_fixed_teb.launch
```

Fixed-TEB navigation regression without RL (five goals):

```bash
roslaunch m2_gazebo m2_fixed_teb_regression.launch seed:=42 \
  report_path:=/tmp/m2_fixed_teb_regression.yaml
```

Both regression launches return a non-zero status if any threshold fails. Their
YAML reports contain per-case measurements, thresholds, pass/fail state, and
failure text. The chassis suite covers spawn/reset, static stability, 5 m and
10 m straight motion, low-speed reverse, left/right fixed-radius circles, stop
response, laser ranging, TF integrity, and seed/reset repeatability. The TEB
suite covers straight, left, right, obstacle-detour, and 1.8 m corridor goals,
including action result, path availability, clearance, command gaps, planner
errors, and controller deadline misses.

The simulation uses `odom -> base_link` and publishes `/odom`; it intentionally
does not fabricate `/gps/odom`. `/cmd_vel.angular.z` is a target yaw rate and
`cmd_angle_instead_rotvel` remains false. `/ackerman_vel.angular.z` is the direct
center steering angle, matching the checked-in M2 driver source.

The measured zero braking distance belongs to this deterministic kinematic
model only. It is not a real M2 braking estimate and remains a sim-to-real TBD.
