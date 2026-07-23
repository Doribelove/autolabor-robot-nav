# Autolabor Operator GUI

ROS Noetic desktop console built with C++14, Qt5 Widgets and `librviz`.
It is an optional visualization and operator layer: it does not replace the
existing `scripts/bringup.sh`, RabbitMQ terminal bridge, or GPS test menu.

## First version

- Embedded RViz with laser scan, costmaps, GPS odometry, global plan, TEB plan,
  robot model and navigation goal. Its Displays dock is hidden by default and
  can be reopened from the overview page.
- Freshness cards for ROS master, CAN, GNSS, dual-antenna heading, laser scan,
  move_base, RabbitMQ and rosbag recording.
- GPS fix, local odometry, heading, velocity and existing static-drift metrics.
- Structured RabbitMQ status/latest-target display and Trigger service buttons.
- Safe 8 m forward GPS goal generation, move_base GoalID cancellation, static
  error reset, and start/stop of the existing mode1 rosbag script.
- Reserved pages for future camera/YOLO and cleaning-device integration.
- High-DPI-aware, maximized layout with larger field-readable type, horizontal
  navigation tabs, balanced status cards, and resizable RViz/event splitters.

The GUI never publishes `/cmd_vel`.
The embedded RViz intentionally omits the direct `SetGoal` tool; use the guarded
8 m action here, or the preserved standalone RViz workflow for advanced manual
goal placement.

## Build

```bash
cd /home/robot/robot_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=''
source devel/setup.bash
```

The explicit empty `CATKIN_WHITELIST_PACKAGES` also clears any package-only
build selection left in the CMake cache, so later builds continue to include
the complete legacy workspace.

## Run

The existing robot stack remains the primary startup path. When using the
embedded RViz, suppress only the old standalone RViz window:

```bash
NAV_START_RVIZ=false ./scripts/bringup.sh gps 0.3 cruise
```

Start the optional GUI in another terminal:

```bash
./scripts/operator_gui.sh
```

Running `./scripts/bringup.sh gps 0.3 cruise` without `NAV_START_RVIZ=false`
keeps the original standalone-RViz workflow unchanged; simply do not start the
optional GUI in that case.

For CI, remote shells, or a machine without OpenGL:

```bash
./scripts/operator_gui.sh enable_rviz:=false
```

The launch file starts the existing `gps_static_error_monitor` by default so
the error cards populate when `/gps/odom` arrives. Disable it when another
instance is already managed elsewhere:

```bash
./scripts/operator_gui.sh start_gps_error_monitor:=false
```

## Degraded startup behavior

The process deliberately creates no ROS subscribers, publishers, services or
RViz instance until a background master probe succeeds. This prevents roscpp
registration retries from blocking the Qt window when `roscore` is absent.
Missing CAN, GNSS, lidar, navigation or RabbitMQ nodes merely show as offline;
they do not prevent the other pages from opening. When a master appears later,
the interface registers automatically.

The RabbitMQ actions call:

- `/rabbitmq_bridge/publish_latest` (`std_srvs/Trigger`)
- `/rabbitmq_bridge/clear_latest` (`std_srvs/Trigger`)

The 8 m action requires fresh `/gps/odom` and `/move_base/status`, finite
`/gps/origin_lat` and `/gps/origin_lon`, and at least one `/gps/goal_fix`
subscriber. It publishes a `sensor_msgs/NavSatFix` to `/gps/goal_fix`.
Cancel publishes an empty `actionlib_msgs/GoalID` to `/move_base/cancel`.
