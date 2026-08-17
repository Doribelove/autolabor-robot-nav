# Autolabor M2 dual LD19 avoidance enhancement

## Data path

FAST-LIO continues to use only the MID360 custom point cloud and IMU. MoveBase
avoidance uses a separate raw-sensor path:

```text
/livox/lidar -> /mid360/scan -------------------+
                                                   +-> /scan -> MoveBase
front LD19 + rear LD19 -> /dual_lidar/scan -------+
/livox/lidar + /livox/imu -> FAST-LIO -> /Odometry
```

The scan fusion keeps the closest finite return in each angular bin. The
required MID360 scan drives output; an absent or stale optional scan is never
allowed to keep `/scan` alive by itself. This makes a MID360 outage fail closed
while an LD19 outage falls back to MID360.

The measured front/rear rotation-centre distance is 0.94 m, so the default
poses are `x=+0.47 m` and `x=-0.47 m`. The USB devices use physical `by-path`
names because both CH340 adapters have the same USB identity.

## Automatic use

`scripts/bringup.sh` defaults to `DUAL_LIDAR_MODE=auto` and
`DUAL_LIDAR_USE_FOR_SCAN=true`:

- both configured USB paths exist: publish `/dual_lidar/scan` and merge it into
  MoveBase `/scan`;
- either path is absent or unwritable: keep MID360-only `/scan`;
- a live LD19 scan becomes stale for 0.35 s: immediately return to MID360-only
  `/scan`.

The normal command is unchanged:

```bash
./scripts/bringup.sh fast_lio
```

Set `DUAL_LIDAR_USE_FOR_SCAN=false` to acquire the LD19 units without using
them for navigation. Disable all automatic LD19 handling with
`DUAL_LIDAR_MODE=off`.

## Manual component start

Use this only for direct LD19 diagnostics:

```bash
cd /home/slam/robot_ws
source .deps/setup.bash
roslaunch autolabor_dual_lidar dual_ld19_fused.launch \
  output_topic:=/dual_lidar/scan target_frame:=base_link
```

Important topics:

- `/dual_lidar/front/scan_raw`
- `/dual_lidar/rear/scan_raw`
- `/dual_lidar/scan`
- `/mid360/scan`
- `/scan`
- `/avoidance/dual_lidar_active`
- `/avoidance/source_mode`

When FAST-LIO is also running, the optional
`/cloud_registered_body_enhanced` topic remains available for RViz/debug only;
MoveBase does not consume it. Standalone 2D visualization is available with
`dual_ld19_fused.launch start_rviz:=true`.
