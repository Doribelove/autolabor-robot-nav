# FAST-LIO known-map localization

This ROS Noetic package integrates the architecture of
[HViktorTsoi/FAST_LIO_LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION)
into the NVIDIA + J6M runtime. The upstream package combines high-frequency
FAST-LIO odometry with low-frequency, coarse-to-fine scan-to-map ICP and a
`map -> camera_init` correction.

Reference revision inspected for this adaptation:
`2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2` (`main`).

The upstream implementation targets Python 2 and Open3D 0.7/0.9. This package
is a C++14/PCL adaptation for ROS Noetic and ARM64; it does not copy the old
Python runtime. It retains the upstream GPL-2.0 licensing designation and
publishes project-specific health state for the navigation velocity gate.

## Runtime graph

```text
FAST-LIO: camera_init -> body (high-rate odometry)
                          |
current /cloud_registered + prior PCD + /initialpose
                          |
coarse ICP -> fine ICP -> map -> camera_init
                          |
                    map -> body /localization
```

The point cloud published by FAST-LIO is already expressed in `camera_init`.
ICP therefore estimates `map_T_camera_init` directly. An RViz 2-D initial pose
is interpreted as the base pose; the configured MID360/base offset and current
FAST-LIO odometry are used to derive the initial map-to-odometry transform.

Velocity is permitted only while `/fast_lio/localization_status` starts with
`state=LOCALIZED;`. Failed ICP, stale scan/odometry, or an expired successful
match changes the state and closes the gate.

This remains rough-initial-pose localization, like the referenced upstream
implementation. It is not descriptor-based kidnapped-robot global search.
