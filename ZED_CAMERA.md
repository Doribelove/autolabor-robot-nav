# ZED 2 Camera

This workspace uses the following ROS 1 integration:

- Stereolabs ZED SDK 4.2.5 for JetPack 5.1.2 / L4T 35.4
- `stereolabs/zed-ros-wrapper` commit `6e5ab7d` as a Git submodule
- ROS Noetic on Ubuntu 20.04
- Installed ZED 2 serial number `23748636`

The robot currently runs JetPack 5.1.3 / L4T 35.5. Stereolabs does not
publish a dedicated L4T 35.5 SDK build, so the closest CUDA 11.4-compatible
L4T 35.4 build is used and must be validated on the physical camera after
installation.

The wrapper is pinned to its final ROS 1 master commit because the older
v4.0.8 release still refers to SDK enums removed in ZED SDK 4.2 and does not
compile against 4.2.5.

## Clone and build

Initialize the wrapper and its nested interface submodule after cloning:

```bash
git submodule update --init --recursive
./scripts/build_workspace.sh
```

The ZED SDK is a system dependency installed under `/usr/local/zed`; it is not
stored in Git.

## Start the camera

```bash
cd /home/slam/robot_ws
./scripts/zed2_camera.sh
```

The project launch disables ZED's `map -> odom` and `odom -> base_link` TF
publication by default so it does not compete with the existing localization
stack. The rectified RGB image, CameraInfo, and registered metric depth use the
application's stable `/fod_camera/image_raw`, `/fod_camera/camera_info`, and
`/fod_camera/depth_registered` names. Point cloud, IMU, pose, and odometry
remain under `/zed2`.

For low-level debugging with the original ZED RGB names:

```bash
./scripts/zed2_camera.sh publish_fod_aliases:=false
```

To run the ZED visual odometry TF as a standalone localization source:

```bash
./scripts/zed2_camera.sh publish_tf:=true publish_map_tf:=true
```

Set the measured camera mounting transform when integrating it on the robot:

```bash
./scripts/zed2_camera.sh \
  cam_pos_x:=0.0 cam_pos_y:=0.0 cam_pos_z:=0.0 \
  cam_roll:=0.0 cam_pitch:=0.0 cam_yaw:=0.0
```

Useful checks while the node is running:

```bash
rostopic list | grep -E '^/zed2/|^/fod_camera/'
rostopic hz /fod_camera/image_raw
rostopic hz /fod_camera/depth_registered
rostopic echo -n 1 /fod/detections
rostopic hz /zed2/zed_node/point_cloud/cloud_registered
rostopic echo -n 1 /zed2/zed_node/imu/data
```
