# Hikrobot MVS Camera for ROS Noetic

This package publishes a Hikrobot industrial camera through the official MVS
SDK. It is configured for the detected `MV-CU013-A0UC` camera with serial
`DA7535899` and enumerates only USB cameras by default.

The default output matches the FOD perception sidecar:

```text
/fod_camera/image_raw    sensor_msgs/Image (bgr8)
/fod_camera/camera_info  sensor_msgs/CameraInfo
```

## Start

Close the MVS desktop client first because the camera is opened exclusively,
then run:

```bash
cd /home/robot/robot_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch hikrobot_mvs_camera fod_camera.launch
```

The node stays alive and retries if the camera is unplugged. Check the stream:

```bash
rostopic hz /fod_camera/image_raw
rqt_image_view /fod_camera/image_raw
```

The driver also exposes runtime controls used by the FOD image-quality
controller:

```text
/fod_camera/driver/get_imaging_controls
/fod_camera/driver/set_imaging_controls
```

The query returns current values and the camera-reported hardware ranges. The
setter can switch exposure and gain between `Continuous` and manual mode and
clamps manual values to those hardware ranges. If a multi-feature update
fails, the driver attempts to restore the preceding exposure/gain state.

The defaults use the calibrated full-sensor `1280x1024` output and free-running
acquisition at 20 Hz. External line triggering can be enabled explicitly:

```bash
roslaunch hikrobot_mvs_camera fod_camera.launch \
  trigger_mode:=true trigger_source:=Line0
```

## MVS client

Run the installed client with:

```bash
hikrobot-mvs
```

The wrapper keeps MVS runtime logs under `/opt/MVS/bin/Temp` instead of the
directory from which it was launched. Close the client before starting the ROS
node.

## Calibration

The supplied `camera_pinhole_my.yaml` is preserved as the original reference.
It uses the FAST-LIVO2/rpg_vikit Pinhole schema. Its active `cam_fx`, `cam_fy`,
`cam_cx`, `cam_cy` and four distortion coefficients were converted to the
standard ROS file
`config/fod_camera_1280x1024.yaml`, which is loaded by default. The four source
distortion values are interpreted as OpenCV `k1, k2, p1, p2`; ROS receives a
fifth radial term `k3=0`.

These values are valid only for the physical camera/lens/focus combination
that was calibrated. If the file came from another camera or the lens/focus
has changed, recalibrate before relying on metric ground projection.

To use a replacement standard ROS calibration file:

```bash
roslaunch hikrobot_mvs_camera fod_camera.launch \
  camera_info_url:=file:///home/robot/.ros/camera_info/fod_camera.yaml
```

The calibration resolution must match the camera output resolution. The
default launch enforces `image_width:=1280 image_height:=1024 offset_x:=0
offset_y:=0` and refuses to stream if the camera cannot apply that ROI.

## Installation boundary

The installed release is MVS 5.0.1 Build 20260512 for Linux x86_64. The
official archive is:

```text
https://www.hikrobotics.com/cn2/source/support/software/MVS_Linux_STD_V5.0.1_260512.zip
SHA-256 cd6c4e3352afb1f6395b9be8a692b4fa8a911ae7eea7ff1f9181970f221bf264
```

MVS is installed in `/opt/MVS`. Runtime linking is provided by
`/etc/ld.so.conf.d/hikrobot-mvs.conf`, and access to USB vendor `2bdf` is
provided by `/etc/udev/rules.d/80-hikrobot-mvs.rules`. The safe client launcher
is `/usr/local/bin/hikrobot-mvs`.

The MVS directory also contains an older private `libusb-1.0.so.0`. Navigation
bringup isolates only its FAST_LIO child onto the Ubuntu system libusb, while
this camera launch keeps the normal MVS runtime environment. Consequently the
camera can run in a second terminal at the same time as
`./scripts/bringup.sh gps ...`; no global `LD_LIBRARY_PATH` override is needed.

The vendor package's broad post-install script is deliberately not used: it
would edit user shell profiles, install boot services, modify network sysctls,
and load unrelated GigE/PCIe drivers. This USB camera package only needs the
SDK files, dynamic-library entry, and targeted udev rule.
