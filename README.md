# Autolabor Arena Navigation

This repository contains the working ROS Noetic setup for the Autolabor chassis,
Livox MID360, FAST-LIO, pointcloud-to-laserscan, and Arena navigation launch flow.

## Workspace Layout

- `autolabor_ws`: Autolabor chassis driver and Livox/FAST-LIO workspace content.
- `arena_ws`: Arena navigation workspace content.
- `start_autolabor_arena.sh`: One-command startup script for the real robot.

## Build Order

```bash
cd ~/autolabor-arena-nav/autolabor_ws
catkin_make

cd ~/autolabor-arena-nav/autolabor_ws/src/livox/Mid_livox_ros_driver2
catkin_make

cd ~/autolabor-arena-nav/arena_ws
catkin_make
```

## Startup

On the robot computer:

```bash
cd ~/autolabor-arena-nav
./start_autolabor_arena.sh
```

The startup script automatically chooses `/dev/ttyUSB0` or `/dev/ttyUSB1` for
the Autolabor CAN bus driver and passes the selected port into
`drive_only.launch`.

## Notes

- Build outputs such as `build/`, `devel/`, `install/`, and `log/` are ignored.
- Large example media and the oversized `agv-ota` STL mesh are ignored because
  GitHub rejects files larger than 100 MB without Git LFS.
- The current real navigation launch flow uses the `dingo` model by default.
