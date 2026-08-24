# Global map storage

Each offline FAST-LIO job creates one subdirectory containing:

- `global_map_raw.pcd`: full accumulated point cloud;
- `global_map.pcd`: voxel-filtered point cloud for normal viewing;
- `generation_info.txt`: source bag and processing parameters;
- processing logs and a copy of `rosbag info`.

View the latest completed map on the robot desktop with:

```bash
./scripts/view_global_map.sh
```

## Three-map static map sets

Qt's `录入静态地图` workflow and `build_static_map_from_bag.sh` write one
self-contained map set below `global_maps/map_sets/<session>/`:

```text
<session>/
├── manifest.yaml
├── map_3d/       # temporally persistent MID360 /cloud_registered voxel PCD
├── map_2d/       # temporally confirmed front/rear LD19 map_server PGM/YAML
└── map_fused_2d/ # LD19 grid plus a persistent fixed-height 3-D slice
```

The LD19 grid uses FAST-LIO `/Odometry` only as its trajectory; MID360 points
are not inserted into `map_2d`. An LD19 occupied cell must be observed in at
least five distinct integrated scans. The third map is created after recording
stops by adding only MID360 slice cells observed in at least 20 distinct cloud
frames. The default band is `Z=-0.4+/-0.2 m`, or `[-0.6,-0.2] m`, in the level
initial FAST-LIO `camera_init` frame. With the current IMU `body` origin at
about `0.95588 m`, it corresponds to approximately `0.356-0.756 m` above the
level ground and supplements the lower LD19 scan plane with mid-height
obstacles. MID360 points beyond 20 m from the matching FAST-LIO pose are
discarded before accumulation.

The persistent MID360 slice is additionally protected against moving-body map
artifacts. Each registered cloud is exact-time paired with FAST-LIO odometry;
slice points inside the contemporaneous `base_link` self envelope
(`X=[-0.75,0.75] m`, `Y=[-0.50,0.50] m`) are ignored. Before saving, the
mapper subtracts every grid cell intersected by the padded navigation footprint
(`1.24 x 0.90 m`) swept along the interpolated mapping trajectory. This affects
only `slice_observations.yaml` and the fused 2-D map, not `map_3d/map.pcd`.
The fuser requires schema-2 crop/sweep evidence and verifies that no accepted
slice cell remains in the swept set.

`global_maps/map_sets/latest` is switched atomically only after all three maps
and their configuration files are complete. A historical bag can be processed
only when it contains `/cloud_registered`, `/dual_lidar/scan` and `/Odometry`:

```bash
./scripts/build_static_map_from_bag.sh rosbags/example.bag example_map
```

Normal startup remains map-free incremental FAST-LIO. To load a map set:

```bash
./scripts/start_dual_host.sh --start --map-set global_maps/map_sets/latest
# Use the raw LD19 static grid instead of the fused grid when needed:
./scripts/start_dual_host.sh --start --map-set global_maps/map_sets/latest \
  --static-map-source lidar2d
```

In map mode, FAST-LIO remains the original high-rate odometry. The separate
`fast_lio_localization` node loads `map_3d/map.pcd`, waits for an approximate
`/initialpose`, and applies low-rate coarse-to-fine ICP corrections through
`map -> camera_init`. `map_server` loads the selected 2-D grid for move_base;
AMCL is not used. Navigation velocity remains blocked until localization is
`LOCALIZED`.
