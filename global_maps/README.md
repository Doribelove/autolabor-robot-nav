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
├── map_2d/       # temporally confirmed MID360 full-azimuth map_server PGM/YAML
└── map_fused_2d/ # MID360 grid plus its persistent fixed-height 3-D slice
```

The MID360 grid uses FAST-LIO `/Odometry` as its trajectory. An occupied cell
must be observed in at least five distinct integrated MID360 scans. The third map is created after recording
stops by adding only MID360 slice cells observed in at least 20 distinct cloud
frames. The default band is `Z=-0.4+/-0.2 m`, or `[-0.6,-0.2] m`, in the level
initial FAST-LIO `camera_init` frame. With the current IMU `body` origin at
about `0.95588 m`, it corresponds to approximately `0.356-0.756 m` above the
level ground. MID360 points beyond 20 m from the matching FAST-LIO pose are
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
only when it contains `/cloud_registered`, `/mid360/scan` and `/Odometry`:

```bash
./scripts/build_static_map_from_bag.sh rosbags/example.bag example_map
```

The current combined branch pins its default prior to
`map_20260829_221335_gimp_cleaned` in `config/conbine_live.json`.
New `latest` map sets do not replace that default. Current commands are:

```bash
bash scripts/conbine.sh live-start
# Stop the live session before starting a new map-free mapping session:
bash scripts/conbine.sh live-stop
bash scripts/conbine.sh mapping-start
```

In map mode, FAST-LIO remains the original high-rate odometry. The separate
`fast_lio_localization` node loads `map_3d/map.pcd`, waits for an approximate
`/initialpose`, and applies low-rate coarse-to-fine ICP corrections through
`map -> camera_init`. `map_server` loads the selected 2-D grid for move_base;
AMCL is not used. Navigation velocity remains blocked until localization is
`LOCALIZED`.
