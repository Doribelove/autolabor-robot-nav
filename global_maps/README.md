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

## Static 2-D maps

Qt fused-mapping sessions and `build_static_map_from_bag.sh` write standard
`map_server` files below `global_maps/static_maps/<session>/`:

- `map.pgm` and `map.yaml`: static occupancy grid;
- `mapping_info.yaml`: source topics, grid statistics and first/final pose;
- `session_info.yaml`: live Qt session and component-scan provenance.

`global_maps/static_maps/latest` is switched only after all required files have
been saved successfully. With `STATIC_MAP_ENABLED=true`, `start_dual_host.sh`
synchronizes this selected map to J6M before launching `map_server`, AMCL and
move_base.

Build a 2-D map from an existing bag with:

```bash
./scripts/build_static_map_from_bag.sh rosbags/example.bag example_map
```
