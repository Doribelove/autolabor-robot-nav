# ROS bag storage

Qt fused-mapping recordings are stored here by default. The same button starts
the bag recorder and the live 2-D mapper; stopping it closes the bag, saves the
static map and updates `global_maps/static_maps/latest`. Only complete `.bag`
files should be used for offline mapping; a `.bag.active` file is still
recording or was not closed cleanly.

Build the newest bag with:

```bash
./scripts/build_global_map.sh
```

For the move_base PGM/YAML map instead of the legacy 3-D PCD map, run:

```bash
./scripts/build_static_map_from_bag.sh rosbags/example.bag example_map
```
