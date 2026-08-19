# ROS bag storage

Qt mode1 recordings are stored here by default. Only complete `.bag` files
should be used for offline mapping; a `.bag.active` file is still recording or
was not closed cleanly.

Build the newest bag with:

```bash
./scripts/build_global_map.sh
```

