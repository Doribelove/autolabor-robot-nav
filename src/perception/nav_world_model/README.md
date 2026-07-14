# nav_world_model

FAM-TEB V2 local geometry, dynamic tracking/prediction and health package.

- `nav_world_model_skeleton_node.py` remains the default fail-closed heartbeat and always
  publishes `valid=false`.
- `nav_world_model_node.py` is the V2-03 uncalibrated simulation candidate. It validates the
  complete LaserScan contract, computes local geometry, performs deterministic scan-cluster
  tracking and constant-velocity prediction, and publishes health.

The V2-03 node only starts when the Gazebo simulation marker and
`allow_unfrozen_simulation_candidate=true` are both present. It does not read scene labels,
Gazebo/Pedsim truth, publish velocity commands, or write TEB parameters. Current thresholds remain
`runtime_ready=false` and are not real-vehicle calibration.
