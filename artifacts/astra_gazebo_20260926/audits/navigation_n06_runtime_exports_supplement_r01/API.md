# ROS package export input supplement

This adds only the three omitted exporter manifests to the 5,214-file disk roster: gazebo_plugins, simulator_setup and turtlebot3_gazebo. m2_gazebo and gazebo_ros manifests were already bound. The actual read-only rospack queries independently repeat 19 plugin, 19 media and 20 model paths. Each output is joined to its package XML and exact `${prefix}` expansion; repeated entries and original path spelling are preserved.

Every configured plugin filename is checked in every exported plugin directory. Six candidates exist, all aliases of the same six already-bound ROS libraries. The supplement adds no configured ELF and does not claim the native SystemPaths plugin executed. The versioned upstream 2.9.3 source explains the ROS export inputs and GAZEBORC override; source-to-installed-binary equivalence is not newly proven here.

GAZEBORC is derived from the actual `rospack find gazebo_ros` package directory plus `/.do_not_use_gazeborc`. That path is currently absent. Absence is a dated disk observation, not an immutable file binding or guarantee against future filesystem changes. The exact eight world/URDF inputs contain no external model/media URI; this does not qualify every unused asset tree behind exported search directories. Actual runtime mapping and resource selection remain future evidence requirements.

The only environment override for rospack queries is ROS_HOME into this supplement's disposable cache directory. This prevents writes to any future run's ROS_HOME. These commands neither start a ROS node/master nor initialize Gazebo. The query-tool binary and its already-bound ELF dependencies are saved as evidence-tool provenance; it is not a new runtime node.

`collect_exports.py --output /ABS/NEW_EMPTY_AUDIT_DIR` refuses overwrite. `attempt01/supplement.json` is the frozen actual collection. Closure r02 copies the parent author closure and merges exactly the three added manifest bindings, links this supplement and the formal negative review. It remains author/pending until independent incremental review; original closure r01 is unchanged and no release/seed authority is created.
