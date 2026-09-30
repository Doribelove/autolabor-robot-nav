# B03 rear-reference integration — author source closure

This is a new source-only engineering package for `actual_teb_rear_reference_b03` and `dwa_rear_reference_b03`. It changes the planner reference, path/goal representation and supporting internal goal tolerances together. It is **not** a pure one-factor rear-reference performance experiment. No ROS master, Gazebo, build, new seed, release or episode was started for this work. B01, B02 and N04 files remain unchanged. Independent review and live qualification are pending.

## Actual odometry and TF

`astra_baseline_rear_adapter_b03.py` consumes the actual `/odom` (`odom` → `base_link`) and publishes `/astra/rear_odom` (`odom` → `rear_axle_kinematic`). The rear frame has the same orientation and the fixed body translation `(-0.325, 0, 0)`. Its unique static TF is `base_link` → `rear_axle_kinematic`; the node does not publish another `odom` transform and does not change the URDF.

The frozen B02 rigid transform supplies the full 3-D pose and body twist, including `v_rear = v_base + omega × (-0.325, 0, 0)`. Actual rear lateral velocity is preserved; it is never forced to zero. All 36 pose and twist covariance entries are transported. Pose covariance supports parent-axis infinitesimal fixed-axis rotation errors (a first-order covariance transformation), not arbitrary finite Euler-coordinate derivatives; twist covariance uses the full linear point-velocity Jacobian. All-zero upstream covariance conveys no uncertainty qualification.

Raw quaternion norm is checked before the existing Pose3 normalization. Inputs require exact canonical frames, finite pose/twist/covariance, valid covariance, and `abs(roll)+abs(pitch) <= 0.05 rad`. Source time and actual ROS time use integer nanoseconds. Duplicate/regressed odometry, clock rollback, more than 200 ms age/lead, queue overflow, wrong TF ownership or publication expiry permanently fault the adapter. A sample up to 200 ms ahead of local `/clock` waits unchanged until the clock covers it. A fresh ROS time is read immediately before publication. Stamp zero can prime bootstrap but never establishes readiness. Original receipt, source and availability times are recorded separately.

The node has no command publisher, Gazebo lifecycle service, model-state/contact subscription or oracle input. A graph guard requires a unique rear-odom publisher and `/gazebo` as the actual `/odom` authority. The graph guard runs periodically, not atomically with every transport callback. A permanent adapter fault is propagated to the coordinator and causes zero output on its existing control cycle; this is not a physical braking or instantaneous reaction guarantee.

## Fixed path and goal reference

The original common map and base-oriented guide stay available through the unchanged common provider. `/astra/baseline_rear/get_plan` uses the existing `astra_common_route/FixedRouteGlobalPlanner` service bridge; no C++ plugin or build change is needed. Every pose of the shared full guide is transformed using its own heading. Nonfinal headings follow the exact shared provider convention; the last pose retains the explicit shared goal yaw. The full sequence and ordering are preserved without smoothing, pruning, start insertion or online search. Base and rear oriented-path hashes are published.

The service maps the requested full rear start back to base using the rigid transform, retaining the original 2 m distance-to-guide guard. It inversely maps the planar rear goal and requires the exact shared base goal within the inherited 1e-6 comparison tolerance. The existing C++ bridge's final-XY check now compares the returned rear endpoint with the requested rear goal. Heading kinks can move adjacent rear points discontinuously; this is a reference conversion of a polyline, not proof of Ackermann feasibility.

`GetPlan` is a stateless request for an immutable geometric path. Its goal header may legitimately date from the earlier gate, and repeated requests are allowed. Request headers' frame IDs and raw poses are validated; their timestamps are not claimed to be sensor-generation or fresh-measurement evidence. Fresh actual odometry and adapter readiness are checked before serving. The strict source-time/replay policy above applies to the dynamic actual `/odom` stream. The common path is not changed in response to a request timestamp.

## Shared outcome versus private planner goal

The new pure coordinator has exactly the original N02 configuration/class ASTs; only the allowed method roster differs. It still observes **original actual base odometry** for warmup, gate and terminal decisions. The shared source-based goal remains 0.25 m XY / 0.25 rad yaw, with the original 22 s deadline and first-terminal immutability. It remains the sole `/cmd_vel_navigation` publisher. The selected planner publishes only `/astra/method_cmd`, and its action goal is sent only at the common gate.

At that gate, the new node sends the converted rear goal to `move_base`. Private action success is published as a diagnostic and cannot close the shared task. Internal planner XY/yaw tolerances are 0.15 m / 0.10 rad, with DWA `latch_xy_goal_tolerance=false`. For the declared tilt envelope, an internal current-pose success implies the conservative base-XY bound

`0.15 + 2*0.325*sin(0.10/2) + 0.325*(1-cos(0.05)) = 0.182892625... m < 0.25 m`.

Yaw remains at most 0.10 rad < 0.25 rad. This establishes set containment at the same physical pose; it does not assert ordering of asynchronous callbacks. The private result is diagnostic even if received before the corresponding actual-base packet. The node tests inject precisely such an early SUCCESS and retain an active shared task until qualifying actual base odometry arrives. No baseline success or simulated performance has been established.

TEB and DWA retain their actual implementations. DWA can still enter a stop/rotate mode incompatible with an Ackermann vehicle's inability to rotate in place. This package does not solve that issue, force rear lateral slip to zero or increase `acc_y`. A reversed sampled lateral interval is not claimed automatically invalid, because the existing iterator can still include zero.

## Footprints, launch and remaining integration

The new YAML is loaded last. Both costmaps use the rear TF, `/astra/rear_odom` feeds both planners, all 64 vertices of the existing Euclidean-0.33-expanded costmap footprint are translated by `(+0.325, 0)`, and all 16 vertices of TEB's existing raw envelope receive the same translation. Footprint padding stays zero; margins, speeds and physical parameters remain inherited. In planar geometry the world vertex set is identical. A tilted vehicle processed by yaw-only costmaps is not exactly equivalent: the previously recorded 0.00040616537 m example remains a limitation requiring an engineering-margin review. No new margin was silently added or removed.

The draft launch includes the unchanged P05 physical model, shared provider, relay and actual selected `move_base` plugin. It leaves spawn/unpause/pause solely to a **future compatible collector**, supplied by the required `collector_script` argument. The adapter and collector are required nodes. The command chain remains `/astra/method_cmd` → coordinator → `/cmd_vel_navigation` → relay → `/cmd_vel`. XML structure and exact config translation are tested, but no `roslaunch --dump-params`, build, ROS graph or live plugin qualification was performed.

Required launch arguments: `world`, `run_id`, `seed`, `method`, `bundle`, `bundle_sha256`, `steering_kd`, `contract_path`, `contract_sha256`, `output`, `bag_path`, `bag_topics`, `collector_script`. Optional defaults are `gui=false`, `python_executable=$THESIS_WS/.venv_train_cuda/bin/python`, and the explicit legacy TEB/DWA baseline config paths. A future release must bind the chosen baseline config, full effective parameters, interpreter and actual loaded libraries.

The future recorder/collector must add `/astra/rear_odom`, `/astra/baseline_rear/{identity,status,input_receipt,served_path,private_action_result}` and `/move_base/FixedRouteGlobalPlanner/provided_path` to the inherited odom/TF/map/common-route/planner/command/evidence roster. It must validate B03 method identities, rear status readiness, both reference-path hashes and the original actual-base terminal authority. Current frozen collectors and actor metadata method whitelists are not claimed B03-compatible. Root owns that new stage, actor metadata, source/runtime closure, seeds and release. These prerequisites must not be bypassed by passing an old collector or world.

## Tests and retained negative fixture

The final explicit `.venv_train_cuda/bin/python` run passed 27 tests: 20 pure/actual generated-message checks and 7 complete actual-node `main()` executions with fake ROS/master/action transports. They cover exact full-field odometry, source-time boundaries, malformed raw inputs, static route/goal inversion, all footprint vertices, preserved coordinator class ASTs, unique TF ownership, future status queues, private early action success, original-base completion and adapter-fault zero output. This is offline component evidence, not a simulated ROS execution.

`node_tests_r0.log` preserves the first 6/7 result: the future-status fixture attempted arm before its main loop had consumed the now-covered ready status. The real guard correctly refused. Only that fixture's arm tick was delayed from tick 2 to tick 3 for the 2 ms lead case; production semantics were not relaxed. `node_tests_r1.log` and `combined_tests_final.log` contain the passing reruns. No physical failure, delay or baseline outcome was removed or relabelled.
