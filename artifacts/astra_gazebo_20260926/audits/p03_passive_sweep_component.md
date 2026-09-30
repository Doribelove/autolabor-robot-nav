P03 passive sweep is implemented and passes 81 offline checks: 46 new component checks, 18 retained dense-sweep checks and 17 retained geometry checks. The new C++ observer passes a standalone `-fsyntax-only -Wall -Wextra -Werror` check against the installed Gazebo/ROS headers. No existing library was built, no ROS node or simulation was started, and no frozen P01/P02/gainpilot source was changed. “P03” here is a component version; the parent's current P03 gainpilot does not load this observer and cannot receive this new qualification retroactively.

The new observer reads actual ODE body rotation mode/axis, enabled/kinematic/damping settings, native mass/inertia, native COM/quaternion and Gazebo's merged COM/collision geometry. BeforePhysicsUpdate and WorldUpdateEnd packets have the same source stamp, because Gazebo advances its clock before both callbacks. Each step records its configuration revision and source-pairing status. Actual native COM/quaternion are compared with the published link pose plus merged COM. The plugin has no physics setters, subscriptions, services, command publisher, hashes or file I/O.

The offline adapter reconstructs full native collision compounds and checks nonrobot collision identities, dimensions and local origins against the saved world. The CLI also compares robot body roster, merged mass/COM, collision count and enclosing radius against the saved expanded URDF. Every 2 ms interval requires complete source-exact Pre/End model/link/status records, stable configuration and normal dynamic ODE bodies. Missing data, reset, unknown movement, a finite-rotation mode, disabled robot body or changed geometry leaves qualification pending/invalid.

For a normal-Euler unit-phase extension, a collision point within radius R of the actual COM has budget
`B = ||delta COM|| + 2 R tan(theta/2)`.
The solver-phase maximum across robot bodies and the preceding End-to-Pre jump maximum are added separately: different bodies can attain those maxima. Actor updates spend the reviewed source-rule `max(initial,current targets)*dt` jump budget; their published zero twist does not erase motion. Tolerated phase translation/rotation also spends explicit variation. Across exact 2 ms steps, sum all relative variation budgets into P. The 20 ms geometry endpoint lower bound is `min(d0,d1) - endpoint_pad - P/2`, where d already subtracts the unchanged 0.33 m margin. Actual compound projection retains physical XY, conservative XY and Z-screened diagnostics. XY intersection is not an exact 3D-contact statement.

The numeric profile uses 10 micrometres per solver/join engineering budget and 10 micrometres at geometry endpoints, with explicit coordinate, COM/radius, quaternion norm and per-step motion domains. The 1,000-case independent 70-digit Decimal reference found a maximum budget discrepancy of 1.112e-13 m; the 200-case independent SciPy/Shapely projection comparison found 1.084e-13 m. These pads are a tested conservative engineering choice, not a formal all-input floating-point theorem. The retained native ODE fixtures also demonstrate why post-damping/ERP endpoint twist is not an integration-speed bound. A synthetic rosbag test offsets receipt time by 75 ms and confirms exact source-stamp evaluation and finite JSON/CSV output without a ROS master.

The only qualified interpretation available to a future fully bound run is the declared Gazebo normal-Euler motion extension plus bounded discrete actor Begin updates. It does not establish real continuous physical safety, robustness to malicious external processes, deployment readiness or a task result. Whole-system contacts, task/input integrity, masks and failure-inclusive completion must still be evaluated separately. Empty obstacle sets are “not applicable,” never a positive clearance pass.

Root integration is still required:

- After the active gainpilot terminates, review the exact hashes in `p03_passive_sweep_component.json`, then add a separate CMake target `astra_physics_metadata_p03_plugin` linked to existing catkin/Gazebo libraries and generated messages.
- Load `libastra_physics_metadata_p03_plugin.so` once as a world plugin in a fresh common environment stage. Keep the existing whole-system contact plugin.
- Record `/astra/p03/pre_models`, `pre_links`, `end_models`, `end_links`, `runtime_metadata` and `step_status`, all under the same `/astra/p03/` prefix.
- Capture exact runtime robot_description and world, actual loaded libraries via process maps, the isolated-master node roster, and authorized launch/service/pose-writer audit. The audit scope excludes malicious processes but covers every authorized node and service path.
- First verify a fresh runtime fixture for lossless 2 ms coverage, stable revisions, native body correspondence, known obstacle/rotation/actor examples and acceptable observation overhead. No such runtime fixture has yet used this component.

Run closure schema is `astra_p03_run_closure_v1`, scope `gazebo_normal_euler_extension_with_actor_begin_jumps`, with the exact `numeric_profile` from the fail-closed template `p03_passive_run_closure_template.json`. Every binding is an array of `{path, sha256}` objects. Required nonempty roles are launch, world, robot_description, driver_source, driver_binary, evidence_source, evidence_binary, engine_binary_maps, node_roster_audit, service_call_audit and pose_writer_review. actor_sources and actor_binaries are arrays and must be nonempty when actors are present. Binary paths can refer to archived copies if the reviewed process-map artifact also records their actual mapped origin.

The four audit roles each bind exactly one JSON object with `pass: true` and the same run_id. Engine maps, roster and service audits must cover the full source window using start_ns/end_ns. Additional facts are:

- node_roster_audit: `loopback_master_isolated: true`, `unexpected_nodes: []`.
- service_call_audit: `scope: authorized_launch_and_nodes`, `forbidden_calls: []`; the authorized paths must prohibit reset/set_model_state and other unreviewed pose writers after bootstrap.
- pose_writer_review: `robot_pose_velocity_writes: []`, `actor_updates: none` or `reviewed_bounded_Begin_updates`.
- engine_binary_maps: loaded_modules are path/SHA objects; independently reviewed module roles must include ode_integrator and gazebo_physics. Driver, observer and actor binaries must appear in this set. implementation_review requires `ode_normal_euler_update_verified`, `gazebo_pre_end_callback_order_verified` and `loaded_module_source_correspondence_reviewed`, plus nonempty rehashed primary `source_bindings`. These booleans represent bound independent audit conclusions; the adapter does not discover or prove source/binary correspondence from a filename.

Offline invocation, using exact recorded source-window endpoints:

```bash
source /path/to/workspace/scripts/astra_env.sh
/usr/bin/python3 /path/to/workspace/scripts/astra_gazebo/dense_sweep_p03.py \
  --bag /path/to/episode.bag --world /path/to/world.sdf \
  --urdf /path/to/robot_description.xml --closure /path/to/run_closure.json \
  --start-ns 1000000000 --end-ns 1020000000 --output /path/to/new_evaluation
```

The evaluator refuses an existing output directory, rehashes sources and closure artifacts around evaluation, and writes report.json, intervals.jsonl and endpoints.csv. CLI inputs and output hashes are retained. Its pure `assess_records` API expects an already authenticated closure result; the CLI performs file authentication. A synthetic test's closure is deliberately not real audit evidence.

Primary implementation references are [ODE integration](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/deps/opende/src/util.cpp), [QuickStep post-integration velocity changes](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/deps/opende/src/quickstep_update_bodies.cpp), [Gazebo ODELink COM/pose semantics](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/ode/ODELink.cc), and [World callback ordering](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/World.cc). Exact downloaded source bindings and the installed-engine standalone reference are retained in the earlier dense_sweep_component report.
