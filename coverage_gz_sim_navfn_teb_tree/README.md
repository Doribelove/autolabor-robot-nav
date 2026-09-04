# Coverage Gazebo simulation tree

This copy is the isolated Navfn+TEB line-transition comparison tree.  It was
cloned from `coverage_gz_sim_tree` after the route-selection changes.  Its only
navigation differences are that every same-region line transition uses the
ordinary Navfn+TEB action and the TEB global-plan lookahead used by those
actions is 8.0 m.  The authoritative project source and the direct-Hybrid
simulation tree are not imported from this comparison copy.

This isolated catkin overlay snapshots the current `autolabor_coverage` and
TEB sources.  It runs only against a loopback ROS master and Gazebo master; it
does not launch, deploy, or contact J6M or the physical robot.

The baseline uses the project's latest fused static map and its saved A/C
coverage regions.  Gazebo contains an empty visual world.  Static obstacles
come from `map_server`; no range sensor or dynamic obstacle model is used.  A
truth-state bicycle plant implements the M2 Twist-to-steering behavior with:

- wheelbase: 0.65 m;
- hard minimum turning radius: 1.35 m;
- physical body: 1.04 x 0.70 m;
- navigation footprint padding: 0.10 m, giving the project's effective
  0.62 m front/rear and 0.45 m half-width envelope.

Build and run from this directory:

```bash
./build.sh -j4
./run_experiment.sh baseline_current_architecture baseline
./run_experiment.sh simplified_1hz_online_unsplit simplified
./run_experiment.sh recommended_hierarchical recommended
./run_experiment.sh fault_overshoot fault_test fault_scenario:=sweep_overshoot
./run_experiment.sh fault_entry fault_test fault_scenario:=entry_offset
```

`baseline` snapshots the current cached/precomputed architecture. `simplified`
uses lightweight swath ordering, no mission-wide Hybrid precomputation, a 12 m
rolling kinematic prefix replanned at 1 Hz, and deliberately does not split
cusps. `simplified_cross` starts at the recorded A-region completion pose and
runs C only, for a repeatable long cross-region diagnostic.

`direct_event` is the current architecture: geometric sweep-angle selection,
a bounded path-free Dubins time proxy that jointly orders and orients those
swaths, one Navfn+TEB action for the first swath, and a direct live-pose Hybrid
path for each same-region line transition. Hybrid paths are split at cusps;
every fixed-gear part is tracked by TEB. After a measured zero-speed cusp the
manager reuses a geometrically joinable cached suffix and replans the complete
remaining path to the final entry only when that join is invalid. It does not
precompute a mission-wide executable trajectory or unconditionally run Hybrid
search at 1 Hz.

`fault_test` runs the same current TEB architecture through a deterministic
command/pose fault-injection mux. It can force a sweep-end overshoot or apply a
map-frame lateral/yaw offset immediately before a selected sweep. An entrance
outside the 0.40 m position/cross-track and 0.436332 rad (25 deg) hand-off
contract remains `TRANSITING`; the
forward-only sweep action is never armed. If a disturbance arrives while the
last fixed-gear connector action is still active, three consecutive path
deviation samples cancel that exact stale action first. Recovery then uses a
reverse-favoured, length-bounded Hybrid/cusp plan and retries the unchanged
swath. A second early-sweep guard covers disturbances that arrive after the
sweep action has actually started.

Results are written under `results/`, including a bag, CSV samples, online
monitor events, terminal summaries, and an automatic static-map footprint and
kinematics audit (`audit_summary.json`). The final A-to-C validation is
`recommended_lightweight_time_order_A_v12_20260902_140008` (the historical
label says `A`, but the launch default ran both A and C). Positive/negative
entrance-error and overshoot regressions are preserved as v14, v15, and v16.
See
[EXPERIMENT_REPORT.md](EXPERIMENT_REPORT.md) for the measured comparison and
architecture conclusion. Current TEB/cusp-join validation is in
[TEB_FIXED_GEAR_VALIDATION_20260904.md](TEB_FIXED_GEAR_VALIDATION_20260904.md).

The current 1.35 m-radius, direct line-to-line architecture and its A/C/custom
three-repeat transition results are documented in
[CURRENT_ARCHITECTURE.md](CURRENT_ARCHITECTURE.md) and
[TRANSITION_EXPERIMENT_REPORT_20260903.md](TRANSITION_EXPERIMENT_REPORT_20260903.md).
The 2026-09-02 report is retained as historical development evidence.

The current component/lifecycle contract is documented in
[CURRENT_ARCHITECTURE.md](CURRENT_ARCHITECTURE.md). Deliberate overshoot and
entry-offset tests, including every transition's duration, shape, planned
length, actual distance, and failure/recovery timeline, are in
[FAULT_INJECTION_EXPERIMENT_REPORT.md](FAULT_INJECTION_EXPERIMENT_REPORT.md).
Each new run also writes `navigation_details.json` from its rosbag in addition
to the footprint/kinematics audit.
