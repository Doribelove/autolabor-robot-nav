# M08 v2: batch occupancy computation and availability-time repair

The immutable v1 source and audit remain unchanged. This new file is a query backend plus narrow public API guards. It does not change tracking, association, map filtering, persistent supports, nominal CV, candidate generation, actions, policy weights, request/learning schema, the external 0.33 m margin or the original 200 ms source freshness rule. There is no ROS launch, new simulator seed, training, calibration or runtime integration.

## API

`SensorTracks` inherits the exact v1 acceptance/tracking logic. Its public `occupancy(now_ns)` returns a guarded scalar compatibility object; `batch_occupancy(now_ns)` returns the new batch object. Direct public constructors `HardOccupancy(snapshot, now_ns)` and `BatchHardOccupancy(snapshot, now_ns)` both reject a time before `snapshot.accepted_now_ns`. `prediction_request(..., decision_ns=...)` applies the same availability guard then delegates to immutable v1; valid request bytes, request hash and schema remain identical. Every old 200 ms source-age check remains in force. Accepting a late scan does not grant a new 200 ms window.

Create a reusable `ConvexFootprint(runtime.footprint.vertices)` from the existing full-body envelope. A convexity/validity check rejects concavity, holes represented as invalid rings, repeated/degenerate edges and nonfinite data; there is no silent convexification or new shape approximation. The actual Q05 expanded robot description produces the same nine-component, 16-plane full-body envelope used in the paired benchmark.

`batch.distance_lower_bounds(footprint, base_poses, offsets_ns)` accepts a Q×3 map-frame **base_link** pose array and Q integer offsets from `now_ns`. Q is bounded to 4096 and each offset to the unchanged configured 8 s support. It returns dynamic-observation distance lower bounds **before** the unchanged external 0.33 m robot margin, static/domain checks and robot/obstacle interval budgets. The actual rear-to-base reference remains the existing candidate generator's responsibility. No action or planned trajectory is modified.

This interface is separate from the old scalar API. A later explicit runtime revision must share the snapshot across analytic/new learned rankers, retain deadline drops and fresh 20 Hz revalidation, and bind the v2 source/backend identity. Nothing automatically imports it into sealed N01/N02/N03 components.

## Geometry and temporal equivalence

All v1 support entries are retained: each track circle, every persistent sparse cell circle, every baseline point, and every tentative static-map point. A point is a zero-radius circle. At one accepted snapshot and query creation time `n`, all terms can be written

`distance(actual_robot_polygon, support_center_i) - radius_i_at_n - 1.35 * horizon_s`.

Consequently the common horizon term can be factored out exactly. Repeated poses reuse only this same spatial minimum; different offsets still receive their own time expansion. Vectorized array construction replaces per-cell NumPy scalar arithmetic, without removing or coarsening any support. The identity records the full count.

For different shapes, AABB distance is only a lower bound used to find potentially deciding support pairs. A selected pair is evaluated against the **actual transformed convex polygon**, using point-to-segment distances and convex inside tests. The AABB never becomes the returned clearance. A row attaining `-max(radius)` already reaches the minimum possible distance; no other circle can improve it. Temporary pair tensors are processed in blocks of at most 1024 pairs, avoiding a full query×support×128-vertex allocation.

The final result subtracts an explicit `1e-10 m` numerical amount. Thus comparisons target the v1 expression minus this tiny conservative subtraction; it is neither a margin relaxation nor a learned uncertainty reduction. Tested coordinates are bounded by the inherited ±10 km map support and query transformed hulls must stay within it. This is engineering floating-point validation, not a formal numerical proof or physical safety certificate.

## Preserved negatives and repair evidence

v1 remains at SHA `3f0cb949640ba93494bfc511538a9776a171b5deaf09dee3c10dd320fd67076b`. Its original 879.565 ms unpruned and 57.427 ms pruned sparse-query results remain in the v1 audit. Its independent availability-time counterexample and limited review are bound by the v2 report; v1 is not retroactively promoted.

The first v2 benchmark attempted a summary lookup on the counterbalanced loop's final v1 object, causing an AttributeError after the loop and before output. The exact candidate source/test/benchmark and failure explanation are retained in `development_r0/`. No timing result is invented for that failed summary. The first successful v2 version measured up to 22.634 ms including construction; its exact inputs/results are retained in `development_r1/`. Vectorizing sparse support construction then reduced the measured maximum below 20 ms.

## Validation and timing scope

Author tests cover random convex hull sizes/orientations, negative inside/overlap/touching distances, large supported coordinates, ambiguous crossing followed by lost/sparse retention, tentative/static and baseline terms, actual source ages/horizons, repeated and 248 distinct geometric queries, invalid footprint rejection, immutable inputs, and all five new public request/occupancy entry points. Availability is tested at accepted-time minus/exact/plus 1 ns, separately from the original source-time 200 ms endpoint. Valid prediction requests are byte-for-byte equivalent to v1.

The paired benchmark binds the actual expanded nine-component URDF and original full-body envelope helper. It compares every one of 248 results per workload directly to v1 before timing, with 20 counterbalanced repetitions on both `/usr/bin/python3` and the original workspace virtualenv. All support construction, pose transformation and query operations are included. The fixed footprint is validated once as expected for a constant robot model. Workloads are the original repeated geometry, 248 different full-domain rotations, 248 candidate-like near-route poses, and 27 s mixed lost-support ages. All are synthetic component inputs/queries, not Gazebo navigation evidence or feasible executable trajectories.

Both environments' measured maxima are below 20 ms for these four workloads. This is an observed component result, not a worst-case execution-time proof or a full-planner/live 50 ms qualification. Scan transformation/tracking, static-map geometry, candidate generation, neural inference, ROS scheduling/transport and publication are excluded. In particular v1's previously measured dense 1440-return tracking acceptance can still cost about 40 ms; adding it to this kernel does not establish a 50 ms end-to-end budget. Real runtime integration must preserve the existing deadline-failure stop behavior and receive its own measurements/review.

The uncalibrated static-prior association, detection/extent blind spots, persistent lost-support conservatism and absence of a learned candidate-conditioned predictor remain exactly the v1 limitations. M08 training/calibration/coverage promotion gates remain pending.
