# M08 sensor multi-track v1: implemented scope and next model boundary

This is a new, offline-tested pure component. It does not alter or replace the sealed N01 route runtime, frozen R2, physical driver, launch, weights, scene, seed registry or release. No ROS, Gazebo episode, training, calibration or final outcome was used. The fixture scans in the tests/benchmark are synthetic component inputs, not simulation evidence.

## Exact runtime API

`SensorTracks(shared_static_grid, TrackConfig()).accept(frame, tf_stamp_ns=..., odom_state=..., now_ns=..., frame_id='map')` accepts the existing `route_sensor_buffer.ScanFrame` after its real scan was transformed using its own exact timestamp TF. Odometry must be at or before the scan and at most 200 ms old; its actual body twist includes `vy`. Odometry is recorded as a causal feature, not reapplied to the already world-transformed cloud. Inputs are copied into immutable tuples. No actor roster, model/link state, contact, future pose, ground-truth velocity or checkpoint is accepted.

The scan source and TF stamp must be exactly equal, positive integer ns. Future, stale, duplicate and out-of-order observations cannot refresh accepted source time. A temporarily missing TF can be retried by the caller with the original stamp within the original 200 ms window; this module invents neither scans nor retries. Clock rollback, input/track/sparse-memory overflow and a greater-than-30 s episode span permanently fault. Failures do not partially commit tracking state. The existing 200 ms fresh-scan health limit is unchanged.

`tracker.occupancy(now_ns).distance_lower_bound(base_footprint_polygon, offset_ns)` returns a **dynamic-observation envelope distance**, before the unchanged external 0.33 m robot margin and interval-motion budgets. It accepts integer horizons 0–8 s and a fresh current snapshot. A caller must use actual full-body/base geometry and preserve the old static-map/domain checks. This is an additive interface for a later explicit runtime revision; no sealed runtime imports it yet.

## Association, static filtering and lost observations

* The latest baseline unknown cloud uses exactly the sealed route core's `grid.clearance(point) <= 0.15` static-prior predicate. Three consecutive distinct scans in the same 0.10 m map cell are needed to label that prior match as multi-scan confirmed. Unconfirmed map matches are extra current occupancy. Stationary returns outside the static prior are never silently learned into the map.
* Map matching is still an **uncalibrated sensing assumption**. Three repeated returns do not prove an actor absent or a return physically static. A previously seen dynamic support remains retained when new returns fall in the map band. A new actor completely hidden in or near the prior can remain undetected; no full detection or safety claim follows.
* Euclidean connected components retain every non-map return. Groups with fewer than three points enter persistent spatial cells, using the oldest source time and the entire cell's enclosing circle. They are not thrown away as noise.
* Track-to-detection association uses radius/position-error/speed reachable gates. A unique one-to-one match supplies a clipped centroid-CV diagnostic. Its admissible velocity support stays the full uncalibrated 1.35 m/s disk; nominal CV never shrinks hard occupancy.
* A connected association component with multiple old/new alternatives is merged into one explicitly unresolved hypothesis. Its circle contains all old reachable circles and new observed clusters. Member IDs remain local labels, not simulator actors. The component never claims to recover identities after an ambiguous crossing. This can be deliberately conservative.
* Unseen tracks keep their last support and expand isotropically; absence is labelled `unseen_not_proven_occluded`. There is no age-based deletion, free-space clearing, occlusion classifier or disappearance-to-free-space inference. A sustained ambiguity or sparse return can stop navigation; reducing that conservatism needs fresh sensing/calibration work.

## Non-shrinking relation to the sealed core

At source `s`, current time `n` and future offset `h`, the sealed dynamic bound is

`distance(robot_shape, latest_nonmap_points) - 1.35 * ((n-s+h) / 1e9)`.

The new result is the minimum of that **exact term** and all additional retained circles/tentative map supports. Therefore it cannot exceed the old dynamic bound on identical inputs. No learned or CV output may replace that term in this version. The tests compare 384 shape/horizon combinations across changing scans, in addition to crossing/lost/sparse fixtures. Empty scans give no new detection evidence; historical supports remain. An empty episode with no observed supports is merely an empty observation model, not proof of free space.

For larger retained memories, a footprint AABB distance is used only as a conservative broad-phase lower bound to prune circles that cannot improve the current minimum. Every deciding circle still uses exact GEOS distance to the actual query geometry; no AABB replaces the footprint. Its output is checked against the unpruned expression on 180 rotated footprint fixtures. A declared ±10 km coordinate support avoids extreme finite-value overflow.

The circular distance operation is analytic (`shape.distance(center)-radius`), so it does not use an inward polygon approximation. These are observation-support envelopes under stated assumptions, not the complete geometry of unseen actors and not continuous physical safety certificates.

## Candidate-conditioned prediction boundary

`prediction_request(snapshot, RouteSensorBuffer.history, candidates, decision_ns=...)` produces `astra_candidate_conditioned_sensor_request_v1`. It requires six **already accepted, content-hash-matching** real scans on six consecutive integer 200 ms buckets. All scan/odom sources are at or before the observation cutoff; no future observation, padded history or silently changed scan can enter. Planned trajectories may extend into the future: these are candidate inputs, not observed future outcomes. Their time origin is the decision timestamp.

Each candidate contains an explicit ID and integer-offset states with base pose, rear-axle pose, actual-reference body twist and center steering. The 0.325 m base/rear relation and frozen ±0.4964 rad steering support are checked. All candidates share the same initial physical reference. Full dynamic feasibility and executed common-prefix qualification remain the shared candidate generator/executor's responsibility. The request binds map bytes, tracker configuration, source receipt, causal track/odom/history features and candidate sequences. A different candidate can change the request hash; this is **not proof of a learned interaction response**.

`nominal_cv` is candidate-independent diagnostic CV. There is no fitted predictor, candidate-conditioned response model, calibrated uncertainty, actor-conditioned future oracle or trained policy in this component. Its identity reports those absences explicitly. A future policy that uses these new features requires a **new observation/checkpoint namespace and new training**, never relabelling/reusing the frozen R2 or old route weights as compatible.

To implement M08's learned direction next, collect new geometry-disjoint Gazebo train and untouched calibration observations, with executed common-prefix/action histories and exact sensor source cutoffs. Teacher future trajectories/IDs may be labels in a separate training-only table; they must not enter policy/predictor request features. Fit candidate-conditioned residual/interaction predictions, then calibrate whole-trajectory/per-horizon reachable envelopes under occlusion, sparse sensing and identity ambiguity. Bind model bytes, feature schema, normalizers, split parent identities, physical bounds and calibration score. No new final outcomes are needed to build or calibrate it.

Promotion follows frozen `mechanism_plan.yaml` M08: held-out coverage must pass and width reduction must reach 10% **or** completion-time improvement 3%, with no task/safety regression; no current component test substitutes for those gates. Primary statistical multiplicity and one final confirmation remain owned by the frozen stage contracts. Until that separate qualification, learned outputs may add ranking diagnostics or extra occupancy only; this implementation cannot authorize shrinking the existing hard envelope.

## Integration and limits

The next runtime revision may call `accept` for every valid fresh scan independently of the six-row model sampling, build one immutable snapshot per decision, and share that snapshot/predictor/verifier across analytic and new learned rankers. Revalidation at 20 Hz must use a new fresh snapshot, retain static-map and 0.33 m checks, and drop results missing the existing transaction deadline. The component itself is pure and does not publish commands or override lifecycle authority.

Offline timing includes scan acceptance and 248 dynamic queries (eight candidates ×31 points) on six specified synthetic workloads plus configured-size stress. It excludes static/full-footprint construction, TF/transport, neural inference, scheduler/ROS load and control publication. Reported component time cannot qualify a live deadline. No broader worst-case bound is claimed, and capacity is not a performance guarantee.
