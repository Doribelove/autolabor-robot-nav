R2 sensor lifecycle independent review
======================================

Status: changes required before new lifecycle integration. Author tests replay60/60; independent3 probes complete. One independent test deliberately demonstrates a bug, so passing test execution is not approval.

R2-LIFECYCLE-01 (high): `accept_scan` validates TF, then the inherited `_scan` performs a second lookup. If only that second lookup fails, the parent swallows it after updating raw scan freshness; the adapter still returns True and drops the packet. With six prior accepted histories, the actual constructor/runtime/checkpoint test held the last committed stamp at2.10s while current time reached2.76s: three parent failures, no pending scan, no lifecycle fault, empty health failures, ready=True. Preserve this frozen source and repair in a new revision by explicitly retaining or faulting a due-but-failed append; keep .20s gates and legacy sampling/numerics.

Actual construction and main transport contain no Gazebo truth subscription/service. Actor parameters are stripped, oracle diagnostics are None, six real accepted histories are counted, and original actor/runtime/execute_sensor/checkpoint behavior is preserved for tested valid streams. Shared timeout/stopped/fault do not falsely produce goal success. Full common route is audited for receipt only; fixed global-x R2 and implicit walls remain disclosed input-use limitations.

No live qualification, primary-matrix readiness, ROS/build or old-file modification is claimed. The current frozen N01 enum/wrapper remains untouched. Exact bindings, logs and discovered counterexample are in review.json.
