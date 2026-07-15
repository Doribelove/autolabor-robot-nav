# CURRENT V2-04G HANDOFF

Date: 2026-07-14

## Outcome

V2-04G was preregistered and started as a simulation-only, calibration-only
mechanism repair. It stopped fail-closed after 31 accepted navigation episodes
because episode 32 failed the frozen runtime-readiness condition on both allowed
attempts. No Anchor/mechanism was frozen and no new held-out validation set was
generated.

The authoritative machine-readable stop record is
`artifacts/v2/calibration/v2_04g/v2_04g_stop_report.yaml`.

## Completed implementation

- Added relative-motion TTC evidence with the disjoint states
  `OBSERVED_CONFLICT`, `NO_CONFLICT_IN_HORIZON`, and `TRACKER_INVALID`.
- Added signed rectangular-footprint LaserScan clearance and evaluator-only
  Gazebo oriented-box clearance.
- Added a label-free V2-04G mechanism controller for episode-local static
  topology preference, corridor centerline residuals, and bounded maneuver
  forward/reverse profile selection.
- Added a V2-04G-only residual-aware typed Anchor transaction path. Historical
  V2-04E/F runners and launches were restored to their original hashes.
- Added 15 new calibration scenes using seeds 4901--4915, including two dynamic
  conflict timings and one no-conflict timing.
- Preregistered 60 navigation episodes and three TTC component probes. Seeds
  5001--5010 remain reserved and unopened for a future held-out validation.

## Accepted evidence

- Fixed reference: 15/15 success, 0 collision, 0 switches.
- Frozen control g0: 15/15 success, 0 collision, maximum 3 switches and mean
  1.333 switches.
- Mechanism-balanced g1: 1 accepted Cruise episode before the stop.
- Both complete profiles produced two `OBSERVED_CONFLICT` and one
  `NO_CONFLICT_IN_HORIZON` dynamic episodes, with zero `TRACKER_INVALID`.
- The TTC component probe produced all three states exactly.
- Targeted ROS acceptance: 102 tests, 0 errors, 0 failures.
- Full workspace build: `catkin_make` passed. The unrelated full `run_tests`
  aggregate remains blocked by the pre-existing third-party `light_scan_sim`
  use of removed OpenCV symbol `CV_FILLED`.

## Clearance conclusion

The investigated 0.000 m sample is a true footprint geometry intrusion. In
Fixed/Dynamic-conflict-s4904 the signed LaserScan clearance was -0.2032 m and
the evaluator-only Gazebo oriented-box clearance was 0.0 m. The contact topic
reported zero contacts, showing that the pose-driven trajectory actor does not
provide an authoritative contact signal. Future safety gates must retain the
signed scan and truth-box audit rather than relying only on Gazebo contacts.

## Stop cause

The g1 Cruise-s4902 episode reached normal ROS/Gazebo startup twice, then timed
out waiting for an activated typed transaction. The primary diagnosis is the
exact sequence join in the V2-04G transaction node: it accepts mechanism input
only when `LocalGeometry.world_model_seq == ContextState.world_model_seq` at a
timer tick. These messages are asynchronous, so the match is intermittent. The
preceding accepted g1 episode had 163 transaction messages but only 26 activated
transactions, which supports this diagnosis.

## Current boundaries

- Do not resume the remaining 29 episodes under the same V2-04G
  preregistration.
- Do not change or delete the 31 accepted episodes or the two failed-attempt
  logs.
- Do not freeze g0, g1, or g2 from incomplete evidence.
- Do not generate seeds 5001--5010 yet.
- SAC training, V2-05, real-vehicle closed loop, and real TEB writes remain
  unauthorized.

## Required next stage

Create a new calibration-only V2-04G-R1 interface synchronization repair with a
new preregistration and new calibration seeds. Replace the exact sequence join
with a bounded timestamp/sequence cache join, publish an explicit mechanism
health reason, and require deterministic activation-readiness probes before
navigation. Only after R1 completes with zero persistent interface failures may
the mechanisms be ranked, frozen, and evaluated on a newly generated held-out
split.
