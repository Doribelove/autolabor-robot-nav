# CURRENT V2-04G-R1 HANDOFF

Date: 2026-07-14

## Outcome

V2-04G-R1 completed its simulation-only, calibration-only budget. The bounded
sequence/time cache join repaired the V2-04G typed-transaction activation
failure, but no candidate passed every preregistered performance/mechanism gate.
Therefore no candidate was frozen and no held-out validation was generated.

The authoritative machine-readable decision is
`artifacts/v2/calibration/v2_04g_r1/v2_04g_r1_stop_report.yaml`.

## What changed

- Replaced the single latest `LocalGeometry` plus exact sequence equality with
  a 32-entry non-future cache join bounded by 2 sequence steps, 0.45 s source
  timestamp difference, and 1.0 s arrival age.
- Retained the V2-04G supervisor, Anchor and mechanism numeric candidate values.
- Added explicit join validity/reason telemetry to the mechanism state.
- Added a deterministic activation-readiness gate before any navigation goal.
- Added fresh calibration navigation seeds 4921--4935 and probe-only seeds
  4941--4946. Seeds 5001--5010 remain unopened and reserved.

## Interface-repair evidence

- Activation readiness: 6/6 probes passed across g1 and g2, with transaction
  activation fraction 1.0 and join validity fraction 1.0 in every repeat.
- TTC component semantics: all three states passed in the frozen order:
  `OBSERVED_CONFLICT`, `NO_CONFLICT_IN_HORIZON`, `TRACKER_INVALID`.
- Navigation: 60/60 valid episodes, 60 successes, 0 collisions, 0 persistent
  interface failures.
- g1 and g2 minimum per-episode join validity fraction: 1.0. All navigation
  joins were recorded as `EXACT_SEQUENCE_JOIN`; the bounded fallback remains
  covered by unit tests and is fail-closed outside its bounds.
- Full workspace `catkin_make` passed. R1 targeted ROS tests were 13/13; the
  accumulated catkin result index reports 115 tests, 0 errors, 0 failures.

## Calibration assessment

| Candidate | Success | Time ratio vs Fixed | Key failed gates |
| --- | ---: | ---: | --- |
| g0 frozen control | 15/15 | 1.1972 | TTC, chatter, total/family time |
| g1 mechanism balanced | 15/15 | 1.0732 | reverse activation, total/family time |
| g2 mechanism aggressive | 15/15 | 1.0181 | reverse activation, family time |

Both non-control candidates passed success non-degradation, collision, minimum
clearance, typed transaction, TTC, chatter, and bounded-join gates. Neither
produced a maneuver reverse sample. Static, Corridor, and Maneuver family time
regressions remained above the preregistered limits; g2 passed only the overall
time-ratio gate.

## Frozen evidence

- Preregistration:
  `experiments/manifests/v2/calibration/v2_04g_r1_preregistration.yaml`
- Contract:
  `config/thesis_experiments/v2/v2_04g_r1_interface_repair_contract.yaml`
- Activation summary:
  `artifacts/v2/calibration/v2_04g_r1/activation_probe/activation_probe_summary.yaml`
- Navigation progress:
  `artifacts/v2/calibration/v2_04g_r1/v2_04g_r1_progress.yaml`
- Assessment:
  `artifacts/v2/calibration/v2_04g_r1/v2_04g_r1_assessment.yaml`

## Current boundaries

- Do not retune and resume inside V2-04G-R1.
- Do not freeze g0, g1, or g2 from this evidence.
- Do not generate or consume seeds 5001--5010.
- Do not reuse seeds 4921--4935 as held-out validation.
- V2-05, SAC training, real-vehicle closed loop, and real TEB writes remain
  unauthorized. `runtime_ready=false` remains unchanged.

## Required next decision

If development continues, create a new preregistered calibration-only mechanism
stage with fresh seeds. It should treat the join repair as frozen infrastructure
and change one mechanism factor: first repair Maneuver reverse observability and
the Static/Corridor/Maneuver time regressions. A new held-out validation is
allowed only after a candidate passes every new calibration hard gate.
