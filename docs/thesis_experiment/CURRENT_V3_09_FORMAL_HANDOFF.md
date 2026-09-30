# V3-SCOPE Formal Result and Paper Closure Handoff

Date: 2026-07-24

## Terminal status

V3-SCOPE has completed the bounded simulation and paper-evidence program from
V3-00 through V3-10-P1. The confirmatory result is the fresh V3-09-F1 paired
evaluation; V3-10-P1 is a pure-offline export and evidence-closure stage.

V3-09-F1 used seeds 6801--6830 as 30 statistical blocks. Each block contained
the cautious, assertive and hesitant evaluator-only reactive-actor policies,
paired across the frozen `v3_spatial_safe` method and the frozen
`fixed_v120_shielded` straight-policy comparator. The exact 180-episode
schedule completed 180/180 at attempt 1, with no retry, resume, replacement or
forfeiture. The exact execution release SHA is
`5b67fde09bb4c76cf9c240a4cde998c862e598155048f787a9bef7039b75a772`.

The deterministic assessment status is
`v3_09_f1_formal_positive_result`; its SHA is
`d950256f4da310f65d604031ec72009e014facfb7cb0c14be8cf19ce1b9c346b`.
The independent result review status is
`v3_09_f1_independent_formal_positive_review`; its SHA is
`206732c86b536e7f9a520c58dac6d386ff6752595bdc767d4957e4804e5f8d5a`.
Two cold `--check` replays reproduced the assessment byte-for-byte.

## Confirmatory result

The preregistered primary result is:

| Metric | V3 spatial-safe | Fixed-v1.20 shielded comparator |
| --- | ---: | ---: |
| Episodes | 90 | 90 |
| Goal success | 100% | 100% |
| Mean restricted goal time | 8.8611 s | 11.4167 s |
| Median restricted goal time | 7.50 s | 11.25 s |

The paired block-level relative reduction is 22.3844%. The one-sided 95%
block-bootstrap lower bound is 21.8235%, using 20,000 resamples and frozen
seed 30909. The point and lower gates were 10% and 5%, respectively. All
30/30 blocks favored V3 after aggregating the three actor policies.

Policy-stratified means expose the mechanism and its limitation:

| Actor policy | V3 spatial-safe | Comparator | Direction |
| --- | ---: | ---: | --- |
| cautious | 7.5000 s | 11.7667 s | V3 faster |
| assertive | 11.5833 s | 10.8917 s | V3 slower |
| hesitant | 7.5000 s | 11.5917 s | V3 faster |

The aggregate positive result is therefore not a claim that V3 wins in every
actor mode. It is a preregistered population result over the equal-weight
three-policy block.

## Safety, integrity and runtime metrics

- completed episodes: 180/180;
- goal success: 180/180;
- planner decisions replayed: 5,320;
- accepted selected-candidate rate: 100%;
- certified-stop availability: 100%;
- exact core replay: 100%;
- correct receipt stage identity: 100%;
- conservative-clearance violations: 0;
- minimum conservative clearance: 0.165663302 m;
- nonfinite metric count: 0;
- overall planner-latency P99: 18.087844 ms, below the frozen 20 ms gate;
- V3 spatial-safe P99: 15.629119 ms;
- fixed-v1.20 shielded P99: 18.548193 ms.

These are audited simulation properties in the frozen scene family. They are
not a formal collision-proof certificate and do not establish general
navigation safety.

## Development chain and preserved negative evidence

The positive result did not replace earlier failures:

- V3-05-B0 is a complete negative formal evaluation of the earlier
  longitudinal analytic planner: 270/270 episodes, 1.2007% point effect,
  0.9647% lower bound and 93.33% V3 goal success.
- V3-06 bounded longitudinal speed/density repairs remained negative and are
  terminal.
- V3-07 established spatial candidate identifiability, an observed-mode safe
  overlap gate and the frozen spatial predictor. Its B1 held-out validation
  completed 20/20 with 100% global/per-head coverage.
- V3-08 established the spatial hard-verifier planner. A1 failed on
  nonfinite rejected-candidate diagnostics; A1-R1 was a valid negative at
  17/18 goals; A3 changed only the duration and receipt identity and completed
  18/18.
- V3-09-Q1 terminated after one complete unit because its shared 0.5 s
  replanning protocol was unfair to the frozen 0.25 s baseline commitment.
  Q1-R1 changed only the shared replanning period to 0.25 s and qualified the
  formal stage on fresh seeds 6721--6726.

None of these identities, seeds, budgets or outcomes may be retried, resumed,
deleted or reclassified.

## Paper evidence closure

V3-10-P1 uses no ROS, Gazebo, seed, schedule, execution or training authority.
It exports the frozen formal result to:

- `v3_10_p1_formal_summary.csv`;
- `v3_10_p1_policy_results.csv`;
- `v3_10_p1_block_results.csv`;
- `v3_10_p1_formal_result.svg`;
- `v3_10_p1_claims.yaml`.

The V3-10-P1 evidence manifest SHA is
`70353b355c6eec2f733901ca7d4a6820de550beff00999cfacdb359dec951938`.
The independent closure review SHA is
`bee5891b08f860a97a25ea71047b3e5fabb0c193695f221de131eacb5dcfe0f4`.
The conventional multiline terminal manifest SHA is
`57f043fdd485f01c6d194623acaedcee50d66447c635ae776bfb2b433c9ccf54`.

The historical V3-09-F1 evidence and terminal manifest writers used literal
backslash-n separators. Those files are preserved byte-for-byte; the formal
assessment and independent replay do not parse them and both passed. V3-10-P1
binds the exact historical bytes in a new conventional multiline manifest.

## Exact claim boundary

The qualified statement is:

> Within 30 fresh parameterized reactive single-actor crossing blocks,
> V3 spatial-safe reduced restricted goal time by 22.3844% relative to the
> frozen fixed-v1.20 shielded straight-policy comparator; the one-sided 95%
> block-bootstrap lower bound was 21.8235%, with 100% goal success for both
> methods and all audited integrity/safety gates passing.

The comparator is not actual TEB. The evidence does not qualify actual-TEB
superiority, multi-actor or cross-scene generalization, learned-policy
benefit, formal safety, deployment readiness or real-vehicle performance.
`runtime_ready=false`, `training_authorized=false` and
`real_vehicle_ready=false`.

No real robot, CAN device, `/cmd_vel`, move_base or stable workspace
`/home/robot/robot_ws` was used or modified.

## Reproduction

```bash
cd /home/robot/robot_ws_base_rl
python3 src/tools/thesis_experiment/scripts/assess_v3_09_f1_formal.py --check
python3 src/tools/thesis_experiment/scripts/review_v3_09_f1_result.py
python3 src/tools/thesis_experiment/scripts/build_v3_10_p1_paper_closure.py --check
python3 src/tools/thesis_experiment/scripts/review_v3_10_p1_paper_closure.py --check
python3 -m pytest -q \
  src/tools/thesis_experiment/tests/test_v3_09_f1.py \
  src/tools/thesis_experiment/tests/test_v3_10_p1.py
```

## Next safe entry

The current bounded V3 study is terminal and must not be rerun merely to seek
a larger effect. A stronger publication claim requires a new, independently
reviewed study with an actual TEB implementation as a paired baseline and
fresh multi-scene/multi-actor identities. Real-robot work remains a separate
on-site authorization and calibration stage. It must retain the existing
`/cmd_vel_navigation` -> GPS limiter -> FOD arbiter -> M2 driver safety
boundary.
