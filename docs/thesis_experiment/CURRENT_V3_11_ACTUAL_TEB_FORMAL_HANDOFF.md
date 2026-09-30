# V3 Actual-TEB Fair Formal Evaluation and Paper Closure Handoff

Date: 2026-07-25

## Terminal status

V3-11 established and executed a fair paired simulation baseline using the
workspace's actual `teb_local_planner/TebLocalPlannerROS` plugin, not the
historical idealized straight-policy comparator. V3-12-P1 then completed the
pure-offline paper export and evidence closure.

The terminal states are:

- `V3-11-Q1-R1`: fair multi-scene qualification pass;
- `V3-11-F1`: 60/60 fresh formal episodes complete at attempt 1;
- `V3-11-F1-ANALYSIS-R1`: formal positive result and independent replay pass;
- `V3-12-P1`: deterministic paper closure and independent export review pass;
- `formal_result=true`;
- `performance_improvement_demonstrated=true`;
- `training_steps=0`;
- `runtime_ready=false`;
- `real_vehicle_ready=false`.

No real robot, CAN/serial device, stable workspace
`/home/robot/robot_ws`, online TEB parameter write or training process was
used.

## Actual TEB baseline

The primary baseline is the built local workspace plugin:

- plugin: `teb_local_planner/TebLocalPlannerROS`;
- package version: 0.8.4;
- source SHA:
  `5ada4149b1827284403c1664ce61dd14188bb30f8073417f3da1b378f00df8c2`;
- built library SHA:
  `7d17474e7901821d7f69bf10cdb909e4445f9010e32ee045452ad42c6ba1883e`;
- configuration:
  `src/simulation/m2_gazebo/config/fixed_teb.yaml`;
- configuration SHA:
  `9e3125eec7f1f9040356f245dae9620d0b866ac94482ac29fd2d600838b14c26`;
- `max_vel_x=1.2 m/s`;
- Ackermann wheelbase/minimum turning radius remain 0.65/1.20 m simulation
  candidates;
- dynamic obstacles are enabled.

Both methods use the same Autolabor M2 Gazebo model and actuator dynamics,
the same `/cmd_vel_navigation` command boundary, the same 1.2 m/s speed cap,
the same start prefix and gate, the same target and timeout, the same
simulator seed within each block, and the same Gazebo-oracle actor track.
Actual TEB receives that track through
`/move_base/TebLocalPlannerROS/obstacles` while retaining its laser/costmap
inputs. This oracle track isolates planner behavior; it does not qualify a
perception claim.

## Qualification and preserved repair history

The initial smoke and qualification work preserved every failure:

- method-dependent startup displacement and sampled-speed aliasing;
- actor response not observable at the common handoff;
- response-state toggling caused by 50 Hz M2 sampling;
- one pre-semantic Gazebo server SIGSEGV;
- Q1's 23/24 valid negative result, where one V3 actor-speed estimate caused
  an `OCCUPANCY_INTERSECTION` fail-closed stop.

Only infrastructure or single-factor estimator repairs were made. The V3
planner, actual TEB parameters, actor policy thresholds and formal effect
gates were not tuned to qualification outcomes. Q1-R1 used fresh seeds
7021--7032 and completed 24/24. Its independent assessment passed all 14
qualification gates:

- both methods: 12/12 goals;
- V3/actual-TEB mean restricted times: 6.7708/7.7583 s;
- exploratory reduction: 12.7282%;
- zero contact states;
- minimum conservative clearance: 0.5588/0.0443 m;
- maximum paired gate actor coordinate/speed differences:
  0.00320 m / 0.02560 m/s.

Qualification assessment SHA:
`d90f44ff5fde177b764bc71c4f42c2c60ebb2a3403d42651983de334e02adbbe`.
Independent qualification review SHA:
`2d7674f52e33f6a002d4719404e9b8b0ab2ea3f59787d7f914dd8b6467bd4c3e`.

## Formal design

The immutable execution release SHA is
`b21835d5d55a55fd130aae836c609fd117481b9d4cd54155ba8fc78b57dc9ea6`.
It froze:

- fresh seeds 7101--7130;
- 30 paired scene-policy-seed blocks / 60 episodes;
- three scenes:
  `open_crossing_90`, `wide_corridor_crossing_70`,
  `bounded_corridor_crossing_110`;
- cautious and hesitant actor policies;
- counterbalanced method order;
- one attempt per identity, with no retry, resume, replacement or selective
  exclusion;
- restricted gate-to-goal time with a 22 s restriction;
- paired mean relative reduction;
- 200,000 paired-block bootstrap resamples using seed 31100;
- one-sided alpha 0.025;
- point-effect gate at 10%;
- one-sided lower-effect gate at 5%;
- nonnegative mean effect in all six scene-policy strata;
- zero contacts, nonnegative conservative clearance and V3 goal-success
  noninferiority.

The pre-execution analysis release SHA is
`61342983f229dd739a4f0330cebaecc94b0ea981be8982d3f0915bc1d6f69929`.

## Formal result

All 60 episodes completed at attempt 1. No identity was retried, replaced or
forfeited.

| Metric | V3 spatial-safe | Actual TebLocalPlannerROS |
| --- | ---: | ---: |
| Episodes | 30 | 30 |
| Goal success | 100% | 100% |
| Mean restricted gate-to-goal time | 6.7683 s | 7.7733 s |
| Minimum conservative clearance | 0.5110 m | 0.0192 m |

The paired mean reduction was **12.9288%**. The preregistered one-sided 97.5%
bootstrap lower bound was **12.7253%**. Both exceed the frozen 10% point and
5% lower gates. All six scene-policy strata were positive:

| Scene / policy | Mean reduction |
| --- | ---: |
| open-90 / cautious | 12.7742% |
| open-90 / hesitant | 13.5723% |
| wide-70 / cautious | 12.5161% |
| wide-70 / hesitant | 12.8700% |
| bounded-110 / cautious | 12.9653% |
| bounded-110 / hesitant | 12.8700% |

Both methods achieved 30/30 goals and the total contact-state count was zero.
The maximum V3 tracking error was 0.03312 m. Every actual TEB episode
published at least 75 local plans and ended with action state `SUCCEEDED`.
The maximum paired gate actor coordinate/speed differences were
0.00384 m / 0.02987 m/s, within the frozen 0.03/0.03 limits.

Formal assessment SHA:
`9b19f8bd29f1d6c06197e932b03359a2f78081da65e636f52f7f610529c3f2f9`.
Independent formal review SHA:
`2d5e8c21ea70f29df97e646f703159297ff0d2da6ef10edb1341da057c133b39`.
Formal terminal freeze SHA:
`f4396e25716dfb13bdc3e8d33a02387c117ab5a8c066e0d6a3e94b2fe3ac7465`.

## Analysis implementation disclosure

The first frozen analysis invocation replayed the raw evidence and computed
the bootstrap, then raised `NameError` while constructing two unquoted
claim-scope labels. It wrote no assessment or evidence freeze. The failure is
preserved at
`artifacts/v3/actual_teb/v3_11_f1_release/formal_analysis_i0_failure.yaml`.

R1 injected only the intended string labels `"cautious"` and `"hesitant"`
into the frozen module namespace. It changed no formal evidence, pair
construction, bootstrap sample, threshold or result. The repair release SHA
is
`677d29954e216665c414ffb9f11b460e1a7f9d32d4a6a44f427177b95a23e0b5`.
Independent R1 replay reproduced the persisted assessment and 200,000-sample
bootstrap exactly.

## Paper assets

V3-12-P1 exports:

- `v3_12_p1_actual_teb_formal_summary.csv`;
- `v3_12_p1_actual_teb_stratum_results.csv`;
- `v3_12_p1_actual_teb_block_results.csv`;
- `v3_12_p1_actual_teb_formal_result.svg`;
- `v3_12_p1_actual_teb_claims.yaml`.

The paper evidence manifest SHA is
`319bce3401859e25fa1e5da68d97669a6aebf593688c475003a8844b91798c0b`.
The independent paper-closure review SHA is
`96d7d0333ef2eba5b62819fb0fadc689c3756e5015804d2ea8fb86b4a4ca17b2`.
The conventional terminal freeze SHA is
`4e94be77a82acd172cb4f1924f80ccb2abe9e47d7cfda0d4d6eefbf55c988cc9`.

## Exact claim boundary

The qualified claim is:

> Within 30 fresh paired blocks from three frozen single-reactive-actor
> Gazebo templates and cautious/hesitant policies, V3 spatial-safe reduced
> restricted gate-to-goal time by 12.9288% relative to actual
> TebLocalPlannerROS 0.8.4; the one-sided 97.5% paired-bootstrap lower bound
> was 12.7253%. Both methods achieved 100% goal success and no contact state
> was observed.

This does not establish perception robustness, multi-actor or
out-of-distribution generalization, formal collision freedom, a learned/RL
benefit, deployment readiness or real-vehicle performance. The minimum
actual-TEB empirical conservative clearance was only 0.0192 m, so the result
must not be presented as an unrestricted safety ranking.

## Reproduction

No simulation rerun is needed or allowed. Pure-offline checks are:

```bash
cd /home/robot/robot_ws_base_rl
python3 src/tools/thesis_experiment/scripts/assess_v3_11_f1_actual_teb_formal_r1.py \
  --analysis-release-sha256 61342983f229dd739a4f0330cebaecc94b0ea981be8982d3f0915bc1d6f69929 \
  --repair-release-sha256 677d29954e216665c414ffb9f11b460e1a7f9d32d4a6a44f427177b95a23e0b5 \
  --check
python3 src/tools/thesis_experiment/scripts/build_v3_12_p1_actual_teb_paper_closure.py \
  --check
```

The independent review writers are immutable one-shot artifacts and must not
be invoked again after their persisted outputs exist.

## Next safe entry

V3-11/V3-12 are terminal and must not be rerun to seek a larger effect.
Training is not required for the demonstrated analytic-planner result and
remains unauthorized by the V3-04 support audit.

A stronger journal claim requires a new study, not an extension of these
seeds: multi-actor and non-crossing interactions, independent static
geometries, perception/localization degradation, distribution-shift tests
and separately authorized low-speed real-robot validation. Optional learning
should be considered only after cloned interventions provide nonzero audited
support for alternative candidates and a new learning-value gate passes.
