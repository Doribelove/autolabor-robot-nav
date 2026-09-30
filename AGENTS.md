# Codex Project Context

## Project

This repository is a ROS workspace for the Autolabor robot navigation setup.

Important local paths:

- Thesis workspace root: `/home/robot/robot_ws_base_rl`
- Stable real-robot workspace: `/home/robot/robot_ws`
- GitHub remote: `git@github.com:Doribelove/autolabor-robot-nav.git`
- Stable branch: `main`
- Thesis development branch: `base_on_rl`

Keep the two workspaces isolated: build them separately and source only one
workspace's `devel/setup.bash` in a terminal. Thesis development belongs in
`/home/robot/robot_ws_base_rl`; do not modify the stable real-robot workspace
unless the user explicitly requests it.

## Git Workflow

- Do not commit generated ROS build outputs:
  - `build/`
  - `devel/`
  - `install/`
  - `log/`
- Do not commit rosbag recordings:
  - `*.bag`
  - `*.bag.active`
- The initial project import commit is `a4f7490 Initial robot workspace import`.
- The project was prepared for pushing to `Doribelove/autolabor-arena-nav`.
- HTTPS push previously failed with GitHub `403 Permission denied`.
- An SSH key was generated at `/home/robot/.ssh/id_ed25519`.
- The public key that needs to be added to GitHub is in:
  - `/home/robot/.ssh/id_ed25519.pub`

## Submodules

Several third-party packages under `src/` are nested Git repositories and are tracked as submodules through `.gitmodules`.

When cloning this project elsewhere, use:

```bash
git clone --recurse-submodules git@github.com:Doribelove/autolabor-arena-nav.git
```

If already cloned, initialize submodules with:

```bash
git submodule update --init --recursive
```

## Common Commands

Check repository state:

```bash
git status --short --branch
git remote -v
```

Push current main branch:

```bash
git push -u origin main
```

Build ROS workspace:

```bash
catkin_make
```

Source workspace:

```bash
source devel/setup.bash
```

## Operating Notes

- Prefer editing source files under `src/`, `scripts/`, and documentation files.
- Avoid deleting or rewriting user-created data files unless explicitly requested.
- Before pushing, verify that no `.bag`, `build/`, or `devel/` content is staged.
- If a new Codex session lacks context, read this `AGENTS.md`, then inspect `git status --short --branch`.
- For the current GPS navigation development thread, also read `CURRENT_GPS_DEV_HANDOFF.md`.
- For thesis TEB reinforcement-learning, Gazebo, and sim-to-real work, read
  `docs/thesis_experiment/CURRENT_TEB_RL_HANDOFF.md` first, then
  `docs/thesis_experiment/DEVELOPMENT_STATUS.md`,
  `docs/thesis_experiment/experiment_contract.yaml`, and
  `docs/thesis_experiment/UBUNTU20_TEB_RL_EXPERIMENT_BOOK.md` before editing or running experiments.
  For FAM-TEB V2 scene-aware multi-mode architecture work, also read
  `docs/thesis_experiment/V2_SYSTEM_GUIDE.md`, then
  `docs/thesis_experiment/CURRENT_V2_FOUNDATION_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V2_02_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V2_03_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V2_04_HANDOFF.md` after the V1/T12 handoff files.
  For the current V2-04G TTC robustness thread, also read
  `docs/thesis_experiment/CURRENT_V2_04G_R4_R1_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V2_04G_R5_DESIGN_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V2_04G_R5_EXECUTION_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V2_04G_TTC_D1_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V2_04G_R6_DESIGN_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V2_04G_R6_I1_EXECUTION_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V2_04G_R6_I1_R6_I2_REPAIR_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V2_04G_R6_I1_R6_I2_R6_I3_AUTHORIZATION_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V2_04G_R6_I1_R6_I2_R6_I3_EXECUTION_READINESS_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V2_04G_R6_I3_RELEASE_PREFLIGHT_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V2_04G_R6_I4_PREFLIGHT_INTEGRITY_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V2_04G_R6_I5_EXECUTION_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V2_04G_R6_I1_R6_I2_R6_I3_R6_I4_R6_I5_R6_I6_RESULT_INTERPRETATION_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V2_04G_P1_R1_OFFLINE_READINESS_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V2_04G_P1_E1_EXECUTION_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V2_04G_P1_I1_RESULT_INTERPRETATION_PAPER_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V2_04G_P2_D1_TRACE_ATTRIBUTION_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V2_04G_P2_D2_SINGLE_FACTOR_DESIGN_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V2_04G_P4_F1_TERMINAL_HANDOFF.md`.
  For future V3-SCOPE work, read the terminal P4-F1 handoff first, then
  `docs/thesis_experiment/CURRENT_V3_SCOPE_FOUNDATION_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V3_00_FOUNDATION_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V3_01_REACTIVE_ACTOR_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V3_02_PREDICTOR_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V3_02_COMMON_PREFIX_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V3_02_FEASIBLE_INTERVENTION_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V3_03_SAFE_PLANNER_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V3_04_LEARNING_AUDIT_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V3_05_FORMAL_EVALUATION_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V3_09_FORMAL_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V3_11_ACTUAL_TEB_FORMAL_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V3_14_MULTIACTION_TERMINAL_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V3_15_PAPER_EXPORT_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V3_16_RESPONSE_GATED_TERMINAL_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V3_17_PAPER_EXPORT_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V3_21_FOUR_METHOD_CHARACTERIZATION_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_V3_22_THESIS_EXPORT_HANDOFF.md`, then
  `docs/future_research/v3_candidate/THEORETICAL_FEASIBILITY_AND_RESEARCH_ROADMAP.md`, then
  `docs/future_research/v3_candidate/literature/README.md`. V3-00--V3-12-P1 are complete.
  V3-11-F1 is the latest fresh formal positive result within three frozen single-reactive-actor
  Gazebo templates against actual TebLocalPlannerROS 0.8.4. It is not a perception,
  multi-actor/OOD, general-safety, deployment or real-vehicle result.
  For the current T12 Residual SAC thread, also read
  `docs/thesis_experiment/CURRENT_T12_RESIDUAL_TRAINING_HANDOFF.md`, then
  `docs/thesis_experiment/CURRENT_T12_RESIDUAL_LEARNING_REVIEW.md`, and check live processes
  before starting anything.
- Thesis stages T00--T11 are complete. T12 safety repair, no-training residual pilot, original
  two-seed Residual SAC run, offline diagnosis, frozen three-method pairing, and curriculum
  single-factor pilot are complete. The curriculum repair restored all five training scenes, but
  validation improved only for seed102 and the cross-seed mean change remained negative. The
  second single-factor episode-anchor pilot was stopped after both bounded seed101 attempts ended
  in move_base SIGSEGV. Boundary atomicity, activation-timeout recovery, and static-footprint
  lifetime amendments then removed the repeatable crash chain: the latest seed101/102 2000-step
  runs both completed with 14/14 test goals and no crash. Validation changed +0.4527/-0.0784, so the
  two-seed learning gate failed and the new frozen three-method pairing was not run. `formal_result`
  remains false. The follow-up offline action/projection diagnosis is complete: training projection
  averaged 67.3%, with Ackermann coupling and post-WARNING return-to-anchor rate limits as the two
  structural sources. Do not restart these pilots or expand the budget. Preregister exactly one
  action/execution-alignment learning factor before any new pilot.
- FAM-TEB V2-00--V2-03 component gates and the V2-04 software/shadow transaction gate are complete.
  V2-04 provides an uncalibrated simulation-candidate Anchor Bank, typed profiles, feasible decoder,
  previous-executed smoothing and a zero-training ROS shadow loop. It does not authorize SAC,
  dynamic-reconfigure writes, real-vehicle operation or performance claims. Read the V2-04 handoff
  before extending it; keep all deployment thresholds `runtime_ready=false`.
- FAM-TEB V2-04G-R5 TTC single-factor bounded calibration is terminally stopped. The only
  attempted identity was readiness `r5_ttc_h450` seed5111 at attempt=1: activation, transaction,
  join and navigation passed, but expected `OBSERVED_CONFLICT` was observed as
  `NO_CONFLICT_IN_HORIZON` with zero finite TTC samples. The failure episode is preserved;
  component and navigation were not started; there is no passing candidate or winner. Do not
  retry or resume R5, consume its remaining budget, freeze a winner, consume held-out seeds
  5001--5010, train SAC or connect a real vehicle. Read the execution handoff before any new work.
- V2-04G-TTC-D1 offline-only diagnosis is complete. It reproduced zero finite TTC, found the
  actor reached the crossing 4.0053 s before the robot, and showed that the frozen 5.0/4.5/4.0 s
  horizons do not distinguish any of the 21 reachable CROSSING samples. The 1.5/1.0 s values are
  distinguishable only as offline future candidates; they are not qualified or authorized.
  Six execution-integrity risks are machine-audited in the D1 report.
- V2-04G-R6-I1 execution integration review was established and its separately authorized
  fresh-seed simulation is terminally stopped. Sequence 1, legacy control / single-conflict /
  seed5141 / attempt1 consumed one of six units, then timed out waiting for the TEB dynamic
  reconfigure service before semantic execution. The confirmed cause is a paused simulated-clock
  bootstrap ordering deadlock: the runner waited for move_base readiness before its later Gazebo
  unpause call. The canonical journal is terminal and non-resumable; the remaining five units are
  forfeited. No aligned profile, semantic episode, winner, held-out seed, training, or real vehicle
  was run. Post-review also found incomplete external hash closure, incomplete authorization
  enforcement, hash/parse TOCTOU, and a broken preauthorized assessor. Do not retry/resume R6-I1
  or reuse its identities/budget. Any future execution requires a new stage, fresh seed/budget,
  repaired bootstrap and integrity closure, an independent review, and new explicit authorization.
- V2-04G-R6-I2 is an independent bootstrap/integrity repair review. It implements and statically
  reviews the positive-progress `/clock` bootstrap barrier, canonical path+SHA closure for all
  inherited/I2 dependencies and five runtime bindings, closed type-sensitive authorization and
  exact-schedule enforcement, single-open/no-follow hash+parse, a deterministic assessor, and
  credential-safe child environments/logging. The semantic factor, seven thresholds, scenes and
  evaluator remain frozen. R6-I2 has no execution authorization, seeds, schedule or budget and did
  not start ROS/Gazebo. Its physical files use the frozen R6 reviewer's predeclared R6-I1 downstream
  ownership prefixes only for compatibility; R6-I1 bytes and stage identity are unchanged. Any
  future execution still requires a new user instruction, fresh seed/budget, independent stage and
  separate authorization.
- V2-04G-R6-I3 authorization and execution-readiness closure are complete as historical offline
  snapshots. A later explicit simulation instruction created the unique canonical release with SHA
  `5c47557f539f5d2dcf91349d1d7fda87d81de4d08f75be174644930879ac7fb6`; its closed schema and
  22 path+SHA bindings pass. Full prejournal validation then failed closed because the validator
  requires string YAML keys for every authorization-bound YAML while the frozen R6-I1 scene
  derivation has integer `seed_roles` keys 5141--5147. `--execute` was not called: all six units are
  unconsumed, none forfeited, and no attempt root, journal, report, ROS or Gazebo exists.
  `execution_ready=false`. Preserve the failed release; do not overwrite/delete it, invoke execute,
  patch frozen hashes or reuse this authorization. Any repair requires a new independent offline
  stage/review and explicit instruction, followed by a separate future execution authorization.
- V2-04G-R6-I4 is the completed independent offline preflight-integrity repair/readiness closure.
  It preserves the failed I3 release at SHA
  `5c47557f539f5d2dcf91349d1d7fda87d81de4d08f75be174644930879ac7fb6`
  and the I3 6 authorized / 0 consumed / 0 forfeited snapshot. Its versioned validator rehashes the
  exact 12-resource authorization roster but parses only preregistration and the inherited I2
  closure; the other 10 resources, including the legacy integer-key scene derivation, remain
  hash-only. Real-roster regressions, runner trusted hashes, inherited I3/I2 target rehashing,
  canonical closure and deterministic machine review pass offline. I4 has no seeds, schedule or
  budget, the future I4 release is absent, no execution state or ROS/Gazebo was created, and
  `execution_ready=false`. Its required later explicit simulation instruction was supplied only for
  the separate I5 stage; I4 itself remains non-executable and does not authorize any further work.
- V2-04G-R6-I5 bounded simulation execution is terminally complete. Its unique release SHA is
  `9cef80f5c4eaf562719a71bb11fadd2cded7208d2ade07a22b09d7b6058b3d43`; the exact six-unit
  paired schedule used fresh execution seeds 5161--5163 at attempt=1 and completed 6/6 with no
  terminal failure, retry, resume or forfeiture. Expected/observed TTC status matched all six rows;
  the deterministic assessment replayed every journal/raw resource and returned
  `simulation_integration_validation_pass` with report SHA
  `8ed096601c13cc45fba34d32d5ae78477cabd345b9730df8ab4eced7fc0e5599` and no integrity
  failures. Final ROS/Gazebo/process isolation is clean; no training or real vehicle was used.
  This proves only fresh simulation semantic/execution integration: `formal_result=false`,
  `runtime_ready=false`, no winner and no downstream authorization. Do not rerun/resume I5 or reuse
  its seeds/budget. The next safe entry is a separately authorized offline result-interpretation/design
  closure; any performance study needs a new preregistered fresh multi-seed stage.
- V2-04G-R6-I6 is the completed pure-offline result-interpretation/design closure. Its 13-resource
  I5 evidence-freeze manifest has SHA
  `40d9eba914840d33a7966f7c5bff972e94d9123239b1cc1cc0c0971752288935`; the deterministic
  interpretation report has SHA
  `c1fd43205d0f3b3c6a029590b33808812dc8db795bdcf4b270c345e033b9dd68` and status
  `offline_result_interpretation_design_closure_pass`. It qualifies only I5 fresh-simulation
  semantic/execution integration and explicitly forbids performance, generalization, safety,
  winner, deployment or real-vehicle claims. The final future performance design, V2-04G-P1,
  preregisters 90 fresh scene-seed blocks / 270 episodes with zero additional training steps, but
  has `execution_authorized=false` and budget 0. Do not interpret I6 or that design as a release or
  authorization; I5 remains frozen and non-rerunnable. Any performance execution requires an
  independent review, new release, fresh budget and separate explicit simulation authorization.
- V2-04G-P1-R1 is the completed zero-authority offline scene/schedule/statistics/integrity closure.
  It materializes seeds 5201--5290 as 90 evidence blocks plus four compile-support seeds,
  94 scenes / 188 compiled children and a deterministic counterbalanced 270-row schedule. The
  claim population is strictly three frozen dynamic templates under fresh simulator stochastic
  seeds, not 90 independent geometries. Its dependency closure rehashes 428 local resources and
  the independent review status is
  `p1_r1_offline_implementation_readiness_closure_pass_release_absent`. Statistical fixtures pass,
  but no P1 evidence assessment ran. `execution_ready=false`, `execution_authorized=false`,
  authorized/consumed budget=0, and release/attempt/journal are absent. Exact P1 runner and raw
  replay adapters remain future release prerequisites; do not start ROS/Gazebo or treat this
  offline pass as execution authorization. I5 stays frozen and no historical seed/budget may be
  reused.
- V2-04G-P1-E1 and P1-I1 are terminally complete. The exact fresh-seed 270-episode matrix retained
  all 270 attempt=1 identities and passed integrity, safety/task noninferiority and mechanism gates,
  but the efficiency effect was only 3.7624% with one-sided lower 2.7904%, below the frozen 5%
  minimum effect. `formal_result=true` and `performance_improvement_demonstrated=false`; do not
  rerun, resume, replace seeds, or reinterpret the valid negative result as positive.
- V2-04G-P2-D1 is a completed pure-offline trace attribution. On the 30 frozen semantic-clear
  treatment traces it found a conservative same-path post-cruise envelope of 12.0144% with
  one-sided lower 11.2649%, and an approximate 10% target speed of 1.38349 m/s. This is physical
  room only: it does not qualify a runtime field, safety, or achieved performance.
- V2-04G-P2-D2 is the completed zero-authority offline single-factor design closure. The only
  behavioral field is `TebLocalPlannerROS.max_vel_x`, with control 1.20 m/s and future calibration
  grid 1.20/1.25/1.30/1.35/1.40 m/s; every nonfactor and safety gate remains frozen. TEB-Default is
  context only, while the primary baseline must be the calibration-frozen strongest safe choice
  between exact TEB-Tuned-V1 and an independently speed-tuned fixed TEB. The global program is
  capped at 3 factor rounds / 2200 simulation evidence units / 120000 training environment steps
  with non-recyclable one-sided alpha 0.025/0.015/0.010. Round 1 reserves calibration seeds
  5301--5330, qualification 5331--5342 and held-out test 5401--5490. This was the historical design
  entry for the now-terminal P2/P3/P4 chain; do not treat its former future work as current.
- V2-04G-P4-F1 is the terminal offline closure for the post-P1 finite performance search. Across
  the three frozen rounds, the 2200-unit capacity closed as 4 consumed plus 2196 retired; no
  training steps or new held-out test were used. Round 3's exact P4-I1 release
  `1bcd0392d475cdf17d7a2bf6540e024a61136150011945d323cae1234b568220`
  consumed fresh seed5924 attempt1, then failed before Gazebo startup because roslaunch could not
  resolve package `m2_gazebo`. The promoted Bank had passed strict offline parsing, but live
  transaction or performance behavior was never reached. Independent terminal review SHA is
  `e0dc93790e092b99684c64a9d2b11e5ce02a0a16f5235649e3657376835270c8`.
  The final conclusion is `obvious_performance_improvement_not_demonstrated` / “未证明明显提升”:
  historical P1 remains 3.7624% point and 2.7904% lower versus the new 10%/5% gate, and no complete
  post-P1 test matrix exists. Performance search is stopped; do not retry/reuse P2/P3/P4 seeds or
  releases, repair-and-rerun P4-I1, add a round, recycle retired budget, train, access new held-out
  outcomes, freeze a winner, or make positive/deployment/real-vehicle claims.
- V3-SCOPE began as the saved future architecture direction and is not a continuation of the terminal
  V2 performance search. It changes the learning action from high-dimensional TEB parameters to a
  discrete choice among pre-generated Ackermann-feasible trajectories, then applies
  candidate-conditioned interaction prediction, calibrated reachable occupancy, a hard backup/
  braking safety verifier, and an analytic ranker before any optional contextual bandit or offline
  RL. The historical V2 Gazebo waypoint actor is open-loop; V3-01 later supplied and qualified a
  separately reviewed reactive-actor and cloned-intervention data stage. Hardware and the stable
  command chain may remain unchanged:
  any future planner must output through `/cmd_vel_navigation` and retain the GPS limiter, FOD mode
  arbiter and M2 driver safety boundary. The original concept handoff itself remains
  non-executable; later V3-00/V3-01 stages supplied their own bounded authority and are now frozen.
  Do not modify `/home/robot/robot_ws`. Read the current V3 handoffs in the order above before new
  V3 work.
- V3-03--V3-12-P1 are terminally complete. V3-03-C6 completed fresh segmented Gazebo integration
  9/9 with 308/308 accepted selections, certified-stop availability and exact replay, zero
  conservative-clearance violations and P99 latency 16.2301 ms. V3-04 correctly closed learning:
  the logged closed-loop contexts had zero propensity for unexecuted alternatives and no sequential
  support, so training remained 0 steps and no learned-policy claim exists. V3-05-B0 preserved a
  complete negative longitudinal formal result (270/270; 1.2007% point, 0.9647% lower; 93.33% V3
  goal success). V3-06 longitudinal repairs remained negative and terminal. V3-07/V3-08 then
  established spatial candidates, the frozen spatial predictor, hard verifier and fresh A3
  integration pass. V3-09-Q1 preserved a fairness failure; Q1-R1 qualified only the shared 0.25 s
  replanning protocol.
- V3-09-F1 is the historical idealized-comparator formal evaluation. Fresh seeds 6801--6830 formed 30 blocks and
  180 paired episodes; all 180 completed at attempt=1 with no retry, resume, replacement or
  forfeiture. V3 spatial-safe and fixed-v1.20 shielded achieved 90/90 goals each; mean restricted
  goal times were 8.8611 s and 11.4167 s. The preregistered paired relative reduction was 22.3844%
  with one-sided 95% block-bootstrap lower 21.8235%; all 30 blocks were positive and all frozen
  safety, task, replay, finite-statistic and latency gates passed. Assessment SHA is
  `d950256f4da310f65d604031ec72009e014facfb7cb0c14be8cf19ce1b9c346b`; independent
  review SHA is `206732c86b536e7f9a520c58dac6d386ff6752595bdc767d4957e4804e5f8d5a`.
  This supports only the frozen parameterized reactive single-actor crossing population and is
  retained as historical mechanism evidence.
  The comparator is a shielded straight policy, not actual TEB; assertive-policy mean time is
  negative for V3 (11.5833 s versus 10.8917 s). Do not rerun/resume F1, reuse seeds/budget,
  suppress negative strata, or claim actual-TEB superiority, general safety, cross-scene
  generalization, deployment, learned benefit or real-vehicle readiness.
- V3-10-P1 is the zero-authority offline paper-evidence closure. Its conventional multiline
  evidence manifest SHA is
  `70353b355c6eec2f733901ca7d4a6820de550beff00999cfacdb359dec951938`; independent
  closure review SHA is
  `bee5891b08f860a97a25ea71047b3e5fabb0c193695f221de131eacb5dcfe0f4`.
  It preserves two historical V3-09-F1 manifests containing literal backslash-n separators and
  binds their exact bytes without changing the formal evidence. V3-10-P1 creates no execution,
  training, seed, schedule, ROS/Gazebo or real-vehicle authority. Any stronger publication claim
  required a new study with an actual TEB baseline; that separately released study is now V3-11.
- V3-11-Q1-R1 and V3-11-F1 are terminally complete. Q1-R1 used fresh seeds 7021--7032 and
  completed 24/24 qualification episodes; both V3 spatial-safe and actual
  `teb_local_planner/TebLocalPlannerROS` 0.8.4 achieved 12/12 goals, zero contact, matched common
  gate/mode semantics and passed all 14 qualification gates. F1 release SHA is
  `b21835d5d55a55fd130aae836c609fd117481b9d4cd54155ba8fc78b57dc9ea6`; fresh seeds
  7101--7130 formed 30 paired scene-policy blocks / 60 episodes over three frozen Gazebo
  templates and cautious/hesitant actors. All 60 completed at attempt=1 with no retry,
  replacement or forfeiture. V3/actual-TEB goal success was 30/30 and mean restricted
  gate-to-goal time 6.7683/7.7733 s. The preregistered paired reduction was 12.9288% with
  one-sided 97.5% lower 12.7253%; all six scene-policy strata were positive, contact count was
  zero and all 17 frozen gates passed. Formal assessment SHA is
  `9b19f8bd29f1d6c06197e932b03359a2f78081da65e636f52f7f610529c3f2f9`; independent
  review SHA is `2d5e8c21ea70f29df97e646f703159297ff0d2da6ef10edb1341da057c133b39`.
  The first analysis invocation's unquoted metadata labels caused a preserved NameError before
  any assessment write; R1 injected only the two intended string labels and changed no evidence,
  statistic or threshold. Do not rerun/resume V3-11, reuse its seeds/budget, tune from formal
  outcomes or claim perception, multi-actor/OOD, formal safety, training, deployment or
  real-vehicle readiness. Both methods shared a Gazebo oracle actor track; actual TEB's minimum
  empirical conservative clearance was 0.0192 m.
- V3-12-P1 is the zero-authority actual-TEB paper closure. Its evidence manifest SHA is
  `319bce3401859e25fa1e5da68d97669a6aebf593688c475003a8844b91798c0b`; independent
  paper review SHA is
  `96d7d0333ef2eba5b62819fb0fadc689c3756e5015804d2ea8fb86b4a4ca17b2`.
  Its conventional terminal freeze SHA is
  `4e94be77a82acd172cb4f1924f80ccb2abe9e47d7cfda0d4d6eefbf55c988cc9`.
  It exports deterministic summary/stratum/block CSV, SVG and claims YAML and grants no
  execution, training or real-vehicle authority. A stronger study needs new multi-actor,
  independent-geometry, perception/OOD and separately authorized real-vehicle evidence.
- V3-00 pure-offline foundation and the V3-01 deterministic crossing reactive-actor qualification
  are complete. V3-01 first exposed a terminal Gazebo pause-flag failure; R1 then completed 36/36
  but failed the frozen cross-seed repeatability gate at 0.2244 m versus 0.20 m. R2 removed only the
  uncalibrated response-margin seed jitter, used fresh seeds 6121--6123, completed 36/36 and passed
  all nine policy-seed response blocks; independent result review SHA is
  `f9363de30f018208d3e2adf0c9cdbe6b19e4c9b2708ec3eba356f03bca2d0ffd`.
  This qualifies deterministic crossing counterfactual data generation only: stochastic actors,
  other scene families, prediction, planning, performance and real-vehicle claims remain
  unqualified. Do not rerun/resume V3-01/R1/R2 or reuse their seeds/budgets. The next safe entry is
  a separately reviewed V3-02 candidate-conditioned prediction and reachable-occupancy calibration
  design.
- V3-02 candidate-conditioned prediction is complete for parameterized crossing. A1 collected
  72 training + 36 calibration episodes. The first frozen T1 test completed 36/36 but was a valid
  negative at coverage 0.7986 versus 0.90. B1 changed only the conformal score from outer-envelope
  nonconformity to actual-mode absolute residual, kept the regression/physical clip/gates frozen,
  and did not use T1 outcomes for fitting. The fresh T2 seeds 6231--6233 completed 36/36 and passed:
  coverage 0.9931, minimum per-horizon coverage 0.9722, mean full width 2.3397 m, actual-mode MAE
  0.0483 m, minimum candidate sensitivity 0.7625 m and zero physical violations. Independent
  result review SHA is `758cc530c74b44655b60a69c1a2b0da49b22dd251b8fe98c386e92c5aee6e7dd`.
  A later preintegration review correctly found that T2 candidates branched before the observation
  time, so T2 was not directly usable for single-state online candidate choice. The C1/C1-R1 chain
  imposed an exact 0--0.5 s common prefix and froze a complete-quadratic model. C2 fresh seeds
  6261--6263 completed 36/36 and passed: coverage 0.9931, minimum horizon 0.9722, mean width
  3.9098 m, MAE 0.1918 m, minimum candidate sensitivity 0.2388 m, zero physical violations and
  exactly zero actor coordinate/speed spread across candidate prefixes. A post-assessment manifest
  scope bug was repaired offline without changing the immutable assessment or rerunning simulation;
  C2-I1 independent integrity review SHA is
  `56ef648a291a9078ad2dce947b3f308e690d09b6ac325b4554afdbeafc0fabb4`.
  This is a finite common-prefix parameterized-crossing predictor result only; do not rerun/reuse
  V3-02 stages, seeds or budgets, and do not claim general prediction, planner safety, navigation
  performance or real-vehicle readiness. The next safe entry is a separately reviewed V3-03 hard
  verifier and analytic safe-candidate planner.
- V3-02-D1/D2 close the later-discovered action-feasibility integration gap. D1 replaced the
  post-prefix velocity discontinuity with one frozen 0.60/0.80 m/s2 acceleration/deceleration
  generator and added certified stop to the 0/0.6/0.8/1.0/1.2 m/s support. Its 135/135 train/cal
  episodes passed trace, acceleration, stop and common-prefix gates; the 90/45 split calibration
  passed. Frozen model SHA is
  `fa4ef83b9b2e534da3031001f5dd0036cd7f983ebfbe73fe20f45c4b73bc55d7`.
  D2 fresh seeds 6281--6283 completed 45/45 and passed all frozen gates: coverage 1.0, all horizon
  coverage 1.0, mean width 3.8779 m, MAE 0.1897 m, minimum sensitivity 0.7379 m, zero physical,
  prefix, trace and stop violations. Independent result review SHA is
  `f0fd7d3908015851820c4512f868a9a5944919612c8f826c914cba309f3fa001`.
  Do not rerun/reuse D1/D2 seeds or budgets. This still does not qualify navigation safety,
  performance, general scenes, deployment or real vehicles. V3-03 may use only the frozen
  feasible-generator/predictor semantics and must retain certified-stop fail-closed behavior.
- Historical V3-00 pure-offline foundation is complete. The independent review status is
  `v3_00_pure_offline_foundation_closure_pass`, review SHA is
  `bb9ce5641ed650f2f3b14688f15f7a3787679f7e06c9bc4d62cd435be7b6b1bf`, and the terminal
  11-resource seal passes. It adds the independent `v3_scope_planner` interface package, five ROS
  messages, a hermetic package-resolution design, deterministic Ackermann candidate fixtures and
  a reactive-actor/cloned-rollout data contract. V3-00 remains `formal_result=false`,
  `execution_authorized=false`, `training_authorized=false`, with budget/seed/schedule all zero;
  its static actor rows are schema fixtures, not simulation evidence. Do not treat the pass as
  actor identifiability, planner runtime, safety or performance evidence. Its prescribed V3-01
  successor is now complete as described above.
- V3-16-D1 and V3-16-S1 are terminally complete. D1 attributed V3-14's common-prefix failure to
  pre-branch reactive-response amplification. S1 changed exactly one factor by delaying actor
  response until 12.55 s, then completed fresh seed7741 preflight plus seeds7742--7749 evidence:
  45/45 attempt-1 units, no retry/replacement/forfeiture. Maximum actor speed spread fell 88.07%
  to 0.008597 m/s and passed, but coordinate spread remained 0.067472 m and failed the 0.05 m
  gate; only 5/8 blocks passed coordinate matching. The result is terminal negative with
  `predictor_fit_authorized=false`. The original assessor's mixed-resource indexing failure is
  preserved; R1 repaired only raw-report path lookup and passed independent review SHA
  `c4469571a1a8f8e999e733138d42466fa0c0a6bd472bcebe5d044fe512a4765c`. Do not rerun/resume,
  reuse seeds 7741--7749, change gates, append samples, fit a multi-actor predictor, or start
  actual-TEB multi-actor/perception/OOD/training/hardware work from this evidence.
- V3-18--V3-22 thesis consolidation is terminally complete. V3-18 provides the reviewed project
  audit and an 825-row historical canonical adapter. V3-21 completed a fresh four-method,
  30-block/120-episode failure-inclusive characterization with seeds 8201--8230. V3/actual-TEB
  success was 28/30 versus 30/30 and restricted mean time was 7.785/7.770 s; the paired relative
  reduction was -0.1931% with two-sided 95% CI [-19.8668%,12.9820%], so actual-TEB superiority was
  not demonstrated. V3 reduced restricted time 49.4535% versus the no-candidate-conditioning
  ablation, which is exploratory mechanism evidence. One retained gate tolerance deviation makes
  `formal_result=false` while `characterization_result_available=true`; exclusion sensitivity did
  not change the conclusion. Do not pool this study with the separately valid V3-11 historical
  positive, rerun/reuse V3-19--V3-21 identities/seeds, delete timeouts, or make general TEB,
  perception/OOD, formal-safety, deployment or real-vehicle claims. V3-22 exports reviewed five-
  figure/five-table paper materials and a terminal 32-page A4 PDF at
  `artifacts/v3/paper/v3_22_p2_thesis_export/FAM_TEB_V3_THESIS_DRAFT.pdf`; PDF SHA is
  `22f4e423a4211590c2d4936c8be4a7f5d6963da83083e0dd8f86b0197af58b7d` and independent
  paper review SHA is `adb4317773484601dd3a1b2006098051521c611fdaa7073138991112a7a3d7a5`.
- V5/V6 end-to-end sensor-to-candidate reinforcement learning is terminally complete. V5 trained
  an asymmetric-PPO actor from only deployable scan/odometry/goal/candidate/prior/mask inputs while
  keeping 32-dimensional simulator privilege critic-only, then evaluated eight Ackermann-feasible
  candidate trajectories behind an analytic prior and fail-closed hard shield. Its frozen 60-block
  fresh formal study was a valid negative (6.0936% versus analytic, 95% CI
  [1.7867%, 11.0069%], below the 8% gate). V6 changed exactly one learning factor: privileged
  counterfactual trajectory-rank distillation (PCT, weight 0.35). Four fresh 120k-step identities
  all improved validation, and the frozen 80-block/320-episode V6-02-F1 formal study achieved
  60/80 versus analytic 49/80 goals, mean restricted time 13.115625 versus 14.318750 s, and a
  paired reduction of 8.402444% with two-sided 97.5% CI [3.484003%, 13.713459%]; all task,
  contact, shield/mask, finite-statistic and latency gates passed. V6 also improved 3.161052%
  over V5 (95% CI [0.493580%, 6.576508%]). Formal assessment SHA is
  `08a0ab06343cad51ee206ba502d2a115c6fd707d58620f9bb4f519a71ec95c1b`. The no-history
  ablation tied V6 exactly, so no temporal-history benefit may be claimed.
- V6 Gazebo sensor-transfer preflights are terminal negative and did not enter qualification or
  formal evaluation. The frozen P0/R1/R2/V6-04 attempts preserved, respectively, a sandbox
  network block, invalid learned/analytic conservative clearances despite completed goals, and a
  reachable-tracker timeout for V6 while analytic/actual TEB reached the goal. Do not rerun these
  identities, access the reserved qualification/formal seeds, claim Gazebo or actual-TEB
  superiority, formal safety, perception/OOD generalization, deployment or real-vehicle readiness.
  A future transfer repair requires a new preregistered stage, factor and fresh authority. The
  reviewed paper export is at
  `artifacts/v6/paper/v6_05_p1_thesis_export/FAM_TEB_V6_PCT_RL_THESIS_DRAFT.pdf` (SHA
  `c1a23074f1524cf967b89f674017f03529e4b54ab171abc5cabfa5749229d829`); independent R2
  paper review SHA is `ff4b6423d8fb610fe29486ab059c86a1e86f284055c8b520cf9e38cf4e470b41`.
  Read `docs/thesis_experiment/CURRENT_V6_PCT_SCOPE_RL_HANDOFF.md` before any successor work.
  Preserve the release-bound V5-00 architecture contract at SHA
  `529ed6fd6776e6b259abc5398352ea0a047948af02dc12331da5848dc5c1ca6d`; its known legacy
  post-action YAML indentation defect is documented in the handoff and must not be fixed in place.
- V6-06-P1-R1 is the terminal pure-offline current-thesis transfer closure. Use
  `artifacts/v6/paper/v6_06_p1_r1_thesis_transfer_package/` and read
  `docs/thesis_experiment/CURRENT_V6_06_THESIS_TRANSFER_HANDOFF.md` before updating an external
  school-template draft. The package contains a self-contained 41-page integrated V3+V6 PDF,
  V3/V5/V6 data, 11 editable figures, five V3 three-line tables, combined claims, an explicitly
  empty real-vehicle chapter and a ready-to-use other-computer Codex prompt. PDF SHA is
  `a2fcacc82ee8f7fc290415661b34bd5324bbc157158d54a0780705a7737eadf1`; independent review SHA is
  `00e4705f783632723018c6920f8e1b68dc6096a08cc28018d72e286e783d1f1b`. Do not replace pending
  real-vehicle fields with invented values or reinterpret V3 Gazebo evidence as V6 evidence.
- Real-robot motion and online TEB parameter writes require explicit on-site user approval;
  Codex must default to simulation, offline replay, or shadow mode.
