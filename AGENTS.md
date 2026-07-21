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
  `docs/thesis_experiment/CURRENT_V2_04G_R6_I1_R6_I2_R6_I3_R6_I4_R6_I5_R6_I6_RESULT_INTERPRETATION_HANDOFF.md`.
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
- Real-robot motion and online TEB parameter writes require explicit on-site user approval;
  Codex must default to simulation, offline replay, or shadow mode.
