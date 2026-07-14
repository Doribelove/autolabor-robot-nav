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
- Real-robot motion and online TEB parameter writes require explicit on-site user approval;
  Codex must default to simulation, offline replay, or shadow mode.
