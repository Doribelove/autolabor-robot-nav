# V4-SCOPE-RL Terminal Handoff

Date: 2026-08-07

## Terminal status

V4-SCOPE-RL is terminally complete as an honest negative learning study. The
independent package `src/application/v4_scope_rl` implements a sequential
Gymnasium crossing environment and an auditable local PyTorch Masked-PPO
actor--critic. The learned policy selects only among six Ackermann candidate
IDs after a hard Boolean safety mask. It cannot change candidate geometry,
kinematics, occupancy, safety thresholds, certified stop, or the reserved
`/cmd_vel_navigation` boundary.

The frozen program consumed exactly three factor rounds and 600,000 training
environment steps. V4-02-R0 and V4-03-F1 produced zero validation improvement.
V4-03-F2 produced small positive directions for both seeds: 0.1039% and
0.0727%, cross-seed mean 0.0883%, far below the frozen 3% gate. V4-03-F3 used
the final 50,000 steps after a 1,000-episode random-mask preflight with zero
contact and zero mask violation, but returned to zero improvement. Do not add
a factor round or extend any seed.

## Fresh terminal characterization

The validation-selected F2 seed4104 checkpoint was frozen before V4-04-N1.
Fresh seeds 430100--430129 formed 30 paired blocks / 60 fast-simulator
episodes. Both receding analytic and Masked-PPO achieved 30/30 goals, zero
contact and zero mask violations. Mean failure-inclusive restricted time was
7.7750/7.7667 s. The paired relative reduction was only 0.0980%, with a
two-sided 95% paired-block bootstrap interval [0, 0.2941%]; only one of 30
blocks was positive. The 3% learning effect gate failed.

Therefore `formal_result=false` and
`meaningful_optimization_demonstrated=false`. The precise permitted statement
is: “实现了带硬安全动作掩码的强化学习候选选择模块，但未证明其相对解析排序器具有有意义的效率提升。”

## Verification and exports

- Component tests: 9 passed, 0 failed, recorded at
  `artifacts/v4/v4_component_tests.xml`.
- Terminal result: `artifacts/v4/v4_terminal_result.yaml`.
- Claims: `artifacts/v4/paper/v4_06_p1_thesis_export/v4_06_p1_claims.yaml`.
- Tables: `v4_04_n1_method_summary.csv`, `v4_04_n1_paired_blocks.csv`, and
  `v4_factor_history.csv` in the same paper directory.
- Figures: architecture, factor history, fresh paired differences and action
  distribution, each exported as SVG and PDF.
- Combined 49-page thesis draft:
  `artifacts/v4/paper/v4_06_p1_thesis_export/FAM_TEB_V4_RL_THESIS_DRAFT.pdf`,
  SHA256
  `ef46bb726459129e8029eeaf94c5783f3310183b96a3a2606bb64c06cbb25c57`.
- Evidence manifest SHA256:
  `beabd2c9b57a48cf7b7796cb9b60010553c106349dfab6c0535f0f3e3577c729`.

## Claim and execution boundary

No V4 Gazebo unit, Actual TEB comparison, ROS command publication, hardware,
real vehicle, perception, multi-actor/OOD study, or formal safety proof was
run. The 240-unit Gazebo cap remained entirely unconsumed because the learning
gate failed; it is not a bank that may now be spent without a new independent
study and explicit authorization. Historical V3 results remain separate and
read-only. Do not claim V4 superiority over Actual TEB, meaningful RL
improvement, deployment readiness, or real-vehicle readiness.
