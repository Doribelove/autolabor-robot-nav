# V4-SCOPE-RL System Guide

## Purpose

V4-SCOPE-RL is an independent successor to the frozen V3 studies. It adds a
reinforcement-learning decision layer without giving the learned policy
authority over vehicle feasibility or safety. Historical V2/V3 evidence is
input context only and is never rewritten, reused for fitting, or filtered.

## Architecture

The online order is fixed:

`snapshot -> Ackermann candidates -> reachable occupancy -> hard verifier -> safe action mask -> masked policy -> activation revalidation -> tracker -> /cmd_vel_navigation`

The learned policy chooses only a candidate ID. Candidate geometry,
kinematics, safety thresholds, the certified-stop fallback, and the downstream
robot command boundary remain outside learning. An empty learned choice is not
possible because the certified-stop candidate is always present; any runtime
inconsistency fails closed before command publication.

## Why this is reinforcement learning

The decision is repeated every 0.25 seconds. The actor can react to the
robot's earlier choices, so a locally fast action can create a slower future
interaction. Masked PPO optimizes the discounted sequence return over these
transitions. This is different from V3's one-shot analytic ranker and from
supervised imitation of that ranker.

Safety is not represented by a tunable collision penalty. Unsafe actions have
zero probability under the hard verifier's Boolean mask. Reward optimizes
progress, elapsed time, and command smoothness only inside the verified safe
set.

## Evidence sequence

1. V4-00 freezes the architecture, action semantics, reward, budgets, gates,
   and claim boundary.
2. V4-01 implements the deterministic fast backend, mask invariants, analytic
   baseline, and a local PyTorch masked-PPO reference.
3. V4-02 audits whether sequential regret and safe multi-action support exist.
   Full training is stopped if learning has no measurable decision value.
4. V4-03 performs bounded multi-seed training. Checkpoints are selected only
   on frozen validation seeds.
5. V4-04 qualifies backend-to-Gazebo semantics and the command boundary.
6. V4-05 performs a fresh paired comparison against frozen V3, actual TEB
   0.8.4, and an imitation control.
7. V4-06 replays evidence and exports deterministic thesis tables, figures,
   claims, a manifest, and an updated PDF.

## Result discipline

The primary learning claim is V4 versus the frozen V3 analytic ranker. Beating
actual TEB alone is insufficient. A positive optimization result requires the
frozen task, hard-safety, mask-integrity, and paired efficiency gates together.
Every failed or mixed result remains in the evidence chain. No test-driven
retuning, seed replacement, hidden timeout removal, or real-vehicle inference
is allowed.

The canonical machine-readable contract is
`src/application/v4_scope_rl/config/v4_00_architecture_contract.yaml`.
