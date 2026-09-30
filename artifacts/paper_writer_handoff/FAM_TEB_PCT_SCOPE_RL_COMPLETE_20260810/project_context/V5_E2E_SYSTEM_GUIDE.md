# V5-E2E-SCOPE-RL System Guide

Current component handoff: `CURRENT_V5_01_SENSOR_BACKEND_HANDOFF.md`.

## Thesis hypothesis

V5 tests whether sensor history can expose long-horizon interaction value that
the V3/V4 memoryless analytic ranker cannot represent. The learned actor maps
only robot-available observations to an Ackermann candidate ID. Simulation
truth may reduce critic variance during training, but it is forbidden from the
actor, exported checkpoint interface and deployment inference path.

## Architecture

The fixed decision order is:

`scan history + odometry + goal -> recurrent sensor encoder -> shared candidate encoder -> analytic prior + learned residual -> hard safe mask -> conservative advantage gate -> candidate ID -> activation revalidation -> tracker -> /cmd_vel_navigation`

Learning owns perception-to-candidate valuation. It does not own candidate
geometry, vehicle feasibility, collision/braking constraints, certified stop,
low-level tracking, or the command chain. This is decision-level end-to-end
reinforcement learning rather than an unshielded scan-to-command controller.

## Privileged information rule

The asymmetric critic may observe actor positions, velocities, response mode,
oracle future candidate clearance and time-to-conflict during simulation
training. The actor receives only scan history, odometry, local goal, previous
execution state, candidate features, analytic prior and the hard mask. Tests
must prove that actor output is identical when privileged tensors are changed
and that the actor-only checkpoint loads without the privileged schema.

## Scientific sequence

1. V5-00 freezes the action support, sensor and privileged schemas, reward,
   budgets, methods, gates and claim boundary.
2. V5-01 implements the deterministic sensor-level interaction backend,
   Ackermann candidates and hard shield.
3. V5-02 implements symmetric and asymmetric recurrent masked PPO, analytic
   residual scoring and advantage gating.
4. V5-03 audits sensor identifiability, privileged leakage, multi-action
   support and sequential regret before full training.
5. V5-04 performs bounded multi-seed training and validation-only selection.
6. V5-05 runs the frozen 60-block fast-simulator test and all preregistered
   ablations.
7. Gazebo qualification and Actual TEB comparison are permitted only if the
   fast positive gate passes. Otherwise the stage stops and exports a negative
   thesis result without consuming Gazebo units.
8. V5-07 exports deterministic paper figures, tables, claims, evidence
   manifest and an updated combined PDF.

## Positive claim

A V5 learning benefit requires direct superiority over the same-candidate
sensor analytic prior, not merely over Actual TEB. A point effect alone is not
enough: task, hard shield, replay and the paired confidence gate must also
pass. Historical V3/V4 evidence remains separate and read-only.
