# V3-03 Safe Planner Handoff

Date: 2026-07-24

## Terminal status

V3-03 is terminally complete for its bounded integration scope. The final
stage is V3-03-C6. Its fresh segmented Gazebo schedule used seeds 6361--6363,
three evaluator-only reactive-actor policies per seed, one attempt per
identity, and completed 9/9 units without retry.

The deterministic result replay passed every preregistered gate:

- goal success: 9/9;
- hard-verifier accepted selected candidates: 308/308;
- certified stop availability: 308/308;
- exact core replay: 308/308;
- yield-before-clear and motion recovery: 9/9;
- conservative clearance violations: 0;
- minimum conservative clearance: 0.21825012646947406 m;
- P99 planner latency: 16.230065 ms;
- maximum goal time: 10.0 s.

The terminal result is
`artifacts/v3/planner/v3_03_c6/v3_03_c6_terminal_result.yaml`; the terminal
freeze is
`artifacts/v3/planner/v3_03_c6/v3_03_c6_terminal_freeze.sha256`.

## Repair history that must remain frozen

V3-03-C1 failed before planner execution because its segment trace parser
required a full-episode row count. V3-03-C2 repaired only that parser and then
failed after a selected commitment left the next braking domain. V3-03-C3
added a 0.25 s commitment-forward-invariance check and passed 810 offline
property cases. V3-03-C4 then completed 9/9 safely but reached the 10 s goal
in only 6/9 episodes.

The C4 failure was not repaired by changing the goal horizon. C5 identified
the post-conflict/pre-clear predictor OOD gap and added a time-indexed
reachable-occupancy certificate using the reviewed actor lower speed
0.12 m/s and maximum deceleration 2.4 m/s2. The learned predictor and hidden
policy label are not used in this bypass. C5 replayed all 330 C4 decisions,
advanced admissible motion by 0.5--1.0 s in every episode, and passed 8,748
reachable-bound checks. C6 validated this single factor on fresh identities.

C1, C2 and C4 are terminal and must not be resumed, retried or have their
seeds/budgets reused. C6 is also terminal and non-rerunnable.

## Claim boundary

The evidence supports only fresh segmented Gazebo closed-loop integration in
the parameterized single-actor crossing scope. It does not establish
continuous ROS runtime, general navigation safety, performance superiority,
deployment readiness or real-vehicle readiness. `formal_result=false`.

No real robot, CAN device, `/cmd_vel`, move_base or stable workspace
`/home/robot/robot_ws` was used or modified.

## Next safe entry

V3-04 must begin with a pure-offline learning-value and support audit using
frozen evidence. A learning policy may rank only candidates already accepted
by the C6 hard verifier. It must not weaken the verifier, remove certified
stop, use evaluator-only policy labels at runtime, or claim value from reused
evaluation outcomes. Any training budget and any fresh simulation evaluation
require explicit preregistration and separate bounded authorization.
