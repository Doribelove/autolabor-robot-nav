# V3-05 Formal Evaluation Handoff

Date: 2026-07-24

## Terminal status

V3-05 is terminally complete. The valid formal result is the repaired
V3-05-B0 stage; it is a complete negative performance result, not an
infrastructure failure.

V3-05-B0 used fresh seeds 6451--6480, three evaluator-only reactive-actor
policies, three frozen methods and one attempt per identity. All 270/270
authorized units completed without retry or terminal failure. The exact
release SHA is
`0cf76ee7115968f60df8da9d11807564c3f2e6f3a31fab3fd29b3331acbcb1cd`.

The deterministic formal assessment has SHA
`3754b6cce7613f4aeb708fb4e708515f6eeaca87f230c08c1e028700139b568a`;
the independent result review has SHA
`2bd0d481b6a8019b5bf39969644766821dbc1de8e73775b9eb582da00a71839a`.
Both cold-start `--check` replays pass. The terminal freeze file has SHA
`dfd131abf875cf65ad2477fe926e2ddd0a02a38f92ac1388dd6bd1ebc3dfb117`.

## Formal result

The primary comparison is the frozen analytic V3 planner against the
fixed-1.20-m/s shielded candidate policy, using 30 geometry blocks as the
statistical units:

- V3 restricted goal-time mean: 8.9861 s;
- fixed-1.20 restricted goal-time mean: 9.0944 s;
- paired relative reduction: 1.2007%;
- one-sided 95% bootstrap lower bound: 0.9647%;
- frozen point/lower gates: 5% / 2%;
- V3 goal success: 84/90 = 93.33%, below the frozen 95% gate;
- fixed-1.20 goal success: 82/90 = 91.11%;
- positive geometry-block fraction: 25/30 = 83.33%.

The point effect, lower effect and absolute V3 goal-success gates fail.
The exact conclusion is `formal_result=true` and
`performance_improvement_demonstrated=false`.

The fixed-0.80 shielded method is only a descriptive low-speed candidate
reference. Its descriptive relative reduction is 9.9992%; it is not an
inferential baseline and is not actual TEB.

## Safety and integrity result

The shared safety/integrity scope passes:

- 270 episodes and 9,563 decisions replayed;
- selected candidate accepted rate: 100%;
- certified stop availability: 100%;
- exact core replay: 100%;
- yield-before-clear and recovery-after-clear rates: 100%;
- collision/conservative-clearance violation count: 0;
- minimum conservative clearance: 0.211004 m;
- P99/max planner latency: 18.006/21.235 ms;
- no nonfinite metric and no retry.

This supports only the frozen parameterized single-actor crossing simulation
scope. It does not establish general navigation safety.

## Preserved A0 infrastructure failure

V3-05-A0 is a separate terminal nonresumable stage. It completed 205 units,
then consumed unit 206 before semantic execution when Gazebo port 35600
collided inside the Linux ephemeral range. Its remaining 64 units were
forfeited and none of its partial outcomes are used for inference.

V3-05-R1 repaired only master-port allocation. B0 used ports 12050--25538;
unit 206 completed on port 22300. A0 must not be resumed or retried, and B0
must not be rerun or have its seeds, identities or outcomes reused for
selection.

## Claim boundary

The defensible claim is:

> In the frozen fresh-seed parameterized-crossing matrix, V3 preserved all
> audited hard-safety and replay properties and was noninferior in task
> success to the fixed-1.20 shielded candidate policy, but it did not meet
> the preregistered absolute-success or efficiency-improvement gates.

Do not claim actual-TEB superiority, learned-policy benefit, generalization,
deployment readiness or real-vehicle readiness. No real robot, CAN device,
`/cmd_vel`, move_base or stable workspace `/home/robot/robot_ws` was used or
modified.

## Next safe entry

The next safe entry is a pure-offline, explicitly post-hoc V3-06 attribution
over the frozen B0 evidence. It may diagnose the six V3 timeouts and the
limited efficiency effect, but it must not turn reused outcomes into a new
confirmatory result. Any planner change requires a single-factor design,
fresh development identities and a separately preregistered held-out
evaluation before a new performance claim.
