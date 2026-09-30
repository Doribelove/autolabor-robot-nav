# V3-13 Multi-Actor Journal-Strength Development Handoff

Date: 2026-07-25

## User objective and order

The active objective is a stronger journal paper:

1. multi-scene / multi-actor V3 with an actual
   `teb_local_planner/TebLocalPlannerROS` fair baseline;
2. perception latency, noise, dropout and OOD degradation;
3. policy/ranker training only after support, propensity and analytic-regret
   gates pass;
4. real vehicle last, under a separate on-site authorization.

The stable real-robot workspace `/home/robot/robot_ws` has not been modified.
No hardware, CAN/serial device, real robot or policy/RL training was used.

## V3-13-D0 offline foundation

The new namespace preserves all V3-11 terminal files and identities. D0
implements a four-action Ackermann bank (left/right/straight/certified
stop), multi-actor candidate-conditioned tubes, every-actor/every-time hard
verification, static/corridor verification, analytic ranking, hidden-label
firewalls and a positive-propensity clone schema.

- tests: 9/9 pass;
- evidence manifest SHA:
  `761958602cd18bdd1978f6ec340e5d4e0ca2827506ed8c954cc210ff04ba2816`;
- independent review SHA:
  `7307ca5deb644ad4ea98564a49beae754ae13199cf27b9ecb762fe2ebf3f7658`.

D0 is a code/fixture claim only, not a predictor or performance result.

## V3-13-C1 multi-actor cloned interventions

One non-evidence preflight seed 7299 passed with two actors, 141 samples over
7.0 s, 0.341 m minimum conservative clearance and no contact. Its result is
explicitly excluded from fitting and performance evidence.

The unique C1 release SHA is
`e0e411679329d4a798f52ecbfdbc4a57a8d72d78040cf5e76717d7094b8210f0`;
the readiness review SHA is
`b6609cf76a12f122d86bb28646ef3beca8006267cb83929c7c9b961b604b506e`.

Fresh seeds 7601--7616 covered:

- four independent scene families;
- four cautious/hesitant actor-policy pairs;
- four cloned actions per block;
- 16 complete blocks / 64 episodes;
- behavior propensity 0.25 for every action;
- attempt=1 only, no retry, resume, replacement or exclusion.

All 64 completed. The evidence contains 9,045 finite trace samples.
Maximum within-block gate ranges were 0.01600 m ego x, 0.00448 m actor
coordinate and 0.04480 m/s actor speed. Every actor/block showed a positive
candidate-conditioned endpoint response spread; mean spread was 0.6004 m.

- C1 assessment SHA:
  `41209bba6bdda908f5cbe8bf11096d8522979992d33782e0fbab95c71f5d7ad6`;
- evidence freeze SHA:
  `c427437d0fd0795efa55374ba19f666d2447d934adb4696e9c26683f9d08459d`;
- independent result review SHA:
  `946d1bb4f959d64a308d1de6827a1a13fe32aca7e4943f957c9999fb49d69a1e`.

C1 authorizes predictor fitting only. It does not authorize policy/ranker
training or an actual-TEB comparison.

## V3-13-C2 predictor fit

The original constant-speed D0 fixture model was rejected during model
development because leave-one-scene maximum trajectory error reached about
1.64 m. C2 instead fits separate observable-mode/candidate ridge heads using
time, gate actor speed/coordinate and conflict-point x geometry. Hidden actor
policy labels are not features.

Internal leave-one-scene-family-out results:

- simultaneous whole-trajectory coverage: 92.1875%;
- mean trajectory RMSE: 0.16961 m;
- maximum trajectory RMSE: 0.61017 m;
- per-family coverage: 87.5%, 90.625%, 96.875%, 93.75%;
- terminal full tube width at 7 s: 1.72 m.

All preregistered internal gates passed, but this was explicitly not treated
as fresh qualification.

- model SHA:
  `53a23d292118021d2d70f46f3083acb5ab8632a034b4ba86a90f02362abe6c95`;
- internal CV SHA:
  `630870fb7209ddcbdfcae8ccd9c45090ad656a9a58ce6107cf51d4b6e0b39472`;
- independent review SHA:
  `03160d4af7995281bc4f20cee1c26b605ce7de9c8a6b308e414274aae901df1e`.

This was supervised system identification with zero policy/ranker training
steps and zero new simulation environment steps.

## V3-13-C3 fresh predictor qualification

The unique release SHA is
`2a36b7ba918dd37ad92a1e218a9a25942cbfbc431712f20dbbe1acaa99eb8cfa`;
the readiness review SHA is
`421f4ab85a776b136cb4ce5b5fa7475bae4bc4bbe94fd02c5806cc1ad44c2a70`.

Fresh seeds 7201--7216 repeated all 16 scene/policy blocks and all four
positive-propensity actions. All 64 units completed at attempt 1 with no
failure or retry. On 128 fresh actor trajectories:

- simultaneous whole-trajectory coverage: 96.875%;
- mean trajectory RMSE: 0.14063 m;
- maximum trajectory RMSE: 0.54079 m;
- per-scene coverage: mixed oncoming 87.5%; the other three scenes 100%;
- maximum paired gate ranges: 0.0000041 m ego x, 0.00384 m actor coordinate,
  0.03627 m/s actor speed.

All frozen gates passed.

- fresh assessment SHA:
  `95244805202bd3540ce1ba75b3715f1df4f1eecb67047cc02349cf4f0ec9847a`;
- fresh evidence freeze SHA:
  `e7754c16cbfe7fad8119818e6804f00999b62f98166e827459f2055533527d69`;
- independent result review SHA:
  `48ebeaa677f69b4415291ce597e02eb5a9fb21eebba5d16735b1852747ffbe49`.

## Current blocker and next safe entry

Pure-offline replay of one fresh gate state per block produced 8 straight,
4 spatial-left and 4 certified-stop initial decisions. The four stops are
the cautious/hesitant high-interaction cases in the occluded and mixed
oncoming families.

Blindly replanning after the stop is not valid. Within roughly one second
the reactive actors return to observable `assertive` speed, while the
qualified C2 model has only cautious/hesitant heads. The existing C1/C3
clones identify alternative actions at the initial common gate, but not
alternative actions from a matched post-stop state. This is the same
structural distinction between one-step support and sequential support that
must be closed before later offline RL.

The next safe stage is a new bounded sequential cloned-intervention study:

1. use fresh seeds and matched common stops;
2. branch to left/right/straight/stop after frozen wait depths;
3. retain complete positive propensity and all unsafe outcomes;
4. fit and freshly qualify an assertive/recovery predictor head;
5. audit whether a hard-stop then recovery policy can reach the goal safely;
6. only then release the actual-TEB multi-actor qualification.

Do not run the actual-TEB matrix yet, reinterpret C3 as sequential support,
fit hidden evaluator labels, train a policy/RL agent, start perception
claims, access hardware, or modify `/home/robot/robot_ws`.

## S1--S5 sequential recovery closure

The former C3 blocker was investigated without reusing C3 identities.

- S1 collected 32/32 fresh sequential clones but failed its frozen matching
  gate because branch time was relative to a variable handoff and actor speed
  used a phase-sensitive instantaneous derivative. The failure is terminally
  frozen; it was not used to fit a predictor.
- S2 changed only synchronization/observation infrastructure: absolute
  8.5/12.5 s simulation barriers and a causal 0.50 s coordinate-slope speed
  estimate. A four-unit non-evidence preflight and the 32-unit fresh evidence
  matrix passed. The evidence-freeze SHA is
  `2354037249885875880599dd6b735bcfb706a546fb1766513c1739f9ceda51af`;
  independent review SHA is
  `0678f5c1fa8bced65dcbeac5c10d52c871ae260ade2609463b7df6e153b7824f`.
- S3 fitted only the missing assertive recovery heads, retaining the C2
  cautious/hesitant heads. Model SHA is
  `94449b8a1a645754c88c1781a5b7ac8fd869adaa5b79c69d81aa4e6abc33b621`.
- S4 used 32 fresh units / 64 actor trajectories. Simultaneous whole-trajectory
  coverage was 93.75%, mean RMSE 0.15326 m and maximum RMSE 0.63522 m.
  Evidence-freeze SHA is
  `21691c4e7eca68bf35534600792989f5358a1b77fd2e6f8cdd22a7e4df0eb8a9`;
  independent review SHA is
  `6d4932730e8c1c4b25b79cb2d8310ef71d9eb57065fa58cef1ff1a32f699ce43`.
- S5 replayed all four high-risk strata through initial/early/late states.
  The frozen data supported stop at the initial and 8.5 s states and straight
  progress at 12.5 s. Audit SHA is
  `3d3d7fd2cd00ee846a75cd3ead2b2d75671094d204f6d608ed148f0755901381`;
  independent review SHA is
  `b4ad0c67e09e6652dfad6c693d2f35ac9c687573dbd9dab08b60e7addc1fc891`.

These stages used supervised system identification only. Policy/ranker/RL
training steps remain zero.

## Actual-TEB multi-actor qualification attempts

Q1 froze the exact historical actual-TEB identity:

- `teb_local_planner/TebLocalPlannerROS` 0.8.4;
- source SHA
  `5ada4149b1827284403c1664ce61dd14188bb30f8073417f3da1b378f00df8c2`;
- built plugin SHA
  `7d17474e7901821d7f69bf10cdb909e4445f9010e32ee045452ad42c6ba1883e`;
- `fixed_teb.yaml` SHA
  `9e3125eec7f1f9040356f245dae9620d0b866ac94482ac29fd2d600838b14c26`;
- the same M2, 1.2 m/s cap, `/cmd_vel_navigation`, handoff gate, goal,
  timeout and Gazebo-oracle dynamic tracks as V3.

The intended qualification remains four scene families x four two-actor
policy pairs x two methods = 32 paired episodes on seeds 7301--7316.
Those qualification seeds remain unconsumed.

The first Q1 non-evidence preflight never reached ROS/Gazebo because the child
inherited a foreign `ROS_PACKAGE_PATH` and resolved the wrong `m2_gazebo`.
Seed7300 was consumed and may not be reused. Terminal freeze SHA is
`c11935454f2f369e4f44cbc552d30e32983453ef271fea3d191114bfe78eeaa1`.

Q1-R1 changed only the child-process overlay. It resolved the correct package
and reached Gazebo, but fresh seed7330 failed closed at the 12.5 s recovery
decision with `NO_SAFE_CANDIDATE_OR_CERTIFIED_STOP`; the actual-TEB paired unit
was not started. Its terminal freeze SHA is
`7cc239ec08104caaddd113188da070c88a86ba8aab24080da84e5ffaa9c2c512`.

Q1-R2 used fresh seed7331 for one read-only diagnostic V3 episode. It recorded
three decisions and no action override:

- 7.384 s: certified stop;
- 8.534 s: certified stop;
- 12.544 s: all four candidates rejected by
  `DYNAMIC_OCCUPANCY_INTERSECTION`;
- certified-stop predicted clearance was exactly about -1.02 m because the
  oncoming actor remained on a future centerline collision course.

Diagnostic analysis SHA is
`d651b8af11641272d064bc217ea5812c8325fce76830a7e1922c8f4644ceecce`;
independent review SHA is
`e435f82ecf701dc86331ec1ba05aa8ed33dfc7b228ddeaca1479f121da984717`.

## Q1-R3 midpoint single-factor result

An offline design scan found a narrow predicted progress window at
10.25--10.75 s and fixed 10.5 s prospectively. R3 then consumed fresh
seeds7332--7335 in four high-risk strata with complete four-action support:
16/16 units completed, attempt=1, no retry.

Collection integrity passed:

- complete support rate 1.0;
- maximum branch clock error 0.008 s;
- maximum paired ego-x range 0.01600 m;
- maximum actor-coordinate range 0.01086 m;
- maximum actor-speed range 0.00434 m/s.

The scientific gates failed:

- S3 simultaneous coverage at 10.5 s: 65.625% (required 90%);
- mean/max trajectory RMSE: 0.52843/1.72831 m (required <=0.25/0.65 m);
- both occluded strata still selected certified stop;
- the two crossing/oncoming strata selected spatial right with analytic
  clearances +0.043/+0.048 m, but cloned truth clearances were
  -0.322/-0.064 m.

Assessment SHA is
`701206c23e264474a2d84b1aa9a54c1debcb623d1596e5efd3722cbea96d788e`;
evidence-freeze SHA is
`2502a41fcc1b935674b0dc71e46e30a18cf56a4fa0726d531bf6b13a76ed3d03`;
independent review SHA is
`f6ea60d979a8c1e993592ea75449f9e9816a59797701669c08756615b7acdc0e`.

## Current state and next safe entry

`actual_TEB_multiactor_qualification_pass=false`. The 32-unit Q1 matrix is not
released and must not be run. No actual-TEB multi-actor performance claim is
available.

The result rules out a global 10.5 s timing-only repair. The next safe entry is
a new pure-offline candidate-bank geometry closure. Preliminary design-only
replay shows that the current 0.25 rad spatial candidate reaches only about
1.05 m lateral offset and cannot clear the calibrated oncoming occupancy.
A stronger emergency lateral candidate becomes jointly positive in the frozen
R2 snapshot only near 1.0 m/s² acceleration and 0.34 rad steering peak, while
remaining inside the 2.0 m corridor. This is not yet a released candidate or
evidence.

Before another simulator unit:

1. define and statically verify exactly one Ackermann-feasible recovery-only
   evasive candidate within the existing M2 actuator bounds;
2. do one fresh non-evidence critical-scene clone preflight;
3. if and only if it is truly nonnegative, collect complete positive-propensity
   sequential support, fit a candidate-specific actor-response head and run a
   separate fresh qualification;
4. only after that gate may a new actual-TEB paired preflight be released.

Do not shrink the calibrated tube, lower the 1.02/1.15 m safety clearances,
exclude the oncoming strata, reuse seeds 7300/7330--7335, run seeds7301--7316,
train a policy/RL agent, start perception claims, access hardware, or modify
`/home/robot/robot_ws`.

## Superseding Q1-R4--R19-R6 closure (2026-07-26)

This section supersedes the former “next safe entry” above. The recovery
geometry, candidate-specific prediction, planner integration and one actual
TEB/V3 pair were subsequently completed, but the fresh five-action support
program did not close its final execution-integrity gate. Predictor fitting,
the formal actual-TEB matrix, perception degradation and training therefore
remain closed.

### Recovery candidate, predictor and planner chain

- R4--R13 established a progress-relative Ackermann-feasible recovery
  candidate and complete positive-propensity development support. The R13
  candidate-specific predictor retained every prior head byte-equivalent.
- The first R14 qualification result was invalid only because its assessor
  expected a predecessor-stage label. R14-R1 used fresh identities and passed;
  assessment SHA
  `3f1125...` and independent result review SHA `eaf75e...`.
- R15 planner integration passed; audit SHA `538a978...`, independent review
  SHA `e2268...`.
- R16 ran one fresh actual-TEB/V3 paired preflight. Actual TEB reached the
  common goal but had conservative dynamic clearance `-0.8965 m`. V3 remained
  dynamically safe at `+0.06883 m` but its r2 candidate missed/overshot the
  common goal. Assessment SHA `120c1b...`, review SHA `709d81...`. This is a
  valid negative preflight, not an actual-TEB performance result.
- R16-D1 attributed the V3 failure to an r2 endpoint/common-goal mismatch and
  missing fail-closed completion. R17 then passed a pure-offline goal-aligned
  r3 geometry and terminal guard closure. R17 contract SHA is
  `3dd9299cbbf39c63442a108c082e3830784084f4e40f9eff9560de7e43f2603f`.

### Goal-aligned runtime and support attempts

- R18 fresh seed7667 failed before producing a trace because runtime
  instrumentation recursively called the monkeypatched executor. Failure
  freeze SHA
  `f80ef3f0538354f8fbd8fdcb549d5948c5e18099ad9cc04dd240ea0dd2fa220c`;
  seed7667 is nonreusable.
- R18-R1 introduced a nonrecursive adapter and passed on fresh seed7668:
  dynamic/static clearance `+0.107229/+0.254805 m`, final goal distance
  `0.109887 m`, guard count 1. Assessment SHA
  `96d4e8e9a7ca904985065ab5366c9b06c72449686da5df090d0a4b7670b66fd7`;
  review SHA
  `4779bca90e9b88453b1e113d1e68860f25f86c49c2f52dd860ee76bd6bdd7be1`.
- R19 used fresh seeds7669--7676 and completed 40/40 five-action units with
  complete support, propensity 0.20 and no contacts. It failed only because
  the r3 terminal-guard execution rate was `6/8=0.75`. R19-D1 proved the two
  misses were exact-horizon samples skipped by the inherited loop ordering.
- R19-R1 changed only the boundary ordering and passed fresh seed7677:
  dynamic/static clearance `+0.160798/+0.255631 m`, goal distance
  `0.046293 m`, guard count 1 and exact boundary evaluation/trigger.
- R19-R2 used fresh seeds7678--7685 and completed 40/40. R3 was safe in all
  eight rows with minimum dynamic clearance `+0.091937 m`, no contact and
  guard/boundary rates 1.0. The full stage nevertheless failed because two
  cloned blocks exceeded the frozen prefix-matching gates: ego range
  `0.048003 m` and actor-coordinate range `0.085763 m`. Assessment SHA
  `ef50fedcfdd12ff5ec0d64dd863c7a78c37dc05216c711446a257f71eb8fa284`;
  review SHA
  `c362f03431515b03e6cb1cafd93812317ede0abd285660495261d4b1f66969b3`.
- R19-R2-D1 attributed that mismatch to the 20 Hz sampled warmup gate:
  one 0.05 s sample changed the stopped ego pose, then the ego-x-conditioned
  reactive-actor activation amplified the prefix difference. Diagnosis SHA
  `79be9b96d4252981677336958185cefcdea1790d7b14b8623a55ef8f65ca2940`;
  review SHA
  `203d167d71759f224baa037daed4d574df639418feac240cd51c3846dbdb635d`.
- R19-R3 changed only the common-prefix command sampling rate from 20 to
  100 Hz. Fresh seed7686 completed all five actions with zero ego range and
  actor-coordinate range `0.008193 m`; all gates passed. Assessment SHA
  `23c8568493d52e6dce138536751538a4609606e40b852771d32a45fb92bc4738`;
  review SHA
  `9f400f27138c8d087e428d39edd968dc72b355eae309963fe422e9d432b3c8be`.
- R19-R4 used fresh seeds7687--7694 and completed 40/40 with zero ego range,
  actor-coordinate range `0.016353 m`, no contacts and r3 minimum dynamic
  clearance `+0.087253 m`. It failed because one r3 boundary was 1.16 mm
  outside the 0.12 m terminal tolerance, so guard/boundary trigger rates were
  `7/8=0.875`. Assessment SHA
  `5e1802ab7083772abc57d6861fbded1d548a518e5b6b0ee84c430b62f5630c41`;
  review SHA
  `7b62d3849383556c4e8b3fcb624eccda6ea6af582db3b6f907e18c772538ccbd`.
- R19-R4-D1 replayed all eight boundaries and preregistered an earlier-stop
  r3 tolerance change `0.12 -> 0.15 m`. Diagnosis SHA
  `cfd5625f1cb7a3115a9cebcdb51f6e39b6b8dc85fbbdd2b0ce892c90db456b31`;
  review SHA
  `e1bcf68ad8f6b3066249969a6bc323b9c947e3899603479662c7bfdb8ca2cfe3`.
- R19-R5 seed7695 failed pre-semantically because the one-unit runner bound
  behavior propensity 1.0 while the inherited runtime released only 0.20.
  The identity is frozen and nonreusable. R19-R5-R1 changed that binding only
  and passed fresh seed7696: dynamic/static clearance
  `+1.874740/+0.057405 m`, guard 1, boundary trigger 1. Assessment SHA
  `593f11983c1389588f225a414cf170393154bc524b2e0d88b4f808495654ee17`;
  review SHA
  `ab90d4900db7b4c2f89d8be655c58810a551215878d65cceeb74c3419a07deef`.
- R19-R6 was the final fresh support attempt. It released seeds7697--7704,
  but the reused one-unit runner hardcoded r3 for every scheduled action.
  Sequence1 and sequence2 were consumed before the mismatch was detected;
  both runtime reports said r3 while the schedule said spatial left/right.
  Execution was immediately stopped. Scientific units are 0, remaining 38
  units are forfeited, and every R19-R6 seed is nonreusable. Terminal failure
  SHA
  `776a28e76b2421856b7bd3b5ebe52ca80dfae3a03b69a68e4d96cd2154e9ad29`;
  independent review SHA
  `b683c0c0e8fc5a3deb296b96eb38a017b875cdca69548ec290bdc00b6eafe483`.

### Current terminal state

`r3_complete_positive_propensity_support_pass=false`.
`predictor_fit_authorized=false`.
`actual_TEB_multiactor_qualification_pass=false`.
`perception_execution_authorized=false`.
`training_authorized=false`.

Do not reuse or resume R18/R19 identities, repair-and-rerun R19-R6, fit an r3
head from any failed matrix, run reserved actual-TEB seeds7301--7316, start
perception degradation, train a policy/RL agent, access hardware, or modify
`/home/robot/robot_ws`. The next safe entry is pure-offline execution-integrity
design: add a fail-closed assertion that every scheduled candidate ID equals
the runtime report ID before any future multi-action release. A new simulator
stage would require explicit fresh budget and a separate authorization.
