# V6 PCT-SCOPE-RL Handoff

Date: 2026-08-08

V5-03-F1 is a frozen valid negative practical-effect result.  Its 6.0936%
failure-inclusive time reduction had two-sided 95% bootstrap interval
[1.7867%, 11.0069%], but missed the preregistered 8% minimum.  It therefore did
not authorize Gazebo or Actual TEB evaluation.  V5 formal seeds 930100--930159
must never be rerun or used for fitting.

V5-04-D1 attributed the shortfall to an unlearned narrow-corridor commitment:
full V5 and the analytic prior were both 0/15 there, while the same candidate
library's development-only privileged side teacher achieved 18/25.  The no-
history delta was only 0.0042 s and the no-gate delta exactly zero.  The V5
block mapping also confounded family and training identity; the aggregate
mixed-policy estimand remains valid, but per-seed/family attribution does not.

V6 changes exactly one architecture factor.  PCT (privileged counterfactual
trajectory ranking) supplies a training-only target candidate ID for partial-
occlusion and narrow-corridor states.  The target ranks the safe spatial
candidate opposite the simulator-truth hazard side, then its low-speed version,
then safe straight candidates and certified stop.  A 0.35-weight auxiliary
cross-entropy epoch follows each PPO update.  The V5 transition, reward,
actions, hard shield, analytic prior, asymmetric critic, recurrent sensor actor
and advantage gate remain byte-identical.

The V6 component gate passed on fresh audit seeds 951200--951399: partial
occlusion 87/100 goals, narrow corridor 63/100, zero contact and zero mask
violations.  Actor observations remain sensor-only and the actor-only checkpoint
contains no privileged modules.  A seed9599 1,536-step `/tmp` code smoke is
ineligible evidence and consumes no V6 budget.

Eligible V6 training is four exact V5 selected initializations, new RNG seeds
9501--9504, 120,000 steps each, total 480,000.  There were no retries,
extensions or extra factor rounds.  Validation used fresh seeds
952100--952259.  All four identities were positive; aggregate PCT / analytic /
frozen-V5 failure-inclusive means were 12.9828 / 14.5281 / 13.3859 s.  The
validation gate passed with zero contact and zero mask violations.

V6-02-F1 is the terminal fresh fast-population positive result.  Seeds
953100--953179 formed 80 Latin-assigned blocks and 320 episodes across analytic,
frozen V5, PCT V6 and PCT V6 no-history.  PCT / analytic goals were 60/80 and
49/80; means were 13.115625 and 14.31875 s.  The primary reduction was
8.402444% with two-sided 97.5% interval [3.484003%, 13.713459%].  All primary
gates passed, with zero contact, zero mask violation and minimum PCT physical
clearance 0.154052 m.  The secondary reduction versus frozen V5 was 3.161052%
with two-sided 95% interval [0.493580%, 6.576508%].  V6 and V6 no-history were
exactly tied; no temporal-history-benefit claim is permitted.  Formal
assessment SHA is `08a0ab06343cad51ee206ba502d2a115c6fd707d58620f9bb4f519a71ec95c1b`.

Gazebo transfer is terminal negative.  The first V6-03 preflight was stopped
before ROS startup by the network namespace and preserved.  R1 then completed
three fresh episodes: all methods reached the goal and had zero contact, but
V6/analytic conservative clearance was -0.0774/-0.0809 m versus Actual TEB
+0.2452 m.  R2 aligned scan-surface inflation by the exact geometry identity
1.15-0.36-0.46=0.33 m; clearance worsened to -0.3366/-0.2581 m.  V6-04 added a
LaserScan+Odometry-only directional reachable-occupancy tracker; its tracker
gate passed, but V6 timed out and V6/analytic clearance was -0.9902/-1.0339 m.
Actual TEB remained positive in all three preflights.  Consequently no Gazebo
qualification or formal seeds were accessed, and there is no Gazebo, Actual-TEB,
deployment or safety-positive claim for V6.  Do not resume these stages or tune
from their preflight identities.

V6-05-P1 paper export is complete.  It contains six editable SVG/PDF figures,
method/effect/family/episode/Gazebo CSV files, claims YAML, primary-source
literature links and evidence manifests.  The standalone V6 supplement is 20
A4 pages with SHA
`6e44188b3b56ceeb883baf5d08c74b5798451864dd9f29dd3959c1870e2abd61`.
The combined 69-page thesis draft is
`artifacts/v6/paper/v6_05_p1_thesis_export/FAM_TEB_V6_PCT_RL_THESIS_DRAFT.pdf`
with SHA `c1a23074f1524cf967b89f674017f03529e4b54ab171abc5cabfa5749229d829`.
Independent R2 paper review passed 15/15 checks; review SHA is
`ff4b6423d8fb610fe29486ab059c86a1e86f284055c8b520cf9e38cf4e470b41`.

The later pure-offline V6-06-P1-R1 thesis transfer closure is now the current
paper handoff.  It converts the former appended-supplement export into a
self-contained 41-page integrated V3+V6 technical draft, carries 11 editable
figures, five V3 three-line tables, V3/V5/V6 machine-readable data, combined
claims, an explicitly empty real-vehicle chapter, and a ready-to-use Codex
prompt for migrating an external school-template draft.  Integrated PDF SHA is
`a2fcacc82ee8f7fc290415661b34bd5324bbc157158d54a0780705a7737eadf1`;
independent review SHA is
`00e4705f783632723018c6920f8e1b68dc6096a08cc28018d72e286e783d1f1b`.
Read `CURRENT_V6_06_THESIS_TRANSFER_HANDOFF.md` and use the R1 package, not the
historical 69-page concatenation, for another-computer thesis writing.

Machine-readability note: the historical, release-bound
`v5_00_architecture_contract.yaml` has SHA
`529ed6fd6776e6b259abc5398352ea0a047948af02dc12331da5848dc5c1ca6d` and
contains a legacy indentation error at its post-action invariant.  It is kept
byte-exact because the V5 training release binds that hash; no training,
runtime, formal-assessment or paper-export path deserializes it.  All eight
successor/repair contracts, including the complete V6 contract, parse as YAML.
Do not silently edit the historical file or rewrite its frozen release hash; a
future clean-up must be a separately named compatibility artifact.

The defensible thesis conclusion is scoped: privileged counterfactual rank
distillation produced a statistically and practically positive improvement in
the frozen four-family sensor-level fast population, especially partial
occlusion.  It did not solve narrow corridor yield and did not transfer through
the current Gazebo scan shield.  Further research requires a new closed-loop
interaction-consistent perception/verifier design and new authority.  Real
robot execution and online TEB writes remain forbidden.
