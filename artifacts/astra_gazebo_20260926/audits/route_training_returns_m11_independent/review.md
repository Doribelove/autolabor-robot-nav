# Independent M11 review

The repaired r02 pure reward/GAE component passes this bounded independent
review. This is not approval of a training run, physical-data provenance or
deployment. No training, ROS, simulator, optimizer, GPU or benchmark was used.
The reviewer did not edit the author component or its tests.

The author source under review is
`workspace/scripts/astra_gazebo/route_training_returns_m11.py`, SHA
`b584dbb2b16ded9931de7996a60c1392f02b935f1cf3834f62cddc1617f21fa6`.
37 author tests and 16 independent tests pass together in 0.05 seconds.

## Mathematical result

Reward uses actual integer source-time duration, signed route-arc progress,
absolute endpoint forward-speed difference, observed candidate-index switches
and a single resolved terminal outcome. It does not assume the nominal 250 ms
decision duration. Negative progress, held zero commands and action changes
through zero remain in the reward. The speed term is the endpoint difference;
it is not total variation or an integral of intermediate acceleration.

For interval i, the component computes gamma_i=gamma^(dt_i/ref) and
lambda_i=lambda^(dt_i/ref), delta_i=r_i+gamma_i*V_next−V_i, followed by
A_i=delta_i+gamma_i*lambda_i*A_next for an open contiguous segment. Independent
70-digit Decimal direct sums match variable-duration output. With lambda=1,
value targets telescope to the source-time discounted observed reward sum;
with lambda=0 they reduce to one-step TD. Discount parameters and reference
duration are caller-supplied, not silently frozen by this review.

PG eligibility is copied as a separate boolean. Changing every PG flag leaves
all advantages/value targets identical. A non-PG timeout reward remains in the
earlier eligible action's credit. Physically removing an intermediate non-PG
row is rejected as an unacknowledged source gap. M10 proposal labels/log q must
remain separate from the executed candidate index used by this reward.

Goal/contact/timeout are finite-horizon terminals and require next_value=0.
An explicit collection cut stops backward recursion but includes its supplied
next critic value in delta; rewards from a later segment do not leak across the
cut. Open tails, inconsistent adjacent critic values, duplicate decisions,
overlapping source intervals and reuse after a physical terminal are rejected.
Infrastructure failure is not accepted as a physical outcome. The API does not
invent an observation, next value or transition for a missing endpoint.

## Preserved defect and repair

The original source b8a84ae4…c1fa computed `(gamma*lambda)**exponent`. Its
declared valid domain includes gamma=lambda=1e−200 and exponent=0.1. Premature
product underflow returned first advantage=0 for a two-row trajectory whose
second reward is 1 and is non-PG. Independent Decimal reference is 1e−40.
This is an extreme supported-domain numerical defect, not an observed
training failure; ordinary proposed discount values were unaffected.

The root author preserved the original source and changed only the continuation
factor to `discount*(lambda**exponent)`, with the existing gamma discount.
The independent frozen-original replay retains 1 failure / 15 passes; repaired
r02 passes all 16 independent cases and its 37 author tests. Ordinary binary64
rounding/range limits still apply; this is not a proof of exact arithmetic for
every possible finite input.

During the initial audit snapshot, root had already applied the reported fix.
The expected-original SHA assertion failed closed before copying source. Thus
the first file named `tests_r0.log` actually exercised r02 and passed 16 tests;
it is not described as an original-source result. `replay_original_negative.log`
is the explicit replay of the authenticated preserved original source, and
`tests_final_r02.log` is the final current-source result. All are retained.

## Integration boundaries

This scalar API cannot prove supplied source timestamps, joined route/speed
measurements, completeness of control receipts or authenticity of next critic
values. The author's pending collector work explicitly owns those obligations.
In particular, a collection-cut boolean plus a finite next_value is only an
API precondition: future integration must bind the real next observation,
source stamp, behavior critic/checkpoint and its numeric value. Zero is a valid
critic output, so rejecting zero cannot establish such provenance.

The caller must authenticate the 22-second shared timeout and resolve simultaneous
contact/goal using the declared contact priority. The primitive itself has no
gate/deadline/physics evidence and cannot verify those decisions. The executed
action stream must be complete and ordered; an empty stream can legitimately
mean a held command, but this function cannot distinguish that from missing
recording. A direct zero/drop command's audit index must be defined explicitly
by the collector and must never be used as a fabricated valid backup action or
actor proposal label.

Infrastructure failure before a valid next observation requires a separate
durable episode/failure record. It must not be converted into a zero-reward
physical terminal or supplied with an invented bootstrap just to satisfy this
function. Valid preceding transitions and already observed costs remain in the
failure-inclusive ledger; what can be used for critic/PG fitting depends on
which next observations actually exist. No full learning backend exists here.

The M10 proposal component remains an author-reviewed candidate pending an
independent reviewer. This M11 review does not independently qualify the
reviewer's own M10 code or authorize its use. No N04/N05 bound source changed.
