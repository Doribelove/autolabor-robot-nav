# M10: explicit exploration and proposal-space likelihood

This is a new offline candidate component. The M09 draft, its initialization
failure, frozen M08 runtime and N04/N05 experiments remain unchanged. This is
not a trainer, environment adapter, checkpoint, execution release or learned
result. Training steps and simulation episodes are zero. The future training
stage must separately preregister its complete objective, budgets, initialization
seeds, deployment rule and admission checks. No seed is allocated here.

## Distribution and identities

Use the original eight candidates, original analytic prior, original actor
forward/normalization/residual scale and **true backup mask** M. No N05 moving
preview eligibility mask is introduced. The mask must come from the actual M08
backup verifier, not an actor prediction or a fabricated always-valid stop.
M10 receives float32 forward outputs. It does not call or change old
`SensorActor.choose_actions`; the new sampler is explicitly a new policy.

Let p be the lowest-index maximum of the masked original prior. Let

    G_theta(z) = z, if z != p and residual[z] - residual[p] >= 0.10
                 p, otherwise.

The residual subtraction and threshold comparison retain the old float32
semantics. `actual_actor_probe.py` exercises the real unchanged method at 96
controlled forward-output cases, including the threshold equality and float32
subtraction edges. Gate identity is checked only on valid actions.

For this candidate, freeze epsilon = 0.20, without adaptation or decay. This is
an interpretable one-in-five branch design, not a value fitted to evidence or a
claim of optimal exploration. Sample two independent uniform draws from the
future bound RNG stream: B ~ Bernoulli(epsilon), z ~ pi_theta(.|observation,M).

    a = z if B is exploration, else G_theta(z)
    q_theta(a) = epsilon*pi_theta(a)
                 + (1-epsilon)*sum_{z:G_theta(z)=a} pi_theta(z).

The component computes pi with a float64 stable softmax of the original
float32 logits and samples exactly that categorical using an injected uniform.
The latent sampler and likelihood must share this definition; do not sample
with a different float32 distribution and describe the result as an exact
implementation of this q. Masked indices have zero mass. Every valid pi must
be numerically positive; underflow is an explicit invalid-proposal failure,
never repaired by an unregistered floor. All-false masks raise before sampling.

At the fresh zero residual head, G(z)=p for all valid z. Hence
q(p)=0.8+0.2*pi(p), and q(a)=0.2*pi(a)>0 for every valid alternative. This is
executable proposal exploration only when another backup-valid candidate
exists. Epsilon is the branch probability, **not** the probability of a
nonanalytic action. A single-valid-action mask cannot explore alternatives.
The actual fresh-actor synthetic observation produces nonanalytic mass
0.13030317752818263; this is a mathematical fixture, not live feasibility.

Training identity: `astra_route_tracks_pushforward_explore_m10_v1`.
Deployment identity: `astra_route_tracks_deterministic_residual_gate_m10_v1`:
no exploration, deterministic argmax of original logits, then the original
0.10 residual gate. This deterministic deployment is not the stochastic
epsilon=0 pushforward. A future checkpoint must bind both identities, epsilon,
gate/numerical definitions, original actor-source hash, true candidate semantics,
observation schema, training provenance and deployment rule. Existing M08
checkpoint format/namespace must not silently admit an M10 trained artifact;
an explicit future versioned loader/adapter is still required.

## Probability, objective and gradients

Log `log q_old(a)` at collection. On the same bound observation, prior and true
mask, recompute the new actor forward, **current** gate map and q_new. PPO ratio
is q_new(a)/q_old(a), not pi_new(a)/pi_old(a), not the sampled branch's joint
probability, not the probability of the latent label treated as a proposal,
and never the probability of a relabeled executed action. `update_likelihood`
validates the observation/candidate hashes and unchanged prior/mask; caller
still must authenticate those hashes against the actual observation bytes.
All q support remains common as long as the nonempty mask and finite positive
pi remain valid. Checkpoint identifiers are provenance bindings, not proof of
the caller's tensor computation.

Conditional on a fixed gate partition, define
w_aj=epsilon*[j=a]+(1-epsilon)*[G(j)=a]. Then

    d log q(a) / d logits[j] = pi(j)*(w_aj-q(a))/q(a).

This is the marginalized categorical score, a.e. in parameters. A future
autograd implementation must differentiate through logits/softmax and sum the
mapped mass; gate membership is discrete and detached. No straight-through
derivative through the threshold is justified. At a gate crossing q can jump.
We recompute the actual new q for the ratio and record changed mapping indices
and exact old-to-new marginal KL; we do not freeze the old map to hide the jump.
There is no global smoothness or PPO monotonic-improvement guarantee. Optimizer,
clipping, KL stopping and entropy coefficients remain future stage contract
items. If entropy regularization is selected, its proposal-space quantity is
H(q); H(pi) is a different objective and must not be substituted silently.

At zero head with fixed gate, expected proposal reward is
0.8*R(p)+0.2*sum_z pi(z)*R(z). Thus the local likelihood gradient can distinguish
executable alternatives; it is epsilon times the categorical reward gradient,
unlike the zero-head collapsed original gate. This does not guarantee eventual
gate crossing, useful exploration, safe live control or learning success.

Marginalization is valid only if downstream control, transition and reward
depend on the proposal a (plus the fixed observation/environment), not on
hidden B or z after conditioning on a. Branch and latent are audit-only. A
future reward that explicitly scores latent intervention or exploration branch
would require a different joint objective and cannot reuse this q claim.

Training maximizes returns under stochastic q; deployment uses a different
deterministic policy and changes state visitation. This target mismatch is
intentional and must be reported and tested on preregistered validation across
all initializations. Proposal ratios do not by themselves give an unbiased
long-horizon deterministic-deployment evaluation. No deployment benefit is
claimed or used to select a winner here.

## Timing, fallback and failure-inclusive credit

`Context` binds decision, observation, candidate semantics and behavior
checkpoint, plus actual source/availability times. Available observation must
precede the decision; scan/odom sources must be real and no more than 200 ms
old. The existing causal six-real-scan M08 input contract remains an external
prerequisite; a hash or this low-level output component does not prove it.

`sample` requires actual actor start/finish on a monotonic clock and at most
20 ms actor duration, with at most 50 ms since the whole transaction started.
`finalize` separately checks the final source and steady times, 50 ms total
budget, clock regressions and committed scan/odom freshness (200 ms). Time
limits are not enlarged. These are receipt guards; this pure module cannot
interrupt a worker, prove supplied timestamps or replace the actual final
prepublish M08 checks. A future backend must provide authenticated receipts.

No valid proposal, invalid actor output, empty mask, source expiry, clock
failure and deadline-dropped proposals receive **no policy-gradient row**.
Caller records the failure and uses the inherited timely analytic fallback
where applicable; complete transaction/source failure keeps the original zero
output rule. Missing/invalid inputs must never be retried under a fresh time
stamp or admitted by labeling a fallback as a sampled action.

A valid, timely submitted proposal may be handled by the existing deterministic
execution shield. This is an environment response: retain original a/log q for
PG and record the executed action separately, even if the shield selects another
action or zero. If the proposal was discarded for a deadline/invalid input,
there is no PG regardless of what fallback later executes. Receipt replay
rejects mutated latent/proposal labels or forged eligibility fields.

Every zero/drop/failure transition remains in episode outcomes, rewards,
critic targets and earlier eligible actions' credit. PG ineligibility does not
cut GAE, erase costs or remove the failure from evaluation. The proposed M11
returns component owns this sequencing. A malformed observation before any
valid Context or an infrastructure break still needs an external durable
failure receipt; `finalize` cannot invent a physical transition. Suppressing PG
on timing/invalid rows follows the declared engineering objective; it is not
a proof that the resulting masked estimator is unbiased for a policy whose
availability itself depends on parameters or sampled action. This limitation
must survive future training reports.

## API and audit fields

`ActorOutput`, `Context`, `distribution`, `sample`, `finalize`,
`update_likelihood`, `deterministic_deployment` are pure and import no NumPy,
Torch, ROS, simulator or checkpoint loader. The component owns no RNG state.
The future caller supplies and binds two independent draws/RNG state; merely
passing two numbers does not prove their independence.

The immutable proposal record contains original prior/mask/logits/residual,
analytic action, full gate map, pi and q, branch and both uniform draws, latent
z/log pi(z), G(z), proposal a/log q(a), observation and checkpoint hashes,
candidate-semantic hash and source/availability/steady timestamps. Final
execution receipt has its independent disposition, executed action, final
times and PG exclusion reason. The actor failure error string, actual backup
and complete runtime receipt remain external ledger fields. No executed label
overwrites a latent or proposal field.

This module exposes the exact local logit score, not a trainable Torch wrapper
or optimizer. A small independent CPU Torch scatter-add/autograd fixture checks
the formula against the actual unchanged actor forward outputs; no gradients
are created on actor parameters and no weights change. Future differentiable
trainer integration must be separately reviewed and checked against this
reference before any training release.

## Validation and retained limits

32 pure tests cover exact rational joint enumeration, fresh/nonempty/empty
masks, original float32 gate edges, partial masks, no artificial stop, actual
q ratios across gate crossings, finite-difference score, invalid output,
support underflow, source/availability/deadline edges, shield versus dropped
receipt, label integrity and train/deploy identity. The first run retained
one assertion failure: binary .7250000000000001 differs from rational .725 by
one binary64 ULP. The test comparison was corrected from 1e-16 to one binary64
epsilon (2.22e-16); no probability, actor or runtime gate changed.

The separate actual-actor probe runs only one small CPU forward and 96 cheap
gate cases. It confirms unchanged actor weights, no parameter gradients,
float32 original-gate parity and marginal score error <3e-16 against native
Torch autograd. This is not a timing benchmark or GPU use. Existing M08 full
chain 69.239986 ms >50 ms and its original environment mismatch remain visible;
neither is repaired by this distribution component. No live exploration,
learning quality, runtime admission, deployment or physical-safety claim is
made. Independent review is pending.
