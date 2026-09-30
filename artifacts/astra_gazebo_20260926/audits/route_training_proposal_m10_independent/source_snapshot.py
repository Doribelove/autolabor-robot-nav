"""M10 pure proposal-space distribution; no trainer, ROS, weights or RNG state.

The actor forward pass is unchanged. A NEW training policy samples an explicit
epsilon branch and latent action, then marginalizes their pushforward. Runtime
timing/acceptance must be supplied from actual receipts by a future integration.
"""
from dataclasses import asdict, dataclass
import math
import struct
from typing import Optional, Tuple


TRAIN_ID = 'astra_route_tracks_pushforward_explore_m10_v1'
DEPLOY_ID = 'astra_route_tracks_deterministic_residual_gate_m10_v1'
SCHEMA = 'astra_route_training_proposal_m10_v1'
EPSILON = 0.20
GATE_MARGIN = 0.10
COUNT = 8
ACTOR_BUDGET_NS = 20_000_000
TRANSACTION_BUDGET_NS = 50_000_000
SOURCE_MAX_AGE_NS = 200_000_000


def _sha(value):
    if type(value) is not str or len(value) != 64 or any(c not in '0123456789abcdef' for c in value):
        raise ValueError('lowercase SHA256 required')
    return value


def _integer(value):
    if type(value) is not int or value < 0:
        raise ValueError('nonnegative integer nanoseconds required')
    return value


def _float(value):
    if type(value) not in (float, int) or not math.isfinite(value):
        raise ValueError('finite scalar required')
    return float(value)


def _f32(value):
    """Match the existing actor's float32 residual subtract/threshold gate."""
    try:
        result = struct.unpack('f', struct.pack('f', _float(value)))[0]
    except OverflowError as exc:
        raise ValueError('outside finite float32 actor domain') from exc
    if not math.isfinite(result):
        raise ValueError('outside finite float32 actor domain')
    return result


def _vector(value):
    if type(value) is not tuple or len(value) != COUNT:
        raise ValueError('immutable eight-action tuple required')
    for x in value:
        _float(x)
    return value


@dataclass(frozen=True)
class ActorOutput:
    """Original forward outputs plus exact original true-backup mask and prior.

    Pass float32 output scalars converted to Python floats. No new preview mask,
    prior normalization, clipping, residual recomputation or stop fabrication.
    """
    logits: Tuple[float, ...]
    residual: Tuple[float, ...]
    prior: Tuple[float, ...]
    mask: Tuple[bool, ...]

    def validate(self):
        for vector in (self.logits, self.residual, self.prior):
            _vector(vector)
            if any(_f32(v) != v for v in vector):
                raise ValueError('actual float32 actor outputs required')
        if type(self.mask) is not tuple or len(self.mask) != COUNT or any(type(x) is not bool for x in self.mask):
            raise ValueError('exact bool true-backup mask required')
        if any(abs(x) > 3.0 for x in self.residual):
            raise ValueError('existing residual-scale domain exceeded')
        return self


@dataclass(frozen=True)
class Context:
    decision_id: str
    observation_sha256: str
    candidate_semantics_sha256: str
    behavior_checkpoint_sha256: str
    decision_source_ns: int
    scan_source_ns: int
    odom_source_ns: int
    available_source_ns: int
    transaction_start_steady_ns: int

    def validate(self):
        if type(self.decision_id) is not str or not self.decision_id:
            raise ValueError('decision identity required')
        for value in (self.observation_sha256, self.candidate_semantics_sha256, self.behavior_checkpoint_sha256):
            _sha(value)
        for value in (self.decision_source_ns, self.scan_source_ns, self.odom_source_ns,
                      self.available_source_ns, self.transaction_start_steady_ns):
            _integer(value)
        if min(self.scan_source_ns, self.odom_source_ns) <= 0:
            raise ValueError('real positive sensor source stamps required')
        if not max(self.scan_source_ns, self.odom_source_ns) <= self.available_source_ns <= self.decision_source_ns:
            raise ValueError('source/availability/decision causality violated')
        if any(self.decision_source_ns - s > SOURCE_MAX_AGE_NS for s in (self.scan_source_ns, self.odom_source_ns)):
            raise ValueError('stale decision source')
        return self


@dataclass(frozen=True)
class Distribution:
    output: ActorOutput
    analytic_action: int
    gate_map: Tuple[Optional[int], ...]
    pi: Tuple[float, ...]
    q: Tuple[float, ...]

    def log_probability(self, action):
        if type(action) is not int or not 0 <= action < COUNT or self.q[action] <= 0:
            raise ValueError('action outside proposal support')
        return math.log(self.q[action])

    def logit_score(self, action):
        """d log q(action)/d logits, discrete gate map held fixed (a.e. only)."""
        self.log_probability(action)
        return tuple(p * (EPSILON * (j == action) + (1 - EPSILON) * (g == action) - self.q[action]) / self.q[action]
                     for j, (p, g) in enumerate(zip(self.pi, self.gate_map)))


def distribution(output):
    output.validate()
    valid = [i for i, m in enumerate(output.mask) if m]
    if not valid:
        raise ValueError('empty_true_backup_mask')
    # Equal maxima use first index, as the inherited torch.argmax does.
    analytic = max(valid, key=lambda i: output.prior[i])
    gate = tuple((i if i != analytic and _f32(output.residual[i] - output.residual[analytic]) >= _f32(GATE_MARGIN)
                  else analytic) if output.mask[i] else None for i in range(COUNT))
    maximum = max(output.logits[i] for i in valid)
    weights = tuple(math.exp(output.logits[i] - maximum) if output.mask[i] else 0.0 for i in range(COUNT))
    total = math.fsum(weights)
    pi = tuple(w / total for w in weights)
    if any(pi[i] <= 0 for i in valid):
        raise ValueError('categorical_support_underflow_no_probability_floor')
    q = tuple(EPSILON * pi[a] + (1 - EPSILON) * math.fsum(pi[z] for z in valid if gate[z] == a)
              if output.mask[a] else 0.0 for a in range(COUNT))
    if any(q[i] <= 0 for i in valid) or not math.isclose(math.fsum(q), 1.0, abs_tol=1e-14):
        raise ValueError('invalid_pushforward_distribution')
    return Distribution(output, analytic, gate, pi, q)


def _uniform(value):
    value = _float(value)
    if not 0 <= value < 1:
        raise ValueError('independent uniform draw must be in [0,1)')
    return value


def _categorical(probabilities, uniform):
    cumulative = 0.0
    last = None
    for i, p in enumerate(probabilities):
        if p <= 0:
            continue
        last = i
        cumulative += p
        if uniform < cumulative:
            return i
    # Roundoff at the final cumulative sum, without assigning masked indices.
    return last


@dataclass(frozen=True)
class Proposal:
    context: Context
    actor_start_steady_ns: int
    actor_finish_steady_ns: int
    distribution: Distribution
    branch: str
    branch_uniform: float
    latent_uniform: float
    latent_action: int
    latent_log_probability: float
    latent_gate_action: int
    proposal_action: int
    proposal_log_probability: float

    def record(self):
        return dict(schema=SCHEMA, policy_id=TRAIN_ID, epsilon=EPSILON,
                    gate_margin=GATE_MARGIN, **asdict(self))


def sample(context, output, actor_start_steady_ns, actor_finish_steady_ns, branch_uniform, latent_uniform):
    """Raises on invalid input. Record inherited fallback/zero and NO PG row.

    This pure function owns no RNG: caller supplies two independent draws from
    its bound RNG stream and logs their provenance. Actor must already be valid.
    """
    context.validate()
    _integer(actor_start_steady_ns)
    _integer(actor_finish_steady_ns)
    if actor_start_steady_ns < context.transaction_start_steady_ns or not 0 <= actor_finish_steady_ns - actor_start_steady_ns <= ACTOR_BUDGET_NS:
        raise ValueError('actor_timeout_or_steady_clock_rollback')
    if actor_finish_steady_ns - context.transaction_start_steady_ns > TRANSACTION_BUDGET_NS:
        raise ValueError('transaction_deadline')
    d = distribution(output)
    u, v = _uniform(branch_uniform), _uniform(latent_uniform)
    z = _categorical(d.pi, v)
    branch = 'exploration' if u < EPSILON else 'residual_gate'
    a = z if branch == 'exploration' else d.gate_map[z]
    return Proposal(context, actor_start_steady_ns, actor_finish_steady_ns, d, branch, u, v, z,
                    math.log(d.pi[z]), d.gate_map[z], a, d.log_probability(a))


def _validate_proposal(proposal):
    replay = sample(proposal.context, proposal.distribution.output,
                    proposal.actor_start_steady_ns, proposal.actor_finish_steady_ns,
                    proposal.branch_uniform, proposal.latent_uniform)
    if replay != proposal:
        raise ValueError('proposal branch/latent/distribution receipt mismatch')


@dataclass(frozen=True)
class FinalReceipt:
    proposal: Optional[Proposal]
    decision_id: str
    disposition: str
    final_source_ns: int
    final_steady_ns: int
    executed_action: Optional[int]
    policy_gradient_eligible: bool
    exclusion_reason: Optional[str]


DISPOSITIONS = frozenset(('proposal_applied', 'shield_fallback', 'actor_invalid',
                         'empty_mask', 'deadline_dropped', 'source_expired', 'clock_fault'))


def finalize(context, proposal, disposition, final_source_ns, final_steady_ns, executed_action=None):
    """Separate execution receipt. PG requires a valid timely submitted proposal.

    A timely proposal handled by an inherited deterministic shield may retain
    PG with its ORIGINAL q/action; execution is an environment response. An
    invalid/expired/dropped proposal has no PG. All failures remain in returns,
    critic/ledger and failure-inclusive episode accounting; no label relabeling.
    """
    context.validate()
    if disposition not in DISPOSITIONS:
        raise ValueError('closed execution disposition required')
    _integer(final_source_ns)
    _integer(final_steady_ns)
    if executed_action is not None and (type(executed_action) is not int or not 0 <= executed_action < COUNT):
        raise ValueError('executed action index or None required')
    if proposal is not None and proposal.context != context:
        raise ValueError('proposal/context identity mismatch')
    if proposal is not None:
        _validate_proposal(proposal)
    reason = None
    if proposal is None:
        reason = 'no_valid_proposal'
    elif disposition not in ('proposal_applied', 'shield_fallback'):
        reason = disposition
    elif final_source_ns < context.decision_source_ns or final_steady_ns < proposal.actor_finish_steady_ns:
        reason = 'clock_rollback'
    elif (final_source_ns - context.decision_source_ns > TRANSACTION_BUDGET_NS or
          final_steady_ns - context.transaction_start_steady_ns > TRANSACTION_BUDGET_NS):
        reason = 'transaction_deadline'
    elif any(final_source_ns - stamp > SOURCE_MAX_AGE_NS for stamp in (context.scan_source_ns, context.odom_source_ns)):
        reason = 'source_expired'
    if disposition == 'proposal_applied' and proposal is not None and executed_action != proposal.proposal_action:
        raise ValueError('applied receipt cannot relabel proposal as executed action')
    return FinalReceipt(proposal, context.decision_id, disposition, final_source_ns,
                        final_steady_ns, executed_action, reason is None, reason)


@dataclass(frozen=True)
class UpdateLikelihood:
    action: int
    old_log_q: float
    new_log_q: float
    ratio: float
    gate_changed_indices: Tuple[int, ...]
    old_to_new_kl: float
    new_logit_score: Tuple[float, ...]
    target_checkpoint_sha256: str


def update_likelihood(receipt, current_output, target_checkpoint_sha256,
                      observation_sha256, candidate_semantics_sha256):
    """Exact marginal ratio on the same observation; no optimizer/backward.

    The map is rebuilt with CURRENT residuals. Holding old membership fixed
    would give the wrong ratio across a gate boundary. The returned score is
    only the within-partition derivative, never a straight-through estimator.
    """
    _sha(target_checkpoint_sha256)
    if not receipt.policy_gradient_eligible or receipt.proposal is None:
        raise ValueError('no_policy_gradient_for_invalid_or_dropped_proposal')
    proposal = receipt.proposal
    if (observation_sha256 != proposal.context.observation_sha256 or
            candidate_semantics_sha256 != proposal.context.candidate_semantics_sha256):
        raise ValueError('same bound observation/candidate semantics required')
    if finalize(proposal.context, proposal, receipt.disposition, receipt.final_source_ns,
                receipt.final_steady_ns, receipt.executed_action) != receipt:
        raise ValueError('final eligibility receipt mismatch')
    old = receipt.proposal.distribution
    current_output.validate()
    if current_output.mask != old.output.mask or current_output.prior != old.output.prior:
        raise ValueError('same observation true-mask/prior required for ratio')
    new = distribution(current_output)
    a = receipt.proposal.proposal_action
    old_log, new_log = receipt.proposal.proposal_log_probability, new.log_probability(a)
    try:
        ratio = math.exp(new_log - old_log)
    except OverflowError as exc:
        raise ValueError('nonfinite_proposal_ratio') from exc
    if not math.isfinite(ratio) or ratio <= 0:
        raise ValueError('nonfinite_proposal_ratio')
    kl = math.fsum(p * math.log(p / new.q[i]) for i, p in enumerate(old.q) if p > 0)
    return UpdateLikelihood(a, old_log, new_log, ratio,
                            tuple(i for i, (x, y) in enumerate(zip(old.gate_map, new.gate_map)) if x != y),
                            kl, new.logit_score(a), target_checkpoint_sha256)


def deterministic_deployment(output):
    """Different identity/objective: deterministic argmax + inherited gate.

    epsilon=0 stochastic gate sampling is NOT this deterministic policy.
    Empty mask raises; integration must zero/fallback without fabricating stop.
    """
    d = distribution(output)
    latent = max((i for i, m in enumerate(output.mask) if m), key=lambda i: output.logits[i])
    return dict(policy_id=DEPLOY_ID, exploration_enabled=False, deterministic=True,
                latent_action=latent, latent_gate_action=d.gate_map[latent],
                proposal_action=d.gate_map[latent], analytic_action=d.analytic_action,
                policy_gradient_eligible=False)
