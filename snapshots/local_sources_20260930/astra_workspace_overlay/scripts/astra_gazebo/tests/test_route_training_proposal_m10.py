"""Small deterministic CPU mathematics/receipt tests; no model or ROS import."""
from dataclasses import replace
from fractions import Fraction
import json
import math
from pathlib import Path
import struct
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import route_training_proposal_m10 as m


def f32(x):
    return struct.unpack('f', struct.pack('f', x))[0]


def out(logits=None, residual=None, prior=None, mask=None):
    return m.ActorOutput(tuple(map(f32, logits or [0.] * 8)),
                         tuple(map(f32, residual or [0.] * 8)),
                         tuple(map(f32, prior or list(range(8)))),
                         tuple(mask if mask is not None else [True] * 8))


def context():
    return m.Context('synthetic_decision_1', 'a' * 64, 'b' * 64, 'c' * 64,
                     1_000_000_000, 900_000_000, 1_000_000_000,
                     1_000_000_000, 2_000_000_000)


def proposal(output=None, u=.1, v=.01, ctx=None):
    return m.sample(ctx or context(), output or out(), 2_010_000_000, 2_025_000_000, u, v)


def accepted(p=None, **kwargs):
    p = p or proposal()
    return m.finalize(p.context, p, 'proposal_applied', 1_040_000_000,
                      2_040_000_000, p.proposal_action, **kwargs)


def update(r, output, **kwargs):
    return m.update_likelihood(r, output, 'd' * 64, kwargs.get('observation', 'a' * 64),
                               kwargs.get('semantics', 'b' * 64))


def test_fresh_zero_head_has_exact_nonanalytic_exploration():
    d = m.distribution(out())
    assert d.gate_map == (7,) * 8
    assert d.q[:7] == (.025,) * 7
    assert d.q[7] == pytest.approx(.825)
    assert math.fsum(d.q[:7]) == pytest.approx(.175)
    p = proposal()
    assert p.branch == 'exploration' and p.latent_action == p.proposal_action == 0
    assert p.latent_gate_action == 7
    assert m.deterministic_deployment(out())['proposal_action'] == 7


def test_exact_joint_enumeration_independent_rational_reference():
    # Analytic=7, only action2 crosses gate. Enumerate 2 branches x8 latents.
    output = out(residual=[0, 0, .2, 0, 0, 0, 0, 0])
    expected = [Fraction(0)] * 8
    for explore, branch_mass in ((True, Fraction(1, 5)), (False, Fraction(4, 5))):
        for z in range(8):
            destination = z if explore or z == 2 else 7
            expected[destination] += branch_mass * Fraction(1, 8)
    d = m.distribution(output)
    assert d.q == pytest.approx(list(map(float, expected)), abs=2.22e-16)
    assert d.q[2] == .125 and d.q[0] == .025


def test_proposal_logq_is_neither_latent_nor_joint_branch_probability():
    p = proposal(u=.5, v=.01)
    assert p.latent_action == 0 and p.proposal_action == 7
    assert p.proposal_log_probability == pytest.approx(math.log(.825))
    assert p.proposal_log_probability != p.latent_log_probability
    assert p.proposal_log_probability != math.log(.8 * .125)
    # Same proposal from distinct latent/branch paths has exactly same log q.
    p2 = proposal(u=.01, v=.99)
    assert (p2.branch, p2.latent_action) != (p.branch, p.latent_action)
    assert p2.proposal_action == p.proposal_action
    assert p2.proposal_log_probability == p.proposal_log_probability


def test_true_backup_mask_and_no_stop_fabrication():
    output = out(prior=[1000, 6, 5, 4, 3, 2, 1, 0], mask=[False, True, False, True, False, False, False, False])
    d = m.distribution(output)
    assert d.analytic_action == 1 and d.q[0] == 0 and d.gate_map[0] is None
    for u in (0., .2, .999):
        for v in (0., .49, .5, .999):
            assert proposal(output, u, v).proposal_action in (1, 3)


def test_single_valid_mask_is_explicitly_no_alternative_exploration():
    d = m.distribution(out(mask=[False, False, True, False, False, False, False, False]))
    assert d.q == (0., 0., 1., 0., 0., 0., 0., 0.)


def test_empty_mask_fails_before_categorical_and_has_no_pg():
    with pytest.raises(ValueError, match='empty_true_backup_mask'):
        proposal(out(mask=[False] * 8))
    receipt = m.finalize(context(), None, 'empty_mask', 1_010_000_000, 2_010_000_000)
    assert not receipt.policy_gradient_eligible and receipt.proposal is None
    with pytest.raises(ValueError, match='no_policy_gradient'):
        update(receipt, out())


@pytest.mark.parametrize('change', [
    {'logits': (float('nan'),) + (0.,) * 7},
    {'residual': (float('inf'),) + (0.,) * 7},
    {'residual': (4.,) + (0.,) * 7},
    {'mask': (1,) + (True,) * 7},
    {'prior': (.1,) + (0.,) * 7},  # Not a float32 actor output scalar.
])
def test_invalid_actor_does_not_make_proposal(change):
    with pytest.raises(ValueError):
        proposal(replace(out(), **change))
    assert not m.finalize(context(), None, 'actor_invalid', 1_020_000_000, 2_020_000_000).policy_gradient_eligible


def test_probability_underflow_is_rejected_not_floored():
    with pytest.raises(ValueError, match='support_underflow'):
        proposal(out(logits=[-1000, 0, 0, 0, 0, 0, 0, 0]))


def test_actor_20ms_budget_distinct_from_complete_50ms_budget():
    p = m.sample(context(), out(), 2_025_000_000, 2_045_000_000, .1, .1)
    assert accepted(p).exclusion_reason == 'clock_rollback'  # final40ms predates finish45ms
    r = m.finalize(context(), p, 'proposal_applied', 1_050_000_000, 2_050_000_000, p.proposal_action)
    assert r.policy_gradient_eligible
    with pytest.raises(ValueError, match='actor_timeout'):
        m.sample(context(), out(), 2_000_000_000, 2_020_000_001, .1, .1)
    with pytest.raises(ValueError, match='transaction_deadline'):
        m.sample(context(), out(), 2_040_000_000, 2_050_000_001, .1, .1)


@pytest.mark.parametrize('source,steady,reason', [
    (1_050_000_001, 2_040_000_000, 'transaction_deadline'),
    (1_040_000_000, 2_050_000_001, 'transaction_deadline'),
    (999_999_999, 2_040_000_000, 'clock_rollback'),
    (1_040_000_000, 2_024_999_999, 'clock_rollback'),
])
def test_final_deadline_or_clock_failure_never_pg(source, steady, reason):
    p = proposal()
    r = m.finalize(context(), p, 'proposal_applied', source, steady, p.proposal_action)
    assert r.exclusion_reason == reason and not r.policy_gradient_eligible
    with pytest.raises(ValueError, match='no_policy_gradient'):
        update(r, out())


def test_raw_freshness_cannot_extend_committed_source_age():
    ctx = replace(context(), scan_source_ns=800_000_000)
    p = proposal(ctx=ctx)
    r = m.finalize(ctx, p, 'proposal_applied', 1_000_000_001, 2_030_000_000, p.proposal_action)
    assert r.exclusion_reason == 'source_expired' and not r.policy_gradient_eligible


@pytest.mark.parametrize('changes', [
    {'available_source_ns': 1_000_000_001},
    {'scan_source_ns': 799_999_999},
    {'odom_source_ns': 1_000_000_001},
    {'decision_source_ns': True},
])
def test_context_source_and_availability_rejected(changes):
    with pytest.raises(ValueError):
        proposal(ctx=replace(context(), **changes))


def test_shield_fallback_never_relabels_latent_or_proposal():
    p = proposal(v=.2)
    r = m.finalize(context(), p, 'shield_fallback', 1_040_000_000, 2_040_000_000, 0)
    assert r.policy_gradient_eligible and r.executed_action != p.proposal_action
    result = update(r, out())
    assert result.action == p.proposal_action and result.ratio == 1
    with pytest.raises(ValueError, match='cannot relabel'):
        m.finalize(context(), p, 'proposal_applied', 1_040_000_000, 2_040_000_000, 0)


def test_explicit_dropped_receipt_has_no_pg_even_if_numerical_times_fit():
    p = proposal()
    r = m.finalize(context(), p, 'deadline_dropped', 1_040_000_000, 2_040_000_000, 0)
    assert not r.policy_gradient_eligible


def test_proposal_and_final_receipts_reject_forged_labels_or_eligibility():
    p = proposal()
    with pytest.raises(ValueError, match='receipt mismatch'):
        accepted(replace(p, latent_action=2))
    r = m.finalize(context(), p, 'deadline_dropped', 1_040_000_000, 2_040_000_000, 0)
    with pytest.raises(ValueError, match='eligibility receipt'):
        update(replace(r, policy_gradient_eligible=True), out())


def test_current_gate_recomputed_ratio_exact_across_boundary():
    p = proposal(v=.27)  # latent and proposal2; old q2=.025
    r = accepted(p)
    changed = out(residual=[0, 0, .2, 0, 0, 0, 0, 0])
    result = update(r, changed)
    assert result.action == 2 and result.ratio == pytest.approx(5.)
    assert result.gate_changed_indices == (2,)
    assert result.old_to_new_kl > 0
    # Reusing old map or π(new)/π(old) would incorrectly give ratio1.
    assert result.ratio != m.distribution(changed).pi[2] / p.distribution.pi[2]


def test_same_bound_observation_mask_prior_and_semantics_required():
    r = accepted()
    for kwargs in ({'observation': 'e' * 64}, {'semantics': 'e' * 64}):
        with pytest.raises(ValueError, match='same bound'):
            update(r, out(), **kwargs)
    with pytest.raises(ValueError, match='same observation'):
        update(r, out(mask=[False] + [True] * 7))
    with pytest.raises(ValueError, match='same observation'):
        update(r, out(prior=list(reversed(range(8)))))


def test_float32_original_gate_equality_boundary():
    below = f32(.09999999)
    assert m.distribution(out(residual=[below] + [0] * 7)).gate_map[0] == 7
    assert m.distribution(out(residual=[f32(.1)] + [0] * 7)).gate_map[0] == 0


def test_piecewise_score_against_independent_softmax_finite_difference():
    # Reference enumerates the joint branches; it doesn't call implementation q.
    output = out(logits=[-.7, -.4, .0, .2, .4, -.2, .9, .5], residual=[0, 0, .2, 0, 0, 0, 0, 0])
    d = m.distribution(output)
    def reference(logits, target):
        weights = [math.exp(x) for x in logits]
        normalizer = sum(weights)
        mass = 0.
        for z, w in enumerate(weights):
            for branch, probability in ((True, .2), (False, .8)):
                a = z if branch or z == 2 else 7
                if a == target:
                    mass += probability * w / normalizer
        return math.log(mass)
    step = 1e-5
    for a in (0, 2, 7):
        score = d.logit_score(a)
        for j in range(8):
            plus, minus = list(output.logits), list(output.logits)
            plus[j] += step
            minus[j] -= step
            numerical = (reference(plus, a) - reference(minus, a)) / (2 * step)
            assert score[j] == pytest.approx(numerical, abs=3e-10)
        assert sum(score) == pytest.approx(0., abs=2e-16)


def test_zero_head_reward_score_is_epsilon_scaled_categorical_not_zero():
    d = m.distribution(out())
    rewards = [1., 0., 0., 0., 0., 0., 0., 0.]
    grad = [sum(d.q[a] * rewards[a] * d.logit_score(a)[j] for a in range(8)) for j in range(8)]
    assert grad[0] == pytest.approx(.2 * .125 * (1 - .125))
    assert grad[0] > 0
    assert grad[7] == pytest.approx(-.2 * .125 * .125)


def test_deployment_identity_not_epsilon_zero_stochastic_distribution():
    o = out(logits=[3, 0, 0, 0, 0, 0, 0, 0], residual=[1, 0, 0, 0, 0, 0, 0, 0])
    deploy = m.deterministic_deployment(o)
    assert deploy['proposal_action'] == 0
    assert deploy['policy_id'] != m.TRAIN_ID and not deploy['policy_gradient_eligible']
    assert m.distribution(o).q[0] < 1  # Training remains stochastic.


def test_record_is_finite_complete_json_with_distinct_labels():
    p = proposal(u=.5, v=.01)
    record = p.record()
    json.dumps(record, allow_nan=False)
    assert record['latent_action'] == 0 and record['proposal_action'] == 7
    assert record['distribution']['output']['mask'] == (True,) * 8
    assert 'prior' in record['distribution']['output'] and 'gate_map' in record['distribution']
    assert record['context']['behavior_checkpoint_sha256'] == 'c' * 64
