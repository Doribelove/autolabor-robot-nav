"""Independent small scalar/Decimal fixtures; no ROS, actor or optimizer."""
from dataclasses import replace
from decimal import Decimal, localcontext
from pathlib import Path
import math
import importlib.util
import os
import sys
import pytest

S = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(S / 'workspace/scripts/astra_gazebo'))
if os.environ.get('M11_REVIEW_SOURCE'):
    spec = importlib.util.spec_from_file_location('review_m11', os.environ['M11_REVIEW_SOURCE'])
    m = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = m
    spec.loader.exec_module(m)
else:
    import route_training_returns_m11 as m


def physical(**overrides):
    args = dict(start_ns=1_000_000_000, end_ns=1_400_000_000,
                route_arc_start_m=2., route_arc_end_m=2.4,
                measured_forward_speed_start_mps=.3,
                measured_forward_speed_end_mps=.7,
                executed_action_before=3,
                executed_actions=((1_100_000_000, 0), (1_300_000_000, 2)),
                outcome='running')
    args.update(overrides)
    return m.physical_reward(**args)


def rows():
    return [m.Transition('episode', 'a', 1_000_000_000, 1_125_000_000,
                         .4, .5, -.25, 'running', True),
            m.Transition('episode', 'b', 1_125_000_000, 1_625_000_000,
                         -2., -.25, .75, 'running', False),
            m.Transition('episode', 'c', 1_625_000_000, 2_000_000_000,
                         -4., .75, 0., 'timeout', False)]


def gae(rr, gamma=.81, lam=.64, ref=.25):
    return m.source_time_gae(rr, gamma_per_reference=gamma,
                             lambda_per_reference=lam, reference_duration_s=ref)


def decimal_closed_form(rr, gamma, lam, ref):
    # Independent high-precision direct sum, not implementation reverse loop.
    with localcontext() as ctx:
        ctx.prec = 70
        g, l, ref = map(lambda x: Decimal(str(x)), (gamma, lam, ref))
        exponents = [Decimal(t.end_ns - t.start_ns) / Decimal(10**9) / ref for t in rr]
        discounts = [g ** exponent for exponent in exponents]
        delta = [Decimal(str(t.reward)) + d * Decimal(str(t.next_value)) - Decimal(str(t.value))
                 for t, d in zip(rr, discounts)]
        answers = []
        for i in range(len(rr)):
            total = Decimal(0)
            for k in range(i, len(rr)):
                if any(t.outcome != 'running' or t.collection_cut for t in rr[i:k]):
                    break
                elapsed = sum(exponents[i:k], Decimal(0))
                coefficient = (g ** elapsed) * (l ** elapsed) if elapsed else Decimal(1)
                total += coefficient * delta[k]
            answers.append(float(total))
        return answers


def test_reward_uses_one_nanosecond_actual_delta_not_nominal_step():
    r = physical(end_ns=1_000_000_001, route_arc_end_m=2.,
                 measured_forward_speed_end_mps=.3, executed_actions=())
    assert r['elapsed_s'] == 1e-9
    assert r['reward'] == pytest.approx(-3e-10, rel=1e-15, abs=0)


def test_actual_control_switches_and_reward_preserved_across_boundary():
    whole = physical()
    first = physical(end_ns=1_200_000_000, route_arc_end_m=2.2,
                     measured_forward_speed_end_mps=.5,
                     executed_actions=((1_100_000_000, 0),))
    second = physical(start_ns=1_200_000_000, route_arc_start_m=2.2,
                      measured_forward_speed_start_mps=.5,
                      executed_action_before=first['last_executed_action'],
                      executed_actions=((1_300_000_000, 2),))
    assert first['executed_switch_count'] + second['executed_switch_count'] == whole['executed_switch_count'] == 2
    assert first['reward'] + second['reward'] == pytest.approx(whole['reward'], abs=1e-15)


def test_reverse_progress_and_zero_hold_are_not_deleted():
    r = physical(route_arc_end_m=1.6, executed_action_before=0,
                 executed_actions=((1_100_000_000, 0), (1_300_000_000, 0)))
    assert r['terms']['progress'] == pytest.approx(-.4)
    assert r['terms']['elapsed'] == pytest.approx(-.12)
    assert r['executed_switch_count'] == 0 and r['reward'] < 0


@pytest.mark.parametrize('outcome,penalty', [('goal', 8.), ('contact', -30.), ('timeout', -4.)])
def test_finite_terminal_reward_exact_once(outcome, penalty):
    assert physical(outcome=outcome)['reward'] - physical()['reward'] == pytest.approx(penalty)


def test_variable_duration_gae_against_high_precision_direct_sum():
    rr = rows()
    expected = decimal_closed_form(rr, .81, .64, .25)
    actual = gae(rr)
    assert [x['advantage'] for x in actual] == pytest.approx(expected, rel=3e-15, abs=1e-15)
    assert actual[-1]['value_target'] == -4.  # Timeout truly finite, not continuing bootstrap.


def test_pg_mask_has_no_effect_on_rewards_advantages_or_value_targets():
    base = gae(rows())
    for mask in ((False, False, False), (True, True, True), (False, True, False)):
        changed = gae([replace(t, policy_gradient_eligible=p) for t, p in zip(rows(), mask)])
        assert [(x['advantage'], x['value_target']) for x in changed] == [(x['advantage'], x['value_target']) for x in base]
        assert tuple(x['policy_gradient_eligible'] for x in changed) == mask


def test_lambda_one_telescopes_to_source_discounted_observed_rewards():
    rr = rows()
    actual = gae(rr, lam=1.)
    for i in range(len(rr)):
        expected = sum(t.reward * .81 ** ((t.start_ns - rr[i].start_ns) / 1e9 / .25)
                       for t in rr[i:])
        assert actual[i]['value_target'] == pytest.approx(expected, rel=2e-15)


def test_lambda_zero_is_declared_one_step_td_not_pg_mask_cut():
    rr = rows()
    actual = gae(rr, lam=0.)
    for t, value in zip(rr, actual):
        assert value['advantage'] == pytest.approx(t.reward + .81 ** ((t.end_ns-t.start_ns)/1e9/.25)*t.next_value-t.value)


def test_explicit_cut_bootstraps_from_value_without_later_segment_reward():
    rr = rows()
    first = replace(rr[0], collection_cut=True)
    other = replace(rr[-1], start_ns=1_625_000_000, reward=1000.)
    actual = gae([first, other])
    assert actual[0]['advantage'] == pytest.approx(.4 + .9 * (-.25) - .5)
    assert actual[1]['value_target'] == 1000.
    # A real critic output may equal zero; zero is not proof of missing bootstrap.
    assert gae([replace(first, next_value=0.)])[0]['value_target'] == pytest.approx(.4)


def test_infrastructure_outcome_and_missing_bootstrap_rejected():
    with pytest.raises(ValueError, match='infrastructure'):
        physical(outcome='infrastructure_failure')
    with pytest.raises(ValueError, match='infrastructure'):
        gae([replace(rows()[0], outcome='infrastructure_failure', collection_cut=True)])
    with pytest.raises(ValueError, match='finite numeric'):
        gae([replace(rows()[0], collection_cut=True, next_value=None)])
    with pytest.raises(ValueError, match='open rollout tail'):
        gae([rows()[0]])


def test_dropping_a_nonpg_interval_is_rejected_as_gap():
    with pytest.raises(ValueError, match='missing source interval'):
        gae([rows()[0], rows()[2]])


def test_inconsistent_next_critic_value_rejected():
    rr = rows()
    rr[1] = replace(rr[1], value=-.250001)
    with pytest.raises(ValueError, match='critic values disagree'):
        gae(rr)


def test_time_reversal_duplicate_and_terminal_reuse_rejected():
    rr = rows()
    with pytest.raises(ValueError, match='duplicate'):
        gae([rr[0], rr[0]])
    with pytest.raises(ValueError, match='overlap'):
        gae([replace(rr[0], collection_cut=True), replace(rr[1], start_ns=1_100_000_000)])
    with pytest.raises(ValueError, match='reused'):
        gae([rr[2], replace(rr[0], start_ns=3_000_000_000, end_ns=3_125_000_000, collection_cut=True)])


def test_extreme_but_accepted_discount_domain_does_not_prematurely_underflow():
    rr = [m.Transition('e', 'a', 1_000_000_000, 1_100_000_000, 0., 0., 0., 'running', True),
          m.Transition('e', 'b', 1_100_000_000, 1_200_000_000, 1., 0., 0., 'goal', False)]
    expected = decimal_closed_form(rr, 1e-200, 1e-200, 1.)
    actual = gae(rr, gamma=1e-200, lam=1e-200, ref=1.)
    assert actual[0]['advantage'] == pytest.approx(expected[0], rel=1e-14, abs=0)
