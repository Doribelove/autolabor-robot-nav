import math
from dataclasses import replace
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from route_training_returns_m11 import physical_reward, source_time_gae, Transition

def reward(**kw):
    args=dict(start_ns=1_000_000_000,end_ns=1_250_000_000,route_arc_start_m=2.,route_arc_end_m=2.2,
        measured_forward_speed_start_mps=.8,measured_forward_speed_end_mps=.6,
        executed_action_before=3,executed_actions=[(1_050_000_000,3),(1_100_000_000,0),(1_150_000_000,0),(1_200_000_000,2)],outcome='running')
    args.update(kw);return physical_reward(**args)

def row(**kw):
    args=dict(episode_id='e1',decision_id='d1',start_ns=1_000_000_000,end_ns=1_250_000_000,
        reward=1.,value=2.,next_value=3.,outcome='running',policy_gradient_eligible=True,collection_cut=True)
    args.update(kw);return Transition(**args)

def gae(rows,**kw):
    args=dict(gamma_per_reference=.9,lambda_per_reference=.8,reference_duration_s=.25)
    args.update(kw);return source_time_gae(rows,**args)

def test_actual_time_and_all_executed_zero_switches_retained():
    a=reward()
    assert a['executed_switch_count']==2 and a['last_executed_action']==2
    assert a['reward']==pytest.approx(.2-.075-.002-.01)
    assert reward(end_ns=1_500_000_000)['terms']['elapsed']==pytest.approx(-.15)

@pytest.mark.parametrize('outcome,bonus',[('goal',8),('contact',-30),('timeout',-4)])
def test_finite_horizon_terminal_rewards(outcome,bonus):
    a=reward(outcome=outcome)
    assert a['terminal'] is True and a['terms']['terminal']==bonus

@pytest.mark.parametrize('kw',[
    {'start_ns':True},{'end_ns':1_000_000_000},{'outcome':'infrastructure_failure'},
    {'route_arc_end_m':float('nan')},{'executed_action_before':False},
    {'executed_actions':[(1_250_000_001,2)]},
    {'executed_actions':[(1_050_000_000,2),(1_050_000_000,0)]},
    {'executed_actions':[(1_000_000_000,2)]},{'executed_actions':[(1_050_000_000,8)]},
    {'route_arc_start_m':-1e308,'route_arc_end_m':1e308}])
def test_invalid_or_fabricated_measurements_rejected(kw):
    with pytest.raises(ValueError):reward(**kw)

def test_fractional_actual_duration_bootstrap():
    a=gae([row(end_ns=1_125_000_000)])[0]
    assert a['discount']==pytest.approx(math.sqrt(.9))
    assert a['advantage']==pytest.approx(1+math.sqrt(.9)*3-2)
    assert a['value_target']==pytest.approx(1+math.sqrt(.9)*3)

def test_nonpg_stops_keep_reward_and_earlier_credit():
    a=row(collection_cut=False,next_value=3.)
    b=row(decision_id='d2',start_ns=a.end_ns,end_ns=1_750_000_000,reward=-4.,value=3.,next_value=0.,
          outcome='timeout',collection_cut=False,policy_gradient_eligible=False)
    r=gae([a,b])
    assert r[1]['advantage']==-7.
    assert r[0]['advantage']==pytest.approx(1+.9*3-2+.9*.8*(-7.))
    assert r[0]['policy_gradient_eligible'] is True and r[1]['policy_gradient_eligible'] is False

def test_cut_bootstraps_without_next_segment_leakage():
    a=row()
    b=row(episode_id='e2',decision_id='d2',reward=100.,value=0.,next_value=0.,outcome='goal',collection_cut=False)
    r=gae([a,b]);assert r[0]['advantage']==pytest.approx(1+.9*3-2)
    assert r[1]['advantage']==100.

def test_gamma_lambda_one_matches_retained_monte_carlo():
    a=row(value=0.,next_value=0.,collection_cut=False)
    b=row(decision_id='d2',start_ns=a.end_ns,end_ns=1_700_000_000,value=0.,next_value=0.,reward=-2.,collection_cut=False,outcome='timeout',policy_gradient_eligible=False)
    r=gae([a,b],gamma_per_reference=1.,lambda_per_reference=1.)
    assert [x['value_target'] for x in r]==[-1.,-2.]

def test_fractional_exponent_does_not_underflow_gamma_lambda_product():
    a=row(end_ns=1_100_000_000,value=0.,next_value=0.,reward=0.,collection_cut=False)
    b=row(decision_id='d2',start_ns=a.end_ns,end_ns=1_200_000_000,
          value=0.,next_value=0.,reward=1.,outcome='goal',collection_cut=False,
          policy_gradient_eligible=False)
    result=gae([a,b],gamma_per_reference=1e-200,
               lambda_per_reference=1e-200,reference_duration_s=1.)
    assert result[0]['advantage']==pytest.approx(1e-40,rel=1e-12)

@pytest.mark.parametrize('mutation',[
    {'next_value':float('nan')},{'outcome':'infrastructure_failure'},
    {'policy_gradient_eligible':1},{'collection_cut':0},
    {'outcome':'timeout','collection_cut':False},
    {'outcome':'timeout','next_value':0.},
    {'end_ns':1_000_000_000},{'reward':True}])
def test_invalid_transition_does_not_enter_optimization(mutation):
    with pytest.raises(ValueError):gae([row(**mutation)])

def test_missing_tail_requires_real_bootstrap_cut():
    with pytest.raises(ValueError,match='open rollout tail'):gae([row(collection_cut=False)])

@pytest.mark.parametrize('change',[{'start_ns':1_260_000_000},{'episode_id':'other'},{'value':3.01}])
def test_unacknowledged_gap_or_critic_mismatch_rejected(change):
    a=row(collection_cut=False)
    b=row(decision_id='d2',start_ns=a.end_ns,end_ns=1_500_000_000,value=3.)
    with pytest.raises(ValueError):gae([a,replace(b,**change)])

def test_duplicate_and_nonadjacent_reused_terminal_rejected():
    a=row(outcome='goal',next_value=0.,collection_cut=False)
    with pytest.raises(ValueError):gae([a,a])
    b=row(episode_id='e2',decision_id='b')
    c=row(decision_id='c',start_ns=2_000_000_000,end_ns=2_250_000_000)
    with pytest.raises(ValueError,match='reused'):gae([a,b,c])

def test_nonadjacent_overlapping_episode_segments_rejected():
    a=row();b=row(episode_id='e2');c=row(decision_id='c',start_ns=1_100_000_000)
    with pytest.raises(ValueError,match='overlap'):gae([a,b,c])

@pytest.mark.parametrize('kw',[{'gamma_per_reference':0},{'lambda_per_reference':1.1},
    {'reference_duration_s':0},{'gamma_per_reference':True}])
def test_invalid_discount_config(kw):
    with pytest.raises(ValueError):gae([row()],**kw)
