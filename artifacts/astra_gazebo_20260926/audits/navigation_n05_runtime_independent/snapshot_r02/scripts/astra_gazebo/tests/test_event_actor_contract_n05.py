"""Actual XML and event-status boundaries for the new shared actor gate."""
import copy
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import event_actor_contract_n05 as event
import evaluate_episode as old
import evaluate_episode_n05 as new


def plugin():
    return ET.fromstring('''<plugin name="actor" filename="libastra_event_actor_n05_plugin.so">
      <expected_run_id>astra_n05_probe</expected_run_id><expected_seed>263002001</expected_seed>
      <expected_method>astra_analytic_preview_eligible_n05</expected_method>
      <gate_topic>/astra/navigation/gate</gate_topic><max_gate_transport_delay_ns>50000000</max_gate_transport_delay_ns>
      <ego_model>autolabor_m2</ego_model><policy>cautious</policy><start>1 -3</start><end>1 3</end><conflict>1 0</conflict>
      <gate_delay>0.2</gate_delay><base_speed>0.8</base_speed><initial_speed>0.9</initial_speed>
      <response_distance>4</response_distance><response_time_margin>1.35</response_time_margin>
    </plugin>''')


@pytest.mark.parametrize('key,value',[
    ('expected_run_id','UNBOUND_TEMPLATE'),('expected_seed','True'),('expected_seed','0'),
    ('expected_seed','4294967296'),('expected_method','oracle'),('gate_topic','/foreign'),
    ('max_gate_transport_delay_ns','50000001'),('max_gate_transport_delay_ns','50000000.0'),
    ('gate_delay','nan'),('gate_delay','-0.1'),('gate_delay','23'),('ego_model','other_robot')])
def test_actor_identity_and_frozen_timing_reject_mismatch(key,value):
    p=plugin();p.find(key).text=value
    with pytest.raises(ValueError):event.plugin_contract(p)


@pytest.mark.parametrize('key',['gate_route_x','route_yaw','route_origin','start_delay','activation_ego_x'])
def test_no_hidden_spatial_or_legacy_activation(key):
    p=plugin();ET.SubElement(p,key).text='0'
    with pytest.raises(ValueError):new.actor_motion_envelope(p)


def test_new_gate_retains_actual_old_policy_speed_envelope():
    p=plugin();current=new.actor_motion_envelope(p)
    previous=copy.deepcopy(p);previous.set('filename','libastra_reactive_actor_plugin.so')
    for key in ('expected_run_id','expected_seed','expected_method','gate_topic','max_gate_transport_delay_ns'):
        previous.remove(previous.find(key))
    for key,value in [('route_origin','0 0'),('route_yaw','0'),('gate_route_x','-4')]:
        ET.SubElement(previous,key).text=value
    retained=old.actor_motion_envelope(previous)
    for key in ('speed_model_bound_mps','target_speeds_mps','initial_speed_mps','derivation'):
        assert current[key]==retained[key]
    assert current['speed_model_bound_mps']==1.35
    with pytest.raises(ValueError):old.actor_motion_envelope(p)


def rows(delay=20_000_000):
    values=dict(schema=event.STATUS_SCHEMA,run_id='astra_n05_probe',seed=263002001,
        method='astra_analytic_preview_eligible_n05',model_name='actor_1',fault=False,reason='',
        status_source_ns=8_000_000_000,gate_source_ns=7_000_000_000,
        receipt_ros_ns=7_010_000_000,receipt_steady_ns=1_000_000_000_000,
        first_begin_ns=7_000_000_000+delay,transport_delay_ns=delay,
        activation_source_ns=7_200_000_000,first_active_begin_ns=7_200_000_000,
        gate_delay_ns=200_000_000,duplicate_count=0,max_gate_transport_delay_ns=50_000_000)
    return [values]


def audit(data):
    return event.audit_status(data,{'actor_1':event.plugin_contract(plugin())},
                              {'source_ns':7_000_000_000},8_000_000_000)


def test_transport_deadline_exact_boundary_and_one_ns_outside():
    assert audit(rows(50_000_000))['pass_']
    assert not audit(rows(50_000_001))['pass_']


@pytest.mark.parametrize('key,value',[
    ('seed',True),('method','dwa_common_route'),('fault',True),('fault',0),
    ('status_source_ns',7_000_000_000),('gate_source_ns',7_000_000_001),
    ('first_begin_ns',6_999_999_999),('transport_delay_ns',0),
    ('activation_source_ns',7_100_000_000),('first_active_begin_ns',7_100_000_000),
    ('gate_delay_ns',0),('duplicate_count',-1),('receipt_ros_ns',None),
    ('model_name','unknown_actor')])
def test_status_counterexamples_cannot_qualify(key,value):
    data=rows();data[0][key]=value
    assert not audit(data)['pass_']


def test_missing_event_and_mutated_latch_are_rejected():
    assert not audit([])['pass_']
    assert not audit(rows()+rows(40_000_000))['pass_']


def test_static_world_has_no_actor_event_requirement():
    assert event.audit_status([],{},None,8_000_000_000)['pass_']


@pytest.mark.parametrize('stamp',[0,9_000_000_000,7_202_000_000])
def test_review_counterexamples_missing_future_or_delayed_first_active(stamp):
    data=rows();data[0]['first_active_begin_ns']=stamp
    assert not audit(data)['pass_']


def test_legal_same_begin_consume_then_active_messages_are_retained():
    consumed=rows()[0];consumed.update(status_source_ns=7_020_000_000,first_active_begin_ns=0)
    at_activation=rows()[0];at_activation['status_source_ns']=7_200_000_000
    assert audit([consumed,at_activation]+rows())['pass_']
    immediate=copy.deepcopy(consumed)
    immediate.update(first_begin_ns=7_200_000_000,transport_delay_ns=0,
        gate_source_ns=7_200_000_000,activation_source_ns=7_200_000_000,gate_delay_ns=0,
        status_source_ns=7_200_000_000)
    active=copy.deepcopy(immediate);active['first_active_begin_ns']=7_200_000_000
    later=copy.deepcopy(active);later['status_source_ns']=8_000_000_000
    expected=event.plugin_contract(plugin());expected['gate_delay_ns']=0
    assert event.audit_status([immediate,active,later],{'actor_1':expected},
                              {'source_ns':7_200_000_000},8_000_000_000)['pass_']


def test_first_active_latch_cannot_clear_or_change():
    first=rows()[0];first['status_source_ns']=7_800_000_000
    second=rows()[0];second['first_active_begin_ns']=0
    assert not audit([first,second])['pass_']


@pytest.mark.parametrize('change',[None,'wrong_gate_owner','missing_actor_subscription','wrong_status_owner'])
def test_actual_registration_gate_authority(change):
    system=[[[event.GATE_TOPIC,['/astra_navigation_coordinator']],
             [event.status_topic('actor_1'),['/gazebo']]],
            [[event.GATE_TOPIC,['/gazebo','/fixture_bag']]],[]]
    if change=='wrong_gate_owner':system[0][0][1].append('/foreign')
    if change=='missing_actor_subscription':system[1][0][1].remove('/gazebo')
    if change=='wrong_status_owner':system[0][1][1]=['/astra_navigation_collector']
    assert event.graph_audit(system,['actor_1'],'/astra_navigation_coordinator')['pass_'] is (change is None)
