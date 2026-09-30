"""Independent actual-method boundary probes; no ROS services/master."""
import importlib.util
import json
from pathlib import Path

S=Path(__file__).resolve().parents[2]
W=S/'workspace'
spec=importlib.util.spec_from_file_location('author_test_support',W/'scripts/astra_gazebo/tests/test_frozen_r2_sensor_lifecycle.py')
support=importlib.util.module_from_spec(spec);spec.loader.exec_module(support)
actual_episode=support.actual_episode


def test_second_exact_tf_lookup_failure_is_dropped_but_health_stays_ready(actual_episode):
    f=actual_episode
    f.lifecycle.on_status(dict(schema=support.PROTOCOL,run_id='offline_r2_lifecycle',seed=260926026,method=support.METHOD,phase='bootstrap'))
    for i in range(6):support.feed(f,1_000_000_000+i*220_000_000)
    assert f.episode.ready()
    old_history=list(f.ingress.accepted_scan_ns)
    original=f.episode.raw_tf_buffer.lookup_transform
    calls={}
    def intermittent(target,source,stamp,*args):
        ns=stamp.to_nsec()
        if ns:
            calls[ns]=calls.get(ns,0)+1
            if calls[ns]==2:raise LookupError('exact TF disappears between precheck and parent callback')
        return original(target,source,stamp,*args)
    f.episode.raw_tf_buffer.lookup_transform=intermittent
    for ns in (2_320_000_000,2_540_000_000,2_760_000_000):support.feed(f,ns)
    result=dict(ready=f.episode.ready(),fault=f.lifecycle.fault_reason,
        history_unchanged=list(f.ingress.accepted_scan_ns)==old_history,
        last_committed_source_ns=f.ingress.accepted_scan_ns[-1],now_ns=f.clock['ns'],
        parent_scan_failures=f.episode.scan_transform_failure_count,
        pending_scan_count=len(f.ingress.queues['scan']),
        health_failures=f.episode.input_health(f.clock['ns']/1e9).failures(.20))
    (Path(__file__).parent/'second_tf_lookup_counterexample.json').write_text(json.dumps(result,indent=2)+'\n')
    # Characterizes a discovered gap; not a desired-behavior regression test.
    assert result['ready'] and result['history_unchanged'] and result['fault'] is None
    assert result['now_ns']-result['last_committed_source_ns']>200_000_000
    assert result['parent_scan_failures']==3 and result['pending_scan_count']==0
    assert not result['health_failures']


def test_shutdown_of_old_truth_cannot_change_terminal_or_policy_pose(actual_episode):
    f=actual_episode;support.feed(f,7_000_000_000);f.lifecycle.on_gate(support.gate());f.lifecycle.active=True
    class ForbiddenTruth:
        def __getattribute__(self,name):raise AssertionError('legacy truth was read')
    f.episode.model_states=ForbiddenTruth();f.episode.model_index=ForbiddenTruth()
    assert f.episode.robot_world_pose()==dict(x_m=-4.,y_m=0.,yaw_rad=0.)
    assert not f.episode.goal_state(dict(x_m=5.,y_m=0.,yaw_rad=0.))[0]
    f.lifecycle.on_status(support.terminal('timeout'))
    try:f.episode.goal_state(f.episode.robot_world_pose())
    except support.SharedTerminal as error:assert error.row['phase']=='timeout'
    else:raise AssertionError('timeout must unwind without success')


def test_source_gap_on_first_lookup_remains_queued_then_faults(actual_episode):
    f=actual_episode
    for i in range(6):support.feed(f,1_000_000_000+i*220_000_000)
    f.clock['tf_fail']=True;support.feed(f,2_320_000_000)
    assert len(f.ingress.queues['scan'])==1
    f.clock['ns']=2_520_000_001
    try:f.episode.drain()
    except support.LifecycleFault as error:assert 'expired' in str(error)
    else:raise AssertionError('unavailable exact scan TF must not be discarded')
