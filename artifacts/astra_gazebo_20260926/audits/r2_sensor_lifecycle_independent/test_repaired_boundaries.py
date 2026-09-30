"""Desired-behavior second-TF failure checks for the separately revised adapter."""
import importlib.util
import json
from pathlib import Path

S=Path(__file__).resolve().parents[2];W=S/'workspace'
spec=importlib.util.spec_from_file_location('revised_author_support',W/'scripts/astra_gazebo/tests/test_frozen_r2_sensor_lifecycle.py')
support=importlib.util.module_from_spec(spec);spec.loader.exec_module(support)
actual_episode=support.actual_episode


def test_second_lookup_failure_never_qualifies_stale_committed_history(actual_episode):
    f=actual_episode
    f.lifecycle.on_status(dict(schema=support.PROTOCOL,run_id='offline_r2_lifecycle',seed=260926026,method=support.METHOD,phase='bootstrap'))
    for i in range(6):support.feed(f,1_000_000_000+i*220_000_000)
    assert f.episode.ready()
    original=f.episode.raw_tf_buffer.lookup_transform;counts={};faults=[]
    def second_fails(target,source,stamp,*args):
        ns=stamp.to_nsec()
        if ns:
            counts[ns]=counts.get(ns,0)+1
            if counts[ns]%2==0:raise LookupError('independent recurring second source-TF failure')
        return original(target,source,stamp,*args)
    f.episode.raw_tf_buffer.lookup_transform=second_fails
    for ns in (2_320_000_000,2_540_000_000,2_760_000_000):
        f.clock['ns']=ns
        try:
            f.episode._odom(support.odom(ns));f.episode._scan(support.scan(ns));f.episode.drain()
        except support.LifecycleFault as error:faults.append(str(error))
    ready=None
    try:ready=f.episode.ready()
    except support.LifecycleFault as error:faults.append(str(error))
    row=dict(ready=ready,exceptions=faults,lifecycle_fault=f.lifecycle.fault_reason,
        last_committed_ns=f.ingress.accepted_scan_ns[-1],now_ns=f.clock['ns'],
        pending=len(f.ingress.queues['scan']),parent_scan_failures=f.episode.scan_transform_failure_count)
    (Path(__file__).parent/'repaired_second_tf_observation.json').write_text(json.dumps(row,indent=2)+'\n')
    assert ready is not True
    assert faults or f.lifecycle.fault_reason is not None or row['pending']>0
    assert row['last_committed_ns']==2_100_000_000


def test_first_lookup_transient_recovers_at_original_exact_stamp(actual_episode):
    f=actual_episode;f.clock['tf_fail']=True
    support.feed(f,1_000_000_000)
    assert not f.ingress.accepted_scan_ns and len(f.ingress.queues['scan'])==1
    f.clock['tf_fail']=False;f.clock['ns']=1_002_000_000;f.episode.drain()
    assert list(f.ingress.accepted_scan_ns)==[1_000_000_000]
    assert f.episode.last_runtime_scan_stamp_s==1.0
    assert len(f.ingress.queues['scan'])==0


def test_five_valid_commits_plus_failed_sixth_cannot_arm(actual_episode):
    f=actual_episode
    f.lifecycle.on_status(dict(schema=support.PROTOCOL,run_id='offline_r2_lifecycle',seed=260926026,method=support.METHOD,phase='bootstrap'))
    for i in range(5):support.feed(f,1_000_000_000+i*220_000_000)
    original=f.episode.raw_tf_buffer.lookup_transform;count=[0]
    def second_fails(target,source,stamp,*args):
        if stamp.to_nsec()==2_100_000_000:
            count[0]+=1
            if count[0]%2==0:raise LookupError('failed sixth history transform')
        return original(target,source,stamp,*args)
    f.episode.raw_tf_buffer.lookup_transform=second_fails
    try:support.feed(f,2_100_000_000)
    except support.LifecycleFault:pass
    assert len(f.ingress.accepted_scan_ns)==5
    try:assert not f.episode.ready()
    except support.LifecycleFault:pass
