"""M08 real core/IO/worker transactions, no ROS or new weights."""
from pathlib import Path
import sys,math,time,threading
import numpy as np
import pytest
P=Path(__file__).resolve().parents[1];sys.path.insert(0,str(P))
from test_common_route import make_grid
from common_route import CommonRoute
from route_sensor_buffer import ScanFrame
from route_candidate_core import RouteAwareRuntime
from route_candidate_core_m08 import RouteTracksRuntime,OBSERVATION_SCHEMA
from route_tracks_actor_adapter_m08 import DeadlineActor,ActorDeadlineExceeded,FEATURE_SHAPES
from route_tracks_sensor_io_m08 import TrackSensorBuffer,AwaitingCausalOdometry
S=P.parents[2];URDF=(S/'runs/passive_sweep_q05_static/robot_description_runtime.xml').read_text();T=2_000_000_000

def actual(t=T,pose=(3.,7.,0.),twist=(.8,0.,0.)):
    return dict(stamp_ns=t,pose_map=pose,body_twist=twist,roll_pitch=(0.,0.),joints=dict(front_left_steer_joint=0.,front_right_steer_joint=0.))
def frame(t=T,points=()):return ScanFrame(t,np.full(72,8.),np.asarray(points,dtype=float).reshape(-1,2),(3,7,0))
def accept(c,f,now=None):return c.accept_scan(f,f.stamp_ns,actual(f.stamp_ns),f.stamp_ns if now is None else now)
def runtime(obstacles=()):
    grid,_,_=make_grid(size=140,obstacles=list(obstacles));return RouteTracksRuntime(grid,CommonRoute([(2,7),(12,7)]),URDF)


def test_generator_physical_state_actions_exact_frozen_function_objects_and_outputs():
    new=runtime();old=RouteAwareRuntime(new.grid,new.route,URDF)
    for name in ('state','project','step','rollout','goal_reached'):
        assert getattr(type(new),name) is getattr(type(old),name)
    state=actual(twist=(.9,.08,.1));state['joints'].update(front_left_steer_joint=.2,front_right_steer_joint=.16)
    for action in range(8):
        for backup in (False,True):assert new.rollout(new.state(state),action,backup)==old.rollout(old.state(state),action,backup)
    assert new.identity['actions']==old.identity['actions']
    assert new.identity['feature_shapes']==old.identity['feature_shapes']
    assert new.identity['schema']!=old.identity['schema']


def test_no_dynamic_support_all_features_and_masks_match_old_exactly():
    new=runtime();old=RouteAwareRuntime(new.grid,new.route,URDF);f=frame();accept(new,f)
    a=new.build(actual(),f,np.full((6,72),8.),1.,T);b=old.build(actual(),f,np.full((6,72),8.),1.,T)
    assert a['previews']==b['previews'] and a['mask']==b['mask']
    for name in a['observation']:np.testing.assert_array_equal(a['observation'][name],b['observation'][name])
    np.testing.assert_array_equal(a['backup_clearances'],b['backup_clearances'])
    assert a['query_count']>248


def test_dynamic_support_features_keep_layout_and_never_relax_old_backups():
    new=runtime();old=RouteAwareRuntime(new.grid,new.route,URDF);f=frame(points=[(5.5,8),(5.52,8.01),(5.54,8)]);accept(new,f)
    a=new.build(actual(),f,np.full((6,72),8.),1.,T);b=old.build(actual(),f,np.full((6,72),8.),1.,T)
    assert a['previews']==b['previews'];assert np.all(np.asarray(a['backup_clearances'])<=np.asarray(b['backup_clearances'])+1e-12)
    assert all(not x or y for x,y in zip(a['mask'],b['mask']))
    for name in ('scans','robot'):np.testing.assert_array_equal(a['observation'][name],b['observation'][name])
    np.testing.assert_array_equal(a['observation']['candidates'][:,[0,1,2,4,5,6]],b['observation']['candidates'][:,[0,1,2,4,5,6]])


def test_stop_is_model_checked_not_physical_certification_and_current_scan_revalidates():
    c=runtime();f=frame();accept(c,f)
    assert c.command(actual(),f,3,T)[2]==3
    f=frame(T+50_000_000,[(3.3,7)]);accept(c,f)
    command=c.command(actual(f.stamp_ns),f,3,f.stamp_ns)
    assert command[:3]==(0.,0.,0) and command[3]=='backup_revalidation_failed'
    result=c.transaction(actual(f.stamp_ns),f,np.ones((6,72)),0,f.stamp_ns,decide=False,committed_action=3,source_clock=lambda:f.stamp_ns)
    assert result['trace']['certified_stop'] is False and result['trace']['backup_stop_model_passed'] is False
    assert result['committed_action']==0


def test_public_build_and_command_cannot_predate_scan_acceptance():
    c=runtime();f=frame();accept(c,f,T+100_000_000)
    with pytest.raises((ValueError,RuntimeError)):c.build(actual(),f,np.ones((6,72)),0,T)
    with pytest.raises((ValueError,RuntimeError)):c.command(actual(),f,0,T)


def test_wrong_unaccepted_current_scan_rejected():
    c=runtime();accept(c,frame())
    with pytest.raises(ValueError):c.build(actual(),frame(T+1),np.ones((6,72)),0,T+1)


def test_actual_io_waits_for_causal_odom_does_not_consume_scan_or_invent_history():
    c=runtime();b=TrackSensorBuffer(c);ranges=np.full(720,np.inf)
    def scan(stamp,now=None):return b.scan(stamp,stamp,ranges,-math.pi,2*math.pi/720,.1,30.,(3,7,0),stamp if now is None else now)
    with pytest.raises(AwaitingCausalOdometry):scan(T)
    assert b.fresh is None and c.tracker.snapshot is None
    b.odom(T+1,(3,7,0),(.8,0,0))
    with pytest.raises(AwaitingCausalOdometry):scan(T)
    b.odom(T-1,(3,7,0),(.8,.1,.02));scan(T)
    assert c.tracker.snapshot.odom_source_ns==T-1 and c.tracker.snapshot.odom_body_twist==( .8,.1,.02)
    for i in range(1,6):
        stamp=T+i*200_000_000;b.odom(stamp,(3,7,0),(.8,0,0));scan(stamp)
    history,buckets,sources=b.model_history()
    assert len(set(sources))==6 and b.accepted_scan_count==6 and c.tracker.source_ns==sources[-1]


def test_io_exact_tf_rejection_is_not_consumed_or_freshened():
    c=runtime();b=TrackSensorBuffer(c);b.odom(T,(3,7,0),(0,0,0))
    with pytest.raises(ValueError):b.scan(T,T+1,np.full(720,np.inf),-math.pi,2*math.pi/720,.1,30.,(3,7,0),T)
    assert b.fresh is None and c.tracker.snapshot is None


def test_async_actor_timeout_and_late_result_cannot_cross_to_next_request():
    release=threading.Event();started=threading.Event()
    def blocked(_):started.set();release.wait(.5);return 7
    actor=DeadlineActor(blocked);begin=time.perf_counter()
    with pytest.raises(ActorDeadlineExceeded):actor({'n':1})
    elapsed=time.perf_counter()-begin
    assert elapsed>=.018 and elapsed<.2 and started.is_set()
    with pytest.raises(ActorDeadlineExceeded):actor({'n':2})
    release.set();time.sleep(.01)
    actor.actor=lambda _:2
    assert actor({'n':3})==2


def test_actor_failure_masking_and20ms_budget_are_actual_code():
    c=runtime();f=frame();accept(c,f);built=c.build(actual(),f,np.full((6,72),8.),0,T)
    c.actor=DeadlineActor(lambda _:99)
    selected,proposed,reason,_=c.select(built);assert reason=='masked_action_drop' and proposed==99 and built['mask'][selected]
    release=threading.Event();c.actor=DeadlineActor(lambda _:release.wait(.3) or 3)
    selected,proposed,reason,latency=c.select(built)
    release.set();assert reason=='inference_deadline_drop' and latency>=19 and built['mask'][selected]


def test_transaction50ms_exact_gate_and_expired_proposal_not_committed():
    c=runtime();f=frame();accept(c,f)
    # Actual core command; clock injection isolates the exact frozen wall gate.
    for duration,drop in ((50_000_000,False),(50_000_001,True)):
        result=c.transaction(actual(),f,np.ones((6,72)),0,T,decide=False,committed_action=3,source_clock=lambda:T,begin_perf_ns=0,clock_ns=lambda:duration)
        assert result['trace']['transaction_deadline_dropped']==drop
        if drop:assert result['command'][:3]==(0.,0.,0) and result['committed_action']==0


def test_expired_source_after_planning_drops_command_and_keeps_reason():
    c=runtime();f=frame();accept(c,f)
    result=c.transaction(actual(),f,np.ones((6,72)),0,T,decide=False,committed_action=3,source_clock=lambda:T+200_000_001)
    assert result['command'][:3]==(0.,0.,0)
    assert result['trace']['stop_reason']=='prepublish_input_health_drop'


def test_new_namespace_rejects_old_checkpoint_even_identical_dimensions():
    c=runtime();metadata=dict(observation_schema='astra_route_observation_v1',candidate_semantics_sha256=c.semantics_sha256,same_generator_predictor_verifier_used_in_training=True,feature_shapes=FEATURE_SHAPES)
    with pytest.raises(ValueError):RouteTracksRuntime(c.grid,c.route,URDF,actor=lambda _:0,checkpoint_metadata=metadata)
