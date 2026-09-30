"""Independent actual-message and fake-transport B03 boundary fixtures."""
from pathlib import Path
from unittest.mock import patch
import hashlib
import importlib.util
import io
import json
import math
import os
import sys
import types

import numpy as np
import pytest

S = Path(__file__).resolve().parents[2]
W = S / 'workspace'
P = W / 'scripts/astra_gazebo'
G = W / 'src/simulation/m2_gazebo'
sys.path[:0] = [str(P), str(P/'tests'), '/opt/ros/noetic/lib/python3/dist-packages']
import genpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from common_route import CommonRoute
from baseline_rear_runtime_b03 import RearOdometryStream, RearFixedRoute, SCHEMA, METHODS, goal_tolerance_contract
# Reuse only the generated-message transport containers, not author's assertions
# or scenario/exercise methods. The actual node main() remains unmodified.
from test_baseline_rear_nodes_b03 import transport_fixture


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


adapter = load('b03_independent_adapter', G/'scripts/astra_baseline_rear_adapter_b03.py')


def quaternion(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r/2), math.sin(r/2), math.cos(p/2), math.sin(p/2), math.cos(y/2), math.sin(y/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


def rotation(r, p, y):
    rx=np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]])
    ry=np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
    rz=np.array([[math.cos(y),-math.sin(y),0],[math.sin(y),math.cos(y),0],[0,0,1]])
    return rz@ry@rx


def message(ns=1_000_000_000, rpy=(.02,-.02,.7)):
    m=Odometry();m.header.stamp=genpy.Time(ns//10**9,ns%10**9)
    m.header.frame_id='odom';m.child_frame_id='base_link'
    m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=(1.1,-2.3,.04)
    q=m.pose.pose.orientation;q.x,q.y,q.z,q.w=quaternion(*rpy)
    m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z=(.9,.2,.1)
    m.twist.twist.angular.x,m.twist.twist.angular.y,m.twist.twist.angular.z=(.3,.4,.5)
    c=np.arange(36,dtype=float).reshape(6,6)/100.;c=c@c.T+np.eye(6)
    m.pose.covariance=c.ravel().tolist();m.twist.covariance=(2*c).ravel().tolist()
    return m


def bytes_of(m):
    out=io.BytesIO();m.serialize(out);return out.getvalue()


@pytest.mark.parametrize('rpy',[(.02,-.02,.7),(-.03,.01,-1.2),(0.,.05,2.9)])
def test_serialized_odom_full_rigid_transform_and_nonidentity_map(rpy):
    raw=message(rpy=rpy);before=bytes_of(raw);published=[];stream=RearOdometryStream()
    adapter.consume_message(stream,raw,lambda:1_000_000_000,published.append,Odometry,
                            lambda ns:genpy.Time(ns//10**9,ns%10**9))
    assert bytes_of(raw)==before and len(published)==1
    result=published[0];offset=np.array([-.325,0.,0.]);R=rotation(*rpy)
    expected=np.array([1.1,-2.3,.04])+R@offset
    actual=np.array([result.pose.pose.position.x,result.pose.pose.position.y,result.pose.pose.position.z])
    np.testing.assert_allclose(actual,expected,rtol=0,atol=1e-14)
    # A separate nonidentity map->odom rigid yaw transform commutes correctly;
    # body twist must NOT be rotated by this map-frame conversion.
    T=rotation(0,0,-.8);origin=np.array([3.,4.,0.])
    np.testing.assert_allclose(origin+T@actual,origin+T@np.array([1.1,-2.3,.04])+T@R@offset,atol=1e-14)
    velocity=np.array([result.twist.twist.linear.x,result.twist.twist.linear.y,result.twist.twist.linear.z])
    np.testing.assert_allclose(velocity,np.array([.9,.2,.1])+np.cross([.3,.4,.5],offset),atol=1e-15)
    assert velocity[1]!=0. and result.header.stamp==raw.header.stamp
    assert result.header.frame_id=='odom' and result.child_frame_id=='rear_axle_kinematic'
    assert len(bytes_of(result))>0


def test_future_two_packet_queue_retains_each_source_and_publication_age():
    stream=RearOdometryStream();output=[]
    for ns in (1_002_000_000,1_004_000_000):
        adapter.consume_message(stream,message(ns),lambda:1_000_000_000,output.append,Odometry,
                                lambda x:genpy.Time(x//10**9,x%10**9))
    assert not output
    adapter.publish_pending(stream,lambda:1_004_000_000,output.append,Odometry,
                            lambda x:genpy.Time(x//10**9,x%10**9))
    assert [m.header.stamp.to_nsec() for m in output]==[1_002_000_000,1_004_000_000]
    assert stream.latest.available_ns==1_004_000_000 and stream.latest.received_ns==1_000_000_000


def test_partial_publication_failure_remains_fault_not_ready_evidence():
    stream=RearOdometryStream();output=[]
    for ns in (1_000_000_000,1_002_000_000):
        stream.ingest(adapter.message_row(message(ns)),999_000_000)
    clock=iter([1_200_000_000,1_200_000_000,1_202_000_001])
    with pytest.raises(ValueError,match='expired'):
        adapter.publish_pending(stream,lambda:next(clock),output.append,Odometry,
                                lambda x:genpy.Time(x//10**9,x%10**9))
    assert len(output)==1 and stream.fault_reason
    with pytest.raises(ValueError,match='permanent_fault'):
        stream.health(1_202_000_001)


def test_all_oriented_path_points_and_explicit_final_yaw_preserved():
    route=CommonRoute([(1.,2.),(3.,4.),(7.,3.)],-.9)
    data={'frame_id':'map','provider_returns_exact_full_polyline':True,'goal_xy_yaw':[7.,3.,-.9]}
    service=RearFixedRoute(route,data)
    expected=[]
    for i,(x,y) in enumerate(route.points):
        yaw=math.atan2(route.points[i+1][1]-y,route.points[i+1][0]-x) if i+1<len(route.points) else -.9
        expected.append((x-.325*math.cos(yaw),y-.325*math.sin(yaw),yaw))
    np.testing.assert_allclose(service.converted['rear_poses'],expected,rtol=0,atol=1e-14)
    goal=expected[-1]
    for header_frame in ('odom','/map'):
        with pytest.raises(ValueError,match='frame'):
            service.request(header_frame,(*expected[0][:2],0),quaternion(0,0,expected[0][2]),'map',(*goal[:2],0),quaternion(0,0,goal[2]))
    # Goal conversion must not assume global +x or recompute final chord yaw.
    assert not math.isclose(expected[-1][0],7.-.325)


def test_internal_goal_contains_base_goal_under_full_tilt_bound():
    contract=goal_tolerance_contract();goal=np.array([3.,-4.]);goal_yaw=.7
    rear_goal=goal-.325*np.array([math.cos(goal_yaw),math.sin(goal_yaw)])
    worst=0.
    for yaw_error in (-.1,.1):
        for roll,pitch in ((0.,.05),(0.,-.05),(.03,.02),(-.03,-.02)):
            for error_yaw in np.linspace(-math.pi,math.pi,17):
                rear=rear_goal+.15*np.array([math.cos(error_yaw),math.sin(error_yaw)])
                base=rear+.325*rotation(roll,pitch,goal_yaw+yaw_error)[:2,0]
                distance=float(np.linalg.norm(base-goal));worst=max(worst,distance)
                assert distance<=contract['maximum_base_xy_at_internal_goal_m']+1e-14
                assert distance<.25
    assert worst>.18


@pytest.mark.parametrize('method',METHODS)
def test_actual_coordinator_main_rotated_map_goal_and_source_terminal(method):
    bundle_path=S/'scenes/navigation_n01_clear_r03/common_bundle'
    angle=.7;c,s=math.cos(angle),math.sin(angle)
    def world(x,y=0.):return (2.+c*x-s*y,-3.+s*x+c*y)
    start=(*world(1.4),angle);goal=(*world(10.4),angle)
    data={'frame_id':'map','provider_returns_exact_full_polyline':True,'start_xy_yaw':list(start),
          'goal_xy_yaw':list(goal),'map_from_odom_xy_yaw':[2.,-3.,angle]}
    params={'/use_sim_time':True,'/m2_gazebo/simulation_only':True,'~run_id':'independent_rotated',
            '~seed':42,'~method':method,'~bundle':str(bundle_path),
            '~bundle_sha256':hashlib.sha256((bundle_path/'common_route.json').read_bytes()).hexdigest()}
    fake,state,pubs,subs,services=transport_fixture('/astra_navigation_coordinator',params)
    state.update(gate_tick=None,first_goal_source=None)
    node=load('b03_independent_coordinator_'+method,G/'scripts/astra_navigation_coordinator_b03.py')
    goals=[];arms=[]
    actions=types.ModuleType('actionlib')
    class Client:
        def __init__(self,*args):pass
        def wait_for_server(self):return True
        def send_goal(self,request,active_cb,done_cb):
            goals.append(request);state['gate_tick']=state['ticks'];active_cb();done_cb(3,None)
        def cancel_all_goals(self):pass
    actions.SimpleActionClient=Client
    graph=types.ModuleType('rosgraph')
    graph.Master=lambda _:types.SimpleNamespace(getSystemState=lambda:([('/cmd_vel_navigation',['/astra_navigation_coordinator']),('/astra/method_cmd',['/move_base'])],[],[]))
    class Thread:
        def __init__(self,target,daemon):self.target=target
        def start(self):self.target()
    def advance(_):
        state['ticks']+=1;state['ns']=4_800_000_000+(state['ticks']-1)*20_000_000
        elapsed=max(0.,(state['ns']-5_000_000_000)/1e9);x=.8*elapsed;y=0.;yaw=0.;v=0. if elapsed==0 else .8
        if state['gate_tick'] is not None and state['ticks']>=state['gate_tick']+6:
            x,y,yaw,v=10.3,.1,.08,.4
            if state['first_goal_source'] is None:state['first_goal_source']=state['ns']-2_000_000
        m=message(state['ns']-2_000_000,(0.,0.,yaw));m.pose.pose.position.x=x;m.pose.pose.position.y=y;m.pose.pose.position.z=0.
        m.twist.twist.linear.x=v;m.twist.twist.linear.y=m.twist.twist.angular.z=0.
        subs['/odom'](m)
        status={'schema':SCHEMA,'run_id':'independent_rotated','seed':42,'method':method,'state':'ready',
                'ready':True,'status_source_ns':state['ns'],'latest_odom_source_ns':m.header.stamp.to_nsec()}
        subs['/astra/baseline_rear/status'](String(data=json.dumps(status)))
        if state['ticks']==2:arms.append(services['/astra/navigation/arm'](None))
        if state['gate_tick'] is not None:
            command=Twist();command.linear.x=.6;command.angular.z=.02;subs['/astra/method_cmd'](command)
    fake.is_shutdown=lambda:state['ticks']>=220 or (state['gate_tick'] is not None and state['ticks']>=state['gate_tick']+10)
    with patch.dict(sys.modules,{'rospy':fake,'rosgraph':graph,'actionlib':actions}),patch.dict(os.environ,{'ROS_MASTER_URI':'http://127.0.0.1:14820','THESIS_WS':str(W)}),patch('common_route.load_bundle',return_value=(None,CommonRoute([start[:2],goal[:2]],angle),data)),patch.object(node.threading,'Thread',Thread),patch.object(node.time,'sleep',advance),patch.object(node.time,'monotonic',side_effect=lambda:100+state['ticks']*.01),patch.object(node.time,'monotonic_ns',side_effect=lambda:10**11+state['ticks']*10**7):
        node.main()
    assert len(goals)==1 and [x.success for x in arms]==[True]
    position=goals[0].target_pose.pose.position
    np.testing.assert_allclose((position.x,position.y),np.array(goal[:2])-.325*np.array([c,s]),atol=1e-14)
    assert goals[0].target_pose.header.frame_id=='map'
    traces=[json.loads(m.data) for m in pubs['/astra/navigation/command_trace'].messages]
    assert traces[-1]['phase']=='complete' and traces[-1]['terminal_source_ns']==state['first_goal_source']
    assert traces[-1]['terminal_time_authority']=='first_actual_odom_header_meeting_base_goal'
    assert any(t['phase']=='active' and t['v']==.6 and t['w']==.02 for t in traces)
    private=json.loads(pubs['/astra/baseline_rear/private_action_result'].messages[0].data)
    assert private['shared_phase']=='active' and private['is_task_terminal_authority'] is False
