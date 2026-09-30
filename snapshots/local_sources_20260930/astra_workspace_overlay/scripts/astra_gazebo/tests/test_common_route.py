"""Offline map/path consumption and conservative geometric-guide checks."""
import importlib.util
import json
import math
from pathlib import Path
import sys
import types
import io
import hashlib

import numpy as np
import pytest
from shapely.geometry import Point,LineString,box
from shapely.ops import unary_union
import yaml

SCRIPTS=Path(__file__).resolve().parents[1];sys.path.insert(0,str(SCRIPTS))
from common_route import SharedStaticMap,CommonRoute,CommonRouteInputAdapter,plan_grid_route,load_bundle,build_bundle
PROVIDER=SCRIPTS.parents[1]/'src/simulation/m2_gazebo/scripts/astra_common_route_provider.py'
spec=importlib.util.spec_from_file_location('provider_offline',PROVIDER)
provider=importlib.util.module_from_spec(spec);spec.loader.exec_module(provider)


def make_grid(size=80,resolution=.1,obstacles=()):
    cells=np.full((size,size),254,dtype=np.uint8);cells[[0,-1],:]=205;cells[:,[0,-1]]=205
    for r0,r1,c0,c1 in obstacles:cells[r0:r1,c0:c1]=0
    pgm=('P5\n%d %d\n255\n'%(size,size)).encode()+cells[::-1].tobytes()
    metadata=yaml.safe_dump({'resolution':resolution,'origin':[0,0,0],'negate':0,'occupied_thresh':.65,'free_thresh':.196,'mode':'trinary'}).encode()
    return SharedStaticMap(pgm,metadata),pgm,metadata


def test_normalized_map_unknown_is_identically_blocked_for_all_consumers():
    grid,_,_=make_grid(obstacles=[(20,25,30,35)])
    assert grid.occupancy[0,0]==-1 and grid.blocking_occupancy[0,0]==100
    assert grid.blocking_occupancy[22,32]==100 and grid.blocking_occupancy[10,10]==0
    assert grid.clearance((-.1,1))==-math.inf and grid.clearance((.05,.05))<0


def test_distance_lower_bound_against_exact_blocked_cell_union():
    grid,_,_=make_grid(size=30,obstacles=[(9,12,8,14),(18,24,21,23)])
    obstacles=unary_union([box(c*.1,r*.1,(c+1)*.1,(r+1)*.1) for r,c in np.argwhere(grid.pixels!=254)])
    rng=np.random.default_rng(4481)
    for xy in rng.uniform(.01,2.99,(500,2)):
        assert grid.clearance(xy)<=Point(xy).distance(obstacles)+1e-12
    for endpoints in rng.uniform(.01,2.99,(80,2,2)):
        assert grid.segment_clearance(*endpoints)<=LineString(endpoints).distance(obstacles)+1e-12


def test_astar_detour_really_uses_static_map_and_preserves_clearance():
    grid,_,_=make_grid(obstacles=[(30,50,35,45)])
    points,audit=plan_grid_route(grid,(1.,4.),(7.,4.),.35)
    assert len(points)>2 and any(abs(p[1]-4.)>1 for p in points)
    assert audit['minimum_segment_clearance_lower_bound_m']>=.35
    assert plan_grid_route(grid,(1.,4.),(7.,4.),.35)[0]==points


@pytest.mark.parametrize('case',['blocked_start','blocked_goal','disconnected','outside'])
def test_impossible_route_fails_without_carving(case):
    grid,_,_=make_grid(obstacles=[(1,79,35,45)]);start,goal=(1.,4.),(7.,4.)
    if case=='blocked_start':start=(4.,4.)
    if case=='blocked_goal':goal=(4.,4.)
    if case=='outside':start=(-1.,4.)
    before=grid.pixels.copy()
    with pytest.raises(ValueError):plan_grid_route(grid,start,goal,.3)
    assert np.array_equal(before,grid.pixels)


def test_polyline_projection_lookahead_and_corner_preservation():
    route=CommonRoute([(1,1),(3,1),(3,4)])
    projection=route.project((3.2,2.))
    assert projection['arc_m']==pytest.approx(3.) and projection['lateral_m']==pytest.approx(-.2)
    assert route.sample(3.5)==pytest.approx((3,2.5,math.pi/2))
    assert (3.,1.) in route.resample(.7)
    assert route.project((3,1))['segment']==0


def test_curved_route_progress_is_not_world_x_progress_and_map_is_consumed():
    grid,_,_=make_grid(obstacles=[(30,40,30,40)])
    adapter=CommonRouteInputAdapter(grid,CommonRoute([(1,1),(2,1),(2,6)]),(0,0,0))
    values=adapter.candidate_metrics([[(2,2,0),(2,3,0)],[(2,2,0),(3.5,3.5,0)]],.2)
    assert values[0]['route_progress_m']==pytest.approx(1.)
    assert values[0]['minimum_static_polyline_clearance_lower_bound_m']>0
    assert values[1]['minimum_static_polyline_clearance_lower_bound_m']<0


def test_nonidentity_map_odom_transform_and_rear_twist_are_real():
    grid,_,_=make_grid();adapter=CommonRouteInputAdapter(grid,CommonRoute([(2,2),(2,6)]),(2,2,math.pi/2))
    state=adapter.state((1,0,0),(1,.2,.4),.5,.65)
    assert state['map_base_pose']==pytest.approx((2,3,math.pi/2))
    assert state['map_rear_pose']==pytest.approx((2,2.675,math.pi/2))
    assert state['rear_body_twist']==pytest.approx((1,.07,.4))
    assert state['lookahead_odom_pose']==pytest.approx((1.5,0,0),abs=1e-12)


@pytest.mark.parametrize('case',['valid_start','advanced_start','frame','new_goal','goal_yaw','far_start','nonfinite'])
def test_provider_fixed_path_requests_do_not_replan(case):
    route=CommonRoute([(1,1),(3,1),(3,4)])
    data={'frame_id':'map','goal_xy_yaw':[3,4,1.]};start=(1,1);goal=(3,4,1.);frame='map'
    if case=='advanced_start':start=(3,3)
    if case=='frame':frame='odom'
    if case=='new_goal':goal=(3,4.1,1.)
    if case=='goal_yaw':goal=(3,4,1.1)
    if case=='far_start':start=(7,7)
    if case=='nonfinite':start=(math.nan,1)
    error=provider.request_error(data,route,start,frame,goal,'map',2.)
    assert (error is None)==(case in ('valid_start','advanced_start'))
    assert route.points==((1.,1.),(3.,1.),(3.,4.))


def test_bundle_identity_and_immutability(tmp_path):
    grid,pgm,metadata=make_grid();source=tmp_path/'source';source.mkdir();out=tmp_path/'out'
    (source/'map.pgm').write_bytes(pgm);(source/'map.yaml').write_bytes(metadata)
    request={'frame_id':'map','map_image_sha256':grid.image_sha256,'map_yaml_sha256':grid.yaml_sha256,
             'policy_start_xy_yaw':[1,4,0],'goal_xy_yaw':[7,4,0],'map_from_odom_xy_yaw':[.5,4,0]}
    (source/'route_request.json').write_text(json.dumps(request))
    data=build_bundle(source,out,.35);loaded_grid,route,loaded=load_bundle(out)
    assert loaded['path_points_sha256']==data['path_points_sha256'] and route.length==pytest.approx(6.)
    assert loaded['ackermann_route_feasibility_qualified']is False
    assert loaded_grid.image_sha256==grid.image_sha256
    with pytest.raises(FileExistsError):build_bundle(source,out,.35)
    text=(out/'common_route.json').read_text();(out/'common_route.json').write_text(text.replace('"start_xy_yaw": [\n    1,','"start_xy_yaw": [\n    2,'))
    with pytest.raises(ValueError,match='endpoints'):load_bundle(out)


def test_all_six_actual_exports_have_map_derived_routes():
    root=SCRIPTS.parents[2]/'audits/common_route_components/samples'
    bundles=sorted(root.glob('*/common_route.json'));assert len(bundles)==6
    for path in bundles:
        grid,route,data=load_bundle(path.parent)
        assert all(grid.segment_clearance(a,b)>=data['planning_clearance_radius_m']-1e-9 for a,b in zip(route.points,route.points[1:]))
        assert data['actor_state_used']is False and data['frozen_R2_consumes_this']is False


def test_plugin_bridge_has_no_hidden_replanning_or_endpoint_insertion():
    source=(SCRIPTS.parents[1]/'src/simulation/m2_gazebo/src/astra_common_route_global_planner.cpp').read_text()
    assert 'plan = call.response.plan.poses' in source
    assert 'push_back' not in source and 'Navfn' not in source and 'createPlan' not in source
    # Static source check supplements, and does not replace, later build/live tests.
    assert 'PLUGINLIB_EXPORT_CLASS' in source


def test_provider_real_ros_message_serialization_with_offline_fake_transport(monkeypatch):
    # Generated message types are used; all ROS transport/initialization is
    # replaced before main() runs. No ROS master or process is started.
    monkeypatch.syspath_prepend('/opt/ros/noetic/lib/python3/dist-packages')
    import genpy
    from nav_msgs.srv import GetPlanRequest
    directory=next((SCRIPTS.parents[2]/'audits/common_route_components/samples').glob('*static_avoidance*'))
    data=json.loads((directory/'common_route.json').read_bytes())
    params={'/use_sim_time':True,'/m2_gazebo/simulation_only':True,'~bundle':str(directory),
            '~bundle_sha256':hashlib.sha256((directory/'common_route.json').read_bytes()).hexdigest()}
    publishers={};services={};fake=types.ModuleType('rospy')
    class Publisher:
        def __init__(self,topic,message_type,**kwargs):self.messages=[];publishers[topic]=self
        def publish(self,message):
            stream=io.BytesIO();message.serialize(stream)
            assert stream.getvalue();self.messages.append(message)
    fake.init_node=lambda name:None;fake.get_param=lambda name,default=None:params.get(name,default)
    fake.Publisher=Publisher;fake.Service=lambda name,kind,callback:services.setdefault(name,callback)
    fake.ServiceException=RuntimeError;fake.Time=types.SimpleNamespace(now=lambda:genpy.Time(4,2000000));fake.spin=lambda:None
    monkeypatch.setitem(sys.modules,'rospy',fake);monkeypatch.setitem(sys.modules,'tf2_ros',types.ModuleType('tf2_ros'))
    monkeypatch.setenv('ROS_MASTER_URI','http://127.0.0.1:14820');monkeypatch.setenv('THESIS_WS',str(SCRIPTS.parents[1]))
    provider.main()
    assert set(services)=={'/static_map','/astra/common_route/get_plan'}
    assert set(publishers)=={'/map','/map_metadata','/astra/common_route/path','/astra/common_route/identity','/astra/common_route/served_path'}
    static=publishers['/map'].messages[0];assert static.data[0]==100 and set(static.data)=={0,100}
    request=GetPlanRequest();request.start.header.frame_id=request.goal.header.frame_id='map'
    request.goal.pose.position.x,request.goal.pose.position.y=data['goal_xy_yaw'][:2]
    request.goal.pose.orientation.z=math.sin(data['goal_xy_yaw'][2]/2);request.goal.pose.orientation.w=math.cos(data['goal_xy_yaw'][2]/2)
    for point in (data['points_xy'][0],data['points_xy'][len(data['points_xy'])//2]):
        request.start.pose.position.x,request.start.pose.position.y=point
        response=services['/astra/common_route/get_plan'](request)
        assert [[p.pose.position.x,p.pose.position.y] for p in response.plan.poses]==data['points_xy']
    request.goal.pose.position.x+=.01
    with pytest.raises(RuntimeError,match='goal_changed'):services['/astra/common_route/get_plan'](request)
