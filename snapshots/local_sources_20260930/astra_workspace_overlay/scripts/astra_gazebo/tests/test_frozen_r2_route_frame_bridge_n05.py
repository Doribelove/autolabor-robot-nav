"""Small pure coordinate fixtures; actual frozen consumer AST, no ROS/GPU."""
import ast
import copy
from dataclasses import replace
import hashlib
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest
from shapely.geometry import Polygon, box
from shapely.ops import unary_union
import yaml

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))
from frozen_r2_route_frame_bridge_n05 import *
from frozen_r2_sensor_lifecycle import authenticate_frozen, verify_static_union
from common_route import SharedStaticMap
W = SCRIPTS.parents[1]
S = W.parent
T = 6_000_000_000
META = SourceMetadata(T, T-2_000_000, T+2_000_000, 999)
TWIST = BodyTwist((.8, -.012, .002), (.001, -.002, .04))


def bridge(transform=SE2(), guide=((-4., 0.), (5., 0.)), reference=((-4., 0.), (5., 0.))):
    # Real odom does not need to be aligned with the route/world frame.
    return FrozenR2RouteFrameBridge(world_from_route=transform, world_from_odom=SE2(2., -3., -.4),
        spawn_world=transform.pose(SPAWN), gate_world=transform.pose(GATE), goal_world=transform.pose(GOAL),
        nominal_reference_world=tuple(transform.point(p) for p in reference),
        guide_world=tuple(transform.point(p) for p in guide), run_id='offline_frame_fixture', seed=1)


def pose(x=-2.4, y=.2, yaw=.12):
    return Pose3((x, y, .18), (0., 0., math.sin(yaw/2), math.cos(yaw/2)))


def scan(metadata=META):
    ranges = tuple(math.inf if i%13==0 else .2+i*.013 for i in range(720))
    return LaserSample(metadata, 'laser_link', ranges, -math.pi, math.pi, 2*math.pi/719,
                       .1, 30., .0001, .05, (1., 0., -0.))


@pytest.mark.parametrize('theta', [0., .3, math.pi/2, -math.pi/2, math.pi, -.87])
def test_rotated_translated_world_and_real_odom_both_recover_private_legacy_frame(theta):
    t = SE2(13., -7., theta); b = bridge(t); canonical = pose()
    world = canonical.transformed(t)
    real_odom = world.transformed(b.world_from_odom.inverse())
    for parent, source_pose in (('map', world), ('odom', real_odom)):
        original = OdometrySample(META, parent, 'base_link', source_pose, TWIST)
        actual = b.odometry(original, T+2_000_000)
        np.testing.assert_allclose((actual.pose.xyz[0]+SPAWN[0], actual.pose.xyz[1], actual.pose.planar()[2]), canonical.planar(), atol=4e-15, rtol=0)
        assert actual.metadata is META and actual.body_twist is TWIST
        assert actual.pose.xyz[2] == canonical.xyz[2]
        assert actual.parent_alias == 'odom' and actual.coordinate_scope == PRIVATE_SCOPE
        assert original.parent_frame == parent and original.pose is source_pose
    assert b.command(TWIST) is TWIST
    assert b.scan(scan(), T+2_000_000).ranges[0] == math.inf
    assert b.goal(t.pose(GOAL)) == GOAL


def test_full_measured_tilt_quaternion_left_rotation_and_inverse_not_yaw_replacement():
    from geometry import Pose3 as ReferencePose3
    q = (.1, -.2, .3, math.sqrt(.86))
    source = Pose3((2., -1., .6), q); t=SE2(5., 2., .8)
    transformed = source.transformed(t)
    rz=np.asarray([[math.cos(.8),-math.sin(.8),0.],[math.sin(.8),math.cos(.8),0.],[0.,0.,1.]])
    expected = rz@np.asarray(ReferencePose3(xyzw=q).rotation)
    np.testing.assert_allclose(ReferencePose3(xyzw=transformed.xyzw).rotation, expected, atol=5e-16, rtol=0)
    np.testing.assert_allclose(transformed.transformed(t.inverse()).xyzw, q, atol=2e-16, rtol=0)
    assert transformed.xyz[2] == .6


@pytest.mark.parametrize('kind', ['bool_source','float_source','negative_receipt','availability_before_source','availability_before_receipt','bool_real','nan_pose','bad_q'])
def test_strict_typed_metadata_and_geometry(kind):
    with pytest.raises(ValueError):
        if kind=='bool_source': replace(META, source_ns=True)
        elif kind=='float_source': replace(META, source_ns=float(T))
        elif kind=='negative_receipt': replace(META, receipt_steady_ns=-1)
        elif kind=='availability_before_source': replace(META, available_source_ns=T-1)
        elif kind=='availability_before_receipt': SourceMetadata(T,T+3,T+2,5)
        elif kind=='bool_real': SE2(True,0.,0.)
        elif kind=='nan_pose': Pose3((math.nan,0.,0.), (0.,0.,0.,1.))
        else: Pose3((0.,0.,0.), (0.,0.,0.,2.))


def test_availability_original_source_expiry_and_exact_scan_tf():
    b=bridge();local=scan();tf=TransformSample(META,'map','laser_link',pose())
    with pytest.raises(ValueError): b.laser_transform(tf,local,T+1_999_999)
    transformed=b.laser_transform(tf,local,T+200_000_000)
    assert transformed.metadata is META and b.scan(local,T+2_000_000) is local
    with pytest.raises(ValueError): b.laser_transform(tf,local,T+200_000_001)
    later=replace(META,source_ns=T+1,available_source_ns=T+2_000_000)
    with pytest.raises(ValueError): b.laser_transform(replace(tf,metadata=later),local,T+2_000_000)
    with pytest.raises(ValueError): b.odometry(OdometrySample(META,'unbound_frame','base_link',pose(),TWIST),T+2_000_000)
    with pytest.raises(ValueError): b.scan(replace(local,frame_id='map'),T+2_000_000)
    assert b.health_transform(tf,T+2_000_000).metadata.source_ns == T


@pytest.mark.parametrize('reference', [((-4.,0.),(0.,1.),(5.,0.)),((-4.,0.),(6.,0.)),((-4.,0.),(0.,0.),(-1.,0.),(5.,0.))])
def test_curved_rescaled_and_reversed_nominal_task_rejected(reference):
    with pytest.raises(ValueError): bridge(reference=reference)


def test_common_detour_is_separate_audit_only_not_rejected_as_task_length():
    detour=((-4.,0.),(-1.,.8),(2.,.8),(5.,0.))
    b=bridge(SE2(7.,-3.,1.2),guide=detour)
    assert b.guide_length_m>9 and b.guide_use=='audit_only' and not b.full_polyline_consumed
    np.testing.assert_allclose(b.guide_route_points,detour,atol=2e-15,rtol=0)
    # Actual existing N04 curved common guides are admissible with the fixed
    # nominal task, without claiming their polyline enters the R2 runtime.
    import csv
    rows=list(csv.DictReader((S/'scenes/navigation_n04_development_r01/diagnostic_blocks.csv').open()))
    count=0
    for row in rows:
        if row['family'] not in ('static_avoidance','occluded_emergence'):continue
        data=json.loads((Path(row['manifest_path']).parent/'common_bundle/common_route.json').read_text())
        actual=bridge(guide=tuple(tuple(p) for p in data['points_xy']))
        assert actual.guide_length_m>9.; count+=1
    assert count==4


def gate_event(b):
    return dict(schema='astra_common_navigation_v1',run_id=b.identity[0],seed=1,method=METHOD,
                source_ns=T,source='actual_odom_header_first_crossing',pose_map=list(b.world_from_route.pose((-3.999,.002,.003))),
                body_twist=[.8,-.01,.02],activation_source_ns=5_000_000_000,
                goal_pose=list(b.world_from_route.pose(GOAL)),deadline_source_ns=T+22_000_000_000,deadline_ns=T+22_000_000_000)


def test_gate_preserves_all_nongeometric_fields_metadata_and_public_method_identity():
    b=bridge(SE2(8.,-9.,1.3));event=gate_event(b);before=copy.deepcopy(event)
    view=b.gate(event,META,T+2_000_000);out=view.event()
    assert event==before and view.metadata is META
    for k in event:
        if k not in ('pose_map','goal_pose'):assert out[k]==event[k] and type(out[k]) is type(event[k])
    assert out['goal_pose']==list(GOAL) and out['method']==METHOD
    np.testing.assert_allclose(out['pose_map'],(-3.999,.002,.003),atol=2e-15,rtol=0)
    out['source_ns']=1;assert view.event()['source_ns']==T
    with pytest.raises(ValueError): b.gate(dict(event,seed=True),META,T+2_000_000)
    with pytest.raises(ValueError): b.gate(dict(event,source_ns=T+1),META,T+2_000_000)
    with pytest.raises(ValueError): b.gate(dict(event,deadline_ns=T+22_000_000_001),META,T+2_000_000)


def raster():
    pixels=np.full((8,12),254,dtype=np.uint8);pixels[[0,-1],:]=205;pixels[:,[0,-1]]=205;pixels[2:6,4:6]=0
    pgm=b'P5\n12 8\n255\n'+pixels[::-1].tobytes()
    metadata=yaml.safe_dump(dict(resolution=.2,origin=[-1.,-.8,0.],negate=0,occupied_thresh=.65,free_thresh=.196,mode='trinary')).encode()
    return pixels,pgm,metadata


def test_canonical_raster_union_and_world_polygons_not_world_AABBs():
    pixels,pgm,metadata=raster();prior=canonical_static_prior(pgm,metadata,image_sha256=hashlib.sha256(pgm).hexdigest(),metadata_sha256=hashlib.sha256(metadata).hexdigest())
    grid=SharedStaticMap(pgm,metadata);assert verify_static_union(prior.legacy_scene_fields(),grid)['cell_union_mismatch']==0
    t=SE2(5.,3.,.7);polygons=unary_union([Polygon(p) for p in prior.world_polygons(t)])
    # Independent matrix transformation of each blocked raster cell, not boxes.
    rotation=np.asarray([[math.cos(.7),-math.sin(.7)],[math.sin(.7),math.cos(.7)]])
    cells=[]
    for row,col in np.argwhere(pixels!=254):
        x=-1.+col*.2;y=-.8+row*.2
        corners=np.asarray([(x,y),(x+.2,y),(x+.2,y+.2),(x,y+.2)])@rotation.T+np.asarray([5.,3.])
        cells.append(Polygon(corners))
    exact=unary_union(cells)
    # GEOS overlay can leave topology slivers for differently partitioned,
    # rotated collinear boundaries; compare area and geometric boundary too.
    assert abs(polygons.area-exact.area)<1e-12
    assert polygons.hausdorff_distance(exact)<1e-12
    inflated=unary_union([box(*Polygon(p).bounds) for p in prior.world_polygons(t)])
    assert inflated.difference(exact).area>.1 # Counterexample to rotated-AABB substitution.
    assert prior.legacy_scene_fields()['r2_static_input_adapter']['legacy_implicit_y_wall_m']==20.
    with pytest.raises(ValueError): canonical_static_prior(pgm,metadata,image_sha256='0'*64,metadata_sha256=hashlib.sha256(metadata).hexdigest())
    with pytest.raises(ValueError): canonical_static_prior(pgm,metadata,image_sha256=hashlib.sha256(pgm).hexdigest(),metadata_sha256=hashlib.sha256(metadata).hexdigest(),frame_id='map')


def extract(path,class_name,names,namespace):
    tree=ast.parse(path.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name==class_name)
    selected=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in names]
    # The actual frozen method bodies are compiled without importing ROS/torch.
    module=ast.Module(body=selected,type_ignores=[]);exec(compile(ast.fix_missing_locations(module),str(path),'exec'),namespace)
    return namespace


def test_actual_frozen_policy_pose_scan_and_resampling_consume_equivalent_private_inputs():
    source=W/'src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_episode.py'
    runtime=W/'src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/runtime_adapter.py'
    yaw=lambda q: math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    ns=dict(math=math,rospy=NS(Duration=lambda s:s,logwarn_throttle=lambda *a:None),execution_core=NS(quaternion_yaw=yaw))
    methods=extract(source,'BenchmarkEpisode',('_scan','policy_pose'),ns)
    resample=extract(runtime,'SensorRuntimeAdapter',('resample_scan',),dict(np=np))['resample_scan']
    local=scan();stamp=NS(to_sec=lambda:T/1e9)
    message=NS(header=NS(stamp=stamp,frame_id='laser_link'),ranges=local.ranges,angle_min=local.angle_min,angle_increment=local.angle_increment,range_max=local.range_max)
    results=[]
    for transform in (SE2(),SE2(8.,-4.,.9)):
        b=bridge(transform);canonical=pose();world=canonical.transformed(transform)
        odom=b.odometry(OdometrySample(META,'map','base_link',world,TWIST),T+2_000_000)
        tf=b.laser_transform(TransformSample(META,'map','laser_link',world),local,T+2_000_000)
        def ros_pose(p):return NS(position=NS(x=p.xyz[0],y=p.xyz[1],z=p.xyz[2]),orientation=NS(**dict(zip(('x','y','z','w'),p.xyzw))))
        captured=[];model=NS(beam_angles=np.linspace(-math.pi,math.pi,72,endpoint=False),maximum_range_m=8.)
        model.resample_scan=lambda *a:resample(model,*a)
        model.append_scan_with_pose=lambda *a:captured.append(a)
        transform_msg=NS(transform=NS(translation=ros_pose(tf.pose).position,rotation=ros_pose(tf.pose).orientation))
        fake=NS(odom=NS(pose=NS(pose=ros_pose(odom.pose))),spawn_world_x=-5.4,runtime=model,scan_stamps=[],collecting=False,last_runtime_scan_stamp_s=None,scan_transform_success_count=0,scan_transform_failure_count=0,tf_buffer=NS(lookup_transform=lambda *a:transform_msg))
        methods['_scan'](fake,message)
        assert fake.scan_transform_success_count==1 and fake.last_runtime_scan_stamp_s==T/1e9
        results.append((methods['policy_pose'](fake),captured[0]))
    np.testing.assert_allclose(results[0][0],results[1][0],rtol=0,atol=2e-15)
    np.testing.assert_array_equal(results[0][1][0],results[1][1][0])
    np.testing.assert_allclose(results[0][1][1],results[1][1][1],rtol=0,atol=2e-15)
    assert results[0][1][2]==results[1][1][2]==T/1e9
    assert len(authenticate_frozen(W))==4
