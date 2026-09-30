"""Independent offline consumed-code and boundary fixtures; no ROS or inference."""
import ast, math, hashlib, json, sys
from pathlib import Path
from types import SimpleNamespace as NS
from dataclasses import replace
import numpy as np
import pytest
import yaml
S=Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');W=S/'workspace';P=W/'scripts/astra_gazebo';sys.path.insert(0,str(P))
import frozen_r2_route_frame_bridge_n05 as b
T=8_123_456_789
M=b.SourceMetadata(T,T-2_000_000,T+3_000_000,987654321)
TW=b.BodyTwist((.7,-.13,.02),(.03,-.04,.21))

def make(t=None):
    t=t or b.SE2(12.,-8.,.8)
    return b.FrozenR2RouteFrameBridge(world_from_route=t,world_from_odom=b.SE2(-3.,6.,-.7),spawn_world=t.pose(b.SPAWN),gate_world=t.pose(b.GATE),goal_world=t.pose(b.GOAL),nominal_reference_world=(t.point((-4.,0.)),t.point((0.,0.)),t.point((5.,0.))),guide_world=(t.point((-4.,0.)),t.point((0.,1.)),t.point((5.,0.))),run_id='independent_offline_fixture',seed=1)
def quat(roll,pitch,yaw):
    a,d,c=[x/2 for x in (roll,pitch,yaw)];ca,sa,cd,sd,cc,sc=math.cos(a),math.sin(a),math.cos(d),math.sin(d),math.cos(c),math.sin(c)
    return (sa*cd*cc-ca*sd*sc,ca*sd*cc+sa*cd*sc,ca*cd*sc-sa*sd*cc,ca*cd*cc+sa*sd*sc)
def rot(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def pose(x=-1.,y=.4,yaw=.2):return b.Pose3((x,y,.6),quat(.11,-.08,yaw))
def laser(metadata=M):return b.LaserSample(metadata,'laser_link',tuple(float('inf') if i%31==0 else .2+i*.005 for i in range(720)),-math.pi,math.pi,2*math.pi/719,.05,8.,.0001,.072,())
def extract(path,cls,name,ns):
    node=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name==cls)
    method=next(n for n in node.body if isinstance(n,ast.FunctionDef) and n.name==name)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),str(path),'exec'),ns)
    return ns[name]
def ros_pose(p):return NS(position=NS(**dict(zip(('x','y','z'),p.xyz))),orientation=NS(**dict(zip(('x','y','z','w'),p.xyzw))))
def policy_methods():
    legacy=W/'src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_episode.py'
    yaw=lambda q:math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    env=dict(math=math,rospy=NS(Duration=lambda x:x,logwarn_throttle=lambda *x:None),execution_core=NS(quaternion_yaw=yaw))
    return extract(legacy,'BenchmarkEpisode','policy_pose',env),extract(legacy,'BenchmarkEpisode','_scan',env)

@pytest.mark.parametrize('yaw',[0.,.4,-.9,math.pi/2,-math.pi/2,math.pi,2.4])
def test_full_rotation_and_independent_odom_private_compensation(yaw):
    t=b.SE2(12.,-8.,yaw);bridge=make(t);p=pose();world=p.transformed(t)
    R=np.array([[math.cos(yaw),-math.sin(yaw),0.],[math.sin(yaw),math.cos(yaw),0.],[0.,0.,1.]])
    np.testing.assert_allclose(rot(world.xyzw),R@rot(p.xyzw),atol=1e-14,rtol=0)
    for parent,measurement in [('map',world),('odom',world.transformed(bridge.world_from_odom.inverse()))]:
        measured=b.OdometrySample(M,parent,'base_link',measurement,TW);out=bridge.odometry(measured,T+3_000_000)
        np.testing.assert_allclose(np.array(out.pose.xyz)+np.array([-5.4,0,0]),p.xyz,atol=1e-14,rtol=0)
        np.testing.assert_allclose(rot(out.pose.xyzw),rot(p.xyzw),atol=1e-14,rtol=0)
        assert out.body_twist is TW and out.metadata is M and measured.pose is measurement
        assert bridge.command(TW) is TW
    assert bridge.full_polyline_consumed is False and bridge.guide_length_m>9.

@pytest.mark.parametrize('parent',['map','odom'])
def test_actual_legacy_consumers_distinct_laser_pose_and_second_tf_failure(parent):
    bridge=make();base=pose();light=pose(x=-.62,y=.44,yaw=.24)
    def sample(p,child,odom=False):
        world=p.transformed(bridge.world_from_route)
        measured=world if parent=='map' else world.transformed(bridge.world_from_odom.inverse())
        return b.OdometrySample(M,parent,child,measured,TW) if odom else b.TransformSample(M,parent,child,measured)
    od=bridge.odometry(sample(base,'base_link',True),T+3_000_000)
    tf=bridge.laser_transform(sample(light,'laser_link'),laser(),T+3_000_000)
    stamp=NS(to_sec=lambda:T/1e9,to_nsec=lambda:T)
    scans=[];requests=[];message=NS(header=NS(stamp=stamp,frame_id='laser_link'),ranges=laser().ranges,angle_min=-math.pi,angle_increment=2*math.pi/719,range_max=8.)
    runtime=W/'src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/runtime_adapter.py'
    resample=extract(runtime,'SensorRuntimeAdapter','resample_scan',{'np':np})
    runtime_obj=NS(beam_angles=np.linspace(-math.pi,math.pi,72,endpoint=False),maximum_range_m=8.)
    runtime_obj.resample_scan=lambda *args:resample(runtime_obj,*args)
    runtime_obj.append_scan_with_pose=lambda *args:scans.append(args)
    def lookup(target,child,source,timeout):
        requests.append((target,child,source,timeout));assert (target,child,source)==('odom','laser_link',stamp)
        return NS(transform=NS(translation=ros_pose(tf.pose).position,rotation=ros_pose(tf.pose).orientation))
    fake=NS(odom=NS(pose=NS(pose=ros_pose(od.pose))),spawn_world_x=-5.4,runtime=runtime_obj,scan_stamps=[],collecting=False,last_runtime_scan_stamp_s=None,scan_transform_success_count=0,scan_transform_failure_count=0,tf_buffer=NS(lookup_transform=lookup))
    policy,scan=policy_methods();np.testing.assert_allclose(policy(fake),base.planar(),atol=1e-14,rtol=0)
    scan(fake,message);np.testing.assert_allclose(scans[0][1],light.planar(),atol=1e-14,rtol=0)
    np.testing.assert_array_equal(scans[0][0],resample(runtime_obj,message.ranges,message.angle_min,message.angle_increment,message.range_max))
    assert scans[0][2]==T/1e9 and requests[0][3]==.03
    # Exact old consumer silently catches second lookup failure. A new wrapper
    # must retain actual-append proof, not claim first precheck commits a scan.
    prior=fake.last_runtime_scan_stamp_s
    next_message=NS(**vars(message));next_message.header=NS(stamp=NS(to_sec=lambda:(T+210_000_000)/1e9),frame_id='laser_link')
    def failed_lookup(*args):raise RuntimeError('independent actual second TF failure')
    fake.tf_buffer.lookup_transform=failed_lookup;scan(fake,next_message)
    assert fake.last_runtime_scan_stamp_s==prior and len(scans)==1 and fake.scan_transform_failure_count==1

@pytest.mark.parametrize('offset,ok',[(2_999_999,False),(3_000_000,True),(200_000_000,True),(200_000_001,False)])
def test_availability_and_original_source_exact_boundary(offset,ok):
    action=lambda:make().odometry(b.OdometrySample(M,'map','base_link',pose(),TW),T+offset)
    if ok:assert action().metadata is M
    else:
        with pytest.raises(ValueError):action()

@pytest.mark.parametrize('fault',['tf_stamp_plus1','late_tf_availability','wrong_laser_child','wrong_real_parent','stale_tf','wrong_scan_frame'])
def test_scan_tf_time_identity_faults_rejected(fault):
    bridge=make();scan=laser();tf=b.TransformSample(M,'map','laser_link',pose())
    if fault=='tf_stamp_plus1':tf=replace(tf,metadata=replace(M,source_ns=T+1))
    if fault=='late_tf_availability':tf=replace(tf,metadata=replace(M,available_source_ns=T+3_000_001))
    if fault=='wrong_laser_child':tf=replace(tf,child_frame='base_link')
    if fault=='wrong_real_parent':tf=replace(tf,parent_frame='world_unbound')
    if fault=='stale_tf':tf=replace(tf,metadata=b.SourceMetadata(T-201_000_000,T-201_000_000,T,999))
    if fault=='wrong_scan_frame':scan=replace(scan,frame_id='base_link')
    with pytest.raises(ValueError):bridge.laser_transform(tf,scan,T+3_000_000)

@pytest.mark.parametrize('kind',['source_bool','source_float','source_int64_overflow','availability_before_receipt','receipt_source_too_far','nan_twist'])
def test_typed_and_no_backdating_metadata(kind):
    with pytest.raises(ValueError):
        if kind=='source_bool':replace(M,source_ns=True)
        elif kind=='source_float':replace(M,source_ns=float(T))
        elif kind=='source_int64_overflow':replace(M,source_ns=2**63)
        elif kind=='availability_before_receipt':replace(M,received_source_ns=T+4_000_000)
        elif kind=='receipt_source_too_far':replace(M,received_source_ns=T-200_000_001).require(T+3_000_000)
        else:b.BodyTwist((1.,float('nan'),0.),(0.,0.,0.))

def test_canonical_cells_asymmetric_orientation_and_unknowns_exact_once():
    pix=np.full((9,13),254,np.uint8);pix[[0,-1],:]=205;pix[:,[0,-1]]=205;pix[1,8:11]=0;pix[3:7,2]=0;pix[6:8,8:10]=205
    pgm=b'P5\n13 9\n255\n'+pix[::-1].tobytes();meta=yaml.safe_dump(dict(resolution=.1,origin=[-2.3,-.8,0.],negate=0,occupied_thresh=.65,free_thresh=.196,mode='trinary')).encode()
    args=dict(image_sha256=hashlib.sha256(pgm).hexdigest(),metadata_sha256=hashlib.sha256(meta).hexdigest())
    p=b.canonical_static_prior(pgm,meta,**args);counts=np.zeros(pix.shape,int)
    for x,y,l,w in p.boxes_xy_length_width:
        x0,x1=round((x-l/2+2.3)/.1),round((x+l/2+2.3)/.1);y0,y1=round((y-w/2+.8)/.1),round((y+w/2+.8)/.1);counts[y0:y1,x0:x1]+=1
    np.testing.assert_array_equal(counts,(pix!=254).astype(int));assert p.blocked_cell_count==int((pix!=254).sum())
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    t=b.SE2(2.,-.7,.73);out=unary_union([Polygon(q) for q in p.world_polygons(t)])
    c,s=math.cos(.73),math.sin(.73);cells=[]
    for iy,ix in np.argwhere(pix!=254):
        corners=[]
        for dx,dy in [(0,0),(1,0),(1,1),(0,1)]:
            x,y=-2.3+(ix+dx)*.1,-.8+(iy+dy)*.1;corners.append((2+c*x-s*y,-.7+s*x+c*y))
        cells.append(Polygon(corners))
    truth=unary_union(cells);assert abs(out.area-truth.area)<1e-12 and out.hausdorff_distance(truth)<1e-12
    with pytest.raises(ValueError):b.canonical_static_prior(pgm[:-1]+b'\xfe',meta,**args)
    yawmeta=yaml.safe_dump(dict(yaml.safe_load(meta),origin=[-2.3,-.8,.1])).encode()
    with pytest.raises(ValueError):b.canonical_static_prior(pgm,yawmeta,image_sha256=args['image_sha256'],metadata_sha256=hashlib.sha256(yawmeta).hexdigest())

def test_gate_field_preservation_and_legacy_identity_not_bypassed():
    bridge=make();e=dict(schema='astra_common_navigation_v1',run_id=bridge.identity[0],seed=1,method=b.METHOD,source_ns=T,source='actual_odom_header_first_crossing',pose_map=list(bridge.world_from_route.pose((-4.,.01,.01))),body_twist=[.8,.12,.2],activation_source_ns=5_000_000_000,goal_pose=list(bridge.world_from_route.pose(b.GOAL)),deadline_source_ns=T+22_000_000_000,deadline_ns=T+22_000_000_000)
    old=json.dumps(e,sort_keys=True,separators=(',',':'),allow_nan=False).encode();out=bridge.gate(e,M,T+3_000_000)
    assert out.original_event_sha256==hashlib.sha256(old).hexdigest()
    for key in e:
        if key not in ('pose_map','goal_pose'):assert out.event()[key]==e[key] and type(out.event()[key])==type(e[key])
    from frozen_r2_sensor_lifecycle import SharedLifecycle,LifecycleFault
    with pytest.raises(LifecycleFault):SharedLifecycle(bridge.identity[0],1).on_gate(out.event())
    for change in [dict(seed=True),dict(deadline_ns=T+22_000_000_001),dict(method='frozen_r2_v060'),dict(source_ns=T+1),dict(extra=1)]:
        with pytest.raises(ValueError):bridge.gate(dict(e,**change),M,T+3_000_000)
