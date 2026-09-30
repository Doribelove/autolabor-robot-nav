"""Actual frozen-v1 vs batch-v2 spatial/time expression comparisons."""
from pathlib import Path
import sys,math,copy
import numpy as np
import pytest
from shapely.geometry import Polygon,box,Point
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from sensor_tracks_v1 import SensorTracks,TrackConfig,HardOccupancy,prediction_request
from sensor_tracks_v2 import ConvexFootprint,BatchHardOccupancy,NUMERICAL_SUBTRACTION_M
from route_sensor_buffer import ScanFrame
BASE=1_000_000_000
class Grid:
    image_sha256='a'*64;yaml_sha256='b'*64
    def clearance(self,p):return math.inf
class Wall(Grid):
    def clearance(self,p):return abs(p[1]-4.)
def accept(t,stamp,points):
    f=ScanFrame(stamp,np.ones(72),np.asarray(points,dtype=float).reshape(-1,2),(0.,0.,0.))
    return t.accept(f,tf_stamp_ns=stamp,odom_state=dict(stamp_ns=stamp,pose_map=(0.,0.,0.),body_twist=(0.,0.,0.)),now_ns=stamp)
def cloud(x,y=0):return [(x-.02,y),(x,y+.03),(x+.02,y)]
def hull(n=16):
    a=np.linspace(0,2*math.pi,n,endpoint=False);return ConvexFootprint(np.column_stack((.6*np.cos(a),.4*np.sin(a))))
def geometry(foot,p):
    c,s=math.cos(p[2]),math.sin(p[2]);return Polygon(foot.vertices@np.asarray([[c,s],[-s,c]])+p[:2])
def reference(snapshot,now,foot,poses,offsets):
    v1=HardOccupancy(snapshot,now);return np.asarray([v1.distance_lower_bound(geometry(foot,p),int(o)) for p,o in zip(poses,offsets)])
def assert_bound(snapshot,now,foot,poses,offsets):
    actual=BatchHardOccupancy(snapshot,now).distance_lower_bounds(foot,poses,offsets)
    expected=reference(snapshot,now,foot,poses,offsets)
    assert np.all(actual<=expected+1e-12)
    np.testing.assert_allclose(actual,expected-NUMERICAL_SUBTRACTION_M,rtol=0,atol=2e-11)
    return actual,expected

@pytest.mark.parametrize('n',[3,4,8,16,32])
def test_random_rotated_actual_hulls_match_v1_at_varied_source_ages_and_horizons(n):
    rng=np.random.RandomState(100+n);t=SensorTracks(Grid())
    for i in range(4):s=accept(t,BASE+i*100_000_000,rng.uniform(-5,5,(64,2)))
    poses=rng.uniform([-7,-7,-math.pi],[7,7,math.pi],(200,3));offs=rng.randint(0,81,200)*100_000_000
    assert_bound(s,s.source_ns+200_000_000,hull(n),poses,offs)


def test_crossing_ambiguity_then_loss_and_sparse_all_supports_retained():
    t=SensorTracks(Grid());accept(t,BASE,cloud(2,-.3)+cloud(2,.3)+[(4,4)])
    accept(t,BASE+100_000_000,cloud(2,-.1)+cloud(2,.1));s=accept(t,BASE+200_000_000,[])
    assert s.tracks[0].ambiguous and s.sparse_supports
    batch=BatchHardOccupancy(s,s.source_ns)
    assert batch.support_count==len(s.tracks)+len(s.sparse_supports)+len(s.baseline_points)+len(s.tentative_map_points)
    poses=np.asarray([[0,0,0],[2,0,0],[4,4,.3],[3,1,-.8]],dtype=float)
    a,b=assert_bound(s,s.source_ns,hull(),poses,np.asarray([0,1,2,3])*1_000_000_000)
    assert a[1]<0 and a[2]<0


def test_tentative_static_and_baseline_both_remain_in_batch():
    t=SensorTracks(Wall());s=accept(t,BASE,cloud(2)+cloud(2,4))
    assert s.baseline_points and s.tentative_map_points
    batch=BatchHardOccupancy(s,BASE);assert batch.support_count==len(s.tracks)+len(s.baseline_points)+len(s.tentative_map_points)
    poses=np.asarray([[2,4,0],[2,0,0],[0,0,0]],dtype=float)
    assert_bound(s,BASE,hull(),poses,np.asarray([0,200_000_000,3_000_000_000]))


def test_repeated_geometry_different_horizons_common_slope_exact():
    t=SensorTracks(Grid());s=accept(t,BASE,cloud(2));poses=np.tile([0.,0.,.2],(248,1));offs=np.arange(248,dtype=np.int64)%31*100_000_000
    a,b=assert_bound(s,BASE,hull(),poses,offs)
    assert len(set(np.round(a+1.35*offs/1e9,12)))==1


def test_2048_sparse_different_rotated_footprints_not_only_cache_hits():
    points=[(x*.4,y*.4) for x in range(64) for y in range(32)];t=SensorTracks(Grid())
    accept(t,BASE,points[:1024]);s=accept(t,BASE+100_000_000,points[1024:]);assert len(s.sparse_supports)==2048
    rng=np.random.RandomState(7);poses=rng.uniform([-2,-2,-math.pi],[27,15,math.pi],(248,3));offs=np.arange(248,dtype=np.int64)%31*100_000_000
    assert_bound(s,s.source_ns,hull(),poses,offs)


def test_empty_occupancy_returns_inf_not_false_safety_claim():
    t=SensorTracks(Grid());s=accept(t,BASE,[]);b=BatchHardOccupancy(s,BASE)
    assert math.isinf(b.distance_lower_bounds(hull(),[[0,0,0]],[0])[0])
    assert b.identity['physical_safety_certified'] is False


@pytest.mark.parametrize('bad',[[[0,0],[1,0],[.2,.2],[1,1],[0,1]],[[0,0],[1,0],[0,0]],[[0,0],[math.nan,0],[0,1]],[[0,0],[1e-12,0],[0,1e-12]]])
def test_no_silent_hull_change_for_invalid_or_concave_footprint(bad):
    with pytest.raises(ValueError):ConvexFootprint(bad)


def test_closed_or_clockwise_vertex_order_matches_same_geometry():
    f=hull(4);clockwise=ConvexFootprint(f.vertices[::-1]);closed=ConvexFootprint(np.vstack((f.vertices,f.vertices[0])))
    t=SensorTracks(Grid());s=accept(t,BASE,cloud(2));b=BatchHardOccupancy(s,BASE)
    poses=[[1,1,.8],[-1,2,-.3]];offs=[0,1_000_000_000]
    np.testing.assert_allclose(b.distance_lower_bounds(f,poses,offs),b.distance_lower_bounds(clockwise,poses,offs),rtol=0,atol=1e-12)
    np.testing.assert_array_equal(b.distance_lower_bounds(f,poses,offs),b.distance_lower_bounds(closed,poses,offs))


@pytest.mark.parametrize('mode',['stale','future','float_offset','negative','over_horizon','wrong_count','nan_pose','too_far','bool_offset','empty','not_footprint'])
def test_causal_and_numeric_input_failures(mode):
    t=SensorTracks(Grid());s=accept(t,BASE,cloud(2));now=BASE;poses=[[0,0,0]];offs=[0];foot=hull()
    if mode=='stale':now+=200_000_001
    elif mode=='future':now-=1
    elif mode=='float_offset':offs=[0.]
    elif mode=='negative':offs=[-1]
    elif mode=='over_horizon':offs=[8_000_000_001]
    elif mode=='wrong_count':offs=[0,1]
    elif mode=='nan_pose':poses=[[0,math.nan,0]]
    elif mode=='too_far':poses=[[10000,0,0]]
    elif mode=='bool_offset':offs=[False]
    elif mode=='empty':poses=[];offs=[]
    elif mode=='not_footprint':foot=foot.vertices
    with pytest.raises(ValueError):BatchHardOccupancy(s,now).distance_lower_bounds(foot,poses,offs)


def test_frozen_snapshot_and_footprint_data_not_aliased_or_mutated():
    t=SensorTracks(Grid());s=accept(t,BASE,cloud(2));f=hull();b=BatchHardOccupancy(s,BASE);before=copy.deepcopy(s)
    b.distance_lower_bounds(f,[[0,0,0]],[0]);assert s==before
    with pytest.raises(ValueError):b.centers[0]=[99,99]
    with pytest.raises(ValueError):f.vertices[0]=[99,99]


def test_geometry_boundary_touch_inside_overlap_negative_and_large_supported_coordinates():
    t=SensorTracks(Grid());s=accept(t,BASE,[(.5,0),(0,0),(-.5,0),(9998,9998)])
    f=ConvexFootprint([[-.5,-.35],[.5,-.35],[.5,.35],[-.5,.35]])
    poses=np.asarray([[0,0,0],[1,0,0],[-1,0,0],[9998,9998,.3]],dtype=float)
    a,b=assert_bound(s,BASE,f,poses,np.zeros(4,dtype=np.int64));assert a[0]<0


def test_metadata_keeps_same_tracking_and_prediction_semantics():
    t=SensorTracks(Grid());s=accept(t,BASE,cloud(2));b=BatchHardOccupancy(s,BASE)
    assert b.identity['tracking_schema']==s.schema
    assert b.identity['predictor_request_schema_unchanged'] is True
    assert b.identity['occupancy_removed'] is False and b.identity['learned_interaction'] is False


def delayed_ready():
    import sensor_tracks_v2 as v2
    t=v2.SensorTracks(Grid());rows=[]
    for i in range(6):
        stamp=BASE+i*200_000_000;f=ScanFrame(stamp,np.ones(72),np.asarray(cloud(2)),(0.,0.,0.))
        s=t.accept(f,tf_stamp_ns=stamp,odom_state=dict(stamp_ns=stamp,pose_map=(0.,0.,0.),body_twist=(0.,0.,0.)),now_ns=stamp+100_000_000);rows.append((stamp,f))
    candidates=[dict(candidate_id='stop',states=[dict(offset_ns=j*1_000_000_000,base_pose=(0,0,0),rear_pose=(-.325,0,0),body_twist=(0,0,0),center_steering=0) for j in range(2)])]
    return t,rows,candidates


@pytest.mark.parametrize('entry',['scalar_bare','batch_bare','request','tracker_scalar','tracker_batch'])
def test_every_new_public_request_and_occupancy_entry_rejects_backdated_availability(entry):
    import sensor_tracks_v2 as v2
    t,rows,candidates=delayed_ready();s=t.snapshot;now=s.source_ns
    with pytest.raises((ValueError,RuntimeError)):
        if entry=='scalar_bare':v2.HardOccupancy(s,now)
        elif entry=='batch_bare':v2.BatchHardOccupancy(s,now)
        elif entry=='request':v2.prediction_request(s,rows,candidates,decision_ns=now)
        elif entry=='tracker_scalar':t.occupancy(now)
        elif entry=='tracker_batch':t.batch_occupancy(now)


def test_valid_prediction_schema_and_bytes_equal_frozen_v1_with_no_action_semantics_change():
    import sensor_tracks_v2 as v2
    t,rows,candidates=delayed_ready();s=t.snapshot;now=s.accepted_now_ns
    assert v2.prediction_request(s,rows,candidates,decision_ns=now)==prediction_request(s,rows,candidates,decision_ns=now)
    scalar=v2.HardOccupancy(s,now).distance_lower_bound(geometry(hull(),np.asarray([0,0,0])),0)
    batch=v2.BatchHardOccupancy(s,now).distance_lower_bounds(hull(),[[0,0,0]],[0])[0]
    assert batch<=scalar and scalar-batch==pytest.approx(NUMERICAL_SUBTRACTION_M,abs=1e-12)
    assert isinstance(t.occupancy(now),v2.HardOccupancy)
    assert isinstance(t.batch_occupancy(now),v2.BatchHardOccupancy)


@pytest.mark.parametrize('entry',['scalar_bare','batch_bare','request'])
@pytest.mark.parametrize('delta',[-1,0,1])
def test_actual_availability_exact_one_nanosecond_boundaries(entry,delta):
    import sensor_tracks_v2 as v2
    t,rows,candidates=delayed_ready();s=t.snapshot;now=s.accepted_now_ns+delta
    def call():
        if entry=='scalar_bare':return v2.HardOccupancy(s,now)
        if entry=='batch_bare':return v2.BatchHardOccupancy(s,now)
        return v2.prediction_request(s,rows,candidates,decision_ns=now)
    if delta<0:
        with pytest.raises(ValueError):call()
    else:assert call() is not None


@pytest.mark.parametrize('entry',['scalar_bare','batch_bare','request'])
def test_late_acceptance_does_not_extend_original_200ms_source_freshness(entry):
    import sensor_tracks_v2 as v2
    t,rows,candidates=delayed_ready();s=t.snapshot
    def call(now):
        if entry=='scalar_bare':return v2.HardOccupancy(s,now)
        if entry=='batch_bare':return v2.BatchHardOccupancy(s,now)
        return v2.prediction_request(s,rows,candidates,decision_ns=now)
    assert call(s.source_ns+200_000_000) is not None
    with pytest.raises(ValueError):call(s.source_ns+200_000_001)
