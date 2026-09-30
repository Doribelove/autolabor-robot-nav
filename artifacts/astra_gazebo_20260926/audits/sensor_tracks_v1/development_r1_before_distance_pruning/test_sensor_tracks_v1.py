"""Offline causal/geometry fault fixtures; none are new Gazebo evidence."""
from dataclasses import replace, FrozenInstanceError
from pathlib import Path
import sys
import math
import copy
import time
import numpy as np
import pytest
from shapely.geometry import Point, box, MultiPoint
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from route_sensor_buffer import ScanFrame, RouteSensorBuffer
from sensor_tracks_v1 import SensorTracks, TrackConfig, TrackerFault, HardOccupancy, prediction_request, nominal_cv, _clusters

BASE=1_000_000_000


class Grid:
    image_sha256='a'*64; yaml_sha256='b'*64
    def __init__(self, obstacle=None):self.obstacle=obstacle
    def clearance(self, point):return math.inf if self.obstacle is None else self.obstacle.distance(Point(point))


def frame(stamp, points, laser=(0.,0.,0.)):
    return ScanFrame(stamp, np.ones(72), np.asarray(points,dtype=float).reshape(-1,2), laser)


def cloud(x,y=0.):return [(x-.025,y),(x,y+.02),(x+.025,y)]


def odom(stamp, pose=(0.,0.,0.), twist=(0.,0.,0.)):
    return dict(stamp_ns=stamp, pose_map=pose, body_twist=twist)


def accept(tracker, stamp, points, **kwargs):
    return tracker.accept(frame(stamp,points), tf_stamp_ns=stamp, odom_state=odom(stamp), now_ns=stamp, **kwargs)


def candidates():
    rows=[]
    for name, y in [('left',.4),('right',-.4)]:
        states=[]
        for t,x,yy in [(0,0.,0.),(1_000_000_000,1.,y)]:
            states.append(dict(offset_ns=t,base_pose=(x,yy,0.),rear_pose=(x-.325,yy,0.),body_twist=(1.,0.,0.),center_steering=0.))
        rows.append(dict(candidate_id=name,states=states))
    return rows


def ready():
    t=SensorTracks(Grid());rows=[]
    for i in range(6):
        stamp=BASE+i*200_000_000;f=frame(stamp,cloud(2.+i*.10));t.accept(f,tf_stamp_ns=stamp,odom_state=odom(stamp),now_ns=stamp);rows.append((stamp,f))
    return t,rows


def test_two_separated_tracks_have_bounded_velocity_and_unique_local_ids():
    t=SensorTracks(Grid());accept(t,BASE,cloud(2,-1)+cloud(2,1));s=accept(t,BASE+200_000_000,cloud(2.1,-1)+cloud(1.9,1))
    assert len(s.tracks)==2 and len({r.track_id for r in s.tracks})==2
    assert all(not r.ambiguous for r in s.tracks)
    assert np.allclose(s.tracks[0].velocity_nominal_mps,(.5,0.))
    assert np.allclose(s.tracks[1].velocity_nominal_mps,(-.5,0.))
    assert all(np.linalg.norm(r.velocity_nominal_mps)<=1.35 for r in s.tracks)


def test_crossing_merges_hypotheses_without_arbitrary_identity_claim():
    t=SensorTracks(Grid());s=accept(t,BASE,cloud(2,-.3)+cloud(2,.3));old=s.tracks
    s=accept(t,BASE+200_000_000,cloud(2,-.12)+cloud(2,.12))
    assert len(s.tracks)==1 and s.tracks[0].ambiguous and s.tracks[0].member_ids==(1,2)
    assert s.tracks[0].velocity_nominal_mps is None
    merged=s.tracks[0]
    for p in old:assert merged.radius_m>=math.dist(merged.center,p.center)+p.radius_m+1.35*.2-1e-12
    assert nominal_cv(s,1_000_000_000)[0]['nominal_center'] is None
    assert s.tracks[0].track_id==1


def test_ambiguous_split_stays_group_and_contains_prior_reachability():
    t=SensorTracks(Grid());accept(t,BASE,cloud(2,-.3)+cloud(2,.3));s=accept(t,BASE+200_000_000,cloud(2,-.12)+cloud(2,.12));before=s.tracks[0]
    s=accept(t,BASE+400_000_000,cloud(2,-.4)+cloud(2,.4));after=s.tracks[0]
    assert len(s.tracks)==1 and after.ambiguous
    assert after.radius_m>=before.radius_m+1.35*.2-1e-12


def test_occlusion_disappearance_never_proves_free_or_discards_track():
    t=SensorTracks(Grid(box(2.9,-2,3.1,2)));s=accept(t,BASE,cloud(2.5));old=s.tracks[0]
    s=accept(t,BASE+200_000_000,[(3.,0.),(3.,.1),(3.,-.1)])
    assert s.tracks==(old,)
    value=t.occupancy(BASE+200_000_000).distance_lower_bound(Point(old.center),0)
    assert value<=-old.radius_m-1.35*.2
    row=nominal_cv(s,0)[0];assert row['status']=='unseen_not_proven_occluded'
    for i in range(2,101):s=accept(t,BASE+i*200_000_000,[])
    assert s.tracks==(old,) and t.occupancy(s.source_ns).distance_lower_bound(Point(2.5,0),0)<-20


def test_sparse_singleton_and_two_returns_never_dropped():
    t=SensorTracks(Grid());s=accept(t,BASE,[(1.,1.),(3.,3.),(3.1,3.)]);assert not s.tracks and len(s.sparse_supports)>=2
    s=accept(t,BASE+200_000_000,[])
    env=t.occupancy(s.source_ns)
    for p in ((1.,1.),(3.,3.),(3.1,3.)):assert env.distance_lower_bound(Point(p),0)<0


def test_multi_scan_map_confirmation_does_not_promote_unmapped_stationary_object():
    t=SensorTracks(Grid(box(5,-1,6,1)));points=cloud(2)+cloud(5)
    for i in range(3):s=accept(t,BASE+i*100_000_000,points)
    stats=dict(s.statistics);assert stats['map_confirmed_count']==3 and stats['baseline_unknown_count']==3
    assert len(s.tracks)==1 and len(s.baseline_points)==3
    assert np.linalg.norm(s.tracks[0].velocity_nominal_mps)==0
    assert t.occupancy(s.source_ns).distance_lower_bound(Point(2,0),0)<0


def test_map_consistency_requires_consecutive_distinct_sources():
    t=SensorTracks(Grid(box(5,-1,6,1)));accept(t,BASE,cloud(5));accept(t,BASE+100_000_000,[])
    s=accept(t,BASE+200_000_000,cloud(5));assert dict(s.statistics)['map_confirmed_count']==0
    with pytest.raises(ValueError):accept(t,s.source_ns,cloud(5))
    assert dict(t.snapshot.statistics)['map_confirmed_count']==0


def test_dynamic_entering_map_band_retains_prior_support_not_erased_by_map_filter():
    t=SensorTracks(Grid(box(3,-1,4,1)));s=accept(t,BASE,cloud(2.5));old=s.tracks[0]
    for i in range(1,4):s=accept(t,BASE+i*100_000_000,cloud(3))
    assert not s.baseline_points and s.tracks==(old,)
    assert t.occupancy(s.source_ns).distance_lower_bound(Point(2.5,0),0)<-.4


def test_no_actor_body_extent_or_complete_detection_claim():
    t=SensorTracks(Grid());ident=t.identity()
    assert ident['learned_candidate_conditioned_prediction_implemented'] is False
    assert ident['calibration_qualified'] is False and ident['physical_safety_certified'] is False
    assert 'explicit sensing assumption' in ident['static_filter']


def test_ego_translation_exact_source_tf_keeps_stationary_map_object_zero_velocity():
    buffer=RouteSensorBuffer();t=SensorTracks(Grid())
    for i,x in enumerate((0.,.1,.2)):
        stamp=BASE+i*100_000_000
        # Three rays whose true endpoints remain fixed as ego translates.
        points=cloud(2)
        local=np.asarray(points)-[x,0.]
        angles=np.arctan2(local[:,1],local[:,0]);ranges=np.linalg.norm(local,axis=1)
        # Exact TF map transform; testing same formula as source buffer plus full
        # RouteSensorBuffer transform in separate full-scan case below.
        mapped=np.column_stack((x+ranges*np.cos(angles),ranges*np.sin(angles)))
        f=frame(stamp,mapped,laser=(x,0.,0.));s=t.accept(f,tf_stamp_ns=stamp,odom_state=odom(stamp,(x,0.,0.),(1.,0.,0.)),now_ns=stamp)
    assert np.linalg.norm(s.tracks[0].velocity_nominal_mps)<1e-12


def test_actual_route_sensor_buffer_rotation_and_translation_are_consumed_once():
    buf=RouteSensorBuffer();t=SensorTracks(Grid());ranges=np.full(720,np.inf);ranges[360:363]=2.
    source=BASE;f=buf.scan(source,source,ranges,-math.pi,2*math.pi/720,.1,30.,(1.,2.,math.pi/2),source)
    s=t.accept(f,tf_stamp_ns=source,odom_state=odom(source,(1,2,math.pi/2)),now_ns=source)
    assert len(s.tracks)==1 and abs(s.tracks[0].center[0]-1)<.03 and abs(s.tracks[0].center[1]-4)<.001


def test_tf_late_retries_same_source_without_consumption_and_never_extends_age():
    t=SensorTracks(Grid());f=frame(BASE,cloud(2))
    with pytest.raises(ValueError):t.accept(f,tf_stamp_ns=BASE-1,odom_state=odom(BASE),now_ns=BASE)
    assert t.source_ns is None and t.snapshot is None
    s=t.accept(f,tf_stamp_ns=BASE,odom_state=odom(BASE),now_ns=BASE+200_000_000)
    assert s.source_ns==BASE and t.health(BASE+200_000_000)==[]
    assert t.health(BASE+200_000_001)
    with pytest.raises(TrackerFault):t.occupancy(BASE+200_000_001)


@pytest.mark.parametrize('delta',[-1,200_000_001])
def test_future_or_stale_input_does_not_advance_source(delta):
    t=SensorTracks(Grid());f=frame(BASE,cloud(2))
    with pytest.raises(ValueError):t.accept(f,tf_stamp_ns=BASE,odom_state=odom(BASE),now_ns=BASE+delta)
    assert t.source_ns is None


@pytest.mark.parametrize('delta',[1,-200_000_001])
def test_future_or_stale_odometry_rejected(delta):
    t=SensorTracks(Grid());f=frame(BASE,cloud(2))
    with pytest.raises(ValueError):t.accept(f,tf_stamp_ns=BASE,odom_state=odom(BASE+delta),now_ns=BASE)
    assert t.source_ns is None


def test_clock_regression_permanent_and_episode_boundary_no_eviction():
    t=SensorTracks(Grid());accept(t,BASE,cloud(2))
    with pytest.raises(TrackerFault):t.health(BASE-1)
    with pytest.raises(TrackerFault):accept(t,BASE+100_000_000,cloud(2))
    t=SensorTracks(Grid(),TrackConfig(maximum_episode_span_ns=200_000_000));old=accept(t,BASE,cloud(2))
    with pytest.raises(TrackerFault):accept(t,BASE+200_000_001,[])
    assert t.snapshot is old


@pytest.mark.parametrize('which',['points','tracks','sparse'])
def test_capacity_fail_closed_without_partial_commit(which):
    cfg={'points':dict(maximum_points=4),'tracks':dict(maximum_tracks=1),'sparse':dict(maximum_sparse_cells=1)}[which]
    t=SensorTracks(Grid(),TrackConfig(**cfg));old=accept(t,BASE,[])
    points={'points':cloud(2)+cloud(4),'tracks':cloud(2)+cloud(4),'sparse':[(2,0),(4,0)]}[which]
    with pytest.raises(TrackerFault):accept(t,BASE+100_000_000,points)
    assert t.snapshot is old and t.source_ns==BASE


@pytest.mark.parametrize('bad',[(math.nan,0),(math.inf,0)])
def test_nonfinite_cloud_cannot_refresh(bad):
    t=SensorTracks(Grid());old=accept(t,BASE,cloud(2))
    with pytest.raises(ValueError):accept(t,BASE+100_000_000,[bad])
    assert t.snapshot is old


def test_snapshot_deep_immutable_and_detached_from_raw_array():
    t=SensorTracks(Grid());f=frame(BASE,cloud(2));s=t.accept(f,tf_stamp_ns=BASE,odom_state=odom(BASE),now_ns=BASE);original=s.baseline_points
    f.points_map[:]=99;assert s.baseline_points==original
    with pytest.raises(FrozenInstanceError):s.source_ns=BASE+1


def test_nonshrinking_hard_distance_against_exact_legacy_cloud_many_shapes_horizons():
    rng=np.random.RandomState(839);t=SensorTracks(Grid(box(4,-2,5,2)))
    for step in range(12):
        stamp=BASE+step*100_000_000;points=np.vstack((rng.uniform([-2,-2],[3,2],(18,2)),cloud(4.5)))
        s=accept(t,stamp,points);env=t.occupancy(stamp+50_000_000);legacy=MultiPoint(s.baseline_points)
        for offset in (0,200_000_000,1_000_000_000,3_000_000_000):
            for _ in range(8):
                x,y=rng.uniform(-3,3,2);query=box(x-.5,y-.35,x+.5,y+.35)
                old=query.distance(legacy)-1.35*(.05+offset/1e9)
                assert env.distance_lower_bound(query,offset)<=old+1e-12


def test_empty_scan_with_no_previous_returns_has_empty_dynamic_envelope_only():
    t=SensorTracks(Grid());s=accept(t,BASE,[])
    assert t.occupancy(BASE).distance_lower_bound(Point(0,0),0)==math.inf
    assert not s.tracks and not s.baseline_points
    assert t.identity()['physical_safety_certified'] is False


def test_clustering_keeps_all_points_permutation_invariant_components():
    points=np.asarray(cloud(1)+cloud(3)+[(7,1)])
    result=_clusters(points,.3);rng=np.random.RandomState(4);other=_clusters(points[rng.permutation(len(points))],.3)
    normalize=lambda groups:sorted(tuple(sorted(map(tuple,g))) for g in groups)
    assert normalize(result)==normalize(other) and sum(map(len,result))==len(points)


def test_request_binds_six_real_sources_candidates_and_new_schema():
    t,rows=ready();s=t.snapshot;r=prediction_request(s,rows,candidates(),decision_ns=s.source_ns)
    assert r['source_cutoff_ns']==s.source_ns and r['learned_interaction'] is False
    assert r['hard_occupancy_replacement_authorized'] is False
    assert len(r['request_sha256'])==64 and len(r['candidates'])==2
    assert max(x['source_ns'] for x in r['real_scan_history'])<=s.source_ns
    assert r['schema']=='astra_candidate_conditioned_sensor_request_v1'


@pytest.mark.parametrize('mode',['padded','future','same_source','not_accepted','content_swap','bucket_gap','stale_decision'])
def test_request_rejects_future_leak_fabricated_or_mutated_history(mode):
    t,rows=ready();s=t.snapshot;rows=copy.deepcopy(rows);now=s.source_ns
    if mode=='padded':rows=rows[1:]
    elif mode=='future':rows[-1]=(s.source_ns+1,frame(s.source_ns+1,cloud(3)))
    elif mode=='same_source':rows[-1]=(rows[-1][0],rows[-2][1])
    elif mode=='not_accepted':rows[0]=(rows[0][0],frame(BASE-1,cloud(2)))
    elif mode=='content_swap':rows[0][1].model_ranges[0]=4.
    elif mode=='bucket_gap':rows[2]=(rows[2][0]+1,rows[2][1])
    elif mode=='stale_decision':now+=200_000_001
    with pytest.raises(ValueError):prediction_request(s,rows,candidates(),decision_ns=now)


@pytest.mark.parametrize('mode',['rear','missing','duplicate','float_time','reverse_time','nonfinite','steer'])
def test_request_rejects_invalid_physical_reference_candidates(mode):
    t,rows=ready();cs=candidates()
    if mode=='rear':cs[0]['states'][0]['rear_pose']=(0,0,0)
    elif mode=='missing':del cs[0]['states'][0]['body_twist']
    elif mode=='duplicate':cs[1]['candidate_id']=cs[0]['candidate_id']
    elif mode=='float_time':cs[0]['states'][0]['offset_ns']=0.
    elif mode=='reverse_time':cs[0]['states'][1]['offset_ns']=0
    elif mode=='nonfinite':cs[0]['states'][0]['body_twist']=(math.nan,0,0)
    elif mode=='steer':cs[0]['states'][0]['center_steering']=.5
    with pytest.raises(ValueError):prediction_request(t.snapshot,rows,cs,decision_ns=t.source_ns)


def test_old_snapshot_request_unchanged_after_future_observations():
    t,rows=ready();s=t.snapshot;r=prediction_request(s,rows,candidates(),decision_ns=s.source_ns)
    accept(t,s.source_ns+100_000_000,cloud(9))
    assert prediction_request(s,rows,candidates(),decision_ns=s.source_ns)==r


def test_deterministic_values_exclude_wall_timing_only():
    one,two=SensorTracks(Grid()),SensorTracks(Grid())
    for i in range(8):
        s=BASE+i*100_000_000;a=accept(one,s,cloud(2+i*.05));b=accept(two,s,cloud(2+i*.05))
        assert a.tracks==b.tracks and a.receipt_sha256==b.receipt_sha256
        assert {k:v for k,v in a.statistics if k!='processing_ns'}=={k:v for k,v in b.statistics if k!='processing_ns'}


def test_configuration_cannot_reduce_legacy_speed_or_extend_freshness():
    for changes in [dict(dynamic_speed_bound_mps=1.34),dict(maximum_source_age_ns=201_000_000),dict(static_tolerance_m=.16),dict(maximum_tracks=2.5)]:
        with pytest.raises(ValueError):TrackConfig(**changes)


def test_empty_or_noninteger_horizon_query_rejected():
    t=SensorTracks(Grid());accept(t,BASE,cloud(2));env=t.occupancy(BASE)
    for off in (-1,1.,8_000_000_001,True):
        with pytest.raises(ValueError):env.distance_lower_bound(Point(0,0),off)
    with pytest.raises(ValueError):env.distance_lower_bound(Point(),0)


def test_candidates_share_initial_reference_and_timestamp_is_decision_not_scan():
    t,rows=ready();cs=candidates();cs[1]['states'][0]['base_pose']=(1,0,0);cs[1]['states'][0]['rear_pose']=(.675,0,0)
    with pytest.raises(ValueError):prediction_request(t.snapshot,rows,cs,decision_ns=t.source_ns)
    r=prediction_request(t.snapshot,rows,candidates(),decision_ns=t.source_ns+50_000_000)
    assert r['candidate_time_origin_ns']==t.source_ns+50_000_000
    assert r['sensor_odometry']['source_ns']<=r['source_cutoff_ns']
    assert r['velocity_support']['calibrated'] is False


def test_module_has_no_ros_or_truth_runtime_imports():
    import ast
    import sensor_tracks_v1 as module
    tree=ast.parse(Path(module.__file__).read_text())
    imports=[n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
    imports += [a.name for n in ast.walk(tree) if isinstance(n,ast.Import) for a in n.names]
    assert not any(n.startswith(('rospy','rosbag','gazebo_msgs','torch')) for n in imports)
