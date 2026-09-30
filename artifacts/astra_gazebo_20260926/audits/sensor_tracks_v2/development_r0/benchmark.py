"""Fresh offline timing of exact/fixed-GEOS and batch kernels. No ROS/episodes."""
from pathlib import Path
import argparse,hashlib,json,platform,sys,time
import numpy as np
from shapely.geometry import Polygon
S=Path(__file__).resolve().parents[2];W=S/'workspace';P=W/'scripts/astra_gazebo';sys.path.insert(0,str(P))
import sensor_tracks_v1 as old
import sensor_tracks_v2 as new
from route_sensor_buffer import ScanFrame
from route_candidate_core import FullFootprintEnvelope,RouteConfig
BASE=1_000_000_000
V1_SHA='3f0cb949640ba93494bfc511538a9776a171b5deaf09dee3c10dd320fd67076b'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def bind(p):return dict(path=str(p),sha256=sha(p),bytes=p.stat().st_size)
class Grid:
    image_sha256='a'*64;yaml_sha256='b'*64
    def clearance(self,p):return float('inf')
def accept(t,stamp,points):
    return t.accept(ScanFrame(stamp,np.ones(72),np.asarray(points,dtype=float).reshape(-1,2),(0.,0.,0.)),tf_stamp_ns=stamp,
                    odom_state=dict(stamp_ns=stamp,pose_map=(0.,0.,0.),body_twist=(0.,0.,0.)),now_ns=stamp)
def fixture(wide_ages=False):
    points=np.asarray([(x*.4,y*.4) for x in range(64) for y in range(32)])
    t=new.SensorTracks(Grid())
    if wide_ages:
        for i,chunk in enumerate(np.array_split(points,4)):s=accept(t,BASE+i*9_000_000_000,chunk)
    else:
        for i,chunk in enumerate(np.array_split(points,2)):s=accept(t,BASE+i*100_000_000,chunk)
    assert len(s.sparse_supports)==2048
    return s

def polygon(foot,p):
    c,s=np.cos(p[2]),np.sin(p[2]);return Polygon(foot.vertices@np.asarray([[c,s],[-s,c]])+p[:2])
def summary(xs):return dict(count=len(xs),median_ms=float(np.median(xs)),p95_ms=float(np.percentile(xs,95)),max_ms=float(np.max(xs)))
def bench(snapshot,foot,poses,offsets,trials):
    # Compare all 248 outputs before timing; this is not a timing assertion.
    ref=old.HardOccupancy(snapshot,snapshot.source_ns)
    expected=np.asarray([ref.distance_lower_bound(polygon(foot,p),int(o)) for p,o in zip(poses,offsets)])
    actual=new.BatchHardOccupancy(snapshot,snapshot.source_ns).distance_lower_bounds(foot,poses,offsets)
    delta=actual-expected
    assert np.all(delta<=1e-12) and np.max(abs(delta+new.NUMERICAL_SUBTRACTION_M))<2e-11
    runs={key:dict(construct=[],query=[],whole=[]) for key in ('v1','v2')}
    for trial in range(trials):
        # Counterbalance timing order; v1 includes per-pose Polygon creation,
        # v2 includes poses->vertices and internal repeated-geometry grouping.
        for key in (('v1','v2') if trial%2==0 else ('v2','v1')):
            start=time.perf_counter_ns()
            env=(old.HardOccupancy if key=='v1' else new.BatchHardOccupancy)(snapshot,snapshot.source_ns)
            mid=time.perf_counter_ns()
            if key=='v1':values=np.asarray([env.distance_lower_bound(polygon(foot,p),int(o)) for p,o in zip(poses,offsets)])
            else:values=env.distance_lower_bounds(foot,poses,offsets)
            end=time.perf_counter_ns()
            assert np.all(np.isfinite(values))
            runs[key]['construct'].append((mid-start)/1e6);runs[key]['query'].append((end-mid)/1e6);runs[key]['whole'].append((end-start)/1e6)
    return dict(query_count=len(poses),unique_base_poses=len(np.unique(poses,axis=0)),support_count=env.support_count,
                scalar_comparisons=len(delta),max_batch_minus_v1_m=float(max(delta)),max_abs_difference_from_declared_subtraction_m=float(max(abs(delta+new.NUMERICAL_SUBTRACTION_M))),
                timing={k:{part:summary(v) for part,v in data.items()} for k,data in runs.items()},raw_trials_ms=runs)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);ap.add_argument('--trials',type=int,default=20);args=ap.parse_args()
    assert args.trials>=10
    assert sha(P/'sensor_tracks_v1.py')==V1_SHA
    urdf=S/'runs/passive_sweep_q05_static/robot_description_runtime.xml'
    footprint=FullFootprintEnvelope(urdf.read_text(),RouteConfig())
    assert footprint.component_count==9
    foot=new.ConvexFootprint(footprint.vertices)
    offsets=np.arange(248,dtype=np.int64)%31*100_000_000
    rng=np.random.RandomState(913) # offline fixture RNG, not a simulator seed
    normal=fixture();aged=fixture(True)
    repeated=np.tile([0.,0.,0.],(248,1))
    varied=rng.uniform([-2,-2,-np.pi],[27,15,np.pi],(248,3))
    route=np.asarray([(k*.07,(action-3)*.10,.3*np.sin(k/9.+action)) for action in range(8) for k in range(31)])
    rows=[]
    for name,snapshot,poses in [('original_repeated_geometry',normal,repeated),('distinct_full_domain_rotations',normal,varied),
                                ('distinct_candidate_like_near_route',normal,route),('distinct_27s_lost_mixed_ages',aged,varied)]:
        rows.append(dict(workload=name,**bench(snapshot,foot,poses,offsets,args.trials)))
    assert sha(P/'sensor_tracks_v1.py')==V1_SHA
    result=dict(status='offline_batch_kernel_timing',scope='synthetic input/query component timing, not Gazebo, live deadlines or full planner WCET',
        python=sys.version,executable=sys.executable,numpy=np.__version__,platform=platform.platform(),trials=args.trials,
        footprint=dict(actual_URDF=bind(urdf),component_count=9,vertices=foot.vertices.tolist(),source=bind(P/'route_candidate_core.py')),
        cases=rows,maximum_all_v2_whole_ms=max(x['timing']['v2']['whole']['max_ms'] for x in rows),
        v2_all_measured_whole_under20ms=all(x['timing']['v2']['whole']['max_ms']<20 for x in rows),
        v2_all_measured_whole_under50ms=all(x['timing']['v2']['whole']['max_ms']<50 for x in rows),
        constructor_included=True,static_footprint_validation_cached_once=True,scanning_tracking_not_included=True,
        full_planner_qualification=False,sources=[bind(Path(__file__).resolve()),bind(P/'sensor_tracks_v1.py'),bind(P/'sensor_tracks_v2.py')])
    Path(args.output).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('maximum_all_v2_whole_ms','v2_all_measured_whole_under20ms','v2_all_measured_whole_under50ms')}))
if __name__=='__main__':main()
